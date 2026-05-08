#include "imu_slave_node.h"

#include <WiFi.h>
#include <esp_wifi.h>
#include <string.h>

namespace {

ImuSlaveApp *g_activeApp = nullptr;

}  // namespace

// 保存节点配置，并用其中的 MPU6500 参数构造传感器驱动。
ImuSlaveApp::ImuSlaveApp(const Config &config) : config_(config), mpu_(config.mpuConfig) {}

// 初始化 CPU 频率、串口、MPU6500、ESP-NOW 和采样/发送任务。
void ImuSlaveApp::begin() {
  g_activeApp = this;
  // Slave 只做 IMU 采样和 ESP-NOW 发送，降频可减小发热和供电压力。
  setCpuFrequencyMhz(config_.cpuFreqMhz);
  Serial.begin(config_.serialBaud);
  delay(300);

  mpu_.begin();
  sendDoneSem_ = xSemaphoreCreateBinary();
  initEspNow();

  // 采样任务只写本地 batch ring；无线任务独立按 slot 发送和重传。
  xTaskCreatePinnedToCore(wirelessTaskStatic, "slaveWireless", 4096, this, 4, nullptr, 0);
  xTaskCreatePinnedToCore(sensorTaskStatic, "slaveSensor", 4096, this, 5, nullptr, 1);
}

// ESP-NOW 发送完成回调的静态转发入口。
void ImuSlaveApp::onEspNowSentStatic(const uint8_t *, esp_now_send_status_t status) {
  if (g_activeApp != nullptr) {
    g_activeApp->onEspNowSent(status);
  }
}

// ESP-NOW 接收回调的静态转发入口。
void ImuSlaveApp::onEspNowRecvStatic(const uint8_t *mac, const uint8_t *data, int len) {
  if (g_activeApp != nullptr) {
    g_activeApp->onEspNowRecv(mac, data, len);
  }
}

// FreeRTOS 无线发送任务的静态转发入口。
void ImuSlaveApp::wirelessTaskStatic(void *arg) {
  static_cast<ImuSlaveApp *>(arg)->wirelessTask();
}

// FreeRTOS 传感器采样任务的静态转发入口。
void ImuSlaveApp::sensorTaskStatic(void *arg) {
  static_cast<ImuSlaveApp *>(arg)->sensorTask();
}

// 记录最近一次 ESP-NOW 发送结果，并唤醒等待发送完成的无线任务。
void ImuSlaveApp::onEspNowSent(esp_now_send_status_t status) {
  // send 回调用信号量通知无线任务，避免无线任务固定等待过久。
  lastSendOk_ = (status == ESP_NOW_SEND_SUCCESS);
  BaseType_t higherPriorityTaskWoken = pdFALSE;
  if (sendDoneSem_ != nullptr) {
    xSemaphoreGiveFromISR(sendDoneSem_, &higherPriorityTaskWoken);
  }
  if (higherPriorityTaskWoken == pdTRUE) {
    portYIELD_FROM_ISR();
  }
}

// 根据下行包类型分发到 ACK 处理或 Beacon 时间同步处理。
void ImuSlaveApp::onEspNowRecv(const uint8_t *mac, const uint8_t *data, int len) {
  // Slave 接收两类下行包：ACK 用于释放缓存，Beacon 用于校准 Master 时间轴。
  if (data == nullptr) {
    return;
  }

  if (len == static_cast<int>(capture::IMU_ACK_WIRE_SIZE) && data[0] == capture::MSG_TYPE_IMU_ACK) {
    handleAckPacket(data, len);
    return;
  }

  handleBeacon(mac, data, len);
}

// 初始化 Slave 的 ESP-NOW STA 模式、固定信道和发送功率。
bool ImuSlaveApp::initEspNow() {
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  WiFi.setSleep(false);
  esp_wifi_set_channel(capture::ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE);
  esp_wifi_set_max_tx_power(config_.espnowTxPowerQdbm);

  if (esp_now_init() != ESP_OK) {
    return false;
  }

  esp_now_register_send_cb(onEspNowSentStatic);
  esp_now_register_recv_cb(onEspNowRecvStatic);
  return true;
}

// 线程安全读取当前 Master-local 时间偏移；未同步时返回 false。
bool ImuSlaveApp::loadTimeSync(int32_t &offsetUs) {
  // offsetMux_ 保护 Beacon 回调和采样/发送任务共享的时间偏移。
  bool synced = false;
  portENTER_CRITICAL(&offsetMux_);
  synced = hasTimeOffset_;
  offsetUs = timeOffsetUs_;
  portEXIT_CRITICAL(&offsetMux_);
  return synced;
}

// 线程安全读取最近收到 Beacon 的 Master MAC，用于后续单播发送。
bool ImuSlaveApp::loadMasterMac(uint8_t outMac[6]) {
  bool valid = false;
  portENTER_CRITICAL(&offsetMux_);
  valid = hasMasterMac_;
  if (valid) {
    memcpy(outMac, masterMac_, 6);
  }
  portEXIT_CRITICAL(&offsetMux_);
  return valid;
}

// 处理 Master Beacon，更新时间偏移和 Master MAC。
void ImuSlaveApp::handleBeacon(const uint8_t *mac, const uint8_t *data, int len) {
  // Beacon 只包含 Master 当前 micros()，Slave 用它估算本地到 Master 的时间偏移。
  if (len != static_cast<int>(sizeof(capture::BeaconPacket))) {
    return;
  }
  capture::BeaconPacket packet{};
  memcpy(&packet, data, sizeof(packet));
  if (packet.type != capture::MSG_TYPE_BEACON) {
    return;
  }

  const uint32_t localNowUs = micros();
  const int32_t measuredOffset = static_cast<int32_t>(packet.masterTimeUs - localNowUs);

  portENTER_CRITICAL_ISR(&offsetMux_);
  if (!hasTimeOffset_) {
    timeOffsetUs_ = measuredOffset;
    hasTimeOffset_ = true;
    largeOffsetErrorCount_ = 0;
  } else {
    // 小误差做低通跟随；大误差需要连续出现几次才重锁，避免偶发 Beacon 抖动拉坏时间轴。
    const int32_t offsetErrorUs = measuredOffset - timeOffsetUs_;
    const int32_t absOffsetErrorUs = offsetErrorUs < 0 ? -offsetErrorUs : offsetErrorUs;
    if (absOffsetErrorUs > TIME_OFFSET_RELOCK_ERROR_US) {
      if (++largeOffsetErrorCount_ >= TIME_OFFSET_RELOCK_CONFIRMATIONS) {
        timeOffsetUs_ = measuredOffset;
        largeOffsetErrorCount_ = 0;
      }
    } else if (absOffsetErrorUs > TIME_OFFSET_SMALL_ERROR_US) {
      timeOffsetUs_ += offsetErrorUs / 16;
      largeOffsetErrorCount_ = 0;
    } else {
      timeOffsetUs_ = (timeOffsetUs_ * 7 + measuredOffset) / 8;
      largeOffsetErrorCount_ = 0;
    }
  }
  if (mac != nullptr) {
    memcpy(masterMac_, mac, 6);
    hasMasterMac_ = true;
  }
  portEXIT_CRITICAL_ISR(&offsetMux_);
}

// 处理 Master ACK，释放本地 ring 中已经被确认的 batch。
void ImuSlaveApp::handleAckPacket(const uint8_t *data, int len) {
  // ACK 到达后遍历本地 ring，释放所有已被 base/bitmap 覆盖的 batch。
  uint16_t ackBaseSeq = 0;
  uint32_t recvBitmap = 0;
  if (!capture::decodeAckPacket(data, len, config_.nodeSource, ackBaseSeq, recvBitmap)) {
    return;
  }

  portENTER_CRITICAL(&ringMux_);
  for (size_t i = 0; i < BATCH_RING_CAPACITY; ++i) {
    BatchSlot &slot = batchRing_[i];
    if (!slot.used) {
      continue;
    }
    if (capture::ackCoversBatchSeq(slot.batchSeq, ackBaseSeq, recvBitmap)) {
      slot.used = false;
      ++ackedBatches_;
    }
  }
  ++ackPackets_;
  advanceOldestUnackedLocked();
  portEXIT_CRITICAL(&ringMux_);
}

// 在持有 ringMux_ 时推进最早未确认 batch 序号。
void ImuSlaveApp::advanceOldestUnackedLocked() {
  // oldestUnackedSeq_ 总是指向最早仍需发送/重传的 batch，发送任务从这里开始找。
  while (oldestUnackedSeq_ != nextBatchSeq_) {
    BatchSlot &slot = batchRing_[oldestUnackedSeq_ % BATCH_RING_CAPACITY];
    if (slot.used && slot.batchSeq == oldestUnackedSeq_) {
      break;
    }
    ++oldestUnackedSeq_;
  }
}

// 记录无法保存或无法读取的样本数量，并标记下一包 link fault。
void ImuSlaveApp::noteDroppedSamples(uint32_t count) {
  // 不能伪造连续样本；一旦本地无法保存，就推进 sampleSeq 并给下一包打 link fault。
  if (count == 0) {
    return;
  }

  portENTER_CRITICAL(&ringMux_);
  nextSampleSeq_ += count;
  linkFaultPending_ = true;
  portEXIT_CRITICAL(&ringMux_);
}

// 将采样任务组好的 batch 写入可靠重传 ring buffer。
bool ImuSlaveApp::storeBatch(const capture::ImuRawSample *samples, uint8_t count) {
  // 采样任务只把 batch 放进 ring，不等待无线发送；这是可靠链路改造的核心。
  const uint16_t batchSeq = nextBatchSeq_;
  const uint32_t sampleStartSeq = nextSampleSeq_;
  capture::ImuBatchWirePacket packet = capture::makeImuBatchPacket(config_.nodeSource, samples, count, batchSeq, sampleStartSeq);
  bool stored = false;

  portENTER_CRITICAL(&ringMux_);
  BatchSlot &slot = batchRing_[batchSeq % BATCH_RING_CAPACITY];
  if (!slot.used) {
    if (linkFaultPending_) {
      // ring 曾经溢出时，下一包显式带上故障标记，提醒这段实验不应参与潜伏期计算。
      packet.bytes[9] |= capture::IMU_FLAG_LINK_FAULT;
      capture::writeImuBatchCrc(packet);
      linkFaultPending_ = false;
    }
    slot.used = true;
    slot.sendAttempts = 0;
    slot.batchSeq = batchSeq;
    slot.sampleStartSeq = sampleStartSeq;
    slot.lastSendUs = 0;
    slot.packet = packet;
    ++nextBatchSeq_;
    nextSampleSeq_ += count;
    stored = true;
  } else {
    // 256 个 batch 都没被 ACK 时才会走到这里，说明无线中断已经超过可缓存范围。
    ++ringOverflows_;
    linkFaultPending_ = true;
    nextSampleSeq_ += count;
  }
  portEXIT_CRITICAL(&ringMux_);
  return stored;
}

// 从 ring 中取出当前最应该发送或重传的 batch。
bool ImuSlaveApp::takeSendCandidate(capture::ImuBatchWirePacket &outPacket, uint32_t nowUs) {
  // 优先发送最早未 ACK 的 batch；如果它刚发过，则跳到后面找可重传/未发送的候选。
  bool found = false;

  portENTER_CRITICAL(&ringMux_);
  advanceOldestUnackedLocked();
  uint16_t seq = oldestUnackedSeq_;
  const uint16_t pendingCount = static_cast<uint16_t>(nextBatchSeq_ - oldestUnackedSeq_);
  for (uint16_t i = 0; i < pendingCount; ++i, ++seq) {
    BatchSlot &slot = batchRing_[seq % BATCH_RING_CAPACITY];
    if (!slot.used || slot.batchSeq != seq) {
      continue;
    }
    if (slot.lastSendUs != 0 &&
        static_cast<uint32_t>(nowUs - slot.lastSendUs) < RETRANSMIT_INTERVAL_US) {
      continue;
    }
    outPacket = slot.packet;
    if (slot.sendAttempts > 0) {
      // 重传时只改 flags 和 CRC，batchSeq/sampleSeq 保持不变，Master 才能去重。
      outPacket.bytes[9] |= capture::IMU_FLAG_RETRANSMIT;
      capture::writeImuBatchCrc(outPacket);
    }
    slot.lastSendUs = nowUs;
    ++slot.sendAttempts;
    ++sendAttempts_;
    found = true;
    break;
  }
  portEXIT_CRITICAL(&ringMux_);
  return found;
}

// 按节点配置读取陀螺仪或加速度，并填入 IMU 原始样本。
bool ImuSlaveApp::readSensor(capture::ImuRawSample &sample) {
  // slave-01/slave-02 复用同一个应用类，通过 SensorMode 决定读取陀螺仪或加速度。
  int16_t x = 0;
  int16_t y = 0;
  int16_t z = 0;
  bool ok = false;
  if (config_.sensorMode == SensorMode::Gyro) {
    ok = mpu_.readGyro(x, y, z);
  } else {
    ok = mpu_.readAccel(x, y, z);
  }
  if (!ok) {
    return false;
  }
  sample.x = x;
  sample.y = y;
  sample.z = z;
  return true;
}

// 等待时间同步后，按错峰发送窗口向 Master 发送和重传 batch。
void ImuSlaveApp::wirelessTask() {
  // 发送 slot 使用 Master 时间轴计算，两个 Slave 因配置的 offset 不同而错峰。
  capture::ImuBatchWirePacket packet{};
  uint8_t masterMac[6] = {0};
  uint32_t lastSlotIndex = UINT32_MAX;

  for (;;) {
    int32_t offsetUs = 0;
    if (!loadTimeSync(offsetUs)) {
      vTaskDelay(pdMS_TO_TICKS(2));
      continue;
    }

    if (!loadMasterMac(masterMac)) {
      vTaskDelay(pdMS_TO_TICKS(2));
      continue;
    }

    if (!esp_now_is_peer_exist(masterMac)) {
      esp_now_peer_info_t peerInfo = {};
      memcpy(peerInfo.peer_addr, masterMac, 6);
      peerInfo.channel = capture::ESPNOW_CHANNEL;
      peerInfo.encrypt = false;
      peerInfo.ifidx = WIFI_IF_STA;
      if (esp_now_add_peer(&peerInfo) != ESP_OK) {
        continue;
      }
    }

    const uint32_t nowUs = micros();
    const uint32_t masterNowUs = static_cast<uint32_t>(static_cast<int64_t>(nowUs) + offsetUs);
    const uint32_t slotIndex = masterNowUs / SEND_SUPERFRAME_US;
    const uint32_t phaseUs = masterNowUs % SEND_SUPERFRAME_US;
    if (slotIndex == lastSlotIndex || phaseUs < config_.sendSlotOffsetUs || phaseUs >= config_.sendSlotOffsetUs + 5000) {
      vTaskDelay(pdMS_TO_TICKS(1));
      continue;
    }
    lastSlotIndex = slotIndex;

    for (uint8_t sendCount = 0; sendCount < MAX_SENDS_PER_SLOT; ++sendCount) {
      // 每个 slot 最多追发 3 个 batch，用恢复后的空口时间追回短时积压。
      if (!takeSendCandidate(packet, micros())) {
        break;
      }

      if (sendDoneSem_ != nullptr) {
        xSemaphoreTake(sendDoneSem_, 0);
      }
      lastSendOk_ = false;
      if (esp_now_send(masterMac, packet.bytes, sizeof(packet.bytes)) != ESP_OK) {
        ++sendFails_;
        continue;
      }
      if (sendDoneSem_ != nullptr && xSemaphoreTake(sendDoneSem_, pdMS_TO_TICKS(8)) == pdTRUE && !lastSendOk_) {
        ++sendFails_;
      }
    }
  }
}

// 以 1kHz 采样 MPU6500，按 IMU_BATCH_SIZE 组包后写入本地 ring。
void ImuSlaveApp::sensorTask() {
  // 1kHz 软件定时采样。无线拥塞不会阻塞这里，只会让 ring 逐渐积压。
  uint32_t nextSampleUs = micros();

  capture::ImuRawSample batchSamples[capture::IMU_BATCH_SIZE] = {};
  uint8_t batchCount = 0;
  uint32_t lastMasterTimestampUs = 0;
  bool hasLastMasterTimestamp = false;

  for (;;) {
    while (static_cast<int32_t>(micros() - nextSampleUs) < 0) {
      taskYIELD();
    }
    nextSampleUs += SAMPLE_PERIOD_US;

    const uint32_t localTimeUs = micros();
    int32_t offsetUs = 0;
    if (!loadTimeSync(offsetUs)) {
      noteDroppedSamples(static_cast<uint32_t>(batchCount) + 1U);
      batchCount = 0;
      hasLastMasterTimestamp = false;
      continue;
    }

    capture::ImuRawSample sample{};
    const uint32_t estimatedMasterTimeUs = static_cast<uint32_t>(static_cast<int64_t>(localTimeUs) + offsetUs);

    if (!hasLastMasterTimestamp) {
      // 新同步段的第一个样本直接采用估计 Master 时间。
      sample.timestampUs = estimatedMasterTimeUs;
      hasLastMasterTimestamp = true;
    } else {
      // 后续样本优先按 1ms 周期推进，只允许小幅校正，避免时间戳被 Beacon 抖动拉出毛刺。
      const uint32_t predictedNextUs = lastMasterTimestampUs + SAMPLE_PERIOD_US;
      const int32_t correctionErrorUs = static_cast<int32_t>(estimatedMasterTimeUs - predictedNextUs);
      if (correctionErrorUs > TIME_OFFSET_RELOCK_ERROR_US || correctionErrorUs < -TIME_OFFSET_RELOCK_ERROR_US) {
        noteDroppedSamples(batchCount);
        sample.timestampUs = estimatedMasterTimeUs;
        batchCount = 0;
      } else {
        const int32_t boundedCorrectionUs =
            correctionErrorUs > 200 ? 200 : (correctionErrorUs < -200 ? -200 : correctionErrorUs);
        sample.timestampUs = predictedNextUs + boundedCorrectionUs;
      }
    }

    if (!readSensor(sample)) {
      noteDroppedSamples(static_cast<uint32_t>(batchCount) + 1U);
      batchCount = 0;
      continue;
    }

    lastMasterTimestampUs = sample.timestampUs;
    batchSamples[batchCount++] = sample;

    if (batchCount >= capture::IMU_BATCH_SIZE) {
      storeBatch(batchSamples, batchCount);
      batchCount = 0;
    }
  }
}
