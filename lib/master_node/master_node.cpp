#include "master_node.h"
#include <esp_log.h> // ✅ 1. 引入 ESP-IDF 日志库
#include <WiFi.h>
#include <esp_wifi.h>
#include <string.h>

namespace {

MasterApp *g_activeApp = nullptr;

}  // namespace

// 保存板级配置，并把 ADS1298 驱动绑定到 Master 的本地 EMG 引脚。
MasterApp::MasterApp(const Config &config) : config_(config), ads_(config.adsPins) {}

// 初始化串口、ADS1298、ESP-NOW 和三个后台任务。
void MasterApp::begin() {
  g_activeApp = this;
  // ✅ 2. 闭嘴！禁止所有底层库（特别是 Wi-Fi）向串口乱喷 ASCII 日志
  esp_log_level_set("*", ESP_LOG_NONE);
  // 串口队列会把 EMG 和两路 IMU 合并输出，TX buffer 适当放大以降低 USB CDC 抖动影响。
  Serial.setTxBufferSize(8192);
  Serial.begin(config_.serialBaud);
  Serial.setTxTimeoutMs(1000);
  delay(300);

  ads_.begin(onAdsDrdyStatic);
  // slaveRxQueue_ 承接 ESP-NOW 回调；serialQueue_ 承接所有准备发给 PC 的样本。
  slaveRxQueue_ = xQueueCreate(512, sizeof(SlaveRxItem));
  serialQueue_ = xQueueCreate(2048, sizeof(capture::PcSample));
  serialWriteMux_ = xSemaphoreCreateMutex();
  initEspNow();

  // 串口写、无线处理、ADS 采样分开跑，避免任一路 I/O 抖动拖慢其他链路。
  xTaskCreatePinnedToCore(wirelessTaskStatic, "masterWireless", 4096, this, 7, nullptr, 0);
  xTaskCreatePinnedToCore(serialTxTaskStatic, "serialTx", 4096, this, 4, nullptr, 0);
  xTaskCreatePinnedToCore(sensorTaskStatic, "masterSensor", 4096, this, 5, nullptr, 1);
  xTaskCreatePinnedToCore(serialCommandTaskStatic, "serialCommand", 4096, this, 3, nullptr, 0);
}

// ADS1298 DRDY 中断的静态转发入口。
void IRAM_ATTR MasterApp::onAdsDrdyStatic() {
  if (g_activeApp != nullptr) {
    g_activeApp->onAdsDrdyFromIsr();
  }
}

// ESP-NOW 接收回调的静态转发入口。
void MasterApp::onEspNowRecvStatic(const uint8_t *mac, const uint8_t *data, int len) {
  if (g_activeApp != nullptr) {
    g_activeApp->onEspNowRecv(mac, data, len);
  }
}

// FreeRTOS 串口发送任务的静态转发入口。
void MasterApp::serialTxTaskStatic(void *arg) {
  static_cast<MasterApp *>(arg)->serialTxTask();
}

// FreeRTOS 无线处理任务的静态转发入口。
void MasterApp::wirelessTaskStatic(void *arg) {
  static_cast<MasterApp *>(arg)->wirelessTask();
}

// FreeRTOS ADS 采样任务的静态转发入口。
void MasterApp::sensorTaskStatic(void *arg) {
  static_cast<MasterApp *>(arg)->sensorTask();
}

void MasterApp::serialCommandTaskStatic(void *arg) {
  static_cast<MasterApp *>(arg)->serialCommandTask();
}

// 在 ISR 中通知 sensorTask 有一帧 ADS1298 数据可读。
void MasterApp::onAdsDrdyFromIsr() {
  // ISR 不读 SPI，只唤醒采样任务；SPI 读写留在普通任务上下文中完成。
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  if (sensorTaskHandle_ != nullptr) {
    vTaskNotifyGiveFromISR(sensorTaskHandle_, &xHigherPriorityTaskWoken);
  }
  if (xHigherPriorityTaskWoken) {
    portYIELD_FROM_ISR();
  }
}

// 把 ESP-NOW 收到的 IMU batch 原样拷贝进队列，留给 wirelessTask 解码。
void MasterApp::onEspNowRecv(const uint8_t *mac, const uint8_t *data, int len) {
  // ESP-NOW 回调运行环境较敏感，这里只做长度检查和拷贝入队。
  if (data == nullptr || len <= 0 || len > static_cast<int>(capture::IMU_BATCH_WIRE_SIZE)) {
    return;
  }
  const uint8_t type = data[0];
  if (type != capture::MSG_TYPE_IMU_BATCH &&
      type != capture::MSG_TYPE_SYNC_DIAG &&
      type != capture::MSG_TYPE_STATE_ACK) {
    return;
  }

  SlaveRxItem item{};
  if (mac != nullptr) {
    memcpy(item.mac, mac, 6);
  }
  item.len = static_cast<uint8_t>(len);
  memcpy(item.bytes, data, len);

  if (slaveRxQueue_ != nullptr) {
    if (xQueueSend(slaveRxQueue_, &item, 0) != pdTRUE) {
      ++slaveQueueDrops_;
    }
  }
}

// 初始化 Master 的 ESP-NOW STA 模式、固定信道和广播 peer。
bool MasterApp::initEspNow() {
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  WiFi.setSleep(false);
  esp_wifi_set_channel(capture::ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE);

  if (esp_now_init() != ESP_OK) {
    return false;
  }

  // ✅ 新增：强制将 ESP-NOW 物理层空口速率从默认的 1Mbps 提升到 2Mbps
  // 大幅减少数据包在空气中的发送耗时，降低被干扰的概率
  esp_wifi_config_espnow_rate(WIFI_IF_STA, WIFI_PHY_RATE_2M_L);

  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, broadcastMac_, sizeof(broadcastMac_));
  peerInfo.channel = capture::ESPNOW_CHANNEL;
  peerInfo.encrypt = false;
  peerInfo.ifidx = WIFI_IF_STA;
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    return false;
  }

  esp_now_register_recv_cb(onEspNowRecvStatic);
  return true;
}

// 确保某个 Slave MAC 已登记为 ESP-NOW peer，ACK 单播前会调用。
bool MasterApp::ensureEspNowPeer(const uint8_t mac[6]) {
  if (mac == nullptr) {
    return false;
  }
  if (esp_now_is_peer_exist(mac)) {
    return true;
  }

  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, mac, 6);
  peerInfo.channel = capture::ESPNOW_CHANNEL;
  peerInfo.encrypt = false;
  peerInfo.ifidx = WIFI_IF_STA;
  return esp_now_add_peer(&peerInfo) == ESP_OK;
}

// 更新 Slave 在线诊断信息，包括最近接收时间和样本累计数。
void MasterApp::trackSlave(const uint8_t mac[6], uint8_t source, uint32_t nowUs, uint32_t sampleCount) {
  // 在线统计按 MAC 记录，和 ACK 窗口分开，避免诊断逻辑影响可靠传输。
  if (mac == nullptr) {
    return;
  }

  int freeIndex = -1;
  int oldestIndex = -1;
  uint32_t oldestSeen = 0;

  for (size_t i = 0; i < MAX_TRACKED_SLAVES; ++i) {
    if (slaveTrackers_[i].used) {
      if (capture::macEquals(slaveTrackers_[i].mac, mac)) {
        slaveTrackers_[i].source = source;
        slaveTrackers_[i].lastSeenUs = nowUs;
        slaveTrackers_[i].totalSamples += sampleCount;
        return;
      }
      if (oldestIndex < 0 || static_cast<int32_t>(slaveTrackers_[i].lastSeenUs - oldestSeen) < 0) {
        oldestIndex = static_cast<int>(i);
        oldestSeen = slaveTrackers_[i].lastSeenUs;
      }
    } else if (freeIndex < 0) {
      freeIndex = static_cast<int>(i);
    }
  }

  int target = freeIndex;
  if (target < 0) {
    target = oldestIndex;
    ++slaveRegistryDrops_;
  }
  if (target < 0) {
    return;
  }

  slaveTrackers_[target].used = true;
  memcpy(slaveTrackers_[target].mac, mac, 6);
  slaveTrackers_[target].source = source;
  slaveTrackers_[target].lastSeenUs = nowUs;
  slaveTrackers_[target].totalSamples = sampleCount;
}

// 统计最近一段时间内仍在发送数据的 Slave 数量。
int32_t MasterApp::countOnlineSlaves(uint32_t nowUs) {
  int32_t online = 0;
  for (size_t i = 0; i < MAX_TRACKED_SLAVES; ++i) {
    if (!slaveTrackers_[i].used) {
      continue;
    }
    const uint32_t silenceUs = static_cast<uint32_t>(nowUs - slaveTrackers_[i].lastSeenUs);
    if (silenceUs <= SLAVE_OFFLINE_TIMEOUT_US) {
      ++online;
    }
  }
  return online;
}

// 获取或创建某个 Slave 的可靠接收窗口状态。
MasterApp::SlaveRxState *MasterApp::getSlaveRxState(const uint8_t mac[6], uint8_t source) {
  if (mac == nullptr) {
    return nullptr;
  }

  int freeIndex = -1;
  for (size_t i = 0; i < MAX_TRACKED_SLAVES; ++i) {
    if (slaveRxStates_[i].used) {
      if (capture::macEquals(slaveRxStates_[i].mac, mac) && slaveRxStates_[i].source == source) {
        return &slaveRxStates_[i];
      }
    } else if (freeIndex < 0) {
      freeIndex = static_cast<int>(i);
    }
  }

  if (freeIndex < 0) {
    ++slaveRegistryDrops_;
    return nullptr;
  }

  SlaveRxState &state = slaveRxStates_[freeIndex];
  state = {};
  state.used = true;
  state.source = source;
  memcpy(state.mac, mac, 6);
  return &state;
}

// 根据 recvBitmap 推进连续 ACK 基准。
void MasterApp::advanceAckWindow(SlaveRxState &state) {
  // 只要 bitmap 的最低位连续为 1，就可以推进连续 ACK 基准。
  while ((state.recvBitmap & 0x01U) != 0) {
    ++state.ackBaseSeq;
    state.ackSampleSeq += capture::IMU_BATCH_SIZE;
    state.recvBitmap >>= 1;
  }
}

// 将一个已通过 CRC 的 batch 写入接收窗口，并判断它是否为重复包。
void MasterApp::markBatchReceived(SlaveRxState &state,
                                  uint16_t batchSeq,
                                  uint32_t sampleStartSeq,
                                  uint8_t flags,
                                  bool &outDuplicate) {
  outDuplicate = false;
  // 重传包本身不是错误；统计它是为了判断无线链路是否开始吃紧。
  if ((flags & capture::IMU_FLAG_RETRANSMIT) != 0) {
    ++state.retransmitPackets;
    ++slaveRetransmitPackets_;
  }

  if (!state.initialized) {
    // 第一个包作为接收窗口锚点，使后续 ACK 可以表达“这个包和它之后 32 个包”的状态。
    state.initialized = true;
    state.ackBaseSeq = static_cast<uint16_t>(batchSeq - 1U);
    state.ackSampleSeq = sampleStartSeq - capture::IMU_BATCH_SIZE;
    state.recvBitmap = 0;
  }

  uint16_t distance = static_cast<uint16_t>(batchSeq - state.ackBaseSeq);
  if (distance == 0 || distance > 32768U) {
    outDuplicate = true;
    ++state.duplicatePackets;
    ++slaveDuplicatePackets_;
    return;
  }

  if (distance > 32U) {
    // 跳出 32 位窗口说明中间缺口已经无法用当前 ACK 表达，显式记为 missing。
    const uint32_t skipped = static_cast<uint32_t>(distance - 1U);
    state.missingPackets += skipped;
    slaveMissingPackets_ += skipped;
    state.ackBaseSeq = static_cast<uint16_t>(batchSeq - 1U);
    state.ackSampleSeq = sampleStartSeq - capture::IMU_BATCH_SIZE;
    state.recvBitmap = 0;
    distance = 1;
  }

  const uint32_t mask = 1UL << (distance - 1U);
  if ((state.recvBitmap & mask) != 0) {
    outDuplicate = true;
    ++state.duplicatePackets;
    ++slaveDuplicatePackets_;
    return;
  }

  state.recvBitmap |= mask;
  advanceAckWindow(state);
}

// 按当前接收窗口状态构造 ACK 并单播回 Slave。
bool MasterApp::sendAck(SlaveRxState &state, uint32_t nowUs) {
  // ACK 单播回原 MAC；如果 peer 未登记，先补登记再发送。
  state.lastAckSentUs = nowUs;
  // ✅ 修复关键：无论底层 Wi-Fi 发送是否成功，先清除 pending 标志。
  // 把重传的责任交还给 Slave 的 40ms 超时机制，防止 Master TX 队列死锁。
  state.ackPending = false;
  if (!ensureEspNowPeer(state.mac)) {
    ++ackSendFails_;
    return false;
  }

  uint8_t ack[capture::IMU_ACK_WIRE_SIZE] = {};
  capture::buildAckPacket(state.source, state.ackBaseSeq, state.ackSampleSeq, state.recvBitmap, nowUs, ack);
  if (esp_now_send(state.mac, ack, sizeof(ack)) != ESP_OK) {
    ++ackSendFails_;
    return false;
  }
  state.ackPending = false;
  return true;
}

// 把同一 Slave 短时间内产生的多个 ACK 合并成一次，避免 ESP-NOW 发送队列被 ACK 风暴打满。
void MasterApp::sendPendingAcks(uint32_t nowUs) {
  for (size_t i = 0; i < MAX_TRACKED_SLAVES; ++i) {
    SlaveRxState &state = slaveRxStates_[i];
    if (!state.used || !state.ackPending) {
      continue;
    }
    if (state.lastAckSentUs != 0 &&
        static_cast<uint32_t>(nowUs - state.lastAckSentUs) < ACK_MIN_INTERVAL_US) {
      continue;
    }
    sendAck(state, nowUs);
  }
}

// 将一条 EMG/IMU/Diag 样本写入串口发送队列。
void MasterApp::writeSerialSample(uint8_t source,
                                  uint32_t timestampUs,
                                  uint32_t sampleSeq,
                                  uint16_t batchSeq,
                                  uint8_t flags,
                                  int32_t x,
                                  int32_t y,
                                  int32_t z) {
  capture::PcSample packet{};
  packet.source = source;
  packet.timestampUs = timestampUs;
  packet.sampleSeq = sampleSeq;
  packet.batchSeq = batchSeq;
  packet.flags = flags;
  packet.x = x;
  packet.y = y;
  packet.z = z;

  if (serialQueue_ != nullptr) {
    if (xQueueSend(serialQueue_, &packet, 0) != pdTRUE) {
      ++serialQueueDrops_;
    }
  }
}

// 阻塞式写完整个 USB CDC 数据块，处理 Serial.write 短写。
void MasterApp::writeSerialBytesAll(const uint8_t *data, size_t size) {
  // Serial.write 可能短写，循环补齐可避免 PC 端收到半个 batch。
  if (serialWriteMux_ != nullptr) {
    xSemaphoreTake(serialWriteMux_, portMAX_DELAY);
  }
  size_t writtenTotal = 0;
  while (writtenTotal < size) {
    const size_t written = Serial.write(data + writtenTotal, size - writtenTotal);
    if (written == 0) {
      ++serialWriteShorts_;
      vTaskDelay(pdMS_TO_TICKS(1));
      continue;
    }
    writtenTotal += written;
  }
  if (serialWriteMux_ != nullptr) {
    xSemaphoreGive(serialWriteMux_);
  }
}

// 生成一条 SOURCE_DIAG 诊断帧，供 PC 端显示链路状态。
void MasterApp::writeDiagFrame(uint32_t timestampUs,
                               int32_t forwardedPerSec,
                               int32_t hardErrorsPerSec,
                               int32_t onlineSlaveCount,
                               uint32_t linkEventsPerSec,
                               uint16_t queueDropsPerSec,
                               uint8_t ackFailsPerSec) {
  // SOURCE_DIAG 复用 PcSample 的额外字段：
  // sampleSeq=无线重传/重复/缺口事件，batchSeq=队列丢弃，flags=ACK 发送失败。
  writeSerialSample(capture::SOURCE_DIAG,
                    timestampUs,
                    linkEventsPerSec,
                    queueDropsPerSec,
                    ackFailsPerSec,
                    forwardedPerSec,
                    hardErrorsPerSec,
                    onlineSlaveCount);
}

void MasterApp::writePcSyncDiag(const capture::SyncDiagPacket &diag) {
  uint8_t txBuffer[capture::PC_SERIAL_MAX_PACKET_SIZE] = {};
  const size_t txSize = capture::buildPcSyncDiagPacket(diag, pcEventSequence_++, txBuffer);
  writeSerialBytesAll(txBuffer, txSize);
}

void MasterApp::writePcStateEvent(const capture::StateAckPacket &event) {
  uint8_t txBuffer[capture::PC_SERIAL_MAX_PACKET_SIZE] = {};
  const size_t txSize = capture::buildPcStateEventPacket(event, pcEventSequence_++, txBuffer);
  writeSerialBytesAll(txBuffer, txSize);
}

uint8_t MasterApp::currentState() const {
  return state_;
}

uint32_t MasterApp::beaconIntervalMs(uint8_t state) const {
  if (state == capture::STATE_SYNC || state == capture::STATE_STREAM_PENDING) {
    return 20;
  }
  if (state == capture::STATE_STREAM) {
    return 200;
  }
  return 2000;
}

void MasterApp::resetStreamingState() {
  emgSampleSeq_ = 0;
  for (size_t i = 0; i < MAX_TRACKED_SLAVES; ++i) {
    slaveRxStates_[i] = {};
  }
  if (serialQueue_ != nullptr) {
    xQueueReset(serialQueue_);
  }
}

void MasterApp::broadcastCommand(uint8_t targetState, uint32_t effectiveMasterTimeUs) {
  uint8_t packet[capture::COMMAND_WIRE_SIZE] = {};
  capture::buildCommandPacket(++commandSeq_, targetState, effectiveMasterTimeUs, packet);
  for (uint8_t i = 0; i < 6; ++i) {
    esp_now_send(broadcastMac_, packet, sizeof(packet));
    vTaskDelay(pdMS_TO_TICKS(8));
  }
}

void MasterApp::transitionTo(uint8_t newState, uint32_t effectiveMasterTimeUs) {
  if (newState == capture::STATE_IDLE || newState == capture::STATE_SYNC || newState == capture::STATE_STREAM_PENDING) {
    resetStreamingState();
  }
  pendingStreamStartUs_ = effectiveMasterTimeUs;
  state_ = newState;
  broadcastCommand(newState, effectiveMasterTimeUs);

  capture::StateAckPacket event{};
  event.source = capture::SOURCE_EMG;
  event.state = newState;
  event.commandSeq = commandSeq_;
  event.effectiveMasterTimeUs = effectiveMasterTimeUs;
  writePcStateEvent(event);
}

void MasterApp::handlePcCommand(String raw) {
  raw.trim();
  raw.toUpperCase();
  if (raw.length() == 0) {
    return;
  }
  if (raw == "PING") {
    capture::StateAckPacket event{};
    event.source = capture::SOURCE_EMG;
    event.state = currentState();
    event.commandSeq = commandSeq_;
    event.effectiveMasterTimeUs = pendingStreamStartUs_;
    writePcStateEvent(event);
    return;
  }
  if (raw == "START_SYNC") {
    transitionTo(capture::STATE_SYNC, 0);
    return;
  }
  if (raw == "START_STREAM") {
    transitionTo(capture::STATE_STREAM_PENDING, micros() + 500000UL);
    return;
  }
  if (raw == "STOP") {
    transitionTo(capture::STATE_IDLE, 0);
    return;
  }
}

void MasterApp::broadcastBeacon(uint32_t nowUs) {
  capture::BeaconPacket beacon{};
  beacon.type = capture::MSG_TYPE_BEACON;
  beacon.beaconSeq = beaconSeq_++;
  beacon.state = currentState();
  beacon.masterTimeUs = nowUs;
  esp_now_send(broadcastMac_, reinterpret_cast<const uint8_t *>(&beacon), sizeof(beacon));
}

// 从 serialQueue_ 取样本并合批编码为 USB CDC 帧。
void MasterApp::serialTxTask() {
  // 小窗口合批发送，减少 USB CDC 包头开销，同时保持毫秒级刷新。
  capture::PcSample sample{};
  capture::PcSample batch[capture::PC_SERIAL_MAX_SAMPLES] = {};
  uint8_t txBuffer[capture::PC_SERIAL_MAX_PACKET_SIZE] = {};

  for (;;) {
    if (xQueueReceive(serialQueue_, &sample, portMAX_DELAY) == pdTRUE) {
      uint8_t sampleCount = 0;
      batch[sampleCount++] = sample;

      while (sampleCount < capture::PC_SERIAL_MAX_SAMPLES &&
             xQueueReceive(serialQueue_, &sample, pdMS_TO_TICKS(1)) == pdTRUE) {
        batch[sampleCount++] = sample;
      }

      const size_t txSize = capture::buildPcBatchPacket(batch, sampleCount, pcSerialSequence_++, txBuffer);
      writeSerialBytesAll(txBuffer, txSize);
    }
  }
}

// 处理 Beacon 广播、Slave batch 解码、ACK 回复和每秒诊断统计。
void MasterApp::wirelessTask() {
  SlaveRxItem rxItem{};
  capture::ImuDecodedSample decodedSamples[capture::IMU_BATCH_SIZE] = {};
  uint8_t decodedCount = 0;
  uint8_t decodedSource = 0;
  uint16_t decodedBatchSeq = 0;
  uint32_t decodedSampleStartSeq = 0;
  uint8_t decodedFlags = 0;

  uint32_t lastForwardedSamples = 0;
  uint32_t lastDecodeFails = 0;
  uint32_t lastCrcFails = 0;
  uint32_t lastQueueDrops = 0;
  uint32_t lastSerialQueueDrops = 0;
  uint32_t lastSerialWriteShorts = 0;
  uint32_t lastRegistryDrops = 0;
  uint32_t lastDuplicatePackets = 0;
  uint32_t lastRetransmitPackets = 0;
  uint32_t lastMissingPackets = 0;
  uint32_t lastAckSendFails = 0;
  uint32_t lastBeaconMs = 0;
  uint32_t lastDiagMs = millis();

  for (;;) {
    const uint32_t nowMs = millis();
    const uint8_t state = currentState();
    if (static_cast<uint32_t>(nowMs - lastBeaconMs) >= beaconIntervalMs(state)) {
      broadcastBeacon(micros());
      lastBeaconMs = nowMs;
    }

    if (state == capture::STATE_STREAM_PENDING &&
        static_cast<int32_t>(micros() - pendingStreamStartUs_) >= 0) {
      resetStreamingState();
      state_ = capture::STATE_STREAM;
      capture::StateAckPacket event{};
      event.source = capture::SOURCE_EMG;
      event.state = capture::STATE_STREAM;
      event.commandSeq = commandSeq_;
      event.effectiveMasterTimeUs = pendingStreamStartUs_;
      writePcStateEvent(event);
    }

    while (xQueueReceive(slaveRxQueue_, &rxItem, 0) == pdTRUE) {
      if (rxItem.len == 0) {
        continue;
      }

      if (rxItem.bytes[0] == capture::MSG_TYPE_STATE_ACK) {
        capture::StateAckPacket event{};
        if (capture::decodeStateAckPacket(rxItem.bytes, rxItem.len, event)) {
          writePcStateEvent(event);
          trackSlave(rxItem.mac, event.source, micros(), 0);
        }
        continue;
      }

      if (rxItem.bytes[0] == capture::MSG_TYPE_SYNC_DIAG) {
        capture::SyncDiagPacket diag{};
        if (capture::decodeSyncDiagPacket(rxItem.bytes, rxItem.len, diag)) {
          writePcSyncDiag(diag);
          trackSlave(rxItem.mac, diag.source, micros(), 0);
        }
        continue;
      }

      if (rxItem.len != capture::IMU_BATCH_WIRE_SIZE || currentState() != capture::STATE_STREAM) {
        continue;
      }

      capture::ImuBatchWirePacket wirePacket{};
      memcpy(wirePacket.bytes, rxItem.bytes, capture::IMU_BATCH_WIRE_SIZE);
      const capture::DecodeStatus status = capture::decodeImuBatchPacket(wirePacket,
                                                                         decodedSamples,
                                                                         decodedCount,
                                                                         decodedSource,
                                                                         decodedBatchSeq,
                                                                         decodedSampleStartSeq,
                                                                         decodedFlags);
      if (status != capture::DECODE_OK) {
        ++slaveDecodeFails_;
        if (status == capture::DECODE_BAD_CRC) {
          ++slaveCrcFails_;
        }
        continue;
      }

      const uint32_t nowUs = micros();
      SlaveRxState *rxState = getSlaveRxState(rxItem.mac, decodedSource);
      if (rxState == nullptr) {
        continue;
      }

      bool duplicate = false;
      markBatchReceived(*rxState, decodedBatchSeq, decodedSampleStartSeq, decodedFlags, duplicate);
      rxState->ackPending = true;

      trackSlave(rxItem.mac, decodedSource, nowUs, duplicate ? 0 : decodedCount);
      if (duplicate) {
        // 重复 batch 仍要 ACK，但不能再次写入 PC，否则 CSV 会出现重复 sample_seq。
        continue;
      }

      for (uint8_t i = 0; i < decodedCount; ++i) {
        writeSerialSample(decodedSource,
                          decodedSamples[i].timestampUs,
                          decodedSamples[i].sampleSeq,
                          decodedSamples[i].batchSeq,
                          decodedSamples[i].flags,
                          static_cast<int32_t>(decodedSamples[i].x),
                          static_cast<int32_t>(decodedSamples[i].y),
                          static_cast<int32_t>(decodedSamples[i].z));
      }
      slaveForwardedSamples_ += decodedCount;
    }

    sendPendingAcks(micros());

    if (static_cast<uint32_t>(nowMs - lastDiagMs) >= 1000) {
      const uint32_t nowUs = micros();
      const int32_t onlineSlaves = countOnlineSlaves(nowUs);
      const int32_t forwardedDelta = static_cast<int32_t>(slaveForwardedSamples_ - lastForwardedSamples);
      const uint32_t queueDropDelta = (slaveQueueDrops_ - lastQueueDrops) +
                                      (serialQueueDrops_ - lastSerialQueueDrops);
      const uint32_t ackFailDelta = ackSendFails_ - lastAckSendFails;
      const uint32_t linkEventDelta = (slaveDuplicatePackets_ - lastDuplicatePackets) +
                                      (slaveRetransmitPackets_ - lastRetransmitPackets) +
                                      (slaveMissingPackets_ - lastMissingPackets);
      const int32_t hardErrorDelta = static_cast<int32_t>((slaveDecodeFails_ - lastDecodeFails) +
                                                          (slaveCrcFails_ - lastCrcFails) +
                                                          queueDropDelta +
                                                          (serialWriteShorts_ - lastSerialWriteShorts) +
                                                          (slaveRegistryDrops_ - lastRegistryDrops) +
                                                          ackFailDelta);

      writeDiagFrame(nowUs,
                     forwardedDelta,
                     hardErrorDelta,
                     onlineSlaves,
                     linkEventDelta,
                     static_cast<uint16_t>(min(queueDropDelta, static_cast<uint32_t>(UINT16_MAX))),
                     static_cast<uint8_t>(min(ackFailDelta, static_cast<uint32_t>(UINT8_MAX))));

      lastForwardedSamples = slaveForwardedSamples_;
      lastDecodeFails = slaveDecodeFails_;
      lastCrcFails = slaveCrcFails_;
      lastQueueDrops = slaveQueueDrops_;
      lastSerialQueueDrops = serialQueueDrops_;
      lastSerialWriteShorts = serialWriteShorts_;
      lastRegistryDrops = slaveRegistryDrops_;
      lastDuplicatePackets = slaveDuplicatePackets_;
      lastRetransmitPackets = slaveRetransmitPackets_;
      lastMissingPackets = slaveMissingPackets_;
      lastAckSendFails = ackSendFails_;
      lastDiagMs = nowMs;
    }

    vTaskDelay(pdMS_TO_TICKS(2));
  }
}

// 等待 ADS1298 DRDY 通知，读取 CH6/CH7/CH8 并送入串口队列。
void MasterApp::sensorTask() {
  sensorTaskHandle_ = xTaskGetCurrentTaskHandle();

  for (;;) {
    // ADS1298 的 DRDY 决定 EMG 采样节奏；Master 给 EMG 单独生成 sampleSeq。
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    const uint32_t timestampUs = micros();
    int32_t ch6 = 0;
    int32_t ch7 = 0;
    int32_t ch8 = 0;

    if (currentState() == capture::STATE_STREAM && ads_.readChannels(ch6, ch7, ch8)) {
      writeSerialSample(capture::SOURCE_EMG, timestampUs, emgSampleSeq_++, 0, 0, ch6, ch7, ch8);
    }
  }
}

void MasterApp::serialCommandTask() {
  String line;
  line.reserve(48);
  for (;;) {
    while (Serial.available() > 0) {
      const char ch = static_cast<char>(Serial.read());
      if (ch == '\n' || ch == '\r') {
        if (line.length() > 0) {
          handlePcCommand(line);
          line = "";
        }
      } else if (line.length() < 47) {
        line += ch;
      } else {
        line = "";
      }
    }
    vTaskDelay(pdMS_TO_TICKS(5));
  }
}
