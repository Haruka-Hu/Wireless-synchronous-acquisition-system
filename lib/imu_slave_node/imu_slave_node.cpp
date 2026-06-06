#include "imu_slave_node.h"

#include <algorithm>
#include <WiFi.h>
#include <esp_wifi.h>
#include <math.h>
#include <string.h>

namespace {

ImuSlaveApp *g_activeApp = nullptr;

wifi_phy_rate_t radioRateFromCode(uint8_t rateCode) {
  return rateCode == capture::ESPNOW_RATE_1M ? WIFI_PHY_RATE_1M_L : WIFI_PHY_RATE_2M_L;
}

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
  dataReadyInterruptEnabled_ = mpu_.hasDataReadyInterrupt();
  if (dataReadyInterruptEnabled_) {
    attachInterrupt(digitalPinToInterrupt(mpu_.dataReadyInterruptPin()), onMpuDataReadyStatic, RISING);
  }
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

// MPU6500 DATA_RDY 中断的静态转发入口。
void IRAM_ATTR ImuSlaveApp::onMpuDataReadyStatic() {
  if (g_activeApp != nullptr) {
    g_activeApp->onMpuDataReadyFromIsr();
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

void ImuSlaveApp::onMpuDataReadyFromIsr() {
  lastDataReadyLocalUs_ = micros();
  BaseType_t higherPriorityTaskWoken = pdFALSE;
  if (sensorTaskHandle_ != nullptr) {
    vTaskNotifyGiveFromISR(sensorTaskHandle_, &higherPriorityTaskWoken);
  }
  if (higherPriorityTaskWoken == pdTRUE) {
    portYIELD_FROM_ISR();
  }
}

// 根据下行包类型分发到 ACK、双向同步或 Beacon 时间同步处理。
void ImuSlaveApp::onEspNowRecv(const uint8_t *mac, const uint8_t *data, int len) {
  // Slave 接收 COMMAND/无线配置/ACK/SYNC_REPLY/Beacon 等下行包。
  const uint32_t localRecvUs = micros();
  if (data == nullptr) {
    return;
  }
  if (mac != nullptr) {
    portENTER_CRITICAL(&offsetMux_);
    memcpy(masterMac_, mac, 6);
    hasMasterMac_ = true;
    portEXIT_CRITICAL(&offsetMux_);
  }

  if (len == static_cast<int>(capture::COMMAND_WIRE_SIZE) && data[0] == capture::MSG_TYPE_COMMAND) {
    handleCommandPacket(data, len);
    return;
  }

  if (len == static_cast<int>(capture::RADIO_CONFIG_WIRE_SIZE) && data[0] == capture::MSG_TYPE_RADIO_CONFIG) {
    handleRadioConfigPacket(data, len);
    return;
  }

  if (len == static_cast<int>(capture::IMU_ACK_WIRE_SIZE) && data[0] == capture::MSG_TYPE_IMU_ACK) {
    handleAckPacket(data, len);
    return;
  }

  if (len == static_cast<int>(capture::SYNC_REPLY_WIRE_SIZE) && data[0] == capture::MSG_TYPE_SYNC_REPLY) {
    handleSyncReply(data, len, localRecvUs);
    return;
  }

  handleBeacon(mac, data, len);
}

// 初始化 Slave 的 ESP-NOW STA 模式、固定信道和发送功率。
bool ImuSlaveApp::initEspNow() {
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  WiFi.setSleep(false);
  esp_wifi_set_channel(currentChannel_, WIFI_SECOND_CHAN_NONE);
  esp_wifi_set_max_tx_power(config_.espnowTxPowerQdbm);

  if (esp_now_init() != ESP_OK) {
    return false;
  }

  esp_wifi_config_espnow_rate(WIFI_IF_STA, radioRateFromCode(currentRateCode_));

  esp_now_register_send_cb(onEspNowSentStatic);
  esp_now_register_recv_cb(onEspNowRecvStatic);
  return true;
}

bool ImuSlaveApp::loadClockModel(double &slope, double &intercept) {
  bool valid = false;
  portENTER_CRITICAL(&offsetMux_);
  valid = clockModelValid_;
  slope = clockSlope_;
  intercept = clockIntercept_;
  portEXIT_CRITICAL(&offsetMux_);
  return valid;
}

// 兼容旧接口；采样时间戳实际使用动态平滑时钟模型。
bool ImuSlaveApp::loadSampleClockModel(double &slope, double &intercept) {
  bool valid = false;
  portENTER_CRITICAL(&offsetMux_);
  valid = sampleClockModelValid_;
  slope = sampleClockSlope_;
  intercept = sampleClockIntercept_;
  portEXIT_CRITICAL(&offsetMux_);
  return valid;
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
  if (len != static_cast<int>(sizeof(capture::BeaconPacket))) {
    return;
  }
  capture::BeaconPacket packet{};
  memcpy(&packet, data, sizeof(packet));
  if (packet.type != capture::MSG_TYPE_BEACON) {
    return;
  }

  const uint32_t localNowUs = micros();
  const uint32_t nowMs = millis();
  portENTER_CRITICAL(&offsetMux_);
  if ((state_ == capture::STATE_SYNC || state_ == capture::STATE_STREAM_PENDING || state_ == capture::STATE_STREAM) &&
      (lastTwoWaySyncMs_ == 0 || static_cast<uint32_t>(nowMs - lastTwoWaySyncMs_) > TWO_WAY_BEACON_SUPPRESS_MS)) {
    recordBeaconLocked(localNowUs, packet.masterTimeUs, packet.beaconSeq);
  }
  if (mac != nullptr) {
    memcpy(masterMac_, mac, 6);
    hasMasterMac_ = true;
  }
  portEXIT_CRITICAL(&offsetMux_);
}

void ImuSlaveApp::handleCommandPacket(const uint8_t *data, int len) {
  capture::CommandPacket command{};
  if (!capture::decodeCommandPacket(data, len, command)) {
    return;
  }
  applyState(command.targetState, command.commandSeq, command.effectiveMasterTimeUs);
}

void ImuSlaveApp::handleRadioConfigPacket(const uint8_t *data, int len) {
  capture::RadioConfigPacket config{};
  if (!capture::decodeRadioConfigPacket(data, len, config)) {
    return;
  }

  portENTER_CRITICAL(&radioMux_);
  if (config.configSeq == lastRadioConfigSeq_ && radioConfigPending_) {
    portEXIT_CRITICAL(&radioMux_);
    return;
  }
  lastRadioConfigSeq_ = config.configSeq;
  pendingRadioChannel_ = config.channel;
  pendingRadioRateCode_ = config.rateCode;
  pendingRadioApplyMs_ = millis() + config.applyDelayMs;
  radioConfigPending_ = true;
  portEXIT_CRITICAL(&radioMux_);
}

bool ImuSlaveApp::applyRadioConfig(uint8_t channel, uint8_t rateCode) {
  if (!capture::isValidRadioChannel(channel) || !capture::isValidRadioRate(rateCode)) {
    return false;
  }

  portENTER_CRITICAL(&offsetMux_);
  state_ = capture::STATE_IDLE;
  streamStarted_ = false;
  pendingStreamStartUs_ = 0;
  clockModelValid_ = false;
  sampleClockModelValid_ = false;
  portEXIT_CRITICAL(&offsetMux_);

  portENTER_CRITICAL(&ringMux_);
  resetRingLocked();
  portEXIT_CRITICAL(&ringMux_);

  currentChannel_ = channel;
  currentRateCode_ = rateCode;
  esp_wifi_set_channel(currentChannel_, WIFI_SECOND_CHAN_NONE);
  esp_wifi_config_espnow_rate(WIFI_IF_STA, radioRateFromCode(currentRateCode_));
  return true;
}

void ImuSlaveApp::applyPendingRadioConfig() {
  uint8_t channel = 0;
  uint8_t rateCode = 0;
  bool shouldApply = false;

  portENTER_CRITICAL(&radioMux_);
  if (radioConfigPending_ && static_cast<int32_t>(millis() - pendingRadioApplyMs_) >= 0) {
    channel = pendingRadioChannel_;
    rateCode = pendingRadioRateCode_;
    radioConfigPending_ = false;
    shouldApply = true;
  }
  portEXIT_CRITICAL(&radioMux_);

  if (shouldApply) {
    applyRadioConfig(channel, rateCode);
  }
}

void ImuSlaveApp::handleSyncReply(const uint8_t *data, int len, uint32_t localRecvUs) {
  capture::SyncReplyPacket reply{};
  if (!capture::decodeSyncReplyPacket(data, len, reply) || reply.source != config_.nodeSource) {
    return;
  }

  const uint32_t rttUs = static_cast<uint32_t>(localRecvUs - reply.slaveT1LocalUs);
  if (rttUs == 0 || rttUs > TWO_WAY_SYNC_RTT_MAX_US) {
    portENTER_CRITICAL(&offsetMux_);
    ++syncProbeDrops_;
    portEXIT_CRITICAL(&offsetMux_);
    return;
  }

  const int64_t t2MinusT1 = static_cast<int32_t>(reply.masterT2RecvUs - reply.slaveT1LocalUs);
  const int64_t t3MinusT4 = static_cast<int32_t>(reply.masterT3SendUs - localRecvUs);
  const int64_t offsetUs = (t2MinusT1 + t3MinusT4) / 2;
  const uint32_t localMidUs = reply.slaveT1LocalUs + rttUs / 2U;
  const uint32_t masterMidUs = static_cast<uint32_t>(static_cast<int64_t>(localMidUs) + offsetUs);

  portENTER_CRITICAL(&offsetMux_);
  if (state_ == capture::STATE_SYNC || state_ == capture::STATE_STREAM_PENDING || state_ == capture::STATE_STREAM) {
    recordSyncPointLocked(localMidUs, masterMidUs, reply.probeSeq);
    lastTwoWaySyncMs_ = millis();
    lastSyncProbeRttUs_ = rttUs;
    ++syncProbeReplies_;
  }
  portEXIT_CRITICAL(&offsetMux_);
}

uint8_t ImuSlaveApp::currentState() const {
  return state_;
}

void ImuSlaveApp::resetRingLocked() {
  for (size_t i = 0; i < BATCH_RING_CAPACITY; ++i) {
    batchRing_[i] = {};
  }
  nextBatchSeq_ = 0;
  oldestUnackedSeq_ = 0;
  nextSampleSeq_ = 0;
  ringOverflows_ = 0;
  lastSampleTimestampUs_ = 0;
  hasLastSampleTimestamp_ = false;
  linkFaultPending_ = false;
  ackPackets_ = 0;
  ackedBatches_ = 0;
  sendAttempts_ = 0;
  sendFails_ = 0;
}

void ImuSlaveApp::resetSyncFitLocked() {
  clockSlope_ = 1.0;
  clockIntercept_ = 0.0;
  sampleClockSlope_ = 1.0;
  sampleClockIntercept_ = 0.0;
  syncOffsetUs_ = 0;
  syncDriftPpm_ = 0;
  syncResidualUs_ = 0;
  clockModelValid_ = false;
  sampleClockModelValid_ = false;
  syncPointCount_ = 0;
  syncPointHead_ = 0;
  lastBeaconSeq_ = 0;
  syncProbeSeq_ = 0;
  lastSyncDiagMs_ = 0;
  lastSyncProbeMs_ = 0;
  lastTwoWaySyncMs_ = 0;
  syncProbeReplies_ = 0;
  syncProbeDrops_ = 0;
  lastSyncProbeRttUs_ = 0;
}

void ImuSlaveApp::applyState(uint8_t newState, uint16_t commandSeq, uint32_t effectiveMasterTimeUs) {
  if (commandSeq == lastCommandSeq_) {
    sendStateAck(commandSeq, effectiveMasterTimeUs);
    return;
  }
  lastCommandSeq_ = commandSeq;

  portENTER_CRITICAL(&ringMux_);
  if (newState == capture::STATE_IDLE || newState == capture::STATE_SYNC || newState == capture::STATE_STREAM_PENDING) {
    resetRingLocked();
  }
  portEXIT_CRITICAL(&ringMux_);

  portENTER_CRITICAL(&offsetMux_);
  state_ = newState;
  pendingStreamStartUs_ = effectiveMasterTimeUs;
  streamStarted_ = false;
  if (newState == capture::STATE_SYNC) {
    resetSyncFitLocked();
  }
  if (newState == capture::STATE_IDLE) {
    streamStarted_ = false;
    sampleClockModelValid_ = false;
  }
  portEXIT_CRITICAL(&offsetMux_);

  sendStateAck(commandSeq, effectiveMasterTimeUs);
}

void ImuSlaveApp::sendStateAck(uint16_t commandSeq, uint32_t effectiveMasterTimeUs) {
  uint8_t masterMac[6] = {0};
  if (!loadMasterMac(masterMac)) {
    return;
  }
  if (!esp_now_is_peer_exist(masterMac)) {
    esp_now_peer_info_t peerInfo = {};
    memcpy(peerInfo.peer_addr, masterMac, 6);
    peerInfo.channel = 0;
    peerInfo.encrypt = false;
    peerInfo.ifidx = WIFI_IF_STA;
    esp_now_add_peer(&peerInfo);
  }
  uint8_t packet[capture::STATE_ACK_WIRE_SIZE] = {};
  capture::buildStateAckPacket(config_.nodeSource, currentState(), commandSeq, effectiveMasterTimeUs, packet);
  esp_now_send(masterMac, packet, sizeof(packet));
}

void ImuSlaveApp::recordSyncPointLocked(uint32_t localUs, uint32_t masterUs, uint16_t beaconSeq) {
  syncPoints_[syncPointHead_] = {localUs, masterUs};
  syncPointHead_ = static_cast<uint8_t>((syncPointHead_ + 1U) % SYNC_WINDOW_CAPACITY);
  if (syncPointCount_ < SYNC_WINDOW_CAPACITY) {
    ++syncPointCount_;
  }
  lastBeaconSeq_ = beaconSeq;
}

void ImuSlaveApp::recordBeaconLocked(uint32_t localRecvUs, uint32_t masterTimeUs, uint16_t beaconSeq) {
  recordSyncPointLocked(localRecvUs, masterTimeUs, beaconSeq);
}

// 对当前同步窗口做普通最小二乘拟合；includeMask 非空时只使用 mask 选中的点。
bool ImuSlaveApp::fitClockModelLocked(const bool *includeMask, double &slope, double &intercept, double &residualRms) {
  if (syncPointCount_ < 2) {
    return false;
  }

  const uint8_t oldestIndex = static_cast<uint8_t>((syncPointHead_ + SYNC_WINDOW_CAPACITY - syncPointCount_) % SYNC_WINDOW_CAPACITY);
  const SyncPoint &base = syncPoints_[oldestIndex];
  double sumX = 0.0;
  double sumY = 0.0;
  double sumXX = 0.0;
  double sumXY = 0.0;
  uint8_t usedCount = 0;

  for (uint8_t i = 0; i < syncPointCount_; ++i) {
    if (includeMask != nullptr && !includeMask[i]) {
      continue;
    }
    const uint8_t index = static_cast<uint8_t>((oldestIndex + i) % SYNC_WINDOW_CAPACITY);
    const double x = static_cast<double>(static_cast<uint32_t>(syncPoints_[index].localUs - base.localUs));
    const double y = static_cast<double>(static_cast<int32_t>(syncPoints_[index].masterUs - base.masterUs));
    sumX += x;
    sumY += y;
    sumXX += x * x;
    sumXY += x * y;
    ++usedCount;
  }

  if (usedCount < 2) {
    return false;
  }

  const double n = static_cast<double>(usedCount);
  const double denom = n * sumXX - sumX * sumX;
  if (denom == 0.0) {
    return false;
  }

  slope = (n * sumXY - sumX * sumY) / denom;
  const double interceptRel = (sumY - slope * sumX) / n;
  intercept = static_cast<double>(base.masterUs) + interceptRel - slope * static_cast<double>(base.localUs);

  double residualSq = 0.0;
  uint8_t residualCount = 0;
  for (uint8_t i = 0; i < syncPointCount_; ++i) {
    if (includeMask != nullptr && !includeMask[i]) {
      continue;
    }
    const uint8_t index = static_cast<uint8_t>((oldestIndex + i) % SYNC_WINDOW_CAPACITY);
    const double predicted = slope * static_cast<double>(syncPoints_[index].localUs) + intercept;
    const double error = predicted - static_cast<double>(syncPoints_[index].masterUs);
    residualSq += error * error;
    ++residualCount;
  }

  if (residualCount == 0) {
    return false;
  }
  residualRms = sqrt(residualSq / static_cast<double>(residualCount));
  return true;
}

// 用两遍拟合更新 Slave 的线性时钟模型，第二遍会剔除单向 ESP-NOW 接收抖动中的离群点。
void ImuSlaveApp::updateClockFitLocked() {
  double slope = 1.0;
  double intercept = 0.0;
  double residualRms = 0.0;
  if (!fitClockModelLocked(nullptr, slope, intercept, residualRms)) {
    clockModelValid_ = false;
    return;
  }

  if (syncPointCount_ >= SYNC_MIN_ROBUST_POINTS) {
    const uint8_t oldestIndex = static_cast<uint8_t>((syncPointHead_ + SYNC_WINDOW_CAPACITY - syncPointCount_) % SYNC_WINDOW_CAPACITY);
    double residualAbs[SYNC_WINDOW_CAPACITY] = {};
    bool includeMask[SYNC_WINDOW_CAPACITY] = {};
    uint8_t residualCount = 0;

    for (uint8_t i = 0; i < syncPointCount_; ++i) {
      const uint8_t index = static_cast<uint8_t>((oldestIndex + i) % SYNC_WINDOW_CAPACITY);
      const double predicted = slope * static_cast<double>(syncPoints_[index].localUs) + intercept;
      residualAbs[residualCount++] = fabs(predicted - static_cast<double>(syncPoints_[index].masterUs));
    }

    double sortedResiduals[SYNC_WINDOW_CAPACITY] = {};
    memcpy(sortedResiduals, residualAbs, residualCount * sizeof(double));
    std::sort(sortedResiduals, sortedResiduals + residualCount);

    size_t keepCount = residualCount * (100U - SYNC_OUTLIER_REJECT_PERCENT) / 100U;
    if (keepCount < SYNC_MIN_ROBUST_POINTS) {
      keepCount = SYNC_MIN_ROBUST_POINTS;
    }
    if (keepCount > residualCount) {
      keepCount = residualCount;
    }

    const double percentileLimit = sortedResiduals[keepCount - 1U];
    const double rejectLimit = std::min(percentileLimit, SYNC_OUTLIER_ABS_US);
    uint8_t includedCount = 0;
    for (uint8_t i = 0; i < syncPointCount_; ++i) {
      includeMask[i] = residualAbs[i] <= rejectLimit;
      if (includeMask[i]) {
        ++includedCount;
      }
    }

    if (includedCount >= SYNC_MIN_ROBUST_POINTS) {
      double robustSlope = 1.0;
      double robustIntercept = 0.0;
      double robustResidualRms = 0.0;
      if (fitClockModelLocked(includeMask, robustSlope, robustIntercept, robustResidualRms)) {
        
        // === 👇 核心数学黑魔法：PPM 平滑与重心约束 👇 ===
        if (clockModelValid_) {
          // 1. 手动计算这批优质数据点（includeMask 为 true）的物理重心
          const uint8_t oldestIndex = static_cast<uint8_t>((syncPointHead_ + SYNC_WINDOW_CAPACITY - syncPointCount_) % SYNC_WINDOW_CAPACITY);
          const SyncPoint &base = syncPoints_[oldestIndex];
          double sumX = 0.0;
          double sumY = 0.0;
          
          for (uint8_t i = 0; i < syncPointCount_; ++i) {
            if (includeMask[i]) {
              const uint8_t index = static_cast<uint8_t>((oldestIndex + i) % SYNC_WINDOW_CAPACITY);
              sumX += static_cast<double>(static_cast<uint32_t>(syncPoints_[index].localUs - base.localUs));
              sumY += static_cast<double>(static_cast<int32_t>(syncPoints_[index].masterUs - base.masterUs));
            }
          }
          
          double centroidX = static_cast<double>(base.localUs) + (sumX / static_cast<double>(includedCount));
          double centroidY = static_cast<double>(base.masterUs) + (sumY / static_cast<double>(includedCount));

          // 2. IIR 滤波：给斜率（PPM）加上极大的惯性，彻底抹平空口延迟带来的震荡
          robustSlope = 0.8 * clockSlope_ + 0.2 * robustSlope;
          
          // 3. 截距补偿：将滤波后的新斜率，绕着真实的“数据重心”强行扭转，防止基准线偏移
          robustIntercept = centroidY - robustSlope * centroidX;
        }
        // === 👆 黑魔法结束 👆 ===

        slope = robustSlope;
        intercept = robustIntercept;
        residualRms = robustResidualRms;
      }
    }
  }

  const uint32_t nowUs = micros();
  clockSlope_ = slope;
  clockIntercept_ = intercept;
  const double nowMaster = clockSlope_ * static_cast<double>(nowUs) + clockIntercept_;
  syncOffsetUs_ = static_cast<int32_t>(nowMaster - static_cast<double>(nowUs));
  syncDriftPpm_ = static_cast<int32_t>((clockSlope_ - 1.0) * 1000000.0);
  syncResidualUs_ = static_cast<int32_t>(residualRms);
  clockModelValid_ = true;
}

// 采样时间戳使用动态平滑模型；保留该函数只兼容 STREAM 进入流程。
void ImuSlaveApp::freezeSampleClockLocked() {
  sampleClockModelValid_ = false;
}

uint32_t ImuSlaveApp::localToMasterTimeUs(uint32_t localUs) {
  double slope = 1.0;
  double intercept = 0.0;
  if (!loadClockModel(slope, intercept)) {
    return localUs;
  }
  return static_cast<uint32_t>(slope * static_cast<double>(localUs) + intercept);
}

uint32_t ImuSlaveApp::localToSampleTimeUs(uint32_t localUs) {
  return localToMasterTimeUs(localUs);
}

void ImuSlaveApp::sendSyncProbe() {
  uint8_t masterMac[6] = {0};
  if (!loadMasterMac(masterMac)) {
    return;
  }
  if (!esp_now_is_peer_exist(masterMac)) {
    esp_now_peer_info_t peerInfo = {};
    memcpy(peerInfo.peer_addr, masterMac, 6);
    peerInfo.channel = 0;
    peerInfo.encrypt = false;
    peerInfo.ifidx = WIFI_IF_STA;
    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
      return;
    }
  }

  uint16_t probeSeq = 0;
  portENTER_CRITICAL(&offsetMux_);
  probeSeq = syncProbeSeq_++;
  portEXIT_CRITICAL(&offsetMux_);

  uint8_t packet[capture::SYNC_PROBE_WIRE_SIZE] = {};
  capture::buildSyncProbePacket(config_.nodeSource, probeSeq, micros(), packet);
  esp_now_send(masterMac, packet, sizeof(packet));
}

void ImuSlaveApp::sendSyncDiag() {
  uint8_t masterMac[6] = {0};
  if (!loadMasterMac(masterMac)) {
    return;
  }

  capture::SyncDiagPacket diag{};
  portENTER_CRITICAL(&offsetMux_);
  updateClockFitLocked();
  diag.source = config_.nodeSource;
  diag.state = state_;
  diag.beaconSeq = lastBeaconSeq_;
  diag.offsetUs = syncOffsetUs_;
  diag.driftPpm = syncDriftPpm_;
  diag.residualUs = syncResidualUs_;
  diag.beaconCount = syncPointCount_;
  portEXIT_CRITICAL(&offsetMux_);

  uint8_t packet[capture::SYNC_DIAG_WIRE_SIZE] = {};
  capture::buildSyncDiagPacket(diag, packet);
  esp_now_send(masterMac, packet, sizeof(packet));
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
      packet.bytes[9] |= capture::IMU_FLAG_LINK_FAULT;
      capture::writeImuBatchCrc(packet);
      linkFaultPending_ = false;
    }
    slot.used = true;
    slot.sendAttempts = 0;
    slot.batchSeq = batchSeq;
    slot.sampleStartSeq = sampleStartSeq;
    // ✅ 优化：新包入队时，将下一次发送时间设为 0，代表“立即发送”
    slot.nextSendUs = 0; 
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
  bool found = false;

  portENTER_CRITICAL(&ringMux_);
  advanceOldestUnackedLocked();
  uint16_t seq = oldestUnackedSeq_;
  const uint16_t pendingCount = static_cast<uint16_t>(nextBatchSeq_ - oldestUnackedSeq_);
  
  // ✅ 核心优化 1：发送窗口限制 (In-Flight Window Limit)
  // 无论本地积压了多少包，我们绝不发送超越 Master 32 位接收窗口的包
  const uint16_t windowLimit = std::min(pendingCount, static_cast<uint16_t>(32));

  for (uint16_t i = 0; i < windowLimit; ++i, ++seq) {
    BatchSlot &slot = batchRing_[seq % BATCH_RING_CAPACITY];
    if (!slot.used || slot.batchSeq != seq) {
      continue;
    }

    // ✅ 核心优化 2 & 4：判断是否到了该发的时间
    // 如果 nextSendUs != 0 且当前时间还未到达 nextSendUs，跳过不发
    if (slot.nextSendUs != 0 && static_cast<int32_t>(nowUs - slot.nextSendUs) < 0) {
      continue; 
    }

    outPacket = slot.packet;
    if (slot.sendAttempts > 0) {
      outPacket.bytes[9] |= capture::IMU_FLAG_RETRANSMIT;
      capture::writeImuBatchCrc(outPacket);
    }

    slot.nextSendUs = nowUs + RETRANSMIT_INTERVAL_US + random(RETRANSMIT_JITTER_US);

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
    applyPendingRadioConfig();

    if (!loadMasterMac(masterMac)) {
      vTaskDelay(pdMS_TO_TICKS(2));
      continue;
    }

    if (!esp_now_is_peer_exist(masterMac)) {
      esp_now_peer_info_t peerInfo = {};
      memcpy(peerInfo.peer_addr, masterMac, 6);
      peerInfo.channel = 0;
      peerInfo.encrypt = false;
      peerInfo.ifidx = WIFI_IF_STA;
      if (esp_now_add_peer(&peerInfo) != ESP_OK) {
        continue;
      }
    }

    const uint8_t state = currentState();
    const uint32_t nowMs = millis();
    if ((state == capture::STATE_SYNC || state == capture::STATE_STREAM_PENDING || state == capture::STATE_STREAM) &&
        static_cast<uint32_t>(nowMs - lastSyncProbeMs_) >= SYNC_PROBE_INTERVAL_MS) {
      sendSyncProbe();
      lastSyncProbeMs_ = nowMs;
    }

    if (static_cast<uint32_t>(nowMs - lastSyncDiagMs_) >= SYNC_DIAG_INTERVAL_MS) {
      if (state == capture::STATE_SYNC || state == capture::STATE_STREAM_PENDING) {
        sendSyncDiag();
      } else if (state == capture::STATE_STREAM) {
        portENTER_CRITICAL(&offsetMux_);
        updateClockFitLocked();
        portEXIT_CRITICAL(&offsetMux_);
      }
      lastSyncDiagMs_ = millis();
    }

    if (state == capture::STATE_STREAM_PENDING &&
        static_cast<int32_t>(localToMasterTimeUs(micros()) - pendingStreamStartUs_) >= 0) {
      portENTER_CRITICAL(&offsetMux_);
      state_ = capture::STATE_STREAM;
      streamStarted_ = true;
      freezeSampleClockLocked();
      portEXIT_CRITICAL(&offsetMux_);
      sendStateAck(lastCommandSeq_, pendingStreamStartUs_);
    }

    if (currentState() != capture::STATE_STREAM || !streamStarted_) {
      vTaskDelay(pdMS_TO_TICKS(2));
      continue;
    }

    const uint32_t nowUs = micros();
    const uint32_t masterNowUs = localToMasterTimeUs(nowUs);
    const uint32_t slotIndex = masterNowUs / SEND_SUPERFRAME_US;
    const uint32_t phaseUs = masterNowUs % SEND_SUPERFRAME_US;
    if (slotIndex == lastSlotIndex || phaseUs < config_.sendSlotOffsetUs || phaseUs >= config_.sendSlotOffsetUs + 5000) {
      vTaskDelay(pdMS_TO_TICKS(1));
      continue;
    }
    lastSlotIndex = slotIndex;

    for (uint8_t sendCount = 0; sendCount < MAX_SENDS_PER_SLOT; ++sendCount) {
      // 每个 slot 最多追发 2 个 batch，避免重发挤占下一批新数据的 ACK 时间。
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
  // 默认 1kHz 软件定时采样；配置 DATA_RDY 引脚后改为 MPU6500 硬件中断定时。
  sensorTaskHandle_ = xTaskGetCurrentTaskHandle();
  uint32_t nextSampleUs = micros();

  capture::ImuRawSample batchSamples[capture::IMU_BATCH_SIZE] = {};
  uint8_t batchCount = 0;
  bool wasStreaming = false;

  for (;;) {
    if (currentState() != capture::STATE_STREAM || !streamStarted_) {
      if (batchCount > 0) {
        batchCount = 0;
      }
      wasStreaming = false;
      if (dataReadyInterruptEnabled_) {
        ulTaskNotifyTake(pdTRUE, 0);
      }
      vTaskDelay(pdMS_TO_TICKS(2));
      nextSampleUs = micros();
      continue;
    }

    if (!wasStreaming) {
      nextSampleUs = micros();
      batchCount = 0;
      wasStreaming = true;
    }

    uint32_t localTimeUs = 0;
    if (dataReadyInterruptEnabled_) {
      if (ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(20)) == 0) {
        noteDroppedSamples(static_cast<uint32_t>(batchCount) + 1U);
        batchCount = 0;
        continue;
      }
      localTimeUs = lastDataReadyLocalUs_;
    } else {
      while (static_cast<int32_t>(micros() - nextSampleUs) < 0) {
        taskYIELD();
      }
      nextSampleUs += SAMPLE_PERIOD_US;
      localTimeUs = micros();
    }

    uint32_t sampleTimeUs = localToSampleTimeUs(localTimeUs);
    portENTER_CRITICAL(&ringMux_);
    if (hasLastSampleTimestamp_ && static_cast<int32_t>(sampleTimeUs - lastSampleTimestampUs_) <= 0) {
      sampleTimeUs = lastSampleTimestampUs_ + 1U;
    }
    lastSampleTimestampUs_ = sampleTimeUs;
    hasLastSampleTimestamp_ = true;
    portEXIT_CRITICAL(&ringMux_);

    capture::ImuRawSample sample{};
    sample.timestampUs = sampleTimeUs;

    if (!readSensor(sample)) {
      noteDroppedSamples(static_cast<uint32_t>(batchCount) + 1U);
      batchCount = 0;
      continue;
    }

    batchSamples[batchCount++] = sample;

    if (batchCount >= capture::IMU_BATCH_SIZE) {
      storeBatch(batchSamples, batchCount);
      batchCount = 0;
    }
  }
}
