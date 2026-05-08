#pragma once

#include <Arduino.h>
#include <esp_now.h>
#include <stdint.h>

#include "capture_common.h"
#include "mpu6500_driver.h"

// ImuSlaveApp 复用在 slave-01 和 slave-02 上。
// 两个节点只通过 Config 区分 source、传感器模式和错峰发送 slot。
class ImuSlaveApp {
 public:
  enum class SensorMode : uint8_t {
    Gyro,
    Accel,
  };

  // 板级入口传入的所有差异化参数。
  // 协议长度、batch 大小等公共参数放在 capture_common，避免两端不一致。
  struct Config {
    uint8_t nodeSource;
    uint32_t sendSlotOffsetUs;
    SensorMode sensorMode;
    Mpu6500Driver::Config mpuConfig;
    uint32_t cpuFreqMhz;
    uint32_t serialBaud;
    int8_t espnowTxPowerQdbm;
  };

  // 构造 Slave 应用并保存节点配置。
  explicit ImuSlaveApp(const Config &config);

  // 初始化硬件、ESP-NOW 和后台任务。
  void begin();

 private:
  // 可靠链路的本地缓存槽。采样任务只写 ring，发送任务根据 ACK 释放槽位。
  struct BatchSlot {
    bool used;
    uint8_t sendAttempts;
    uint16_t batchSeq;
    uint32_t sampleStartSeq;
    uint32_t lastSendUs;
    capture::ImuBatchWirePacket packet;
  };

  struct SyncPoint {
    uint32_t localUs;
    uint32_t masterUs;
  };

  static constexpr size_t BATCH_RING_CAPACITY = 256;
  static constexpr uint32_t SAMPLE_PERIOD_US = 1000;
  static constexpr uint32_t SEND_SUPERFRAME_US = 20000;
  static constexpr uint8_t MAX_SENDS_PER_SLOT = 3;
  static constexpr uint32_t RETRANSMIT_INTERVAL_US = 40000;
  static constexpr size_t SYNC_WINDOW_CAPACITY = 64;
  static constexpr uint32_t SYNC_DIAG_INTERVAL_MS = 200;

  // ESP-NOW/FreeRTOS C 风格回调转发到当前 ImuSlaveApp。
  static void onEspNowSentStatic(const uint8_t *mac, esp_now_send_status_t status);
  static void onEspNowRecvStatic(const uint8_t *mac, const uint8_t *data, int len);
  static void wirelessTaskStatic(void *arg);
  static void sensorTaskStatic(void *arg);

  // ESP-NOW 发送完成的实例处理函数。
  void onEspNowSent(esp_now_send_status_t status);
  // ESP-NOW 下行包的实例处理函数。
  void onEspNowRecv(const uint8_t *mac, const uint8_t *data, int len);
  // 错峰发送和重传任务主体。
  void wirelessTask();
  // 1kHz 采样和组 batch 任务主体。
  void sensorTask();

  // 时间同步、ACK 和 ring buffer 管理。
  // 初始化 ESP-NOW 和发送功率。
  bool initEspNow();
  bool loadClockModel(double &slope, double &intercept);
  // 读取 Master MAC 地址。
  bool loadMasterMac(uint8_t outMac[6]);
  // 处理 Master Beacon。
  void handleBeacon(const uint8_t *mac, const uint8_t *data, int len);
  void handleCommandPacket(const uint8_t *data, int len);
  // 处理 Master ACK。
  void handleAckPacket(const uint8_t *data, int len);
  void applyState(uint8_t newState, uint16_t commandSeq, uint32_t effectiveMasterTimeUs);
  void sendStateAck(uint16_t commandSeq, uint32_t effectiveMasterTimeUs);
  void resetRingLocked();
  void resetSyncFitLocked();
  void recordBeaconLocked(uint32_t localRecvUs, uint32_t masterTimeUs, uint16_t beaconSeq);
  void updateClockFitLocked();
  void sendSyncDiag();
  uint32_t localToMasterTimeUs(uint32_t localUs);
  uint8_t currentState() const;
  // 推进最早未确认 batch 指针。
  void advanceOldestUnackedLocked();
  // 记录本地丢样并标记 link fault。
  void noteDroppedSamples(uint32_t count);
  // 将 batch 写入 ring buffer。
  bool storeBatch(const capture::ImuRawSample *samples, uint8_t count);
  // 选择一个待发送或待重传 batch。
  bool takeSendCandidate(capture::ImuBatchWirePacket &outPacket, uint32_t nowUs);
  // 读取当前节点配置对应的 IMU 传感器数据。
  bool readSensor(capture::ImuRawSample &sample);

  Config config_;
  Mpu6500Driver mpu_;
  SemaphoreHandle_t sendDoneSem_ = nullptr;

  // offsetMux_ 保护时间同步状态；ringMux_ 保护 batch ring 和样本序号。
  portMUX_TYPE offsetMux_ = portMUX_INITIALIZER_UNLOCKED;
  portMUX_TYPE ringMux_ = portMUX_INITIALIZER_UNLOCKED;
  volatile uint8_t state_ = capture::STATE_IDLE;
  uint16_t lastCommandSeq_ = 0;
  uint32_t pendingStreamStartUs_ = 0;
  double clockSlope_ = 1.0;
  double clockIntercept_ = 0.0;
  int32_t syncOffsetUs_ = 0;
  int32_t syncDriftPpm_ = 0;
  int32_t syncResidualUs_ = 0;
  bool clockModelValid_ = false;
  bool streamStarted_ = false;
  SyncPoint syncPoints_[SYNC_WINDOW_CAPACITY] = {};
  uint8_t syncPointCount_ = 0;
  uint8_t syncPointHead_ = 0;
  uint16_t lastBeaconSeq_ = 0;
  uint32_t lastSyncDiagMs_ = 0;
  uint8_t masterMac_[6] = {0};
  bool hasMasterMac_ = false;
  volatile bool lastSendOk_ = false;
  BatchSlot batchRing_[BATCH_RING_CAPACITY] = {};

  // batchSeq 用于无线 ACK；sampleSeq 用于 PC/MATLAB 检查样本是否连续。
  uint16_t nextBatchSeq_ = 0;
  uint16_t oldestUnackedSeq_ = 0;
  uint32_t nextSampleSeq_ = 0;
  uint32_t ringOverflows_ = 0;

  // 一旦出现丢样或缓存溢出，下一包会带 IMU_FLAG_LINK_FAULT，CSV 可据此标记实验段无效。
  bool linkFaultPending_ = false;
  uint32_t ackPackets_ = 0;
  uint32_t ackedBatches_ = 0;
  uint32_t sendAttempts_ = 0;
  uint32_t sendFails_ = 0;
};
