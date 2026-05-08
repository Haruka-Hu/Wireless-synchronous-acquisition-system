#pragma once

#include <Arduino.h>
#include <esp_now.h>
#include <stdint.h>

#include "ads1298_driver.h"
#include "capture_common.h"

// MasterApp 是采集链路中心节点：
// 1. 采集本地 ADS1298 EMG；
// 2. 广播 Beacon 给 Slave 做时间同步；
// 3. 接收 IMU batch、去重、回 ACK；
// 4. 把 EMG/IMU/Diag 统一发到 PC。
class MasterApp {
 public:
  // 板级配置只保留硬件引脚和串口速率，协议参数在 capture_common 中统一定义。
  struct Config {
    Ads1298Driver::Pins adsPins;
    uint32_t serialBaud;
  };

  // 构造 Master 应用并保存板级配置。
  explicit MasterApp(const Config &config);

  // 初始化硬件和后台任务。
  void begin();

 private:
  // ESP-NOW 回调只负责把原始包塞进队列，后续解码在无线任务中做。
  struct SlaveRxItem {
    uint8_t mac[6];
    capture::ImuBatchWirePacket packet;
  };

  // 用于诊断在线状态，不参与可靠传输确认窗口。
  struct SlaveTracker {
    bool used;
    uint8_t mac[6];
    uint8_t source;
    uint32_t lastSeenUs;
    uint32_t totalSamples;
  };

  // Master 为每个 Slave 维护一个 32 位接收窗口。
  // ackBaseSeq 表示连续收到的最后一个 batch；recvBitmap 表示后续乱序到达的 batch。
  struct SlaveRxState {
    bool used;
    bool initialized;
    uint8_t mac[6];
    uint8_t source;
    uint16_t ackBaseSeq;
    uint32_t ackSampleSeq;
    uint32_t recvBitmap;
    uint32_t duplicatePackets;
    uint32_t retransmitPackets;
    uint32_t missingPackets;
  };

  static constexpr size_t MAX_TRACKED_SLAVES = 16;
  static constexpr uint32_t SLAVE_OFFLINE_TIMEOUT_US = 3000000;

  // Arduino/ESP-NOW/FreeRTOS 需要 C 风格回调，用静态函数转发到当前 MasterApp 实例。
  static void onAdsDrdyStatic();
  static void onEspNowRecvStatic(const uint8_t *mac, const uint8_t *data, int len);
  static void serialTxTaskStatic(void *arg);
  static void wirelessTaskStatic(void *arg);
  static void sensorTaskStatic(void *arg);

  // ADS1298 DRDY ISR 的实例处理函数。
  void onAdsDrdyFromIsr();
  // ESP-NOW 收包的实例处理函数。
  void onEspNowRecv(const uint8_t *mac, const uint8_t *data, int len);
  // USB CDC 批量发送任务主体。
  void serialTxTask();
  // Beacon、IMU 解码、ACK 和诊断任务主体。
  void wirelessTask();
  // ADS1298 采样任务主体。
  void sensorTask();

  // 无线和 ACK 窗口管理。
  // 初始化 ESP-NOW 通道和广播 peer。
  bool initEspNow();
  // 确保 Slave MAC 已被登记为可单播 peer。
  bool ensureEspNowPeer(const uint8_t mac[6]);
  // 更新 Slave 在线诊断状态。
  void trackSlave(const uint8_t mac[6], uint8_t source, uint32_t nowUs, uint32_t sampleCount);
  // 统计当前在线 Slave 数量。
  int32_t countOnlineSlaves(uint32_t nowUs);
  // 获取或创建 Slave 的 ACK 接收窗口。
  SlaveRxState *getSlaveRxState(const uint8_t mac[6], uint8_t source);
  // 推进连续 ACK 基准。
  void advanceAckWindow(SlaveRxState &state);
  // 标记 batch 已收到，并判断是否重复。
  void markBatchReceived(SlaveRxState &state, uint16_t batchSeq, uint32_t sampleStartSeq, uint8_t flags, bool &outDuplicate);
  // 发送当前接收窗口对应的 ACK。
  void sendAck(const SlaveRxState &state);

  // 串口输出统一入口。所有来源最终都先进入 serialQueue_，再由 serialTxTask 合批写出。
  void writeSerialSample(uint8_t source,
                         uint32_t timestampUs,
                         uint32_t sampleSeq,
                         uint16_t batchSeq,
                         uint8_t flags,
                         int32_t x,
                         int32_t y,
                         int32_t z);
  // 将指定字节全部写入 USB CDC。
  void writeSerialBytesAll(const uint8_t *data, size_t size);
  // 写入一条诊断帧。
  void writeDiagFrame(uint32_t timestampUs, int32_t forwardedPerSec, int32_t errorsPerSec, int32_t onlineSlaveCount);

  Config config_;
  Ads1298Driver ads_;
  TaskHandle_t sensorTaskHandle_ = nullptr;
  QueueHandle_t slaveRxQueue_ = nullptr;
  QueueHandle_t serialQueue_ = nullptr;
  uint8_t broadcastMac_[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

  // 下面这些计数器每秒汇总为 SOURCE_DIAG，方便 PC 端观察链路质量。
  uint32_t serialQueueDrops_ = 0;
  uint32_t slaveQueueDrops_ = 0;
  uint32_t slaveDecodeFails_ = 0;
  uint32_t slaveCrcFails_ = 0;
  uint32_t slaveForwardedSamples_ = 0;
  uint32_t serialWriteShorts_ = 0;
  uint32_t slaveRegistryDrops_ = 0;
  uint32_t slaveDuplicatePackets_ = 0;
  uint32_t slaveRetransmitPackets_ = 0;
  uint32_t slaveMissingPackets_ = 0;
  uint32_t ackSendFails_ = 0;
  uint8_t pcSerialSequence_ = 0;
  uint32_t emgSampleSeq_ = 0;

  SlaveTracker slaveTrackers_[MAX_TRACKED_SLAVES] = {};
  SlaveRxState slaveRxStates_[MAX_TRACKED_SLAVES] = {};
};
