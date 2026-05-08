#pragma once

#include <Arduino.h>
#include <stddef.h>
#include <stdint.h>

namespace capture {

// capture_common 是固件侧协议的唯一来源。
// Master、Slave 和 PC 上位机解析都应以这里的字段顺序和长度为准。

// Master -> PC 的 USB CDC 帧头和帧类型。
constexpr uint8_t PACKET_HEAD_0 = 0xAA;
constexpr uint8_t PACKET_HEAD_1 = 0x55;
constexpr uint8_t PC_MSG_SAMPLE_BATCH = 0x30;

// 统一 source 编号。0x01..0x6F 留给无线 Slave，0x7E 单独作为诊断帧。
constexpr uint8_t SOURCE_EMG = 0x00;
constexpr uint8_t SOURCE_DIAG = 0x7E;
constexpr uint8_t SLAVE_SOURCE_MIN = 0x01;
constexpr uint8_t SLAVE_SOURCE_MAX = 0x6F;

// ESP-NOW 链路的消息类型。Beacon 做时间同步，IMU_BATCH 传数据，IMU_ACK 做确认。
constexpr uint8_t MSG_TYPE_BEACON = 0x10;
constexpr uint8_t MSG_TYPE_IMU_ACK = 0x11;
constexpr uint8_t MSG_TYPE_IMU_BATCH = 0x20;
constexpr uint8_t ESPNOW_CHANNEL = 1;

// IMU batch 固定传 20 个 sample 槽位。count 可以小于 20，但空口长度保持固定，便于队列和解析。
constexpr size_t IMU_BATCH_SIZE = 20;
constexpr size_t IMU_SAMPLE_WIRE_SIZE = 8;
constexpr size_t IMU_BATCH_HEADER_SIZE = 14;
constexpr size_t IMU_BATCH_WIRE_SIZE = IMU_BATCH_HEADER_SIZE + IMU_BATCH_SIZE * IMU_SAMPLE_WIRE_SIZE + 2;
constexpr size_t IMU_ACK_WIRE_SIZE = 18;

// flags 会原样进入 CSV 的 rx_flags 字段，用来判断重传和实验段是否有效。
constexpr uint8_t IMU_FLAG_RETRANSMIT = 0x01;
constexpr uint8_t IMU_FLAG_TIME_SYNCED = 0x02;
constexpr uint8_t IMU_FLAG_LINK_FAULT = 0x04;

// PC 串口每帧最多合批 24 条样本。单样本 24 字节，包含 sample_seq/batch_seq/flags。
constexpr size_t PC_SAMPLE_WIRE_SIZE = 24;
constexpr uint8_t PC_SERIAL_MAX_SAMPLES = 24;
constexpr size_t PC_SERIAL_HEADER_SIZE = 5;
constexpr size_t PC_SERIAL_MAX_PACKET_SIZE = PC_SERIAL_HEADER_SIZE + PC_SERIAL_MAX_SAMPLES * PC_SAMPLE_WIRE_SIZE + 2;

// Master 广播的时间信标。结构体 packed 后直接通过 ESP-NOW 发送。
struct BeaconPacket {
  uint8_t type;
  uint32_t masterTimeUs;
} __attribute__((packed));

// Slave 采样任务内部使用的原始样本：时间戳已经换算到 Master 时间轴。
struct ImuRawSample {
  uint32_t timestampUs;
  int16_t x;
  int16_t y;
  int16_t z;
} __attribute__((packed));

// Master 解码后的 IMU 样本，额外带上连续性检查所需的 sampleSeq/batchSeq/flags。
struct ImuDecodedSample {
  uint32_t timestampUs;
  uint32_t sampleSeq;
  uint16_t batchSeq;
  uint8_t flags;
  int16_t x;
  int16_t y;
  int16_t z;
} __attribute__((packed));

// 无线层固定长度字节容器。不要把它当字段结构体直接 reinterpret_cast。
struct ImuBatchWirePacket {
  uint8_t bytes[IMU_BATCH_WIRE_SIZE];
};

// Master -> PC 的统一样本。EMG、IMU、Diag 都走这一种串口样本格式。
struct PcSample {
  uint8_t source;
  uint32_t timestampUs;
  uint32_t sampleSeq;
  uint16_t batchSeq;
  uint8_t flags;
  int32_t x;
  int32_t y;
  int32_t z;
} __attribute__((packed));

// 解码失败原因用于 Master 统计诊断，不直接发给 Slave。
enum DecodeStatus {
  DECODE_OK,
  DECODE_BAD_TYPE,
  DECODE_BAD_COUNT,
  DECODE_BAD_SOURCE,
  DECODE_BAD_CRC,
};

// 计算 CCITT CRC16，所有二进制帧共用这一套校验。
uint16_t crc16Ccitt(const uint8_t *data, size_t len);
// 从 wire buffer 按小端读取 uint16_t。
uint16_t decodeU16LE(const uint8_t *src);
// 从 wire buffer 按小端读取 uint32_t。
uint32_t decodeU32LE(const uint8_t *src);
// 从 wire buffer 按小端读取 int16_t。
int16_t decodeI16LE(const uint8_t *src);
// 将 uint16_t 按小端写入 wire buffer。
void encodeU16LE(uint8_t *dst, uint16_t value);
// 将 uint32_t 按小端写入 wire buffer。
void encodeU32LE(uint8_t *dst, uint32_t value);
// 将 int16_t 按小端写入 wire buffer。
void encodeI16LE(uint8_t *dst, int16_t value);
// 将 int32_t 按小端写入 wire buffer。
void encodeI32LE(uint8_t *dst, int32_t value);
// 比较两个 ESP-NOW MAC 地址是否一致。
bool macEquals(const uint8_t a[6], const uint8_t b[6]);

// 重新写入 batch 末尾 CRC。重传或故障 flags 变化后必须调用。
void writeImuBatchCrc(ImuBatchWirePacket &packet);

// 按当前 wire format 构造 Slave -> Master 的 IMU batch。
ImuBatchWirePacket makeImuBatchPacket(uint8_t source,
                                      const ImuRawSample *samples,
                                      uint8_t count,
                                      uint16_t batchSeq,
                                      uint32_t sampleStartSeq);

// 判断某个 batchSeq 是否已经被 ACK 的 base 或 bitmap 覆盖。
bool ackCoversBatchSeq(uint16_t seq, uint16_t ackBaseSeq, uint32_t recvBitmap);

// 校验并解码 IMU batch。只有返回 DECODE_OK 时 outSamples/outCount 等输出才可信。
DecodeStatus decodeImuBatchPacket(const ImuBatchWirePacket &wirePacket,
                                  ImuDecodedSample *outSamples,
                                  uint8_t &outCount,
                                  uint8_t &outSource,
                                  uint16_t &outBatchSeq,
                                  uint32_t &outSampleStartSeq,
                                  uint8_t &outFlags);

// 校验并提取 ACK 中的确认窗口。MasterTimeUs 当前只保留在 wire format 中，不参与 Slave 决策。
bool decodeAckPacket(const uint8_t *data, int len, uint8_t expectedSource, uint16_t &ackBaseSeq, uint32_t &recvBitmap);

// 构造 Master -> Slave 的 ACK 包。
void buildAckPacket(uint8_t source,
                    uint16_t ackBaseSeq,
                    uint32_t ackSampleSeq,
                    uint32_t recvBitmap,
                    uint32_t masterTimeUs,
                    uint8_t outBytes[IMU_ACK_WIRE_SIZE]);

// 构造 Master -> PC 的 USB CDC 批量帧。
size_t buildPcBatchPacket(const PcSample *samples, uint8_t count, uint8_t sequence, uint8_t *outBytes);

}  // namespace capture
