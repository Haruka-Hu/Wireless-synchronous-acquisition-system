#include "capture_common.h"

#include <string.h>

namespace capture {

// 计算 CCITT CRC16，用于 ESP-NOW 和 USB CDC 两类二进制帧的完整性校验。
uint16_t crc16Ccitt(const uint8_t *data, size_t len) {
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < len; ++i) {
    crc ^= static_cast<uint16_t>(data[i]) << 8;
    for (uint8_t bit = 0; bit < 8; ++bit) {
      crc = (crc & 0x8000U) ? static_cast<uint16_t>((crc << 1) ^ 0x1021U) : static_cast<uint16_t>(crc << 1);
    }
  }
  return crc;
}

// 从小端字节流读取 uint16_t，避免不同平台结构体对齐/端序差异。
uint16_t decodeU16LE(const uint8_t *src) {
  return static_cast<uint16_t>(src[0]) |
         (static_cast<uint16_t>(src[1]) << 8);
}

// 从小端字节流读取 uint32_t，专门服务 timestamp/sample_seq 等 32 位字段。
uint32_t decodeU32LE(const uint8_t *src) {
  return static_cast<uint32_t>(src[0]) |
         (static_cast<uint32_t>(src[1]) << 8) |
         (static_cast<uint32_t>(src[2]) << 16) |
         (static_cast<uint32_t>(src[3]) << 24);
}

// 从小端字节流读取 int16_t，用于 IMU 三轴原始值。
int16_t decodeI16LE(const uint8_t *src) {
  return static_cast<int16_t>(decodeU16LE(src));
}

// 将 uint16_t 按小端写入 wire buffer。
void encodeU16LE(uint8_t *dst, uint16_t value) {
  dst[0] = static_cast<uint8_t>(value & 0xFFU);
  dst[1] = static_cast<uint8_t>((value >> 8) & 0xFFU);
}

// 将 uint32_t 按小端写入 wire buffer。
void encodeU32LE(uint8_t *dst, uint32_t value) {
  dst[0] = static_cast<uint8_t>(value & 0xFFU);
  dst[1] = static_cast<uint8_t>((value >> 8) & 0xFFU);
  dst[2] = static_cast<uint8_t>((value >> 16) & 0xFFU);
  dst[3] = static_cast<uint8_t>((value >> 24) & 0xFFU);
}

// 将 int16_t 按小端写入 wire buffer。
void encodeI16LE(uint8_t *dst, int16_t value) {
  encodeU16LE(dst, static_cast<uint16_t>(value));
}

// 将 int32_t 按小端写入 wire buffer。
void encodeI32LE(uint8_t *dst, int32_t value) {
  encodeU32LE(dst, static_cast<uint32_t>(value));
}

// 比较两个 6 字节 MAC 地址是否完全一致。
bool macEquals(const uint8_t a[6], const uint8_t b[6]) {
  for (uint8_t i = 0; i < 6; ++i) {
    if (a[i] != b[i]) {
      return false;
    }
  }
  return true;
}

void writeImuBatchCrc(ImuBatchWirePacket &packet) {
  const uint16_t crc = crc16Ccitt(packet.bytes, IMU_BATCH_WIRE_SIZE - 2);
  encodeU16LE(&packet.bytes[IMU_BATCH_WIRE_SIZE - 2], crc);
}

// Slave 打包时只记录第一个样本的绝对时间，其余样本用 uint16_t 相对时间。
// 这样 20 个 1kHz 样本仍可放进 ESP-NOW 的常见负载限制。
ImuBatchWirePacket makeImuBatchPacket(uint8_t source,
                                      const ImuRawSample *samples,
                                      uint8_t count,
                                      uint16_t batchSeq,
                                      uint32_t sampleStartSeq) {
  ImuBatchWirePacket packet = {};
  packet.bytes[0] = MSG_TYPE_IMU_BATCH;
  packet.bytes[1] = source;
  encodeU16LE(&packet.bytes[2], batchSeq);
  encodeU32LE(&packet.bytes[4], sampleStartSeq);
  packet.bytes[8] = count;
  packet.bytes[9] = IMU_FLAG_TIME_SYNCED;
  const uint32_t baseTimeUs = samples[0].timestampUs;
  encodeU32LE(&packet.bytes[10], baseTimeUs);

  size_t offset = IMU_BATCH_HEADER_SIZE;
  for (uint8_t i = 0; i < count; ++i) {
    const uint16_t dtUs = static_cast<uint16_t>(samples[i].timestampUs - baseTimeUs);
    encodeU16LE(&packet.bytes[offset + 0], dtUs);
    encodeI16LE(&packet.bytes[offset + 2], samples[i].x);
    encodeI16LE(&packet.bytes[offset + 4], samples[i].y);
    encodeI16LE(&packet.bytes[offset + 6], samples[i].z);
    offset += IMU_SAMPLE_WIRE_SIZE;
  }

  writeImuBatchCrc(packet);
  return packet;
}

// ACK 窗口采用“连续基准 + 32 位位图”：
// diff=0 表示已经落在连续确认区，diff=1..32 对应 bitmap 的 bit0..bit31。
bool ackCoversBatchSeq(uint16_t seq, uint16_t ackBaseSeq, uint32_t recvBitmap) {
  const uint16_t diff = static_cast<uint16_t>(seq - ackBaseSeq);
  if (diff == 0) {
    return true;
  }
  if (diff >= 1 && diff <= 32) {
    return (recvBitmap & (1UL << (diff - 1))) != 0;
  }
  return false;
}

// 解码顺序必须和 makeImuBatchPacket 完全一致。
// Master 只有在类型、source、count、CRC 都通过后才会向 PC 转发。
DecodeStatus decodeImuBatchPacket(const ImuBatchWirePacket &wirePacket,
                                  ImuDecodedSample *outSamples,
                                  uint8_t &outCount,
                                  uint8_t &outSource,
                                  uint16_t &outBatchSeq,
                                  uint32_t &outSampleStartSeq,
                                  uint8_t &outFlags) {
  if (wirePacket.bytes[0] != MSG_TYPE_IMU_BATCH) {
    return DECODE_BAD_TYPE;
  }

  const uint8_t source = wirePacket.bytes[1];
  if (source < SLAVE_SOURCE_MIN || source > SLAVE_SOURCE_MAX || source == SOURCE_DIAG) {
    return DECODE_BAD_SOURCE;
  }

  const uint16_t batchSeq = decodeU16LE(&wirePacket.bytes[2]);
  const uint32_t sampleStartSeq = decodeU32LE(&wirePacket.bytes[4]);
  const uint8_t count = wirePacket.bytes[8];
  const uint8_t flags = wirePacket.bytes[9];
  const uint32_t baseTimeUs = decodeU32LE(&wirePacket.bytes[10]);
  if (count == 0 || count > IMU_BATCH_SIZE) {
    return DECODE_BAD_COUNT;
  }

  const uint16_t receivedCrc = decodeU16LE(&wirePacket.bytes[IMU_BATCH_WIRE_SIZE - 2]);
  const uint16_t calculatedCrc = crc16Ccitt(wirePacket.bytes, IMU_BATCH_WIRE_SIZE - 2);
  if (receivedCrc != calculatedCrc) {
    return DECODE_BAD_CRC;
  }

  size_t offset = IMU_BATCH_HEADER_SIZE;
  for (uint8_t i = 0; i < count; ++i) {
    ImuDecodedSample sample{};
    sample.timestampUs = baseTimeUs + static_cast<uint32_t>(decodeU16LE(&wirePacket.bytes[offset + 0]));
    sample.sampleSeq = sampleStartSeq + i;
    sample.batchSeq = batchSeq;
    sample.flags = flags;
    sample.x = decodeI16LE(&wirePacket.bytes[offset + 2]);
    sample.y = decodeI16LE(&wirePacket.bytes[offset + 4]);
    sample.z = decodeI16LE(&wirePacket.bytes[offset + 6]);
    outSamples[i] = sample;
    offset += IMU_SAMPLE_WIRE_SIZE;
  }

  outCount = count;
  outSource = source;
  outBatchSeq = batchSeq;
  outSampleStartSeq = sampleStartSeq;
  outFlags = flags;
  return DECODE_OK;
}

// Slave 只接受发给自己 source 的 ACK，防止两个 Slave 之间误释放缓存。
bool decodeAckPacket(const uint8_t *data, int len, uint8_t expectedSource, uint16_t &ackBaseSeq, uint32_t &recvBitmap) {
  if (data == nullptr || len != static_cast<int>(IMU_ACK_WIRE_SIZE) || data[0] != MSG_TYPE_IMU_ACK || data[1] != expectedSource) {
    return false;
  }

  const uint16_t receivedCrc = decodeU16LE(&data[IMU_ACK_WIRE_SIZE - 2]);
  const uint16_t calculatedCrc = crc16Ccitt(data, IMU_ACK_WIRE_SIZE - 2);
  if (receivedCrc != calculatedCrc) {
    return false;
  }

  ackBaseSeq = decodeU16LE(&data[2]);
  recvBitmap = decodeU32LE(&data[8]);
  return true;
}

// ACK 中保留 masterTimeUs，是为了以后需要时可把确认和时间校正合并；
// 当前时间同步仍由 Beacon 负责，避免改变已有采样时序。
void buildAckPacket(uint8_t source,
                    uint16_t ackBaseSeq,
                    uint32_t ackSampleSeq,
                    uint32_t recvBitmap,
                    uint32_t masterTimeUs,
                    uint8_t outBytes[IMU_ACK_WIRE_SIZE]) {
  memset(outBytes, 0, IMU_ACK_WIRE_SIZE);
  outBytes[0] = MSG_TYPE_IMU_ACK;
  outBytes[1] = source;
  encodeU16LE(&outBytes[2], ackBaseSeq);
  encodeU32LE(&outBytes[4], ackSampleSeq);
  encodeU32LE(&outBytes[8], recvBitmap);
  encodeU32LE(&outBytes[12], masterTimeUs);
  const uint16_t crc = crc16Ccitt(outBytes, IMU_ACK_WIRE_SIZE - 2);
  encodeU16LE(&outBytes[IMU_ACK_WIRE_SIZE - 2], crc);
}

// USB CDC 帧也带 CRC，PC 端可以发现串口半帧、错位或解析版本不一致。
size_t buildPcBatchPacket(const PcSample *samples, uint8_t count, uint8_t sequence, uint8_t *outBytes) {
  outBytes[0] = PACKET_HEAD_0;
  outBytes[1] = PACKET_HEAD_1;
  outBytes[2] = PC_MSG_SAMPLE_BATCH;
  outBytes[3] = count;
  outBytes[4] = sequence;

  size_t offset = PC_SERIAL_HEADER_SIZE;
  for (uint8_t i = 0; i < count; ++i) {
    outBytes[offset + 0] = samples[i].source;
    encodeU32LE(&outBytes[offset + 1], samples[i].timestampUs);
    encodeU32LE(&outBytes[offset + 5], samples[i].sampleSeq);
    encodeU16LE(&outBytes[offset + 9], samples[i].batchSeq);
    outBytes[offset + 11] = samples[i].flags;
    encodeI32LE(&outBytes[offset + 12], samples[i].x);
    encodeI32LE(&outBytes[offset + 16], samples[i].y);
    encodeI32LE(&outBytes[offset + 20], samples[i].z);
    offset += PC_SAMPLE_WIRE_SIZE;
  }

  const uint16_t crc = crc16Ccitt(outBytes, offset);
  encodeU16LE(&outBytes[offset], crc);
  return offset + 2;
}

}  // namespace capture
