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

const char *stateName(uint8_t state) {
  switch (state) {
    case STATE_IDLE:
      return "IDLE";
    case STATE_SYNC:
      return "SYNC";
    case STATE_STREAM_PENDING:
      return "STREAM_PENDING";
    case STATE_STREAM:
      return "STREAM";
    default:
      return "UNKNOWN";
  }
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

void buildCommandPacket(uint16_t commandSeq,
                        uint8_t targetState,
                        uint32_t effectiveMasterTimeUs,
                        uint8_t outBytes[COMMAND_WIRE_SIZE]) {
  memset(outBytes, 0, COMMAND_WIRE_SIZE);
  outBytes[0] = MSG_TYPE_COMMAND;
  encodeU16LE(&outBytes[1], commandSeq);
  outBytes[3] = targetState;
  encodeU32LE(&outBytes[4], effectiveMasterTimeUs);
  const uint16_t crc = crc16Ccitt(outBytes, COMMAND_WIRE_SIZE - 2);
  encodeU16LE(&outBytes[COMMAND_WIRE_SIZE - 2], crc);
}

bool decodeCommandPacket(const uint8_t *data, int len, CommandPacket &outCommand) {
  if (data == nullptr || len != static_cast<int>(COMMAND_WIRE_SIZE) || data[0] != MSG_TYPE_COMMAND) {
    return false;
  }
  const uint16_t receivedCrc = decodeU16LE(&data[COMMAND_WIRE_SIZE - 2]);
  const uint16_t calculatedCrc = crc16Ccitt(data, COMMAND_WIRE_SIZE - 2);
  if (receivedCrc != calculatedCrc) {
    return false;
  }
  outCommand.commandSeq = decodeU16LE(&data[1]);
  outCommand.targetState = data[3];
  outCommand.effectiveMasterTimeUs = decodeU32LE(&data[4]);
  return outCommand.targetState <= STATE_STREAM;
}

void buildStateAckPacket(uint8_t source,
                         uint8_t state,
                         uint16_t commandSeq,
                         uint32_t effectiveMasterTimeUs,
                         uint8_t outBytes[STATE_ACK_WIRE_SIZE]) {
  memset(outBytes, 0, STATE_ACK_WIRE_SIZE);
  outBytes[0] = MSG_TYPE_STATE_ACK;
  outBytes[1] = source;
  outBytes[2] = state;
  encodeU16LE(&outBytes[3], commandSeq);
  encodeU32LE(&outBytes[5], effectiveMasterTimeUs);
  const uint16_t crc = crc16Ccitt(outBytes, STATE_ACK_WIRE_SIZE - 2);
  encodeU16LE(&outBytes[STATE_ACK_WIRE_SIZE - 2], crc);
}

bool decodeStateAckPacket(const uint8_t *data, int len, StateAckPacket &outAck) {
  if (data == nullptr || len != static_cast<int>(STATE_ACK_WIRE_SIZE) || data[0] != MSG_TYPE_STATE_ACK) {
    return false;
  }
  const uint16_t receivedCrc = decodeU16LE(&data[STATE_ACK_WIRE_SIZE - 2]);
  const uint16_t calculatedCrc = crc16Ccitt(data, STATE_ACK_WIRE_SIZE - 2);
  if (receivedCrc != calculatedCrc) {
    return false;
  }
  outAck.source = data[1];
  outAck.state = data[2];
  outAck.commandSeq = decodeU16LE(&data[3]);
  outAck.effectiveMasterTimeUs = decodeU32LE(&data[5]);
  return outAck.state <= STATE_STREAM;
}

void buildSyncDiagPacket(const SyncDiagPacket &diag, uint8_t outBytes[SYNC_DIAG_WIRE_SIZE]) {
  memset(outBytes, 0, SYNC_DIAG_WIRE_SIZE);
  outBytes[0] = MSG_TYPE_SYNC_DIAG;
  outBytes[1] = diag.source;
  outBytes[2] = diag.state;
  encodeU16LE(&outBytes[3], diag.beaconSeq);
  encodeI32LE(&outBytes[5], diag.offsetUs);
  encodeI32LE(&outBytes[9], diag.driftPpm);
  encodeI32LE(&outBytes[13], diag.residualUs);
  encodeU16LE(&outBytes[17], diag.beaconCount);
  const uint16_t crc = crc16Ccitt(outBytes, SYNC_DIAG_WIRE_SIZE - 2);
  encodeU16LE(&outBytes[SYNC_DIAG_WIRE_SIZE - 2], crc);
}

bool decodeSyncDiagPacket(const uint8_t *data, int len, SyncDiagPacket &outDiag) {
  if (data == nullptr || len != static_cast<int>(SYNC_DIAG_WIRE_SIZE) || data[0] != MSG_TYPE_SYNC_DIAG) {
    return false;
  }
  const uint16_t receivedCrc = decodeU16LE(&data[SYNC_DIAG_WIRE_SIZE - 2]);
  const uint16_t calculatedCrc = crc16Ccitt(data, SYNC_DIAG_WIRE_SIZE - 2);
  if (receivedCrc != calculatedCrc) {
    return false;
  }
  outDiag.source = data[1];
  outDiag.state = data[2];
  outDiag.beaconSeq = decodeU16LE(&data[3]);
  outDiag.offsetUs = static_cast<int32_t>(decodeU32LE(&data[5]));
  outDiag.driftPpm = static_cast<int32_t>(decodeU32LE(&data[9]));
  outDiag.residualUs = static_cast<int32_t>(decodeU32LE(&data[13]));
  outDiag.beaconCount = decodeU16LE(&data[17]);
  return outDiag.state <= STATE_STREAM;
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

size_t buildPcSyncDiagPacket(const SyncDiagPacket &diag, uint8_t sequence, uint8_t *outBytes) {
  outBytes[0] = PACKET_HEAD_0;
  outBytes[1] = PACKET_HEAD_1;
  outBytes[2] = PC_MSG_SYNC_DIAG;
  outBytes[3] = sequence;
  outBytes[4] = diag.source;
  outBytes[5] = diag.state;
  encodeU16LE(&outBytes[6], diag.beaconSeq);
  encodeI32LE(&outBytes[8], diag.offsetUs);
  encodeI32LE(&outBytes[12], diag.driftPpm);
  encodeI32LE(&outBytes[16], diag.residualUs);
  encodeU16LE(&outBytes[20], diag.beaconCount);
  const size_t payloadEnd = 22;
  const uint16_t crc = crc16Ccitt(outBytes, payloadEnd);
  encodeU16LE(&outBytes[payloadEnd], crc);
  return payloadEnd + 2;
}

size_t buildPcStateEventPacket(const StateAckPacket &event, uint8_t sequence, uint8_t *outBytes) {
  outBytes[0] = PACKET_HEAD_0;
  outBytes[1] = PACKET_HEAD_1;
  outBytes[2] = PC_MSG_STATE_EVENT;
  outBytes[3] = sequence;
  outBytes[4] = event.source;
  outBytes[5] = event.state;
  encodeU16LE(&outBytes[6], event.commandSeq);
  encodeU32LE(&outBytes[8], event.effectiveMasterTimeUs);
  const size_t payloadEnd = 12;
  const uint16_t crc = crc16Ccitt(outBytes, payloadEnd);
  encodeU16LE(&outBytes[payloadEnd], crc);
  return payloadEnd + 2;
}

}  // namespace capture
