#include "mpu6500_driver.h"

namespace {

constexpr uint8_t REG_PWR_MGMT_1 = 0x6B;
constexpr uint8_t REG_USER_CTRL = 0x6A;
constexpr uint8_t REG_SMPLRT_DIV = 0x19;
constexpr uint8_t REG_CONFIG = 0x1A;
constexpr uint8_t REG_GYRO_CONFIG = 0x1B;
constexpr uint8_t REG_ACCEL_CONFIG = 0x1C;
constexpr uint8_t REG_ACCEL_CONFIG2 = 0x1D;
constexpr uint8_t REG_WHO_AM_I = 0x75;
constexpr uint8_t REG_ACCEL_XOUT_H = 0x3B;
constexpr uint8_t REG_GYRO_XOUT_H = 0x43;

static const SPISettings MPU_CFG_SPI(1000000, MSBFIRST, SPI_MODE0);
static const SPISettings MPU_READ_SPI(8000000, MSBFIRST, SPI_MODE0);

}  // namespace

// 保存 MPU6500 SPI 引脚和量程寄存器配置。
Mpu6500Driver::Mpu6500Driver(const Config &config) : config_(config) {}

// 初始化 MPU6500，完成复位、滤波、采样率和量程设置。
bool Mpu6500Driver::begin() {
  // 配置阶段使用较低 SPI 频率，完成复位、时钟源、低通滤波和量程设置。
  SPI.begin(config_.pins.sck, config_.pins.miso, config_.pins.mosi, config_.pins.cs);
  pinMode(config_.pins.cs, OUTPUT);
  digitalWrite(config_.pins.cs, HIGH);

  writeReg(REG_PWR_MGMT_1, 0x80);
  delay(100);
  writeReg(REG_PWR_MGMT_1, 0x01);
  delay(10);
  writeReg(REG_USER_CTRL, 0x10);
  writeReg(REG_SMPLRT_DIV, 0x00);
  writeReg(REG_CONFIG, 0x01);
  writeReg(REG_GYRO_CONFIG, config_.gyroConfig);
  writeReg(REG_ACCEL_CONFIG, config_.accelConfig);
  writeReg(REG_ACCEL_CONFIG2, 0x01);

  // WHO_AM_I 只做基本连通性检查：0x00/0xFF 通常表示 SPI 断线或 CS 错误。
  const uint8_t whoAmI = readReg(REG_WHO_AM_I);
  return whoAmI != 0x00 && whoAmI != 0xFF;
}

// 读取三轴陀螺仪原始值。
bool Mpu6500Driver::readGyro(int16_t &gx, int16_t &gy, int16_t &gz) {
  // MPU6500 连续寄存器为大端格式，高字节在前。
  uint8_t raw[6] = {0};
  readBytes(REG_GYRO_XOUT_H, raw, sizeof(raw));

  gx = static_cast<int16_t>((static_cast<uint16_t>(raw[0]) << 8) | raw[1]);
  gy = static_cast<int16_t>((static_cast<uint16_t>(raw[2]) << 8) | raw[3]);
  gz = static_cast<int16_t>((static_cast<uint16_t>(raw[4]) << 8) | raw[5]);
  return true;
}

// 读取三轴加速度原始值。
bool Mpu6500Driver::readAccel(int16_t &ax, int16_t &ay, int16_t &az) {
  // 这里返回原始值，保持无线包紧凑，也避免固件和分析脚本重复维护比例系数。
  uint8_t raw[6] = {0};
  readBytes(REG_ACCEL_XOUT_H, raw, sizeof(raw));

  ax = static_cast<int16_t>((static_cast<uint16_t>(raw[0]) << 8) | raw[1]);
  ay = static_cast<int16_t>((static_cast<uint16_t>(raw[2]) << 8) | raw[3]);
  az = static_cast<int16_t>((static_cast<uint16_t>(raw[4]) << 8) | raw[5]);
  return true;
}

// 通过 SPI 写 MPU6500 单个寄存器。
void Mpu6500Driver::writeReg(uint8_t reg, uint8_t val) {
  SPI.beginTransaction(MPU_CFG_SPI);
  digitalWrite(config_.pins.cs, LOW);
  SPI.transfer(reg & 0x7FU);
  SPI.transfer(val);
  digitalWrite(config_.pins.cs, HIGH);
  SPI.endTransaction();
}

// 通过 SPI 读取 MPU6500 单个寄存器。
uint8_t Mpu6500Driver::readReg(uint8_t reg) {
  SPI.beginTransaction(MPU_READ_SPI);
  digitalWrite(config_.pins.cs, LOW);
  SPI.transfer(reg | 0x80U);
  const uint8_t val = SPI.transfer(0x00);
  digitalWrite(config_.pins.cs, HIGH);
  SPI.endTransaction();
  return val;
}

// 从 MPU6500 连续寄存器区读取多字节数据。
void Mpu6500Driver::readBytes(uint8_t startReg, uint8_t *dst, size_t len) {
  // 读操作最高位必须置 1；多字节读取依赖 MPU6500 自动地址递增。
  SPI.beginTransaction(MPU_READ_SPI);
  digitalWrite(config_.pins.cs, LOW);
  SPI.transfer(startReg | 0x80U);
  for (size_t i = 0; i < len; ++i) {
    dst[i] = SPI.transfer(0x00);
  }
  digitalWrite(config_.pins.cs, HIGH);
  SPI.endTransaction();
}
