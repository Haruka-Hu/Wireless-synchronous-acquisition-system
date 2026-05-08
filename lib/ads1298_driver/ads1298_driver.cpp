#include "ads1298_driver.h"

namespace {

static const SPISettings ADS_SPI_SETTINGS(2000000, MSBFIRST, SPI_MODE1);

}  // namespace

// 保存 ADS1298 的 SPI/控制引脚配置。
Ads1298Driver::Ads1298Driver(const Pins &pins) : pins_(pins) {}

// 完成 ADS1298 复位、寄存器配置、连续转换启动和 DRDY 中断绑定。
void Ads1298Driver::begin(void (*drdyIsr)()) {
  // ADS1298 对 SPI 模式和上电时序较敏感，初始化顺序尽量保持和原型代码一致。
  SPI.begin(pins_.sck, pins_.miso, pins_.mosi, pins_.cs);

  pinMode(pins_.cs, OUTPUT);
  digitalWrite(pins_.cs, HIGH);
  pinMode(pins_.reset, OUTPUT);
  digitalWrite(pins_.reset, HIGH);
  pinMode(pins_.drdy, INPUT_PULLUP);

  digitalWrite(pins_.reset, LOW);
  delay(20);
  digitalWrite(pins_.reset, HIGH);
  delay(150);

  // SDATAC 后才能稳定写寄存器；后面配置采样率、通道输入和偏置相关寄存器。
  sendCmd(0x11);
  delay(10);

  writeReg(0x03, 0xCC);
  writeReg(0x01, 0x84);

  for (int i = 0; i < 5; ++i) {
    writeReg(static_cast<uint8_t>(0x05 + i), 0x81);
  }
  writeReg(0x0A, 0x60);
  writeReg(0x0B, 0x60);
  writeReg(0x0C, 0x60);

  writeReg(0x0D, 0xE0);
  writeReg(0x0E, 0xE0);
  writeReg(0x00, 0x00);

  sendCmd(0x08);
  delay(2);
  sendCmd(0x10);

  attachInterrupt(digitalPinToInterrupt(pins_.drdy), drdyIsr, FALLING);
}

// 读取一次 ADS1298 转换帧，并返回本项目使用的 CH6/CH7/CH8 原始值。
bool Ads1298Driver::readChannels(int32_t &ch6, int32_t &ch7, int32_t &ch8) {
  // RDATA 帧为 3 字节状态 + 8 路 24-bit 通道，本项目只使用最后三路。
  uint8_t rx[27] = {0};
  SPI.beginTransaction(ADS_SPI_SETTINGS);
  digitalWrite(pins_.cs, LOW);
  SPI.transferBytes(nullptr, rx, sizeof(rx));
  digitalWrite(pins_.cs, HIGH);
  SPI.endTransaction();

  ch6 = signExtend24((static_cast<uint32_t>(rx[18]) << 16) |
                     (static_cast<uint32_t>(rx[19]) << 8) |
                     static_cast<uint32_t>(rx[20]));
  ch7 = signExtend24((static_cast<uint32_t>(rx[21]) << 16) |
                     (static_cast<uint32_t>(rx[22]) << 8) |
                     static_cast<uint32_t>(rx[23]));
  ch8 = signExtend24((static_cast<uint32_t>(rx[24]) << 16) |
                     (static_cast<uint32_t>(rx[25]) << 8) |
                     static_cast<uint32_t>(rx[26]));
  return true;
}

// 向 ADS1298 发送单字节命令，例如 SDATAC/RDATAC/START。
void Ads1298Driver::sendCmd(uint8_t cmd) {
  SPI.beginTransaction(ADS_SPI_SETTINGS);
  digitalWrite(pins_.cs, LOW);
  SPI.transfer(cmd);
  digitalWrite(pins_.cs, HIGH);
  SPI.endTransaction();
}

// 写入一个 ADS1298 寄存器，并在写后留出芯片处理时间。
void Ads1298Driver::writeReg(uint8_t addr, uint8_t val) {
  SPI.beginTransaction(ADS_SPI_SETTINGS);
  digitalWrite(pins_.cs, LOW);
  SPI.transfer(0x40 | (addr & 0x1FU));
  SPI.transfer(0x00);
  SPI.transfer(val);
  digitalWrite(pins_.cs, HIGH);
  SPI.endTransaction();
  delay(2);
}

// 将 ADS1298 的 24-bit 二补码通道值扩展为 int32_t。
int32_t Ads1298Driver::signExtend24(uint32_t data) {
  // ADS1298 通道数据是 24-bit 二补码，需要扩展到 int32_t 后再发给 PC。
  if (data & 0x800000U) {
    return static_cast<int32_t>(data | 0xFF000000U);
  }
  return static_cast<int32_t>(data);
}
