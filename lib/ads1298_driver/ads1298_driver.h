#pragma once

#include <Arduino.h>
#include <SPI.h>
#include <stdint.h>

// ADS1298 底层驱动：
// 只封装 SPI、寄存器配置和 CH6/CH7/CH8 读取。
// 采样节奏、DRDY 中断计数和串口发送由 MasterApp 负责。
class Ads1298Driver {
 public:
  // ADS1298 使用独立 SPI 引脚，DRDY/RESET 也由驱动初始化。
  struct Pins {
    uint8_t cs;
    uint8_t mosi;
    uint8_t miso;
    uint8_t sck;
    uint8_t drdy;
    uint8_t reset;
  };

  // 保存 ADS1298 引脚配置。
  explicit Ads1298Driver(const Pins &pins);

  // begin 会完成芯片复位、寄存器配置，并把 DRDY ISR 绑定到引脚。
  void begin(void (*drdyIsr)());
  // 读取一次完整转换结果，仅返回本项目需要的 CH6/CH7/CH8。
  bool readChannels(int32_t &ch6, int32_t &ch7, int32_t &ch8);

 private:
  // 发送 ADS1298 单字节命令。
  void sendCmd(uint8_t cmd);
  // 写 ADS1298 单个寄存器。
  void writeReg(uint8_t addr, uint8_t val);
  // 将 24-bit 通道值符号扩展为 int32_t。
  int32_t signExtend24(uint32_t data);

  Pins pins_;
};
