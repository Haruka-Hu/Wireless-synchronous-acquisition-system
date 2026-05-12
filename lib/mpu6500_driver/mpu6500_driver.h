#pragma once

#include <Arduino.h>
#include <SPI.h>
#include <stdint.h>

// MPU6500 SPI 驱动：
// 同一套驱动同时服务 Gyro 节点和 Accel 节点，具体读取哪个量由上层配置。
class Mpu6500Driver {
 public:
  // 每个 slave 独占一组 SPI 引脚，避免多设备总线仲裁复杂度。
  struct Pins {
    uint8_t cs;
    uint8_t mosi;
    uint8_t miso;
    uint8_t sck;
    int8_t intPin;
  };

  // gyroConfig/accelConfig 直接对应 MPU6500 量程寄存器值。
  struct Config {
    Pins pins;
    uint8_t gyroConfig;
    uint8_t accelConfig;
  };

  // 保存 MPU6500 引脚和量程配置。
  explicit Mpu6500Driver(const Config &config);

  // 初始化 MPU6500 并做基本连通性检查。
  bool begin();
  // 返回原始 ADC 值；物理单位换算放在上位机/分析脚本中处理。
  bool readGyro(int16_t &gx, int16_t &gy, int16_t &gz);
  // 读取三轴加速度原始值。
  bool readAccel(int16_t &ax, int16_t &ay, int16_t &az);
  // INT/DATA_RDY 引脚是可选接线；未配置时采样任务继续使用软件定时。
  bool hasDataReadyInterrupt() const;
  int8_t dataReadyInterruptPin() const;

 private:
  // 写 MPU6500 单个寄存器。
  void writeReg(uint8_t reg, uint8_t val);
  // 读 MPU6500 单个寄存器。
  uint8_t readReg(uint8_t reg);
  // 从 MPU6500 连续读取多个寄存器。
  void readBytes(uint8_t startReg, uint8_t *dst, size_t len);

  Config config_;
};
