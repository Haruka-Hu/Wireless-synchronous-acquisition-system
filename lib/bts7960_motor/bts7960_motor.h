#pragma once

#include <Arduino.h>
#include <stdint.h>

// BTS7960 双半桥电机驱动的最小封装。
// 这里只处理使能脚和两路 PWM，不解析 BLE 命令，也不包含叩击时序。
class Bts7960Motor {
 public:
  // pwmChannelR/L 是 ESP32 LEDC 通道，不是 GPIO 编号。
  struct Config {
    int pinRpwm;
    int pinLpwm;
    int pinRen;
    int pinLen;
    int pwmChannelR;
    int pwmChannelL;
    int pwmFreq;
    int pwmResolutionBits;
  };

  // 保存 BTS7960 引脚和 PWM 配置。
  explicit Bts7960Motor(const Config &config);

  // 初始化使能脚和 PWM 通道。
  void begin();
  // 停止电机输出。
  void stop();
  // 正反转均会保证另一侧 PWM 为 0，避免桥臂同时驱动。
  void driveForward(int pwm);
  // 反向驱动电机。
  void driveReverse(int pwm);
  // 直接写入正反两路 PWM。
  void writeDuty(int dutyForward, int dutyReverse);
  // 返回当前 PWM 分辨率对应的最大值。
  int maxPwm() const;

 private:
  // 将 PWM 限制到合法范围。
  int clampPwm(int value) const;

  Config config_;
};
