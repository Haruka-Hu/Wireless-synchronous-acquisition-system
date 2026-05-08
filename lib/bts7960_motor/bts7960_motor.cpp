#include "bts7960_motor.h"

// 保存 BTS7960 引脚、LEDC 通道和 PWM 分辨率配置。
Bts7960Motor::Bts7960Motor(const Config &config) : config_(config) {}

// 初始化使能脚和两路 LEDC PWM，并默认停机。
void Bts7960Motor::begin() {
  // REN/LEN 拉高后 BTS7960 两侧半桥才响应 PWM。
  pinMode(config_.pinRen, OUTPUT);
  pinMode(config_.pinLen, OUTPUT);
  digitalWrite(config_.pinRen, HIGH);
  digitalWrite(config_.pinLen, HIGH);

  ledcSetup(config_.pwmChannelR, config_.pwmFreq, config_.pwmResolutionBits);
  ledcSetup(config_.pwmChannelL, config_.pwmFreq, config_.pwmResolutionBits);
  ledcAttachPin(config_.pinRpwm, config_.pwmChannelR);
  ledcAttachPin(config_.pinLpwm, config_.pwmChannelL);

  stop();
}

// 将正反两路 PWM 都置零，实现电机停止。
void Bts7960Motor::stop() {
  writeDuty(0, 0);
}

// 以指定 PWM 正转，反向通道保持 0。
void Bts7960Motor::driveForward(int pwm) {
  writeDuty(pwm, 0);
}

// 以指定 PWM 反转，正向通道保持 0。
void Bts7960Motor::driveReverse(int pwm) {
  writeDuty(0, pwm);
}

// 直接写入两路 PWM，占空比会先被限制到合法范围。
void Bts7960Motor::writeDuty(int dutyForward, int dutyReverse) {
  // 所有入口都经过 clamp，防止档位表或调试命令给出超出 LEDC 分辨率的 PWM。
  ledcWrite(config_.pwmChannelR, clampPwm(dutyForward));
  ledcWrite(config_.pwmChannelL, clampPwm(dutyReverse));
}

// 根据 LEDC 分辨率计算最大 PWM 数值。
int Bts7960Motor::maxPwm() const {
  return (1 << config_.pwmResolutionBits) - 1;
}

// 将输入 PWM 限制在 0..maxPwm() 范围。
int Bts7960Motor::clampPwm(int value) const {
  if (value < 0) {
    return 0;
  }
  const int maxValue = maxPwm();
  if (value > maxValue) {
    return maxValue;
  }
  return value;
}
