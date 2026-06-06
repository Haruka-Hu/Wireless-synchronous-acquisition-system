#pragma once

#include <Arduino.h>
#include <stdint.h>
#include <map>  // 引入 map 容器用于动态管理档位

#include "bts7960_motor.h"

// 电机节点应用层：
// - BLE 只负责收发文本命令；
// - BTS7960Motor 只负责底层 PWM；
// - MotorApp 在中间解析命令，并用非阻塞状态机完成叩击/手动转动。
class MotorApp {
 public:
  // 单次叩击分三段：先正向小力触碰，再反向回撤，最后正向敲击。
  struct StrikeProfile {
    int touchPwm;       // 新增：触碰肌腱时的 PWM（力度）
    uint32_t touchMs;   // 新增：触碰肌腱的持续时间
    int pullPwm;
    uint32_t pullMs;
    int strikePwm;
    uint32_t strikeMs;
  };

  // Nordic UART 风格 BLE 服务配置，由入口文件集中给出。
  struct BleConfig {
    const char *deviceName;
    const char *serviceUuid;
    const char *rxUuid;
    const char *txUuid;
  };

  // 节点运行参数。这里保持为简单 POD，便于静态全局对象初始化。
  struct Config {
    BleConfig ble;
    Bts7960Motor::Config motor;
    const StrikeProfile *strikeProfiles;
    uint8_t strikeProfileCount;
    int manualPwm;
    uint32_t serialBaud;
  };

  // 构造电机应用并保存 BLE、电机和档位参数。
  explicit MotorApp(const Config &config);

  // 初始化串口、BLE、电机驱动和命令队列。
  void begin();
  // loopOnce 不阻塞；它只推进当前状态机，让 STOP 可以快速打断。
  void loopOnce();
  // BLE 回调最终都回到应用层，避免 BLE 类直接操作电机。
  // BLE 连接建立后的处理入口。
  void onBleConnect();
  // BLE 断开后的处理入口。
  void onBleDisconnect();
  // BLE RX characteristic 收到文本后的处理入口。
  void onBleWrite(const String &rawValue);

 private:
  enum class CommandType : uint8_t {
    Ping,
    Stop,
    Strike,
    ManualForward,
    ManualReverse,
    SetStrike,  // 设置/更新档位参数
    DelStrike,  // 删除档位
    Unknown,
  };

  // 命令队列把 BLE 回调线程和主 loop 解耦，避免回调里做耗时控制。
  struct MotorCommand {
    CommandType type;
    uint8_t gear;
    StrikeProfile profile; // 用于在队列中传递新的参数设定
  };

  // 叩击状态机的运行阶段。
  enum class MotorMode : uint8_t {
    Idle,
    StrikeTouch,    // 新增：触碰阶段
    StrikePull,
    StrikeFire,
    ManualForward,
    ManualReverse,
  };

  // 静态命令投递入口，供 BLE 回调转发。
  static void enqueueCommandStatic(const MotorCommand &command);
  // 将文本命令解析为内部命令。
  static MotorCommand parseCommand(String raw);

  // 初始化 BLE NUS 服务。
  void initBle();
  // 将命令放入队列。
  void enqueueCommand(const MotorCommand &command);
  // 向上位机发送状态信息。
  void notifyHost(const String &message);
  // 立即停止电机并清空状态。
  void stopMotor();
  // 启动指定档位叩击。
  void beginStrike(uint8_t gear);
  // 启动手动正转。
  void beginManualForward();
  // 启动手动反转。
  void beginManualReverse();
  // 执行一条内部命令。
  void handleCommand(const MotorCommand &command);
  // 推进非阻塞叩击状态机。
  void updateMotorState();

  Config config_;
  Bts7960Motor motor_;
  QueueHandle_t commandQueue_ = nullptr;
  bool bleConnected_ = false;
  MotorMode mode_ = MotorMode::Idle;
  uint8_t activeGear_ = 0;
  // 当前阶段的结束时间，使用 millis() 时间域。
  uint32_t phaseDeadlineMs_ = 0;

  // 动态存储档位参数字典 (Key: 档位号, Value: 动作参数)
  std::map<uint8_t, StrikeProfile> strikeProfiles_;
};