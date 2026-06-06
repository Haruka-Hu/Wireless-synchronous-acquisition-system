#include <Arduino.h>

#include "motor_node.h"

namespace {

constexpr char BLE_DEVICE_NAME[] = "Neuro_Hammer_BLE";
// BLE 使用 Nordic UART 风格 UUID：RX 接收上位机命令，TX 返回 ACK/ERR。
constexpr char NUS_SERVICE_UUID[] = "6E400001-B5A3-F393-E0A9-E50E24DCCA9E";
constexpr char NUS_RX_UUID[] = "6E400002-B5A3-F393-E0A9-E50E24DCCA9E";
constexpr char NUS_TX_UUID[] = "6E400003-B5A3-F393-E0A9-E50E24DCCA9E";

// 开机默认档位参数：{触碰 PWM, 触碰 ms, 回撤 PWM, 回撤 ms, 敲击 PWM, 敲击 ms}。
constexpr MotorApp::StrikeProfile DEFAULT_STRIKE_PROFILES[5] = {
    {200, 100, 300, 200, 400, 150},
    {200, 100, 400, 200, 600, 120},
    {200, 100, 400, 200, 800, 100},
    {200, 100, 500, 200, 950, 80},
    {200, 100, 500, 200, 1023, 70},
};

// BTS7960 引脚沿用原配置：RPWM=41, LPWM=42, REN=39, LEN=40。
MotorApp g_app({
    {BLE_DEVICE_NAME, NUS_SERVICE_UUID, NUS_RX_UUID, NUS_TX_UUID},
    {41, 42, 39, 40, 0, 1, 20000, 10},
    DEFAULT_STRIKE_PROFILES,
    5,
    512,
    115200,
});

}  // namespace

// Arduino 启动入口：初始化 BLE、电机驱动和电机应用状态。
void setup() {
  g_app.begin();
}

// Arduino 主循环：周期性处理 BLE 命令和电机状态机。
void loop() {
  // 电机节点用 loopOnce 推进非阻塞状态机，保证 STOP 能快速响应。
  g_app.loopOnce();
}