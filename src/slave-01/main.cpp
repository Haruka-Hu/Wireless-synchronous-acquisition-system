#include <Arduino.h>

#include "imu_slave_node.h"

namespace {

// slave-01：膝跳反射小腿端陀螺仪节点。
// source=0x01，20ms superframe 内 +2ms 发送，和加速度节点错峰。
ImuSlaveApp g_app({
    0x01,
    2000,
    ImuSlaveApp::SensorMode::Gyro,
    {{10, 11, 13, 12}, 0x18, 0x10},
    240,
    2000000,
    32,
});

}  // namespace

// Arduino 启动入口：初始化陀螺仪 Slave 应用。
void setup() {
  g_app.begin();
}

// Arduino 主循环：Slave 的采样和无线发送都在后台任务中运行。
void loop() {
  // 采样和无线发送均由 ImuSlaveApp 的 FreeRTOS 任务负责。
  vTaskDelay(portMAX_DELAY);
}
