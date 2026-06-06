#include <Arduino.h>

#include "imu_slave_node.h"

namespace {

// slave-02：锤头加速度节点。
// source=0x02，20ms superframe 内 +10ms 发送，避开 slave-01 的窗口。
ImuSlaveApp g_app({
    0x02,
    10000,
    ImuSlaveApp::SensorMode::Accel,
    {{10, 11, 13, 12, -1}, 0x10, 0x18},
    240,
    2000000,
    84,  // ESP-NOW TX power: 84 qdBm = 21 dBm max.
});

}  // namespace

// Arduino 启动入口：初始化加速度 Slave 应用。
void setup() {
  g_app.begin();
}

// Arduino 主循环：Slave 的采样和无线发送都在后台任务中运行。
void loop() {
  // 采样和无线发送均由 ImuSlaveApp 的 FreeRTOS 任务负责。
  vTaskDelay(portMAX_DELAY);
}
