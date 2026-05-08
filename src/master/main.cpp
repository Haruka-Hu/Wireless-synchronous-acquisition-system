#include <Arduino.h>

#include "master_node.h"

namespace {

// Master 节点：本地采 ADS1298 EMG，同时接收两个 IMU Slave 的 ESP-NOW batch。
// 参数依次为 ADS1298 引脚 {CS, MOSI, MISO, SCK, DRDY, RESET} 和 USB CDC 波特率。
MasterApp g_app({
    {10, 11, 13, 12, 9, 14},
    2000000,
});

}  // namespace

// Arduino 启动入口：把 Master 应用初始化并交给 FreeRTOS 任务运行。
void setup() {
  g_app.begin();
}

// Arduino 主循环：Master 无周期逻辑，永久挂起以节省调度开销。
void loop() {
  // MasterApp 已经创建 FreeRTOS 任务，Arduino loop 不再承担周期逻辑。
  vTaskDelay(portMAX_DELAY);
}
