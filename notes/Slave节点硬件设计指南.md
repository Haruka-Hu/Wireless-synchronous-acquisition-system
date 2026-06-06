# Slave 节点硬件设计指南

本文档用于指导后续绘制两类无线 Slave 节点的原理图和 PCB：

- `IMU Slave`：使用 `MPU6500`，用于小腿或锤子端运动采集。
- `MMG Slave`：使用数字 `I2S MEMS` 麦克风，用于肌音图（MMG）采集。

两类节点建议采用同一套公共硬件底板架构，仅替换传感器前端。这样可以复用电源、主控、无线、充电、调试、外壳、电池和固定结构。

## 1. 总体架构

推荐架构如下：

```text
1S LiPo 电池
    |
充电管理 + 电源路径管理
    |
SYS_RAW
    |
3.3 V 稳压
    |
ESP32-S3-MINI-1U + 柔性天线
    |
传感器前端
    |-- IMU 版：MPU6500, SPI
    |-- MMG 版：I2S MEMS 麦克风
```

公共部分包括：

- ESP32-S3-MINI-1U 主控。
- 外接柔性天线与 U.FL/IPEX 天线座。
- 双 Type-C 接口。
- 单节锂电池接口。
- 充电管理与电源路径管理。
- 3.3 V 稳压。
- BOOT / EN 按键。
- 状态 LED。
- 电池电压采样。
- 充电状态与 USB 插入状态检测。
- 调试测试点。

传感器部分包括：

- IMU 版：`MPU6500 + SPI + INT`。
- MMG 版：`I2S MEMS 麦克风 + 声孔/机械耦合结构`。

## 2. 原理图页面划分

建议原理图按以下页面组织。

### 2.1 主控与无线页

核心器件：

- `ESP32-S3-MINI-1U`
- U.FL/IPEX 天线座
- 40 MHz 晶振相关内容按模组参考设计处理（如果模组内部已集成则不需要外部晶振）
- BOOT 按键
- EN/RESET 按键
- 状态 LED
- 必要测试点

必须保留的信号：

```text
USB_D+
USB_D-
BOOT
EN
UART_TX_TEST
UART_RX_TEST
STATUS_LED
USER_BTN
BAT_ADC
VBUS_DATA_DET
VBUS_CHG_DET
CHG_STAT
```

设计要点：

- 使用 `ESP32-S3-MINI-1U`，优先外接柔性天线，改善穿戴场景无线质量。
- U.FL/IPEX 到模组天线脚的射频走线按 50 ohm 控制阻抗设计。
- 天线区域必须设置 keep-out：不要铺铜、不要走线、不要放电池、螺丝或金属结构。
- BOOT 和 EN 必须保留实体按键或测试点，避免后续无法救砖。
- USB D+/D- 从 USB-C DATA 口直接到 ESP32-S3，走差分线，尽量短、等长、少过孔。

### 2.2 双 Type-C 与电源输入页

两个 Type-C 口定义如下：

```text
USB-C DATA/PWR
    用途：USB 烧录、串口调试、上位机通信、临时外部供电

USB-C CHARGE ONLY
    用途：专门给锂电池充电
```

`USB-C DATA/PWR` 连接：

```text
VBUS_DATA -> ESD/TVS -> 保险丝/限流 -> 电源 OR-ing / 电源 MUX -> SYS_RAW
D+        -> ESD -> ESP32-S3 USB_D+
D-        -> ESD -> ESP32-S3 USB_D-
CC1/CC2   -> 5.1k 下拉到 GND
GND       -> 系统地
```

`USB-C CHARGE ONLY` 连接：

```text
VBUS_CHG -> ESD/TVS -> 保险丝/限流 -> 充电管理芯片 IN
CC1/CC2  -> 5.1k 下拉到 GND
GND      -> 系统地
```

注意事项：

- 两个 Type-C 的 VBUS 不允许直接短接。
- 两个 Type-C 同时插入时，不能发生倒灌。
- 丝印必须明确标注：`USB/DATA` 和 `CHARGE ONLY`。
- 两个 Type-C 的外壳地可以通过地平面连接；如后续出现噪声问题，可预留 RC/磁珠位置。

## 3. 充电与电源路径

### 3.1 推荐方案

推荐使用带 power-path 的单节锂电池充电管理芯片，例如：

- `BQ24074`
- `BQ24075`

推荐电源路径：

```text
USB-C CHARGE ONLY VBUS
    -> BQ24074/BQ24075 IN
    -> BAT 接 1S LiPo
    -> OUT/SYS_RAW

USB-C DATA/PWR VBUS
    -> 理想二极管 / 负载开关 / 电源 MUX
    -> SYS_RAW

SYS_RAW
    -> 3.3 V 稳压
    -> ESP32-S3 + 传感器
```

如果希望电源切换更稳，推荐使用电源 MUX，例如 `TPS2121`。如果第一版想简化，也可以使用理想二极管芯片或肖特基 OR-ing，但需要注意压降和发热。

### 3.2 3.3 V 稳压

系统电压统一为 `3.3 V`。

可选方案：

- 低噪声 LDO：简单、噪声低，但电池电压低于约 3.5 V 后余量不足。
- Buck-boost：电池电压较低时仍能维持 3.3 V，更适合完整放电范围。

建议第一版优先考虑 buck-boost 或高质量低压差 LDO，取决于板子空间和续航要求。

3.3 V 供电对象：

- ESP32-S3-MINI-1U
- MPU6500 或 I2S 麦克风
- 状态 LED
- 电压检测分压

### 3.3 电池接口

建议使用 2-pin 或 3-pin 电池座：

```text
BAT+
BAT-
NTC，可选
```

如果所选充电芯片支持 NTC，建议预留 NTC 引脚。即使第一版不用，也可以通过电阻配置为正常温度范围。

## 4. 电源与状态检测

建议给 ESP32-S3 保留以下检测信号。

```text
BAT_ADC        电池电压采样
VBUS_DATA_DET  USB/DATA 口插入检测
VBUS_CHG_DET   CHARGE ONLY 口插入检测
CHG_STAT       充电状态检测
```

### 4.1 电池电压采样

推荐结构：

```text
BAT+ -> Rtop -> ADC 节点 -> Rbottom -> GND
ADC 节点 -> 小电容 -> GND
```

建议：

- 分压后最大电压不要超过 ESP32 ADC 输入范围。
- 电阻可选较大阻值，例如 `470k + 220k` 附近的量级，用于降低静态耗电。
- ADC 节点加 `10 nF ~ 100 nF` 电容滤波。
- 如果非常在意静态功耗，可用 MOS 管控制分压，仅采样时打开。

### 4.2 USB 插入检测

`VBUS_DATA_DET` 和 `VBUS_CHG_DET` 可以通过分压接入 GPIO/ADC。

用途：

- 判断 USB/DATA 是否接入。
- 判断充电口是否接入。
- 后续实现插入充电口时自动挂起无线通信、进入低功耗或充电状态。

### 4.3 充电状态检测

如果充电芯片提供 `STAT` 或类似引脚，连接到 ESP32 GPIO。

用途：

- 显示正在充电 / 充满 / 异常。
- 上位机诊断时上报电源状态。

## 5. IMU Slave 传感器页

IMU 传感器使用 `MPU6500`，建议继续使用 SPI。

推荐信号：

```text
MPU6500_SCK
MPU6500_MOSI
MPU6500_MISO
MPU6500_CS
MPU6500_INT
MPU6500_3V3
MPU6500_GND
```

设计要点：

- `MPU6500_INT` 接 ESP32 GPIO，用于 DATA_RDY 中断。
- SPI 线尽量短，SCK 旁边保持连续参考地。
- MPU6500 靠近真正需要测量的位置，不要放在板子柔软或容易晃动的位置。
- 传感器远离天线、电源电感、Type-C、充电芯片和 USB 差分线。
- MPU6500 下方尽量保留完整地平面，不走大电流线。
- PCB 丝印标出 `X/Y/Z` 方向。

建议原理图增加：

- MPU6500 电源去耦电容。
- CS 上拉或默认电平配置，按最终驱动需求决定。
- INT 线上可预留小串联电阻或 0 ohm 电阻，便于调试。

## 6. MMG Slave 传感器页

MMG 传感器使用数字 `I2S MEMS` 麦克风，第一版推荐选择成熟器件，例如：

- `ICS-43434`
- `INMP441`

注意：MMG 关注低频肌肉机械振动，选型时必须确认麦克风低频响应。如果数字 MEMS 麦克风低频截止不理想，后续需要考虑模拟 MEMS/驻极体麦克风 + 低频前端。

推荐信号：

```text
I2S_BCLK
I2S_WS / LRCLK
I2S_DATA_IN
MIC_LR_SELECT
MIC_3V3
MIC_GND
```

`MIC_LR_SELECT` 可固定接 GND 或 3.3 V，用于选择左/右声道。也可以通过 0 ohm 电阻选择，方便改板。

布局要点：

- 麦克风声孔必须有明确开孔，不能被外壳、胶带、绑带或电池遮挡。
- 麦克风远离电源电感、天线、USB-C 和 USB D+/D-。
- 麦克风区域要有稳定机械固定方式。
- 如果要贴在肌肉表面，建议预留软垫、硅胶圈或小腔体的安装空间。
- 声孔/耦合结构应在 PCB 和外壳设计中一起考虑，不要只画电路。

建议第一版预留：

- 麦克风声孔定位丝印。
- 麦克风附近机械固定孔或外壳压紧区域。
- I2S 线上串联 0 ohm 电阻位置，便于调试信号完整性。

## 7. 建议引脚分配

具体 GPIO 需要结合最终 ESP32-S3-MINI-1U 封装引脚和 PCB 走线再确认。第一版可以按以下逻辑分配。

### 7.1 公共引脚

```text
USB_D+          ESP32-S3 原生 USB D+
USB_D-          ESP32-S3 原生 USB D-
BOOT            下载按键
EN              复位按键
STATUS_LED      状态灯
USER_BTN        用户按键
BAT_ADC         电池电压采样
VBUS_DATA_DET   USB/DATA 插入检测
VBUS_CHG_DET    CHARGE ONLY 插入检测
CHG_STAT        充电状态
UART_TX_TEST    测试点
UART_RX_TEST    测试点
```

### 7.2 IMU 版引脚

```text
SPI_SCK
SPI_MOSI
SPI_MISO
MPU6500_CS
MPU6500_INT
```

### 7.3 MMG 版引脚

```text
I2S_BCLK
I2S_WS
I2S_DATA_IN
MIC_LR_SELECT，可用 0 ohm 电阻固定
```

建议不要让 IMU 和 MMG 传感器引脚完全随机分配。最好让两块板的公共引脚完全一致，传感器接口使用同一组预留 GPIO 区域，这样后续固件配置和调试更简单。

## 8. PCB 布局建议

### 8.1 分区

推荐 PCB 分为四个区域：

```text
[天线区]      ESP32-S3-MINI-1U + U.FL
[电源区]      Type-C + 充电 + 稳压 + 电池座
[传感器区]    MPU6500 或 I2S 麦克风
[调试区]      BOOT / EN / LED / 测试点
```

布局原则：

- 天线区靠板边，保留 keep-out。
- 电源区靠近 Type-C 和电池座。
- 传感器区远离电源区和天线区。
- 调试区放在容易接触的位置。

### 8.2 射频

- U.FL/IPEX 附近按参考设计保留净空。
- 天线下面不要铺铜。
- 天线附近不要放电池、屏蔽罩、螺丝、Type-C 外壳。
- 柔性天线尽量伸出人体遮挡区域。

### 8.3 电源

- 充电芯片、电源 MUX、稳压器靠近输入和负载。
- 电感和开关节点远离传感器。
- 3.3 V 主电源走线足够宽。
- 每个芯片电源脚附近放置去耦电容。

### 8.4 USB

- USB D+/D- 走差分，短、直、少过孔。
- USB 差分线远离麦克风、MPU6500 和天线。
- Type-C 接口附近放 ESD 保护。

### 8.5 传感器

IMU：

- 放在机械稳定点。
- 标注坐标轴。
- 远离电源噪声和射频区域。

MMG：

- 声孔必须对外。
- 预留机械耦合结构。
- 远离高频数字线和电源开关节点。

## 9. 丝印与接口标注

必须清楚标注：

```text
USB/DATA
CHARGE ONLY
BAT+
BAT-
BOOT
EN
X/Y/Z
MIC PORT 或 SOUND HOLE
ANT KEEP OUT
```

建议在板边标注：

```text
IMU Slave v1.0
MMG Slave v1.0
3V3
GND
```

## 10. 上电与硬件测试清单

### 10.1 裸板检查

- 检查 3.3 V 与 GND 是否短路。
- 检查 BAT+ 与 GND 是否短路。
- 检查两个 Type-C 的 VBUS 是否被直接短接。
- 检查 USB D+/D- 是否短路。
- 检查天线区域是否误铺铜。

### 10.2 电源测试

- 只接电池，确认 3.3 V 正常。
- 只接 USB/DATA，确认系统可供电。
- 只接 CHARGE ONLY，确认电池充电路径正常。
- 两个 Type-C 同时插入，确认无倒灌、无异常发热。
- 测试 `VBUS_DATA_DET`、`VBUS_CHG_DET`、`CHG_STAT` 电平是否正确。

### 10.3 ESP32-S3 测试

- USB/DATA 口能识别串口。
- BOOT + EN 能进入下载模式。
- 状态 LED 可控。
- 能读取电池电压 ADC。
- 外接柔性天线后无线通信稳定。

### 10.4 IMU 版测试

- MPU6500 `WHO_AM_I` 读取通过。
- SPI 波形正常。
- INT/DATA_RDY 中断正常。
- 静止时三轴数据稳定。
- 1 kHz 连续采样无明显丢样。

### 10.5 MMG 版测试

- I2S BCLK、WS、DATA 波形正常。
- I2S DMA 连续采样稳定。
- 轻敲或肌肉振动时波形有响应。
- 声孔没有被外壳或固定结构堵住。
- 改变机械耦合方式时波形变化可观察。

## 11. 第一版 PCB 建议

第一版不要追求一次做到最小、最美观。建议优先保证：

- 电源路径稳定。
- 双 Type-C 不倒灌。
- ESP32-S3 可稳定烧录和调试。
- 天线区域足够干净。
- IMU 和 MMG 传感器有足够调试空间。
- 关键电源和信号都有测试点。

推荐第一版保留较多 0 ohm 电阻、测试点和跳线选项。等 IMU 和 MMG 两类节点都验证稳定后，再做第二版小型化和结构优化。

