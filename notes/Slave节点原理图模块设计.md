# Slave 节点原理图模块设计

本文档用于指导绘制两类无线 Slave 节点的原理图：

- `IMU Slave`：传感器使用 `BMI270`。
- `MMG Slave`：传感器使用 `LSM6DSOX`，作为肌肉机械振动/低频运动采集节点使用。

两类节点建议共用同一套主控、电源、充电、双 Type-C、调试和无线模块，只替换传感器页。原理图建议按模块分页绘制，后续 PCB 也按这些模块分区布局。

## 1. 原理图页面建议

建议原理图分为以下页面：

```text
01_MCU_RF
02_USB_TYPEC_POWER
03_CHARGER_POWER_PATH
04_3V3_POWER_AND_BATTERY_MONITOR
05_USER_IO_DEBUG
06_SENSOR_BMI270_IMU
07_SENSOR_LSM6DSOX_MMG
08_TEST_POINTS_AND_MECHANICAL
```

如果只画 IMU 版，可以只保留 `06_SENSOR_BMI270_IMU`。如果只画 MMG 版，可以只保留 `07_SENSOR_LSM6DSOX_MMG`。公共页面尽量保持完全一致。

## 2. 01_MCU_RF：主控与无线模块

### 需要的器件

- `ESP32-S3-MINI-1U` 模组
- U.FL/IPEX 天线座
- 柔性天线
- `BOOT` 按键
- `EN/RESET` 按键
- 若干去耦电容
- 状态 LED 和限流电阻
- 必要测试点

### 主要网络

```text
3V3
GND
ESP_EN
ESP_BOOT
USB_D+
USB_D-
UART_TX_TEST
UART_RX_TEST
STATUS_LED
USER_BTN
BAT_ADC
VBUS_DATA_DET
VBUS_CHG_DET
CHG_STAT
SPI_SCK
SPI_MOSI
SPI_MISO
SENSOR_CS
SENSOR_INT1
SENSOR_INT2
```

### 连接方式

`ESP32-S3-MINI-1U`：

```text
3V3 -> ESP32-S3-MINI-1U 3V3
GND -> ESP32-S3-MINI-1U GND
USB_D+ -> USB-C DATA/PWR D+
USB_D- -> USB-C DATA/PWR D-
ESP_EN -> EN 按键 + 上拉
ESP_BOOT -> BOOT 按键 + 上拉/下拉按官方参考设计
STATUS_LED -> GPIO + 限流电阻 + LED
USER_BTN -> GPIO + 按键
```

天线：

```text
ESP32-S3-MINI-1U RF 引脚 -> 50 ohm 射频走线 -> U.FL/IPEX -> 柔性天线
```

### 设计注意

- `ESP32-S3-MINI-1U` 的射频走线必须按 50 ohm 控制阻抗。
- U.FL/IPEX 和柔性天线附近设置 `ANT_KEEP_OUT` 区域。
- 天线区域不要铺铜、不要走线、不要放电池、螺丝、Type-C 外壳。
- USB D+/D- 走差分线，短、直、少过孔。
- `BOOT` 和 `EN` 必须保留，避免后续无法烧录或复位。

## 3. 02_USB_TYPEC_POWER：双 Type-C 接口

### 需要的器件

- `USB-C DATA/PWR` 母座
- `USB-C CHARGE ONLY` 母座
- USB D+/D- ESD 保护器件
- VBUS TVS 或 ESD 保护器件
- CC 下拉电阻，通常 `5.1k`
- 输入保险丝或限流器件，可选
- 0 ohm 电阻或跳线位，可选

### USB-C DATA/PWR 连接方式

用途：

- ESP32-S3 烧录
- USB CDC 串口调试
- 上位机通信
- 临时外部供电

连接：

```text
DATA_VBUS -> VBUS_DATA_RAW
DATA_D+   -> ESD -> USB_D+
DATA_D-   -> ESD -> USB_D-
DATA_CC1  -> 5.1k -> GND
DATA_CC2  -> 5.1k -> GND
DATA_GND  -> GND
SHIELD    -> GND，或预留 RC/磁珠到 GND
```

### USB-C CHARGE ONLY 连接方式

用途：

- 专门给单节锂电池充电

连接：

```text
CHG_VBUS -> VBUS_CHG_RAW
CHG_CC1  -> 5.1k -> GND
CHG_CC2  -> 5.1k -> GND
CHG_GND  -> GND
SHIELD   -> GND，或预留 RC/磁珠到 GND
```

### 设计注意

- `VBUS_DATA_RAW` 和 `VBUS_CHG_RAW` 不允许直接短接。
- 两个 Type-C 同时插入时不能互相倒灌。
- 丝印必须明确：

```text
USB/DATA
CHARGE ONLY
```

## 4. 03_CHARGER_POWER_PATH：充电与电源路径

### 需要的器件

- 单节锂电池充电管理芯片，推荐 `BQ24074` 或 `BQ24075`
- 电源 MUX 或理想二极管/负载开关，推荐 `TPS2121` 或等效方案
- 充电电流设置电阻
- 输入/输出去耦电容
- 电池接口
- 可选 NTC 电阻或 NTC 接口
- 充电状态 LED，可选

### 推荐连接方式

充电路径：

```text
VBUS_CHG_RAW -> BQ24074/BQ24075 IN
BQ24074/BQ24075 BAT -> BAT+
BQ24074/BQ24075 OUT/SYS -> SYS_FROM_CHARGER
BQ24074/BQ24075 STAT -> CHG_STAT
```

USB/DATA 外部供电路径：

```text
VBUS_DATA_RAW -> 电源 MUX / 理想二极管 -> SYS_FROM_DATA
```

系统电源汇合：

```text
SYS_FROM_CHARGER
SYS_FROM_DATA
    -> 电源 MUX / OR-ing
    -> SYS_RAW
```

### 状态检测

```text
VBUS_DATA_RAW -> 分压 -> VBUS_DATA_DET
VBUS_CHG_RAW  -> 分压 -> VBUS_CHG_DET
CHG_STAT      -> ESP32 GPIO
```

### 设计注意

- 第一版建议优先使用带 power-path 的充电芯片，不建议直接使用简单 TP4056 式充电方案。
- 如果使用 `BQ24074/BQ24075`，按数据手册配置充电电流、输入限流和电池温度检测。
- 如果 NTC 暂时不用，要按数据手册用固定电阻配置为正常温度范围。
- `SYS_RAW` 是后级 3.3 V 稳压输入，不直接给 ESP32 供电。

## 5. 04_3V3_POWER_AND_BATTERY_MONITOR：3.3 V 电源与电池检测

### 需要的器件

- 3.3 V 稳压器
- 输入/输出电容
- 可选磁珠
- 电池电压采样分压电阻
- ADC 滤波电容
- 可选 MOS 控制分压

### 稳压器选择

可选方案：

- Buck-boost：电池低电压时仍能稳定输出 3.3 V，推荐用于最终穿戴节点。
- 低噪声 LDO：结构简单、噪声低，但电池电压较低时可能掉压。

推荐连接：

```text
SYS_RAW -> 3.3 V 稳压器 IN
3.3 V 稳压器 OUT -> 3V3
3.3 V 稳压器 GND -> GND
```

### 电池电压采样

推荐结构：

```text
BAT+ -> Rtop -> BAT_ADC_NODE -> Rbottom -> GND
BAT_ADC_NODE -> Cfilter -> GND
BAT_ADC_NODE -> ESP32 ADC GPIO
```

建议初始取值：

```text
Rtop = 470k
Rbottom = 220k
Cfilter = 10 nF ~ 100 nF
```

### 设计注意

- 分压后 ADC 电压不能超过 ESP32-S3 ADC 允许范围。
- 如果长期待机功耗重要，建议用 MOS 管控制分压，仅采样时打开。
- `3V3` 应给主控和传感器分别就近去耦。

## 6. 05_USER_IO_DEBUG：按键、LED 与调试测试点

### 需要的器件

- `BOOT` 按键
- `EN/RESET` 按键
- `USER` 按键
- `STATUS` LED
- 可选 `CHG` LED
- 测试点

### 建议测试点

```text
3V3
GND
SYS_RAW
BAT+
USB_D+
USB_D-
UART_TX_TEST
UART_RX_TEST
SPI_SCK
SPI_MOSI
SPI_MISO
SENSOR_CS
SENSOR_INT1
SENSOR_INT2
I2S_BCLK，可选
I2S_WS，可选
I2S_DATA，可选
```

### 设计注意

- 测试点要放在 PCB 边缘或容易探针接触的位置。
- `BOOT` 和 `EN` 的丝印要清楚。
- 状态 LED 不要放在天线 keep-out 区。

## 7. 06_SENSOR_BMI270_IMU：BMI270 IMU 传感器页

`BMI270` 用作 IMU Slave 的传感器。建议优先使用 SPI，和当前无线采集系统的 IMU 固件思路一致。

### 需要的器件

- `BMI270`
- 电源去耦电容
- SPI 串联 0 ohm 电阻，可选
- INT 串联 0 ohm 电阻，可选
- CS 上拉电阻，可选

### 推荐信号

```text
BMI270_VDD
BMI270_VDDIO
GND
BMI270_SCK
BMI270_MOSI
BMI270_MISO
BMI270_CS
BMI270_INT1
BMI270_INT2，可选
```

### 推荐连接方式

```text
3V3 -> BMI270 VDD
3V3 -> BMI270 VDDIO
GND -> BMI270 GND
SPI_SCK  -> BMI270 SCK/SCL
SPI_MOSI -> BMI270 SDI/SDA
SPI_MISO -> BMI270 SDO
SENSOR_CS -> BMI270 CSB
SENSOR_INT1 -> BMI270 INT1
SENSOR_INT2 -> BMI270 INT2，可选
```

### 设计注意

- `BMI270` 同时支持 SPI/I2C，原理图上要明确按 SPI 模式连接。
- `CSB` 默认电平和上拉/下拉按 BMI270 数据手册确认。
- `VDD` 和 `VDDIO` 每个电源脚附近都要放去耦电容。
- `INT1` 建议用于 data-ready 中断。
- `INT2` 可以预留给未来事件中断或不用。
- 传感器附近不要走 USB、天线、电源开关节点。
- PCB 丝印标出 `X/Y/Z` 方向。

### PCB 位置建议

- BMI270 放在机械固定稳定的位置。
- 如果用于小腿节点，传感器坐标轴应与佩戴方向有明确对应。
- 如果用于锤子节点，传感器尽量靠近锤头或实际敲击结构。
- 传感器下方保持完整地平面，不走大电流线。

## 8. 07_SENSOR_LSM6DSOX_MMG：LSM6DSOX MMG 传感器页

`LSM6DSOX` 本质是 6 轴 IMU。这里将其用于 MMG 节点，实际测量的是肌肉表面低频机械振动，而不是声学麦克风信号。建议优先读取加速度轴数据，用于 MMG/机械振动分析。

### 需要的器件

- `LSM6DSOX`
- 电源去耦电容
- SPI 串联 0 ohm 电阻，可选
- INT 串联 0 ohm 电阻，可选
- CS 上拉电阻，可选

### 推荐信号

```text
LSM6DSOX_VDD
LSM6DSOX_VDDIO
GND
LSM6DSOX_SCK
LSM6DSOX_MOSI
LSM6DSOX_MISO
LSM6DSOX_CS
LSM6DSOX_INT1
LSM6DSOX_INT2，可选
```

### 推荐连接方式

```text
3V3 -> LSM6DSOX VDD
3V3 -> LSM6DSOX VDDIO
GND -> LSM6DSOX GND
SPI_SCK  -> LSM6DSOX SPC/SCL
SPI_MOSI -> LSM6DSOX SDI/SDA
SPI_MISO -> LSM6DSOX SDO/SA0
SENSOR_CS -> LSM6DSOX CS
SENSOR_INT1 -> LSM6DSOX INT1
SENSOR_INT2 -> LSM6DSOX INT2，可选
```

### 设计注意

- 建议 MMG 版也使用 SPI，这样可以和 BMI270 版共用同一组传感器接口网络。
- `INT1` 用于 data-ready 中断。
- `INT2` 预留，后续可用于 FIFO watermark 或运动事件。
- 如果后续希望降低引脚数量，也可以改成 I2C，但第一版不建议混用接口。
- `VDD` 和 `VDDIO` 去耦电容必须靠近芯片电源脚。

### 机械耦合建议

由于 `LSM6DSOX` 用于测 MMG，PCB 机械结构比普通 IMU 更重要。

建议：

- LSM6DSOX 放在贴近肌肉表面的位置。
- 传感器区域下面避免悬空或柔性过强。
- 预留绑带、胶贴、硅胶垫或外壳压紧结构。
- 传感器附近不要放高大的 Type-C、按键或电池座，以免影响贴合。
- 丝印标注贴合面和坐标轴方向。

## 9. 两类传感器共用接口建议

为了让 IMU 版和 MMG 版 PCB 尽量统一，建议两者使用同一组传感器网络：

```text
SENSOR_SCK
SENSOR_MOSI
SENSOR_MISO
SENSOR_CS
SENSOR_INT1
SENSOR_INT2
SENSOR_3V3
SENSOR_GND
```

然后在不同原理图页中分别连接：

```text
IMU 版：
SENSOR_* -> BMI270

MMG 版：
SENSOR_* -> LSM6DSOX
```

这样做的好处：

- 公共底板和传感器区域布线规则一致。
- 固件只需要切换传感器驱动和初始化参数。
- 测试点、逻辑分析仪接线、调试流程可以复用。

## 10. PCB 分区建议

建议 PCB 按以下区域排布：

```text
┌──────────────────────────────┐
│ 天线区 / U.FL / keep-out      │
├──────────────┬───────────────┤
│ ESP32-S3     │ 调试按键/LED   │
├──────────────┼───────────────┤
│ 电源/Type-C  │ 电池接口       │
├──────────────┴───────────────┤
│ 传感器区：BMI270 或 LSM6DSOX  │
└──────────────────────────────┘
```

原则：

- 天线区靠板边。
- 电源区靠 Type-C 和电池座。
- 传感器区远离电源电感、Type-C、天线和 USB 差分线。
- 测试点放在容易接触的位置。

## 11. 上电检查清单

画完原理图和 PCB 后，第一版上电按这个顺序检查。

### 11.1 不焊主控前

- `3V3` 对 `GND` 不短路。
- `BAT+` 对 `GND` 不短路。
- `VBUS_DATA_RAW` 和 `VBUS_CHG_RAW` 不直接短接。
- USB D+/D- 不短路。
- 传感器电源对地不短路。

### 11.2 只测电源

- 电池供电时，`SYS_RAW` 和 `3V3` 正常。
- 插 `USB/DATA` 时，系统可供电。
- 插 `CHARGE ONLY` 时，充电芯片状态正常。
- 两个 Type-C 同时插入时，无异常发热、无倒灌。
- `VBUS_DATA_DET`、`VBUS_CHG_DET`、`CHG_STAT` 电平符合预期。

### 11.3 主控测试

- ESP32-S3 可以进入下载模式。
- USB CDC 串口可识别。
- `BOOT`、`EN` 按键有效。
- `STATUS_LED` 可控制。
- `BAT_ADC` 可以读到合理电池电压。

### 11.4 BMI270 IMU 版测试

- SPI 通信正常。
- 读取 chip ID 正常。
- `INT1` data-ready 中断正常。
- 静止时三轴数据稳定。
- 运动时波形方向与丝印坐标轴一致。

### 11.5 LSM6DSOX MMG 版测试

- SPI 通信正常。
- 读取 WHO_AM_I 正常。
- `INT1` data-ready 中断正常。
- 轻敲或贴在肌肉表面时，加速度波形有明显响应。
- 改变固定方式时，波形变化可观察。

## 12. 第一版建议保留的调试余量

第一版 PCB 建议保留：

- 传感器 SPI 每根线的 0 ohm 电阻位。
- INT1/INT2 的 0 ohm 电阻位。
- 传感器电源的 0 ohm 或磁珠位。
- Type-C VBUS 路径上的电流测量跳线。
- 电池电流测量跳线。
- 充电状态测试点。
- 关键 GPIO 测试点。

第一版目标是验证硬件链路和机械固定方式，不要过早追求极限小型化。等 BMI270 和 LSM6DSOX 两类节点都稳定后，再做第二版尺寸优化。

