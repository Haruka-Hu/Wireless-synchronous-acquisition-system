# Wireless Synchronous Acquisition System

ESP32-S3 wireless synchronous acquisition system for knee-jerk reflex experiments.

The project contains four PlatformIO firmware targets and Python host tools:

- `master`: ADS1298 EMG acquisition, ESP-NOW beacon/ACK, USB CDC output.
- `slave-01`: MPU6500 gyroscope node on the shank.
- `slave-02`: MPU6500 accelerometer node on the hammer head.
- `motor`: BLE-controlled BTS7960 motor node for five-level tapping therapy.
- `tools/imu_scope.py`: PySide6/pyqtgraph host oscilloscope and CSV recorder.
- `tools/motor_controller.py`: BLE motor control GUI.

## Quick Start

Install dependencies with Pixi, then build firmware:

```bash
pixi run build-master
pixi run build-slave-01
pixi run build-slave-02
pixi run build-motor
```

Run host tools:

```bash
pixi run host
pixi run motor-host
```

Upload firmware:

```bash
pixi run upload-master
pixi run upload-slave-01
pixi run upload-slave-02
pixi run upload-motor
```

## Documentation

Start from [notes/文档索引.md](notes/文档索引.md). The main architecture and protocol documents are:

- [notes/系统架构与实现指南.md](notes/系统架构与实现指南.md)
- [notes/代码运行顺序.md](notes/代码运行顺序.md)
- [notes/通信协议说明.md](notes/通信协议说明.md)
- [notes/烧录与运行指令.md](notes/烧录与运行指令.md)
- [notes/调试复盘.md](notes/调试复盘.md)

