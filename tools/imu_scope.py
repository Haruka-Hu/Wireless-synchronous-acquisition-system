#!/usr/bin/env python3
"""
无线采集系统上位机示波器。

面向当前固件架构：
1. SOURCE 0x00: Master 本地 ADS1298 的 CH6/CH7/CH8
2. SOURCE 0x01: slave-01 的陀螺仪 gx/gy/gz
3. SOURCE 0x02: slave-02 的加速度 ax/ay/az
4. SOURCE 0x7E: 诊断帧

串口协议使用当前 CDC 批量帧：
- 帧头: AA 55
- 类型: 0x30
- 单样本结构: <BIIHBiii
"""

from __future__ import annotations

import argparse
import csv
import struct
import sys
import threading
import time
from collections import deque
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path

import numpy as np
import pyqtgraph as pg
import serial
import serial.tools.list_ports
import scipy.signal as signal
from PySide6.QtCore import QTimer
from PySide6.QtWidgets import (
    QApplication,
    QHBoxLayout,
    QLabel,
    QMainWindow,
    QMessageBox,
    QPushButton,
    QTabWidget,
    QVBoxLayout,
    QWidget,
)


FRAME_HEADER = b"\xAA\x55"
PC_MSG_SAMPLE_BATCH = 0x30
PC_BATCH_HEADER_SIZE = 5
PC_SAMPLE_SIZE = 24
PC_MAX_BATCH_SAMPLES = 24
PC_BATCH_MIN_SIZE = PC_BATCH_HEADER_SIZE + PC_SAMPLE_SIZE + 2
PC_SAMPLE_STRUCT = struct.Struct("<BIIHBiii")

SOURCE_EMG = 0x00
SOURCE_GYRO = 0x01
SOURCE_ACCEL = 0x02
SOURCE_DIAG = 0x7E

# ADS1298 PGA=12, 2.4V reference: 1 LSB ~= 0.02384 uV.
EMG_SCALE_FACTOR_UV = 2.4 * 1_000_000.0 / (12.0 * (2**23 - 1))
EMG_MIN_SHARED_Y_RANGE_UV = 100.0
EMG_Y_RANGE_HEADROOM = 1.15

SOURCE_NAMES = {
    SOURCE_EMG: "EMG",
    SOURCE_GYRO: "Gyro",
    SOURCE_ACCEL: "Accel",
    SOURCE_DIAG: "Diag",
}

SOURCE_PLOT_META = {
    SOURCE_EMG: {
        "title": "Master ADS1298 Filtered",
        "channels": ("CH6", "CH7", "CH8"),
        "colors": ("#FF6B6B", "#FFD166", "#4D96FF"),
        "y_label": "uV",
    },
    SOURCE_GYRO: {
        "title": "Slave-01 Gyroscope",
        "channels": ("gx", "gy", "gz"),
        "colors": ("#00C853", "#FF9100", "#AA00FF"),
        "y_label": "Raw",
    },
    SOURCE_ACCEL: {
        "title": "Slave-02 Accelerometer",
        "channels": ("ax", "ay", "az"),
        "colors": ("#26C6DA", "#EC407A", "#9CCC65"),
        "y_label": "Raw",
    },
}

UINT32_WRAP = 2**32
INT32_HALF_WRAP = 2**31
STREAM_MIN_SIZE = PC_BATCH_MIN_SIZE
# CRC 已经证明帧边界有效；时间戳只用于发现断线/重同步后的时间轴跳变。
TIMESTAMP_RELOCK_GAP_US = 250_000
TIMESTAMP_RELOCK_BACKWARD_US = 2_000


def signed_delta_us(timestamp_us: np.ndarray, reference_us: int) -> np.ndarray:
    """计算带 uint32 回绕处理的相对时间差。"""
    raw_delta = (timestamp_us.astype(np.int64) - int(reference_us) + INT32_HALF_WRAP) % UINT32_WRAP
    return raw_delta - INT32_HALF_WRAP


def crc16_ccitt(data: bytes) -> int:
    """计算和固件一致的 CCITT CRC16。"""
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc


def parse_args() -> argparse.Namespace:
    """解析 IMU/EMG 示波器的命令行参数。"""
    parser = argparse.ArgumentParser(description="按当前 CDC 协议显示 EMG / Gyro / Accel 波形。")
    parser.add_argument("--port", help="可选，手动指定串口；若不提供则自动搜索。")
    parser.add_argument("--baud", type=int, default=2_000_000, help="串口波特率，默认 2000000。")
    parser.add_argument("--window-seconds", type=float, default=2.0, help="显示窗口长度，默认 2 秒。")
    parser.add_argument("--expected-rate", type=float, default=2000.0, help="缓冲区按最高通道采样率预估，默认 2000Hz。")
    parser.add_argument("--emg-display-rate", type=float, default=2000.0, help="EMG 显示滤波器采样率，默认 2000Hz。")
    parser.add_argument("--csv-dir", type=Path, default=Path("data"), help="CSV 保存目录，默认 data/。")
    parser.add_argument("--list-ports", action="store_true", help="列出当前所有串口后退出。")
    return parser.parse_args()


def list_ports() -> None:
    """列出当前系统可见的串口设备。"""
    ports = list(serial.tools.list_ports.comports())
    if not ports:
        print("未发现可用串口。")
        return

    print("可用串口：")
    for port in ports:
        print(f"  {port.device}  {port.description}")


def auto_find_port() -> str | None:
    """按常见 ESP32/USB 串口关键字自动选择最可能的端口。"""
    preferred_keywords = ("esp32", "espressif", "usb jtag", "usb serial", "cdc", "tinyusb")
    fallback_keywords = ("usbmodem", "usbserial", "wchusb", "ttyacm", "ttyusb", "com", "cp210", "ch340", "uart")

    ports = list(serial.tools.list_ports.comports())
    if not ports:
        return None

    scored: list[tuple[int, str]] = []
    for port in ports:
        haystack = f"{port.device} {port.description} {port.manufacturer or ''}".lower()
        score = 0
        if any(keyword in haystack for keyword in preferred_keywords):
            score += 10
        if any(keyword in haystack for keyword in fallback_keywords):
            score += 3
        if score > 0:
            scored.append((score, port.device))

    if scored:
        scored.sort(reverse=True)
        return scored[0][1]

    return ports[0].device


@dataclass
class SourceBuffer:
    timestamps_us: deque[int]
    sample_seq: deque[int]
    batch_seq: deque[int]
    rx_flags: deque[int]
    x: deque[int]
    y: deque[int]
    z: deque[int]
    display_x: deque[float]
    display_y: deque[float]
    display_z: deque[float]
    last_host_rx_time: float = 0.0
    received_count: int = 0


@dataclass
class ParserStats:
    crc_fails: int = 0
    bad_headers: int = 0
    bad_sources: int = 0
    bad_counts: int = 0
    bad_timestamps: int = 0
    resync_bytes: int = 0
    reader_errors: int = 0
    last_reader_error: str = ""


class AcquisitionModel:
    def __init__(self, max_points: int, emg_display_rate: float) -> None:
        """初始化采集缓存、EMG 显示滤波器和 CSV/诊断状态。"""
        # GUI 线程和串口读取线程共享这些缓存，所有读写都必须经过 self.lock。
        self.lock = threading.Lock()
        self.sources = {
            source_id: SourceBuffer(
                timestamps_us=deque(maxlen=max_points),
                sample_seq=deque(maxlen=max_points),
                batch_seq=deque(maxlen=max_points),
                rx_flags=deque(maxlen=max_points),
                x=deque(maxlen=max_points),
                y=deque(maxlen=max_points),
                z=deque(maxlen=max_points),
                display_x=deque(maxlen=max_points),
                display_y=deque(maxlen=max_points),
                display_z=deque(maxlen=max_points),
            )
            for source_id in (SOURCE_EMG, SOURCE_GYRO, SOURCE_ACCEL)
        }
        notch_b, notch_a = signal.iirnotch(50.0, 30.0, emg_display_rate)
        notch_sos = signal.tf2sos(notch_b, notch_a)
        bandpass_sos = signal.butter(4, [20.0, 500.0], btype="bandpass", fs=emg_display_rate, output="sos")
        self.emg_sos = np.vstack((notch_sos, bandpass_sos))
        self.emg_filter_state = {
            axis: signal.sosfilt_zi(self.emg_sos) * 0.0
            for axis in ("x", "y", "z")
        }
        self.emg_filter_ready = False
        self.is_recording = False
        self.csv_fp = None
        self.csv_writer = None
        self.diag_forwarded_per_sec = 0
        self.diag_errors_per_sec = 0
        self.diag_online_slave_count = 0
        self.parser_stats = ParserStats()

    def clear_source(self, source_id: int) -> None:
        """清空某一路 source 的显示缓存。"""
        with self.lock:
            source = self.sources.get(source_id)
            if source is None:
                return
            source.timestamps_us.clear()
            source.sample_seq.clear()
            source.batch_seq.clear()
            source.rx_flags.clear()
            source.x.clear()
            source.y.clear()
            source.z.clear()
            source.display_x.clear()
            source.display_y.clear()
            source.display_z.clear()
            if source_id == SOURCE_EMG:
                self._reset_emg_filter_locked()

    def _reset_emg_filter_locked(self, initial_uv: tuple[float, float, float] | None = None) -> None:
        """重置 EMG IIR 滤波器状态；调用者必须已经持有 lock。"""
        if initial_uv is None:
            initial_uv = (0.0, 0.0, 0.0)
            self.emg_filter_ready = False
        for axis, value in zip(("x", "y", "z"), initial_uv):
            self.emg_filter_state[axis] = signal.sosfilt_zi(self.emg_sos) * value

    def _filter_emg_locked(self, x: int, y: int, z: int) -> tuple[float, float, float]:
        """将 ADS1298 原始值转换为 uV 并做显示用滤波；调用者必须已经持有 lock。"""
        # EMG 只在显示端滤波，CSV 仍保存原始值，后续分析可以重新选择滤波参数。
        values = (
            float(x) * EMG_SCALE_FACTOR_UV,
            float(y) * EMG_SCALE_FACTOR_UV,
            float(z) * EMG_SCALE_FACTOR_UV,
        )
        if not self.emg_filter_ready:
            self._reset_emg_filter_locked(values)
            self.emg_filter_ready = True

        filtered: list[float] = []
        for axis, value in zip(("x", "y", "z"), values):
            out, zi = signal.sosfilt(self.emg_sos, np.array([value], dtype=np.float64), zi=self.emg_filter_state[axis])
            self.emg_filter_state[axis] = zi
            filtered.append(float(out[0]))
        return filtered[0], filtered[1], filtered[2]

    def append_sample(
        self,
        source_id: int,
        timestamp_us: int,
        sample_seq: int,
        batch_seq: int,
        rx_flags: int,
        x: int,
        y: int,
        z: int,
    ) -> None:
        """追加一条来自串口的样本到缓存，并在录制时写入 CSV。"""
        with self.lock:
            source = self.sources.get(source_id)
            if source is None:
                return

            if source_id == SOURCE_EMG:
                display_x, display_y, display_z = self._filter_emg_locked(x, y, z)
            else:
                display_x, display_y, display_z = float(x), float(y), float(z)

            source.timestamps_us.append(timestamp_us)
            source.sample_seq.append(sample_seq)
            source.batch_seq.append(batch_seq)
            source.rx_flags.append(rx_flags)
            source.x.append(x)
            source.y.append(y)
            source.z.append(z)
            source.display_x.append(display_x)
            source.display_y.append(display_y)
            source.display_z.append(display_z)
            source.last_host_rx_time = time.time()
            source.received_count += 1

            if self.is_recording and self.csv_writer is not None:
                # sample_seq / batch_seq / rx_flags 是可靠链路验收的关键字段，必须写入 CSV。
                vector_norm = float(np.sqrt(float(x) * float(x) + float(y) * float(y) + float(z) * float(z)))
                self.csv_writer.writerow(
                    [
                        f"{time.time():.6f}",
                        SOURCE_NAMES.get(source_id, f"Unknown({source_id})"),
                        source_id,
                        timestamp_us,
                        sample_seq,
                        batch_seq,
                        rx_flags,
                        x,
                        y,
                        z,
                        f"{vector_norm:.3f}",
                    ]
                )

    def update_diag(self, forwarded_per_sec: int, errors_per_sec: int, online_slave_count: int) -> None:
        """更新 Master 发来的每秒链路诊断信息。"""
        with self.lock:
            self.diag_forwarded_per_sec = forwarded_per_sec
            self.diag_errors_per_sec = errors_per_sec
            self.diag_online_slave_count = online_slave_count

    def reference_timestamp_us(self) -> int | None:
        """选择最近收到数据的 source 作为显示时间轴参考。"""
        with self.lock:
            latest_timestamp: int | None = None
            latest_host_rx_time = -1.0
            for source_id in (SOURCE_EMG, SOURCE_GYRO, SOURCE_ACCEL):
                source = self.sources[source_id]
                if source.timestamps_us and source.last_host_rx_time >= latest_host_rx_time:
                    latest_host_rx_time = source.last_host_rx_time
                    latest_timestamp = int(source.timestamps_us[-1])
            return latest_timestamp

    def snapshot(self, source_id: int, reference_timestamp_us: int | None) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, int]:
        """复制某一路 source 的绘图数据快照，供 Qt 定时器线程使用。"""
        with self.lock:
            source = self.sources[source_id]
            if not source.timestamps_us or reference_timestamp_us is None:
                empty = np.array([], dtype=np.float64)
                return empty, empty, empty, empty, source.received_count

            timestamps = np.fromiter(source.timestamps_us, dtype=np.int64)
            x = np.fromiter(source.display_x, dtype=np.float64)
            y = np.fromiter(source.display_y, dtype=np.float64)
            z = np.fromiter(source.display_z, dtype=np.float64)
            rel_time_s = signed_delta_us(timestamps, reference_timestamp_us).astype(np.float64) / 1_000_000.0
            if rel_time_s.size > 1:
                # IMU 可能因补发而乱序到达，显示时按 timestamp 排序，不按串口到达顺序画线。
                order = np.argsort(rel_time_s)
                rel_time_s = rel_time_s[order]
                x = x[order]
                y = y[order]
                z = z[order]
            return rel_time_s, x, y, z, source.received_count

    def source_status_snapshot(self) -> dict[int, tuple[int, float]]:
        """返回各 source 的接收计数和最近主机接收时间。"""
        with self.lock:
            return {
                source_id: (source.received_count, source.last_host_rx_time)
                for source_id, source in self.sources.items()
            }

    def diag_snapshot(self) -> tuple[int, int, int, ParserStats]:
        """返回诊断统计快照，避免 UI 直接读取共享状态。"""
        with self.lock:
            return (
                self.diag_forwarded_per_sec,
                self.diag_errors_per_sec,
                self.diag_online_slave_count,
                ParserStats(
                    crc_fails=self.parser_stats.crc_fails,
                    bad_headers=self.parser_stats.bad_headers,
                    bad_sources=self.parser_stats.bad_sources,
                    bad_counts=self.parser_stats.bad_counts,
                    bad_timestamps=self.parser_stats.bad_timestamps,
                    resync_bytes=self.parser_stats.resync_bytes,
                    reader_errors=self.parser_stats.reader_errors,
                    last_reader_error=self.parser_stats.last_reader_error,
                ),
            )

    def note_crc_fail(self) -> None:
        """记录一帧串口数据 CRC 校验失败。"""
        with self.lock:
            self.parser_stats.crc_fails += 1

    def note_bad_header(self) -> None:
        """记录串口帧类型或帧头异常。"""
        with self.lock:
            self.parser_stats.bad_headers += 1

    def note_bad_source(self) -> None:
        """记录样本 source 字段异常或解包失败。"""
        with self.lock:
            self.parser_stats.bad_sources += 1

    def note_bad_count(self) -> None:
        """记录 batch 中样本数量字段异常。"""
        with self.lock:
            self.parser_stats.bad_counts += 1

    def note_bad_timestamp(self) -> None:
        """记录时间戳顺序异常。"""
        with self.lock:
            self.parser_stats.bad_timestamps += 1

    def note_resync_bytes(self, count: int) -> None:
        """记录为了重新对齐帧头而丢弃的字节数。"""
        if count <= 0:
            return
        with self.lock:
            self.parser_stats.resync_bytes += count

    def note_reader_error(self, exc: Exception) -> None:
        """记录串口读取线程捕获到的非致命异常。"""
        with self.lock:
            self.parser_stats.reader_errors += 1
            self.parser_stats.last_reader_error = f"{type(exc).__name__}: {exc}"

    def start_recording(self, csv_dir: Path) -> Path:
        """开始 CSV 录制并写入表头，返回新建文件路径。"""
        with self.lock:
            if self.is_recording:
                raise RuntimeError("CSV 已经处于录制状态。")

            csv_dir.mkdir(parents=True, exist_ok=True)
            filename = f"imu_capture_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
            csv_path = csv_dir / filename
            self.csv_fp = csv_path.open("w", newline="", encoding="utf-8")
            self.csv_writer = csv.writer(self.csv_fp)
            self.csv_writer.writerow(
                [
                    "host_rx_time",
                    "source",
                    "source_id",
                    "timestamp_us",
                    "sample_seq",
                    "batch_seq",
                    "rx_flags",
                    "x",
                    "y",
                    "z",
                    "vector_norm",
                ]
            )
            self.is_recording = True
            return csv_path

    def stop_recording(self) -> None:
        """停止 CSV 录制并关闭文件句柄。"""
        with self.lock:
            self.is_recording = False
            if self.csv_fp is not None:
                self.csv_fp.close()
            self.csv_fp = None
            self.csv_writer = None


class SerialReader(threading.Thread):
    def __init__(self, port: str, baud: int, model: AcquisitionModel) -> None:
        """创建串口读取线程并保存解析状态。"""
        super().__init__(daemon=True)
        self.port = port
        self.baud = baud
        self.model = model
        self.stop_event = threading.Event()
        self.serial_port: serial.Serial | None = None
        self.last_timestamp_us: dict[int, int] = {}

    def open(self) -> None:
        """打开串口并设置 DTR/RTS，准备接收 Master CDC 数据。"""
        self.serial_port = serial.Serial(self.port, self.baud, timeout=0.05)
        self.serial_port.dtr = True
        self.serial_port.rts = True

    def close(self) -> None:
        """关闭串口资源。"""
        if self.serial_port is not None and self.serial_port.is_open:
            self.serial_port.close()

    def stop(self) -> None:
        """请求串口读取线程退出。"""
        self.stop_event.set()

    def note_timestamp(self, source_id: int, timestamp_us: int) -> None:
        """检查到达顺序中的时间戳异常，并在必要时触发 EMG 重锁。"""
        if source_id == SOURCE_DIAG:
            return

        if source_id not in (SOURCE_EMG, SOURCE_GYRO, SOURCE_ACCEL):
            return

        # IMU batches can be retransmitted and forwarded after recovery. The display sorts
        # by timestamp, so only EMG uses strict arrival-order relock checks here.
        if source_id != SOURCE_EMG:
            return

        last_timestamp = self.last_timestamp_us.get(source_id)
        if last_timestamp is None:
            self.last_timestamp_us[source_id] = timestamp_us
            return

        delta_us = (int(timestamp_us) - int(last_timestamp) + INT32_HALF_WRAP) % UINT32_WRAP - INT32_HALF_WRAP
        if source_id == SOURCE_EMG and (
            delta_us < -TIMESTAMP_RELOCK_BACKWARD_US or delta_us > TIMESTAMP_RELOCK_GAP_US
        ):
            self.model.clear_source(source_id)
        elif delta_us <= 0:
            self.model.note_bad_timestamp()

        self.last_timestamp_us[source_id] = timestamp_us

    def run(self) -> None:
        """持续读取串口字节流，按当前 CDC batch 协议解析样本。"""
        if self.serial_port is None:
            raise RuntimeError("串口尚未打开。")

        # 使用流式状态机解析 CDC 数据：找帧头、检查长度、校验 CRC，再解样本。
        # 一旦遇到坏帧，丢 1 字节重新找帧头，尽量不中断长时间采集。
        raw_buffer = bytearray()
        while not self.stop_event.is_set():
            try:
                chunk = self.serial_port.read(4096)
                if chunk:
                    raw_buffer += chunk
                else:
                    time.sleep(0.001)

                while len(raw_buffer) >= STREAM_MIN_SIZE:
                    idx = raw_buffer.find(FRAME_HEADER)
                    if idx == -1:
                        self.model.note_resync_bytes(max(0, len(raw_buffer) - 1))
                        raw_buffer = raw_buffer[-1:]
                        break

                    if idx > 0:
                        self.model.note_resync_bytes(idx)
                        raw_buffer = raw_buffer[idx:]

                    if len(raw_buffer) < PC_BATCH_HEADER_SIZE:
                        break

                    if raw_buffer[2] != PC_MSG_SAMPLE_BATCH:
                        self.model.note_bad_header()
                        raw_buffer = raw_buffer[1:]
                        continue

                    sample_count = raw_buffer[3]
                    if sample_count == 0 or sample_count > PC_MAX_BATCH_SAMPLES:
                        self.model.note_bad_count()
                        raw_buffer = raw_buffer[1:]
                        continue

                    packet_size = PC_BATCH_HEADER_SIZE + sample_count * PC_SAMPLE_SIZE + 2
                    if len(raw_buffer) < packet_size:
                        break

                    frame = raw_buffer[:packet_size]
                    received_crc = int.from_bytes(frame[-2:], byteorder="little", signed=False)
                    calculated_crc = crc16_ccitt(frame[:-2])
                    if received_crc != calculated_crc:
                        self.model.note_crc_fail()
                        raw_buffer = raw_buffer[1:]
                        continue

                    payload_offset = PC_BATCH_HEADER_SIZE
                    for _ in range(sample_count):
                        # 固件 PcSample 的二进制布局是 <BIIHBiii>，修改协议时这里必须同步改。
                        sample_bytes = frame[payload_offset : payload_offset + PC_SAMPLE_SIZE]
                        payload_offset += PC_SAMPLE_SIZE
                        try:
                            source_id, timestamp_us, sample_seq, batch_seq, rx_flags, x, y, z = PC_SAMPLE_STRUCT.unpack(sample_bytes)
                        except struct.error:
                            self.model.note_bad_source()
                            continue

                        if source_id not in (SOURCE_EMG, SOURCE_GYRO, SOURCE_ACCEL, SOURCE_DIAG):
                            self.model.note_bad_source()
                            continue

                        if source_id == SOURCE_DIAG:
                            self.model.update_diag(x, y, z)
                            continue

                        self.note_timestamp(source_id, timestamp_us)
                        self.model.append_sample(source_id, timestamp_us, sample_seq, batch_seq, rx_flags, x, y, z)

                    raw_buffer = raw_buffer[packet_size:]
            except serial.SerialException:
                break
            except Exception as exc:
                self.model.note_reader_error(exc)
                continue


class MainWindow(QMainWindow):
    def __init__(self, model: AcquisitionModel, csv_dir: Path, port: str, baud: int, window_seconds: float) -> None:
        """创建示波器主窗口、曲线、状态栏和定时刷新器。"""
        super().__init__()
        self.model = model
        self.csv_dir = csv_dir
        self.port = port
        self.baud = baud
        self.window_seconds = window_seconds
        self.last_counts = {
            SOURCE_EMG: -1,
            SOURCE_GYRO: -1,
            SOURCE_ACCEL: -1,
        }
        self.source_rates = {
            SOURCE_EMG: 0.0,
            SOURCE_GYRO: 0.0,
            SOURCE_ACCEL: 0.0,
        }
        self.rate_last_counts = {
            SOURCE_EMG: 0,
            SOURCE_GYRO: 0,
            SOURCE_ACCEL: 0,
        }
        self.rate_last_time = time.time()
        self.curves: dict[int, tuple] = {}
        self.emg_plots: list[pg.PlotItem] = []

        self.setWindowTitle("Wireless Capture Scope - EMG Bandpass Mode")
        self.resize(1440, 1000) # 调高一点窗口高度容纳更多图表

        central = QWidget()
        self.setCentralWidget(central)
        main_layout = QVBoxLayout(central)

        top_row = QHBoxLayout()
        self.record_button = QPushButton("开始录制 CSV")
        self.record_button.setStyleSheet(
            "background-color: #2E7D32; color: white; font-weight: bold; padding: 8px 14px;"
        )
        self.record_button.clicked.connect(self.toggle_recording)
        top_row.addWidget(self.record_button)
        status_column = QVBoxLayout()
        self.primary_status_label = QLabel(f"串口 {self.port} @ {self.baud}")
        self.link_status_label = QLabel("Slave-01 -- | Slave-02 --")
        status_column.addWidget(self.primary_status_label)
        status_column.addWidget(self.link_status_label)
        top_row.addLayout(status_column)
        top_row.addStretch()
        main_layout.addLayout(top_row)

        self.tabs = QTabWidget()
        self.emg_graph = pg.GraphicsLayoutWidget()
        self.imu_graph = pg.GraphicsLayoutWidget()
        self.tabs.addTab(self.emg_graph, "EMG")
        self.tabs.addTab(self.imu_graph, "IMU")
        main_layout.addWidget(self.tabs)

        emg_meta = SOURCE_PLOT_META[SOURCE_EMG]
        emg_curves = []
        first_emg_plot = None
        for label, color in zip(emg_meta["channels"], emg_meta["colors"]):
            plot = self.emg_graph.addPlot(title=f"EMG {label} (20-500Hz Bandpass, 50Hz Notch)")
            self._setup_plot(plot, emg_meta["y_label"])
            self.emg_plots.append(plot)

            if first_emg_plot is None:
                first_emg_plot = plot
            else:
                plot.setXLink(first_emg_plot)

            curve = plot.plot(pen=pg.mkPen(color, width=1.6), name=label)
            emg_curves.append(curve)
            self.emg_graph.nextRow()
        self.curves[SOURCE_EMG] = tuple(emg_curves)

        first_imu_plot = None
        for source_id in (SOURCE_GYRO, SOURCE_ACCEL):
            meta = SOURCE_PLOT_META[source_id]
            plot = self.imu_graph.addPlot(title=meta["title"])
            self._setup_plot(plot, meta["y_label"])
            if first_imu_plot is None:
                first_imu_plot = plot
            else:
                plot.setXLink(first_imu_plot)
            plot.addLegend(offset=(12, 12))
            curves = []
            for label, color in zip(meta["channels"], meta["colors"]):
                curve = plot.plot(pen=pg.mkPen(color, width=1.6), name=label)
                curves.append(curve)
            self.curves[source_id] = tuple(curves)
            self.imu_graph.nextRow()

        self.timer = QTimer(self)
        self.timer.timeout.connect(self.update_plots)
        self.timer.start(40)

    def _setup_plot(self, plot: pg.PlotItem, y_label: str) -> None:
        """统一配置单个 pyqtgraph 图的网格、坐标轴和降采样。"""
        plot.showGrid(x=True, y=True, alpha=0.25)
        plot.setLabel("left", y_label)
        plot.setLabel("bottom", "Time", units="s")
        plot.setXRange(-self.window_seconds, 0.0, padding=0.0)
        plot.setClipToView(True)
        plot.setDownsampling(mode="peak")

    def _sync_emg_y_range(self, filtered_channels: tuple[np.ndarray, np.ndarray, np.ndarray]) -> None:
        """根据三个 EMG 通道的幅值同步设置共享 Y 轴范围。"""
        max_abs = 0.0
        for values in filtered_channels:
            if values.size:
                channel_max = float(np.nanmax(np.abs(values)))
                if np.isfinite(channel_max):
                    max_abs = max(max_abs, channel_max)

        half_range = max(EMG_MIN_SHARED_Y_RANGE_UV, max_abs * EMG_Y_RANGE_HEADROOM)
        for plot in self.emg_plots:
            plot.setYRange(-half_range, half_range, padding=0.0)

    def _format_emg_quality(self, filtered_channels: tuple[np.ndarray, np.ndarray, np.ndarray]) -> str:
        """生成 EMG RMS 和峰值的状态栏文本。"""
        parts = []
        for label, values in zip(SOURCE_PLOT_META[SOURCE_EMG]["channels"], filtered_channels):
            finite_values = values[np.isfinite(values)]
            if finite_values.size == 0:
                parts.append(f"{label} --")
                continue
            rms = float(np.sqrt(np.mean(np.square(finite_values))))
            peak = float(np.nanmax(np.abs(finite_values)))
            parts.append(f"{label} rms {rms:.1f}uV pk {peak:.1f}uV")
        return " | ".join(parts)

    def _smooth(self, values: np.ndarray) -> np.ndarray:
        """对 IMU 显示曲线做简单滑动平均平滑。"""
        if values.size < 5:
            return values
        kernel = np.ones(5, dtype=np.float64) / 5.0
        return np.convolve(values, kernel, mode="same")

    def _update_source_rates(self, counts: dict[int, int], now: float) -> None:
        """每秒更新一次各 source 的接收速率。"""
        elapsed = now - self.rate_last_time
        if elapsed < 1.0:
            return
        for source_id, count in counts.items():
            previous_count = self.rate_last_counts.get(source_id, count)
            self.source_rates[source_id] = max(0.0, float(count - previous_count) / elapsed)
            self.rate_last_counts[source_id] = count
        self.rate_last_time = now

    def _format_link_status(self, name: str, source_id: int, last_rx_time: float, now: float) -> str:
        """按最近接收时间和速率生成某个 Slave 的在线状态文本。"""
        is_online = last_rx_time > 0.0 and (now - last_rx_time) <= 1.5
        state = "在线" if is_online else "离线"
        return f"{name} {state} {self.source_rates[source_id]:.0f}/s"

    def update_plots(self) -> None:
        """Qt 定时器回调：刷新曲线、速率和诊断状态栏。"""
        reference_timestamp_us = self.model.reference_timestamp_us()
        current_counts: dict[int, int] = {}

        for source_id in (SOURCE_EMG, SOURCE_GYRO, SOURCE_ACCEL):
            t, x, y, z, count = self.model.snapshot(source_id, reference_timestamp_us)
            current_counts[source_id] = count
            if count == self.last_counts[source_id]:
                continue
            curves = self.curves[source_id]
            
            # 针对不同来源调用不同的信号处理函数
            if source_id == SOURCE_EMG:
                filtered_emg = (x, y, z)
                curves[0].setData(t, filtered_emg[0])
                curves[1].setData(t, filtered_emg[1])
                curves[2].setData(t, filtered_emg[2])
                self._sync_emg_y_range(filtered_emg)
                self.emg_quality_text = self._format_emg_quality(filtered_emg)
            else:
                curves[0].setData(t, self._smooth(x))
                curves[1].setData(t, self._smooth(y))
                curves[2].setData(t, self._smooth(z))
                
            self.last_counts[source_id] = count

        now = time.time()
        status_snapshot = self.model.source_status_snapshot()
        for source_id, (count, _) in status_snapshot.items():
            current_counts[source_id] = count
        self._update_source_rates(current_counts, now)

        diag_forwarded, diag_errors, diag_online, parser_stats = self.model.diag_snapshot()
        emg_count = self.last_counts[SOURCE_EMG] if self.last_counts[SOURCE_EMG] >= 0 else 0
        gyro_count = self.last_counts[SOURCE_GYRO] if self.last_counts[SOURCE_GYRO] >= 0 else 0
        accel_count = self.last_counts[SOURCE_ACCEL] if self.last_counts[SOURCE_ACCEL] >= 0 else 0
        record_state = "录制中" if self.model.is_recording else "未录制"
        reader_error_text = (
            f" | reader err {parser_stats.reader_errors} {parser_stats.last_reader_error}"
            if parser_stats.reader_errors
            else ""
        )
        slave1_last_rx = status_snapshot.get(SOURCE_GYRO, (0, 0.0))[1]
        slave2_last_rx = status_snapshot.get(SOURCE_ACCEL, (0, 0.0))[1]
        slave1_text = self._format_link_status("Slave-01 Gyro", SOURCE_GYRO, slave1_last_rx, now)
        slave2_text = self._format_link_status("Slave-02 Accel", SOURCE_ACCEL, slave2_last_rx, now)
        self.primary_status_label.setText(
            f"串口 {self.port} @ {self.baud} | "
            f"EMG {emg_count} 点 {self.source_rates[SOURCE_EMG]:.0f}/s | "
            f"{getattr(self, 'emg_quality_text', 'CH6 -- | CH7 -- | CH8 --')} | "
            f"{record_state}"
        )
        self.link_status_label.setText(
            f"{slave1_text} | {slave2_text} | "
            f"Gyro {gyro_count} 点 | Accel {accel_count} 点 | "
            f"slave fwd {diag_forwarded}/s | err {diag_errors}/s | master online {diag_online} | "
            f"crc {parser_stats.crc_fails} | bad hdr {parser_stats.bad_headers} | "
            f"bad src {parser_stats.bad_sources} | bad cnt {parser_stats.bad_counts} | "
            f"ts rej {parser_stats.bad_timestamps} | resync {parser_stats.resync_bytes}B"
            f"{reader_error_text}"
        )

    def toggle_recording(self) -> None:
        """切换 CSV 录制状态，并同步按钮样式。"""
        if not self.model.is_recording:
            try:
                csv_path = self.model.start_recording(self.csv_dir)
            except Exception as exc:
                QMessageBox.critical(self, "录制失败", str(exc))
                return
            self.record_button.setText("停止录制 CSV")
            self.record_button.setStyleSheet(
                "background-color: #C62828; color: white; font-weight: bold; padding: 8px 14px;"
            )
            self.primary_status_label.setText(f"正在录制: {csv_path}")
        else:
            self.model.stop_recording()
            self.record_button.setText("开始录制 CSV")
            self.record_button.setStyleSheet(
                "background-color: #2E7D32; color: white; font-weight: bold; padding: 8px 14px;"
            )


def main() -> int:
    """启动串口读取线程和 Qt 示波器主窗口。"""
    args = parse_args()

    if args.list_ports:
        list_ports()
        return 0

    port = args.port or auto_find_port()
    if port is None:
        print("未找到可用串口，请检查 ESP32-S3 是否已连接并开启 USB CDC。", file=sys.stderr)
        return 2

    max_points = max(100, int(args.window_seconds * args.expected_rate))
    model = AcquisitionModel(max_points=max_points, emg_display_rate=args.emg_display_rate)
    reader = SerialReader(port=port, baud=args.baud, model=model)

    try:
        reader.open()
    except Exception as exc:
        print(f"串口打开失败: {exc}", file=sys.stderr)
        return 1

    reader.start()

    app = QApplication(sys.argv)
    pg.setConfigOptions(antialias=False, foreground="#EAEAEA", background="#101418")

    window = MainWindow(model, args.csv_dir, port, args.baud, args.window_seconds)
    window.show()

    exit_code = 0
    try:
        exit_code = app.exec()
    finally:
        reader.stop()
        reader.join(timeout=1.0)
        reader.close()
        model.stop_recording()

    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
