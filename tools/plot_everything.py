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
import html
import queue
import subprocess
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
from PySide6.QtCore import QTimer, Qt
from PySide6.QtWidgets import (
    QApplication,
    QHBoxLayout,
    QLabel,
    QMainWindow,
    QMessageBox,
    QPushButton,
    QComboBox,
    QTabWidget,
    QVBoxLayout,
    QWidget,
    QGroupBox,
)


FRAME_HEADER = b"\xAA\x55"
PC_MSG_SAMPLE_BATCH = 0x30
PC_MSG_SYNC_DIAG = 0x31
PC_MSG_STATE_EVENT = 0x32
PC_BATCH_HEADER_SIZE = 5
PC_SAMPLE_SIZE = 24
PC_MAX_BATCH_SAMPLES = 16
PC_BATCH_MIN_SIZE = PC_BATCH_HEADER_SIZE + PC_SAMPLE_SIZE + 2
PC_SAMPLE_STRUCT = struct.Struct("<BIIHBiii")
PC_SYNC_DIAG_SIZE = 24
PC_SYNC_DIAG_STRUCT = struct.Struct("<BBHiiiH")
PC_STATE_EVENT_SIZE = 14
PC_STATE_EVENT_STRUCT = struct.Struct("<BBHI")

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
        "colors": ("#FF5252", "#FFD740", "#448AFF"),
        "y_label": "uV",
    },
    SOURCE_GYRO: {
        "title": "Slave-01 Gyroscope",
        "channels": ("gx", "gy", "gz"),
        "colors": ("#69F0AE", "#FFAB40", "#E040FB"),
        "y_label": "Raw",
    },
    SOURCE_ACCEL: {
        "title": "Slave-02 Accelerometer",
        "channels": ("ax", "ay", "az"),
        "colors": ("#18FFFF", "#FF4081", "#B2FF59"),
        "y_label": "Raw",
    },
}

UINT32_WRAP = 2**32
INT32_HALF_WRAP = 2**31
STREAM_MIN_SIZE = PC_BATCH_MIN_SIZE
MIN_FRAME_SIZE = 7
RAW_QUEUE_MAX_CHUNKS = 256
SERIAL_READ_SIZE = 8192
MAX_RESEND_REQUESTS_PER_GAP = 64
# CRC 已经证明帧边界有效；时间戳只用于发现断线/重同步后的时间轴跳变。
TIMESTAMP_RELOCK_GAP_US = 250_000
TIMESTAMP_RELOCK_BACKWARD_US = 2_000
SYNC_STABLE_WINDOW_S = 3.0
SYNC_MIN_BEACON_COUNT = 30
SYNC_USABLE_RESIDUAL_US = 3_000
SYNC_STABLE_RESIDUAL_US = 1_000
SYNC_EXCELLENT_RESIDUAL_US = 500
SYNC_STABLE_DRIFT_RANGE_PPM = 100
SYNC_EXCELLENT_DRIFT_RANGE_PPM = 50

STATE_NAMES = {
    0: "IDLE",
    1: "SYNC",
    2: "STREAM_PENDING",
    3: "STREAM",
}


def signed_delta_us(timestamp_us: np.ndarray, reference_us: int) -> np.ndarray:
    """计算带 uint32 回绕处理的相对时间差。"""
    raw_delta = (timestamp_us.astype(np.int64) - int(reference_us) + INT32_HALF_WRAP) % UINT32_WRAP
    return raw_delta - INT32_HALF_WRAP


# ✅ 1. 预计算好的 CRC16-CCITT 查表字典（放在全局，只占极小内存）
_CRC16_TABLE = [
    0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50a5, 0x60c6, 0x70e7,
    0x8108, 0x9129, 0xa14a, 0xb16b, 0xc18c, 0xd1ad, 0xe1ce, 0xf1ef,
    0x1231, 0x0210, 0x3273, 0x2252, 0x52b5, 0x4294, 0x72f7, 0x62d6,
    0x9339, 0x8318, 0xb37b, 0xa35a, 0xd3bd, 0xc39c, 0xf3ff, 0xe3de,
    0x2462, 0x3443, 0x0420, 0x1401, 0x64e6, 0x74c7, 0x44a4, 0x5485,
    0xa56a, 0xb54b, 0x8528, 0x9509, 0xe5ee, 0xf5cf, 0xc5ac, 0xd58d,
    0x3653, 0x2672, 0x1611, 0x0630, 0x76d7, 0x66f6, 0x5695, 0x46b4,
    0xb75b, 0xa77a, 0x9719, 0x8738, 0xf7df, 0xe7fe, 0xd79d, 0xc7bc,
    0x48c4, 0x58e5, 0x6886, 0x78a7, 0x0840, 0x1861, 0x2802, 0x3823,
    0xc9cc, 0xd9ed, 0xe98e, 0xf9af, 0x8948, 0x9969, 0xa90a, 0xb92b,
    0x5af5, 0x4ad4, 0x7ab7, 0x6a96, 0x1a71, 0x0a50, 0x3a33, 0x2a12,
    0xdbfd, 0xcbdc, 0xfbbf, 0xeb9e, 0x9b79, 0x8b58, 0xbb3b, 0xab1a,
    0x6ca6, 0x7c87, 0x4ce4, 0x5cc5, 0x2c22, 0x3c03, 0x0c60, 0x1c41,
    0xedae, 0xfd8f, 0xcdec, 0xddcd, 0xad2a, 0xbd0b, 0x8d68, 0x9d49,
    0x7e97, 0x6eb6, 0x5ed5, 0x4ef4, 0x3e13, 0x2e32, 0x1e51, 0x0e70,
    0xff9f, 0xefbe, 0xdfdd, 0xcffc, 0xbf1b, 0xaf3a, 0x9f59, 0x8f78,
    0x9188, 0x81a9, 0xb1ca, 0xa1eb, 0xd10c, 0xc12d, 0xf14e, 0xe16f,
    0x1080, 0x00a1, 0x30c2, 0x20e3, 0x5004, 0x4025, 0x7046, 0x6067,
    0x83b9, 0x9398, 0xa3fb, 0xb3da, 0xc33d, 0xd31c, 0xe37f, 0xf35e,
    0x02b1, 0x1290, 0x22f3, 0x32d2, 0x4235, 0x5214, 0x6277, 0x7256,
    0xb5ea, 0xa5cb, 0x95a8, 0x8589, 0xf56e, 0xe54f, 0xd52c, 0xc50d,
    0x34e2, 0x24c3, 0x14a0, 0x0481, 0x7466, 0x6447, 0x5424, 0x4405,
    0xa7db, 0xb7fa, 0x8799, 0x97b8, 0xe75f, 0xf77e, 0xc71d, 0xd73c,
    0x26d3, 0x36f2, 0x0691, 0x16b0, 0x6657, 0x7676, 0x4615, 0x5634,
    0xd94c, 0xc96d, 0xf90e, 0xe92f, 0x99c8, 0x89e9, 0xb98a, 0xa9ab,
    0x5844, 0x4865, 0x7806, 0x6827, 0x18c0, 0x08e1, 0x3882, 0x28a3,
    0xcb7d, 0xdb5c, 0xeb3f, 0xfb1e, 0x8bf9, 0x9bd8, 0xabbb, 0xbb9a,
    0x4a75, 0x5a54, 0x6a37, 0x7a16, 0x0af1, 0x1ad0, 0x2ab3, 0x3a92,
    0xfd2e, 0xed0f, 0xdd6c, 0xcd4d, 0xbdaa, 0xad8b, 0x9de8, 0x8dc9,
    0x7c26, 0x6c07, 0x5c64, 0x4c45, 0x3ca2, 0x2c83, 0x1ce0, 0x0cc1,
    0xef1f, 0xff3e, 0xcf5d, 0xdf7c, 0xaf9b, 0xbfba, 0x8fd9, 0x9ff8,
    0x6e17, 0x7e36, 0x4e55, 0x5e74, 0x2e93, 0x3eb2, 0x0ed1, 0x1ef0
]

# ✅ 2. 替换原有的双层循环算法，直接查表！
def crc16_ccitt(data: bytes) -> int:
    crc = 0xFFFF
    for byte in data:
        crc = ((crc << 8) ^ _CRC16_TABLE[(crc >> 8) ^ byte]) & 0xFFFF
    return crc


def missing_pc_frames(previous_sequence: int | None, current_sequence: int) -> int:
    """返回 8-bit PC sample batch sequence 中间跳过的帧数。"""
    if previous_sequence is None:
        return 0
    diff = (current_sequence - previous_sequence) & 0xFF
    if diff <= 1:
        return 0
    return diff - 1


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
    last_bad_count: int = 0
    pc_sequence_gaps: int = 0
    pc_resend_requests: int = 0
    pc_resend_recovered: int = 0
    pc_duplicate_frames: int = 0
    bad_timestamps: int = 0
    resync_bytes: int = 0
    host_queue_drops: int = 0
    host_queue_drop_bytes: int = 0
    reader_errors: int = 0
    last_reader_error: str = ""


@dataclass
class SyncDiag:
    host_rx_time: float
    source_id: int
    state: int
    beacon_seq: int
    offset_us: int
    drift_ppm: int
    residual_us: int
    beacon_count: int


@dataclass
class StateEvent:
    host_rx_time: float
    source_id: int
    state: int
    command_seq: int
    effective_master_time_us: int


class AcquisitionModel:
    def __init__(self, max_points: int, emg_display_rate: float) -> None:
        """初始化采集缓存、EMG 显示滤波器和 CSV/诊断状态。"""
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
        self.sync_csv_fp = None
        self.sync_csv_writer = None
        self.diag_forwarded_per_sec = 0
        self.diag_errors_per_sec = 0
        self.diag_online_slave_count = 0
        self.diag_link_events_per_sec = 0
        self.diag_queue_drops_per_sec = 0
        self.diag_ack_fails_per_sec = 0
        self.diag_errors_total = 0
        self.diag_link_events_total = 0
        self.diag_queue_drops_total = 0
        self.diag_ack_fails_total = 0
        self.system_state = 0
        self.state_events: deque[StateEvent] = deque(maxlen=200)
        self.sync_diag = {
            SOURCE_GYRO: deque(maxlen=500),
            SOURCE_ACCEL: deque(maxlen=500),
        }
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
        self.append_samples([(source_id, timestamp_us, sample_seq, batch_seq, rx_flags, x, y, z, time.time())])

    def append_samples(
        self,
        samples: list[tuple[int, int, int, int, int, int, int, int, float]],
    ) -> None:
        """按 PC 帧批量追加样本，减少串口解析线程在锁和 CSV I/O 上的开销。"""
        if not samples:
            return

        with self.lock:
            csv_rows: list[list[object]] = []
            for source_id, timestamp_us, sample_seq, batch_seq, rx_flags, x, y, z, host_rx_time in samples:
                source = self.sources.get(source_id)
                if source is None:
                    continue

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
                source.last_host_rx_time = host_rx_time
                source.received_count += 1

                if self.is_recording and self.csv_writer is not None:
                    vector_norm = float(np.sqrt(float(x) * float(x) + float(y) * float(y) + float(z) * float(z)))
                    csv_rows.append(
                        [
                            f"{host_rx_time:.6f}",
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

            if csv_rows and self.csv_writer is not None:
                self.csv_writer.writerows(csv_rows)

    def update_diag(
        self,
        forwarded_per_sec: int,
        errors_per_sec: int,
        online_slave_count: int,
        link_events_per_sec: int = 0,
        queue_drops_per_sec: int = 0,
        ack_fails_per_sec: int = 0,
    ) -> None:
        """更新 Master 发来的每秒链路诊断信息。"""
        with self.lock:
            self.diag_forwarded_per_sec = forwarded_per_sec
            self.diag_errors_per_sec = errors_per_sec
            self.diag_online_slave_count = online_slave_count
            self.diag_link_events_per_sec = link_events_per_sec
            self.diag_queue_drops_per_sec = queue_drops_per_sec
            self.diag_ack_fails_per_sec = ack_fails_per_sec
            self.diag_errors_total += max(0, errors_per_sec)
            self.diag_link_events_total += max(0, link_events_per_sec)
            self.diag_queue_drops_total += max(0, queue_drops_per_sec)
            self.diag_ack_fails_total += max(0, ack_fails_per_sec)

    def update_sync_diag(
        self,
        source_id: int,
        state: int,
        beacon_seq: int,
        offset_us: int,
        drift_ppm: int,
        residual_us: int,
        beacon_count: int,
    ) -> None:
        """更新某个 Slave 的同步拟合诊断，并在录制时写入 sync CSV。"""
        diag = SyncDiag(
            host_rx_time=time.time(),
            source_id=source_id,
            state=state,
            beacon_seq=beacon_seq,
            offset_us=offset_us,
            drift_ppm=drift_ppm,
            residual_us=residual_us,
            beacon_count=beacon_count,
        )
        with self.lock:
            if source_id not in self.sync_diag:
                self.sync_diag[source_id] = deque(maxlen=500)
            self.sync_diag[source_id].append(diag)
            if self.is_recording and self.sync_csv_writer is not None:
                self.sync_csv_writer.writerow(
                    [
                        f"{diag.host_rx_time:.6f}",
                        SOURCE_NAMES.get(source_id, f"Unknown({source_id})"),
                        source_id,
                        STATE_NAMES.get(state, str(state)),
                        beacon_seq,
                        offset_us,
                        drift_ppm,
                        residual_us,
                        beacon_count,
                    ]
                )

    def update_state_event(self, source_id: int, state: int, command_seq: int, effective_master_time_us: int) -> None:
        """更新 Master 或 Slave 的状态 ACK 事件。"""
        event = StateEvent(time.time(), source_id, state, command_seq, effective_master_time_us)
        with self.lock:
            self.state_events.append(event)
            if source_id == SOURCE_EMG:
                self.system_state = state

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

    def diag_snapshot(self) -> tuple[int, int, int, int, int, int, int, int, int, int, ParserStats]:
        """返回诊断统计快照，避免 UI 直接读取共享状态。"""
        with self.lock:
            return (
                self.diag_forwarded_per_sec,
                self.diag_errors_per_sec,
                self.diag_online_slave_count,
                self.diag_link_events_per_sec,
                self.diag_queue_drops_per_sec,
                self.diag_ack_fails_per_sec,
                self.diag_errors_total,
                self.diag_link_events_total,
                self.diag_queue_drops_total,
                self.diag_ack_fails_total,
                ParserStats(
                    crc_fails=self.parser_stats.crc_fails,
                    bad_headers=self.parser_stats.bad_headers,
                    bad_sources=self.parser_stats.bad_sources,
                    bad_counts=self.parser_stats.bad_counts,
                    last_bad_count=self.parser_stats.last_bad_count,
                    pc_sequence_gaps=self.parser_stats.pc_sequence_gaps,
                    pc_resend_requests=self.parser_stats.pc_resend_requests,
                    pc_resend_recovered=self.parser_stats.pc_resend_recovered,
                    pc_duplicate_frames=self.parser_stats.pc_duplicate_frames,
                    bad_timestamps=self.parser_stats.bad_timestamps,
                    resync_bytes=self.parser_stats.resync_bytes,
                    host_queue_drops=self.parser_stats.host_queue_drops,
                    host_queue_drop_bytes=self.parser_stats.host_queue_drop_bytes,
                    reader_errors=self.parser_stats.reader_errors,
                    last_reader_error=self.parser_stats.last_reader_error,
                ),
            )

    def sync_snapshot(self) -> tuple[int, dict[int, list[SyncDiag]], list[StateEvent]]:
        """返回同步诊断和状态事件快照。"""
        with self.lock:
            return (
                self.system_state,
                {source_id: list(values) for source_id, values in self.sync_diag.items()},
                list(self.state_events),
            )

    def note_crc_fail(self) -> None:
        with self.lock:
            self.parser_stats.crc_fails += 1

    def note_bad_header(self) -> None:
        with self.lock:
            self.parser_stats.bad_headers += 1

    def note_bad_source(self) -> None:
        with self.lock:
            self.parser_stats.bad_sources += 1

    def note_bad_count(self, count: int) -> None:
        with self.lock:
            self.parser_stats.bad_counts += 1
            self.parser_stats.last_bad_count = count

    def note_pc_sequence_gap(self, missing_frames: int) -> None:
        if missing_frames <= 0:
            return
        with self.lock:
            self.parser_stats.pc_sequence_gaps += missing_frames

    def note_pc_resend_request(self, count: int = 1) -> None:
        if count <= 0:
            return
        with self.lock:
            self.parser_stats.pc_resend_requests += count

    def note_pc_resend_recovered(self) -> None:
        with self.lock:
            self.parser_stats.pc_resend_recovered += 1

    def note_pc_duplicate_frame(self) -> None:
        with self.lock:
            self.parser_stats.pc_duplicate_frames += 1

    def note_bad_timestamp(self) -> None:
        with self.lock:
            self.parser_stats.bad_timestamps += 1

    def note_resync_bytes(self, count: int) -> None:
        if count <= 0:
            return
        with self.lock:
            self.parser_stats.resync_bytes += count

    def note_host_queue_drop(self, byte_count: int) -> None:
        with self.lock:
            self.parser_stats.host_queue_drops += 1
            self.parser_stats.host_queue_drop_bytes += max(0, byte_count)

    def note_reader_error(self, exc: Exception) -> None:
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
            if self.system_state != 3:
                sync_path = csv_dir / f"sync_diag_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
                self.sync_csv_fp = sync_path.open("w", newline="", encoding="utf-8")
                self.sync_csv_writer = csv.writer(self.sync_csv_fp)
                self.sync_csv_writer.writerow(
                    [
                        "host_rx_time",
                        "source",
                        "source_id",
                        "state",
                        "beacon_seq",
                        "offset_us",
                        "drift_ppm",
                        "residual_us",
                        "beacon_count",
                    ]
                )
            else:
                self.sync_csv_fp = None
                self.sync_csv_writer = None
            self.is_recording = True
            return csv_path

    def stop_recording(self) -> None:
        """停止 CSV 录制并关闭文件句柄。"""
        with self.lock:
            self.is_recording = False
            if self.csv_fp is not None:
                self.csv_fp.close()
            if self.sync_csv_fp is not None:
                self.sync_csv_fp.close()
            self.csv_fp = None
            self.csv_writer = None
            self.sync_csv_fp = None
            self.sync_csv_writer = None


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
        self.write_lock = threading.Lock()
        self.raw_queue: queue.Queue[bytes] = queue.Queue(maxsize=RAW_QUEUE_MAX_CHUNKS)
        self.raw_reader_thread: threading.Thread | None = None
        self.last_sample_frame_sequence: int | None = None
        self.pending_resend_sequences: set[int] = set()

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

    def _raw_reader_loop(self) -> None:
        """只负责从 CDC 端口搬运原始字节，避免被解析、绘图和 CSV I/O 阻塞。"""
        if self.serial_port is None:
            return

        while not self.stop_event.is_set():
            try:
                chunk = self.serial_port.read(SERIAL_READ_SIZE)
                if not chunk:
                    continue
                try:
                    self.raw_queue.put_nowait(chunk)
                except queue.Full:
                    self.model.note_host_queue_drop(len(chunk))
            except serial.SerialException as exc:
                self.model.note_reader_error(exc)
                self.stop_event.set()
                break
            except Exception as exc:
                self.model.note_reader_error(exc)

    def send_text_command(self, command: str) -> None:
        """向 Master 发送一条以换行结尾的文本控制命令。"""
        if self.serial_port is None or not self.serial_port.is_open:
            raise RuntimeError("串口尚未打开。")
        payload = f"{command.strip()}\n".encode("utf-8")
        with self.write_lock:
            self.serial_port.write(payload)
            self.serial_port.flush()

    def request_resend(self, sequence: int) -> None:
        """请求 Master 重发某个 PC sample batch sequence。"""
        seq = sequence & 0xFF
        if seq in self.pending_resend_sequences:
            return
        self.pending_resend_sequences.add(seq)
        try:
            self.send_text_command(f"RESEND {seq}")
            self.model.note_pc_resend_request()
        except Exception as exc:
            self.model.note_reader_error(exc)

    def classify_sample_frame_sequence(self, sequence: int) -> str:
        """返回 current/recovered/duplicate，用于处理正常帧与乱序重发帧。"""
        if self.last_sample_frame_sequence is None:
            self.last_sample_frame_sequence = sequence
            return "current"

        diff = (sequence - self.last_sample_frame_sequence) & 0xFF
        if diff == 0:
            self.model.note_pc_duplicate_frame()
            return "duplicate"
        if diff < 128:
            missing = diff - 1
            if missing > 0:
                self.model.note_pc_sequence_gap(missing)
                for offset in range(1, min(missing, MAX_RESEND_REQUESTS_PER_GAP) + 1):
                    self.request_resend((self.last_sample_frame_sequence + offset) & 0xFF)
            self.last_sample_frame_sequence = sequence
            return "current"

        if sequence in self.pending_resend_sequences:
            self.pending_resend_sequences.remove(sequence)
            self.model.note_pc_resend_recovered()
            return "recovered"

        self.model.note_pc_duplicate_frame()
        return "duplicate"

    def note_timestamp(self, source_id: int, timestamp_us: int) -> None:
        """检查到达顺序中的时间戳异常，并在必要时触发 EMG 重锁。"""
        if source_id == SOURCE_DIAG:
            return

        if source_id not in (SOURCE_EMG, SOURCE_GYRO, SOURCE_ACCEL):
            return

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

        self.raw_reader_thread = threading.Thread(target=self._raw_reader_loop, name="serialRawReader", daemon=True)
        self.raw_reader_thread.start()
        raw_buffer = bytearray()
        while not self.stop_event.is_set():
            try:
                try:
                    chunk = self.raw_queue.get(timeout=0.05)
                    raw_buffer += chunk
                except queue.Empty:
                    continue

                while len(raw_buffer) >= MIN_FRAME_SIZE:
                    idx = raw_buffer.find(FRAME_HEADER)
                    if idx == -1:
                        self.model.note_resync_bytes(max(0, len(raw_buffer) - 1))
                        raw_buffer = raw_buffer[-1:]
                        break

                    if idx > 0:
                        self.model.note_resync_bytes(idx)
                        raw_buffer = raw_buffer[idx:]

                    if len(raw_buffer) < 4:
                        break

                    frame_type = raw_buffer[2]
                    if frame_type == PC_MSG_SYNC_DIAG:
                        if len(raw_buffer) < PC_SYNC_DIAG_SIZE:
                            break
                        frame = raw_buffer[:PC_SYNC_DIAG_SIZE]
                        received_crc = int.from_bytes(frame[-2:], byteorder="little", signed=False)
                        calculated_crc = crc16_ccitt(frame[:-2])
                        if received_crc != calculated_crc:
                            self.model.note_crc_fail()
                            raw_buffer = raw_buffer[1:]
                            continue
                        source_id, state, beacon_seq, offset_us, drift_ppm, residual_us, beacon_count = PC_SYNC_DIAG_STRUCT.unpack(
                            frame[4:-2]
                        )
                        self.model.update_sync_diag(source_id, state, beacon_seq, offset_us, drift_ppm, residual_us, beacon_count)
                        raw_buffer = raw_buffer[PC_SYNC_DIAG_SIZE:]
                        continue

                    if frame_type == PC_MSG_STATE_EVENT:
                        if len(raw_buffer) < PC_STATE_EVENT_SIZE:
                            break
                        frame = raw_buffer[:PC_STATE_EVENT_SIZE]
                        received_crc = int.from_bytes(frame[-2:], byteorder="little", signed=False)
                        calculated_crc = crc16_ccitt(frame[:-2])
                        if received_crc != calculated_crc:
                            self.model.note_crc_fail()
                            raw_buffer = raw_buffer[1:]
                            continue
                        source_id, state, command_seq, effective_master_time_us = PC_STATE_EVENT_STRUCT.unpack(frame[4:-2])
                        self.model.update_state_event(source_id, state, command_seq, effective_master_time_us)
                        raw_buffer = raw_buffer[PC_STATE_EVENT_SIZE:]
                        continue

                    if frame_type != PC_MSG_SAMPLE_BATCH:
                        self.model.note_bad_header()
                        raw_buffer = raw_buffer[1:]
                        continue

                    if len(raw_buffer) < PC_BATCH_HEADER_SIZE:
                        break

                    sample_count = raw_buffer[3]
                    if sample_count == 0 or sample_count > PC_MAX_BATCH_SAMPLES:
                        self.model.note_bad_count(sample_count)
                        raw_buffer = raw_buffer[1:]
                        continue

                    frame_sequence = raw_buffer[4]
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

                    sequence_status = self.classify_sample_frame_sequence(frame_sequence)
                    if sequence_status == "duplicate":
                        raw_buffer = raw_buffer[packet_size:]
                        continue

                    payload_offset = PC_BATCH_HEADER_SIZE
                    decoded_samples: list[tuple[int, int, int, int, int, int, int, int, float]] = []
                    host_rx_time = time.time()
                    for _ in range(sample_count):
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
                            self.model.update_diag(
                                x,
                                y,
                                z,
                                link_events_per_sec=sample_seq,
                                queue_drops_per_sec=batch_seq,
                                ack_fails_per_sec=rx_flags,
                            )
                            continue

                        if sequence_status == "current":
                            self.note_timestamp(source_id, timestamp_us)
                        decoded_samples.append((source_id, timestamp_us, sample_seq, batch_seq, rx_flags, x, y, z, host_rx_time))

                    self.model.append_samples(decoded_samples)

                    raw_buffer = raw_buffer[packet_size:]
            except Exception as exc:
                self.model.note_reader_error(exc)
                continue


class MainWindow(QMainWindow):
    def __init__(self, model: AcquisitionModel, reader: SerialReader, csv_dir: Path, port: str, baud: int, window_seconds: float) -> None:
        """创建示波器主窗口、曲线、状态栏和定时刷新器。"""
        super().__init__()
        self.model = model
        self.reader = reader
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
        self.control_status_text = ""
        self.control_status_color = "#FFD740"
        self.control_status_until = 0.0
        self.current_recording_path: Path | None = None

        self.setWindowTitle("Wireless Capture Scope - EMG Bandpass Mode")
        self.setMinimumWidth(1200) # 设定最小宽度保障排版
        self.resize(1200, 850) 

        central = QWidget()
        self.setCentralWidget(central)
        main_layout = QVBoxLayout(central)
        main_layout.setContentsMargins(10, 10, 10, 10)
        main_layout.setSpacing(10)

        # --- 顶部控制与状态栏结构化重构 ---
        top_row = QVBoxLayout()

        # 1. 控制面板 Group
        control_group = QGroupBox("系统控制")
        control_layout = QVBoxLayout(control_group)
        action_layout = QHBoxLayout()
        radio_layout = QHBoxLayout()
        self.sync_button = QPushButton("开始同步")
        self.stream_button = QPushButton("开始采集")
        self.stop_button = QPushButton("停止")
        self.record_button = QPushButton("开始录制 CSV")
        self.channel_combo = QComboBox()
        for channel in range(1, 14):
            self.channel_combo.addItem(f"CH{channel}", channel)
        self.channel_combo.setCurrentIndex(0)
        self.rate_combo = QComboBox()
        self.rate_combo.addItem("2Mbps", "2M")
        self.rate_combo.addItem("1Mbps", "1M")
        self.radio_button = QPushButton("应用无线")

        for button in (self.sync_button, self.stream_button, self.stop_button, self.radio_button):
            button.setMinimumWidth(96)
            button.setMinimumHeight(34)
        self.record_button.setMinimumWidth(132)
        self.record_button.setMinimumHeight(34)
        self.channel_combo.setMinimumWidth(92)
        self.rate_combo.setMinimumWidth(92)
        
        button_style = "padding: 6px 12px; font-weight: bold; border-radius: 4px;"
        self.sync_button.setStyleSheet(button_style + "background-color: #3949AB; color: white;")
        self.stream_button.setStyleSheet(button_style + "background-color: #00897B; color: white;")
        self.stop_button.setStyleSheet(button_style + "background-color: #E53935; color: white;")
        self.record_button.setStyleSheet(button_style + "background-color: #2E7D32; color: white;")
        self.radio_button.setStyleSheet(button_style)

        self.sync_button.clicked.connect(lambda: self.send_control_command("START_SYNC"))
        self.stream_button.clicked.connect(lambda: self.send_control_command("START_STREAM"))
        self.stop_button.clicked.connect(lambda: self.send_control_command("STOP"))
        self.radio_button.clicked.connect(self.send_radio_command)
        self.record_button.clicked.connect(self.toggle_recording)
        
        action_layout.addWidget(self.sync_button)
        action_layout.addWidget(self.stream_button)
        action_layout.addWidget(self.stop_button)
        action_layout.addWidget(self.record_button)
        action_layout.addStretch()
        radio_layout.addWidget(QLabel("无线信道"))
        radio_layout.addWidget(self.channel_combo)
        radio_layout.addWidget(QLabel("速率"))
        radio_layout.addWidget(self.rate_combo)
        radio_layout.addWidget(self.radio_button)
        radio_layout.addStretch()
        control_layout.addLayout(action_layout)
        control_layout.addLayout(radio_layout)
        top_row.addWidget(control_group)

        info_row = QHBoxLayout()

        # 2. 核心状态 Group
        status_group = QGroupBox("实时状态")
        status_group.setMinimumWidth(380) # 阻止向内过度挤压引发文字高度跳动
        status_layout = QVBoxLayout(status_group)
        self.primary_status_label = QLabel(f"串口 {self.port} @ {self.baud}")
        self.sync_status_label = QLabel("State IDLE | Sync --")
        
        # 核心防抖动设定：允许内部文字强制折行，避免拉伸窗口
        self.primary_status_label.setWordWrap(True)
        self.sync_status_label.setWordWrap(True)
        self.primary_status_label.setTextFormat(Qt.RichText)
        self.sync_status_label.setTextFormat(Qt.RichText)
        
        status_layout.addWidget(self.primary_status_label)
        status_layout.addWidget(self.sync_status_label)
        info_row.addWidget(status_group, stretch=1)

        # 3. 链路诊断 Group
        diag_group = QGroupBox("无线与底层诊断")
        diag_group.setMinimumWidth(450) # 保障诊断信息的最小显示空间
        diag_layout = QVBoxLayout(diag_group)
        self.link_status_label = QLabel("等待数据接入...")
        
        # 核心防抖动设定：允许文字换行
        self.link_status_label.setWordWrap(True)
        self.link_status_label.setTextFormat(Qt.RichText)
        self.link_status_label.setStyleSheet("font-size: 12px; color: #BBBBBB; line-height: 1.4;") 
        
        diag_layout.addWidget(self.link_status_label)
        info_row.addWidget(diag_group, stretch=2)
        top_row.addLayout(info_row)

        main_layout.addLayout(top_row)
        # --- 顶部重构结束 ---

        self.tabs = QTabWidget()
        self.emg_graph = pg.GraphicsLayoutWidget()
        self.imu_graph = pg.GraphicsLayoutWidget()
        self.emg_graph.ci.layout.setContentsMargins(5, 5, 5, 5)
        self.emg_graph.ci.layout.setSpacing(10)
        self.imu_graph.ci.layout.setContentsMargins(5, 5, 5, 5)
        self.imu_graph.ci.layout.setSpacing(10)
        
        self.tabs.addTab(self.emg_graph, "EMG 波形")
        self.tabs.addTab(self.imu_graph, "IMU 波形")
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
        plot.showGrid(x=True, y=True, alpha=0.3)
        plot.setLabel("left", y_label)
        plot.setLabel("bottom", "Time", units="s")
        plot.setXRange(-self.window_seconds, 0.0, padding=0.0)
        plot.setClipToView(True)
        plot.setDownsampling(mode="peak")
        plot.getAxis("bottom").setPen("#546E7A")
        plot.getAxis("left").setPen("#546E7A")
        plot.getAxis("bottom").setTextPen("#B0BEC5")
        plot.getAxis("left").setTextPen("#B0BEC5")

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
        return " &nbsp;|&nbsp; ".join(parts) # 优化排版分隔符

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
        return f"{name}: {state} ({self.source_rates[source_id]:.0f}/s)"

    def _latest_master_state_event(self, state_events: list[StateEvent]) -> StateEvent | None:
        """返回最近一条 Master 自身的状态事件，用于判断 STREAM 是否已经真正开始。"""
        for event in reversed(state_events):
            if event.source_id == SOURCE_EMG:
                return event
        return None

    def _format_emg_status(
        self,
        system_state: int,
        emg_count: int,
        emg_quality_text: str,
        state_events: list[StateEvent],
        now: float,
    ) -> str:
        """根据系统状态生成 EMG 状态栏，避免把非 STREAM 阶段的 0 点误判为故障。"""
        if system_state == 0:
            return "IDLE (无采集)"
        if system_state == 1:
            return "SYNC (等待同步)"
        if system_state == 2:
            return "STREAM_PENDING (等待启动)"

        latest_master_event = self._latest_master_state_event(state_events)
        stream_elapsed = now - latest_master_event.host_rx_time if latest_master_event is not None else 0.0
        if emg_count == 0 and stream_elapsed >= 1.0:
            return "<span style='color:#FF5252;'>STREAM 进入但无数据，请检查接线</span>"
        
        # 分层级显示 EMG 数据状态
        return f"当前速率: {self.source_rates[SOURCE_EMG]:.0f} Hz (共 {emg_count} 点)<br><b>[信号质量]</b> {emg_quality_text}"

    def _stream_sync_warning(self) -> str:
        """在开始采集前给出同步质量提示；提示不阻止用户手动进入 STREAM。"""
        _, sync_diag, _ = self.model.sync_snapshot()
        warnings = []
        for source_id, name in ((SOURCE_GYRO, "Gyro"), (SOURCE_ACCEL, "Accel")):
            values = sync_diag.get(source_id, [])
            if not values:
                warnings.append(f"{name} 无同步诊断")
                continue
            latest = values[-1]
            if latest.beacon_count < SYNC_MIN_BEACON_COUNT:
                warnings.append(f"{name} Beacon 不足")
            elif abs(latest.residual_us) > SYNC_USABLE_RESIDUAL_US:
                warnings.append(f"{name} residual {latest.residual_us}us")
        if not warnings:
            return ""
        return "同步质量不足: " + ", ".join(warnings)

    def send_control_command(self, command: str) -> None:
        """发送 START_SYNC/START_STREAM/STOP 到 Master。"""
        try:
            warning_text = self._stream_sync_warning() if command == "START_STREAM" else ""
            self.reader.send_text_command(command)
            self.reader.send_text_command("PING")
            if warning_text:
                self.control_status_text = f"已发送命令: {command} <br><span style='color:#FFAB40;'>{warning_text}</span>"
            else:
                self.control_status_text = f"已发送命令: {command}"
            self.control_status_color = "#FFD740"
            self.control_status_until = time.time() + 3.0
        except Exception as exc:
            QMessageBox.warning(self, "命令发送失败", str(exc))

    def send_radio_command(self) -> None:
        """发送 RADIO channel/rate 到 Master；切换后需要重新同步再采集。"""
        try:
            channel = int(self.channel_combo.currentData())
            rate = str(self.rate_combo.currentData())
            command = f"RADIO {channel} {rate}"
            self.reader.send_text_command(command)
            self.reader.send_text_command("PING")
            self.control_status_text = f"已发送无线配置: CH{channel} / {rate}，请重新同步后采集"
            self.control_status_color = "#FFD740"
            self.control_status_until = time.time() + 5.0
        except Exception as exc:
            QMessageBox.warning(self, "无线配置失败", str(exc))

    def _format_sync_line(self, name: str, values: list[SyncDiag], now: float) -> str:
        """格式化某个 Slave 的同步诊断和稳定性提示。"""
        if not values:
            return f"{name}: 缺"
        latest = values[-1]
        recent = [item for item in values if now - item.host_rx_time <= SYNC_STABLE_WINDOW_S]
        quality = "未同步"
        if latest.beacon_count >= SYNC_MIN_BEACON_COUNT:
            quality = "可用" if abs(latest.residual_us) <= SYNC_USABLE_RESIDUAL_US else "未稳"
        if len(recent) >= 5:
            drift_values = [item.drift_ppm for item in recent]
            drift_range = max(drift_values) - min(drift_values)
            stable = (
                latest.beacon_count >= SYNC_MIN_BEACON_COUNT
                and all(abs(item.residual_us) <= SYNC_STABLE_RESIDUAL_US for item in recent)
                and drift_range <= SYNC_STABLE_DRIFT_RANGE_PPM
            )
            excellent = (
                latest.beacon_count >= SYNC_MIN_BEACON_COUNT
                and all(abs(item.residual_us) <= SYNC_EXCELLENT_RESIDUAL_US for item in recent)
                and drift_range <= SYNC_EXCELLENT_DRIFT_RANGE_PPM
            )
            if excellent:
                quality = "优秀"
            elif stable:
                quality = "稳定"
        return (
            f"<b>{name}</b> ({quality}) &nbsp; "
            f"off: {latest.offset_us}us &nbsp;|&nbsp; drift: {latest.drift_ppm}ppm &nbsp;|&nbsp; "
            f"res: {latest.residual_us}us &nbsp;|&nbsp; n: {latest.beacon_count}"
        )

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

        (
            diag_forwarded,
            diag_errors,
            diag_online,
            diag_link_events,
            diag_queue_drops,
            diag_ack_fails,
            diag_errors_total,
            diag_link_events_total,
            diag_queue_drops_total,
            diag_ack_fails_total,
            parser_stats,
        ) = self.model.diag_snapshot()
        system_state, sync_diag, state_events = self.model.sync_snapshot()
        emg_count = self.last_counts[SOURCE_EMG] if self.last_counts[SOURCE_EMG] >= 0 else 0
        gyro_count = self.last_counts[SOURCE_GYRO] if self.last_counts[SOURCE_GYRO] >= 0 else 0
        accel_count = self.last_counts[SOURCE_ACCEL] if self.last_counts[SOURCE_ACCEL] >= 0 else 0
        record_state = "<span style='color:#69F0AE;'>[录制中]</span>" if self.model.is_recording else "<span style='color:#FF5252;'>[未录制]</span>"
        reader_error_text = (
            f" &nbsp;|&nbsp; reader_err {parser_stats.reader_errors} ({parser_stats.last_reader_error})"
            if parser_stats.reader_errors
            else ""
        )
        host_queue_text = (
            f" &nbsp;&nbsp; 主机队列丢: {parser_stats.host_queue_drops}次/{parser_stats.host_queue_drop_bytes}B"
            if parser_stats.host_queue_drops
            else " &nbsp;&nbsp; 主机队列丢: 0"
        )
        slave1_last_rx = status_snapshot.get(SOURCE_GYRO, (0, 0.0))[1]
        slave2_last_rx = status_snapshot.get(SOURCE_ACCEL, (0, 0.0))[1]
        slave1_text = self._format_link_status("Gyro", SOURCE_GYRO, slave1_last_rx, now)
        slave2_text = self._format_link_status("Accel", SOURCE_ACCEL, slave2_last_rx, now)
        
        emg_status_formatted = self._format_emg_status(
            system_state,
            emg_count,
            getattr(self, "emg_quality_text", "CH6 -- | CH7 -- | CH8 --"),
            state_events,
            now,
        )
        
        control_status = (
            f"<br><span style='color:{self.control_status_color};'>{self.control_status_text}</span>"
            if now < self.control_status_until
            else ""
        )
        
        # 1. 核心状态显示重构 (折行排版)
        self.primary_status_label.setText(
            f"<b>[本地]</b> 串口 {self.port} @ {self.baud} <br>"
            f"<b>[状态]</b>: {record_state}<br>"
            f"<b>[EMG]</b> {emg_status_formatted}"
            f"{control_status}"
        )

        gyro_sync = self._format_sync_line("Gyro", sync_diag.get(SOURCE_GYRO, []), now)
        accel_sync = self._format_sync_line("Accel", sync_diag.get(SOURCE_ACCEL, []), now)
        last_event = state_events[-1] if state_events else None
        event_text = ""
        if last_event is not None:
            event_text = (
                f" &nbsp;|&nbsp; <b>最后收到ACK:</b> {SOURCE_NAMES.get(last_event.source_id, str(last_event.source_id))} "
                f"进入 {STATE_NAMES.get(last_event.state, str(last_event.state))}"
            )
            
        self.sync_status_label.setText(
            f"<b>[全局系统]</b> 模式: <span style='color:#448AFF;'>{STATE_NAMES.get(system_state, str(system_state))}</span>{event_text}<br>"
            f"<b>[时钟拟合]</b> {gyro_sync}<br>"
            f"<b>[时钟拟合]</b> {accel_sync}"
        )

        # 2. 诊断显示重构 (结构化行高与分类排版)
        self.link_status_label.setText(
            f"<b>[连接状态]</b> {slave1_text} &nbsp;&nbsp; {slave2_text}<br>"
            f"<b>[缓冲队列]</b> Gyro: {gyro_count} &nbsp;&nbsp; Accel: {accel_count}<br>"
            f"<b>[无线吞吐]</b> Master收到: {diag_forwarded}/s &nbsp;&nbsp; 错包(err): {diag_errors_total} &nbsp;&nbsp; 无线补包: {diag_link_events_total}<br>"
            f"<b>[链路丢包]</b> Slave抛弃(qdrop): {diag_queue_drops_total} &nbsp;&nbsp; 确认失败(ackfail): {diag_ack_fails_total}<br>"
            f"<b>[底层串流]</b> CRC错: {parser_stats.crc_fails} &nbsp;&nbsp; 帧头错: {parser_stats.bad_headers} "
            f"&nbsp;&nbsp; count错: {parser_stats.bad_counts}({parser_stats.last_bad_count}) &nbsp;&nbsp; PC跳帧: {parser_stats.pc_sequence_gaps}<br>"
            f"<b>[USB重发]</b> 请求: {parser_stats.pc_resend_requests} &nbsp;&nbsp; 恢复: {parser_stats.pc_resend_recovered} "
            f"&nbsp;&nbsp; 重复/过期: {parser_stats.pc_duplicate_frames}<br>"
            f"<b>[主机接收]</b> 丢弃: {parser_stats.resync_bytes}B{host_queue_text}{reader_error_text}"
        )

    def toggle_recording(self) -> None:
        """切换 CSV 录制状态，并同步按钮样式。"""
        if not self.model.is_recording:
            try:
                csv_path = self.model.start_recording(self.csv_dir)
            except Exception as exc:
                QMessageBox.critical(self, "录制失败", str(exc))
                return
            self.current_recording_path = csv_path
            self.record_button.setText("停止录制 CSV")
            self.record_button.setStyleSheet(
                "padding: 6px 12px; font-weight: bold; border-radius: 4px; background-color: #C62828; color: white;"
            )
        else:
            csv_path = self.current_recording_path
            self.model.stop_recording()
            self.current_recording_path = None
            self.record_button.setText("开始录制 CSV")
            self.record_button.setStyleSheet(
                "padding: 6px 12px; font-weight: bold; border-radius: 4px; background-color: #2E7D32; color: white;"
            )
            if csv_path is not None:
                self.control_status_text = f"录制已停止，正在分析: {html.escape(csv_path.name)}"
                self.control_status_color = "#000000"
                self.control_status_until = time.time() + 60.0
                threading.Thread(target=self._analyze_recording_csv, args=(csv_path,), daemon=True).start()

    def _analyze_recording_csv(self, csv_path: Path) -> None:
        """后台运行 analyze_data.py，并把丢包摘要显示到状态栏。"""
        try:
            project_root = Path(__file__).resolve().parent.parent
            result = subprocess.run(
                [sys.executable, str(project_root / "tools" / "analyze_data.py"), str(csv_path)],
                cwd=project_root,
                text=True,
                capture_output=True,
                timeout=60,
                check=False,
            )
            if result.returncode != 0:
                detail = result.stderr.strip() or result.stdout.strip() or f"exit {result.returncode}"
                self.control_status_text = f"录制分析失败: {html.escape(detail[-240:])}"
                self.control_status_color = "#000000"
                self.control_status_until = time.time() + 60.0
                return

            summary_lines = []
            for line in result.stdout.splitlines():
                stripped = line.strip()
                if stripped.startswith("===") or stripped.startswith("总接收包数") or stripped.startswith("丢包率"):
                    summary_lines.append(stripped)
            if not summary_lines:
                summary_lines = ["分析完成，但没有解析到丢包摘要。"]
            escaped_summary = "<br>".join(html.escape(line) for line in summary_lines[:12])
            self.control_status_text = f"录制分析完成: {html.escape(csv_path.name)}<br>{escaped_summary}"
            self.control_status_color = "#000000"
            self.control_status_until = time.time() + 120.0
        except Exception as exc:
            self.control_status_text = f"录制分析异常: {html.escape(type(exc).__name__)}: {html.escape(str(exc))}"
            self.control_status_color = "#000000"
            self.control_status_until = time.time() + 60.0


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
    pg.setConfigOptions(antialias=True, foreground="#CFD8DC", background="#1E1E1E")

    window = MainWindow(model, reader, args.csv_dir, port, args.baud, args.window_seconds)
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
