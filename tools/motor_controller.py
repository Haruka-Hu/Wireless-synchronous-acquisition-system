#!/usr/bin/env python3
"""
ESP32-S3 BLE 电机节点上位机控制界面。

默认连接设备名 Neuro_Hammer_BLE，通过 Nordic UART 风格 BLE 服务发送文本命令。
"""

from __future__ import annotations

import argparse
import asyncio
import sys
import threading
import time
from collections import deque
from dataclasses import dataclass

from bleak import BleakClient, BleakScanner
from bleak.backends.device import BLEDevice
from PySide6.QtCore import QObject, Qt, QTimer, Signal
from PySide6.QtWidgets import (
    QApplication,
    QGridLayout,
    QHBoxLayout,
    QLabel,
    QMainWindow,
    QMessageBox,
    QPushButton,
    QPlainTextEdit,
    QVBoxLayout,
    QWidget,
)


DEFAULT_DEVICE = "Neuro_Hammer_BLE"
NUS_SERVICE_UUID = "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
NUS_RX_UUID = "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
NUS_TX_UUID = "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"


def parse_args() -> argparse.Namespace:
    """解析电机 BLE 上位机的命令行参数。"""
    parser = argparse.ArgumentParser(description="通过 BLE 控制 ESP32-S3 电机叩击节点。")
    parser.add_argument(
        "--device",
        default=DEFAULT_DEVICE,
        help="BLE 设备名或地址，默认 Neuro_Hammer_BLE。",
    )
    parser.add_argument(
        "--scan-timeout",
        type=float,
        default=8.0,
        help="按设备名扫描的超时时间，默认 8 秒。",
    )
    return parser.parse_args()


@dataclass
class LogLine:
    ts: float
    text: str


# Bleak 的 asyncio 循环放在后台线程，Qt 主线程只接收 Signal 更新界面。
# 这样扫描、连接、写 GATT 都不会阻塞按钮响应。
class BleWorker(QObject):
    status_changed = Signal(str)
    connected_changed = Signal(bool)
    notification_received = Signal(str)
    command_sent = Signal(str)
    error_occurred = Signal(str)

    def __init__(self, device_identifier: str, scan_timeout: float) -> None:
        """创建 BLE 后台工作器，并启动独立 asyncio 事件循环线程。"""
        super().__init__()
        self.device_identifier = device_identifier
        self.scan_timeout = scan_timeout
        self.loop = asyncio.new_event_loop()
        self.thread = threading.Thread(target=self._run_loop, daemon=True)
        self.client: BleakClient | None = None
        self.lock = threading.Lock()
        self.thread.start()

    def _run_loop(self) -> None:
        """在线程中运行 bleak 所需的 asyncio 事件循环。"""
        asyncio.set_event_loop(self.loop)
        self.loop.run_forever()

    def close(self) -> None:
        """关闭 BLE 连接、停止后台事件循环并等待线程退出。"""
        try:
            future = asyncio.run_coroutine_threadsafe(self._disconnect(), self.loop)
            future.result(timeout=3.0)
        except Exception:
            pass
        self.loop.call_soon_threadsafe(self.loop.stop)
        self.thread.join(timeout=2.0)

    def connect(self) -> None:
        """异步发起 BLE 扫描和连接。"""
        self._submit(self._connect())

    def disconnect(self) -> None:
        """异步断开 BLE 连接。"""
        self._submit(self._disconnect())

    def send_command(self, command: str) -> None:
        """异步发送一条文本命令到电机节点。"""
        self._submit(self._send_command(command))

    def _submit(self, coroutine) -> None:
        """把协程提交到后台事件循环，并把异常转成 Qt 信号。"""
        # 所有 BLE 协程都通过这里投递到后台 loop，并把异常转成 UI 可见的错误信号。
        future = asyncio.run_coroutine_threadsafe(coroutine, self.loop)

        def on_done(done_future) -> None:
            """处理后台协程结束结果，向 UI 报告异常。"""
            try:
                done_future.result()
            except Exception as exc:
                self.error_occurred.emit(str(exc))
                self.connected_changed.emit(False)

        future.add_done_callback(on_done)

    async def _find_device(self) -> BLEDevice:
        """按设备名或地址扫描目标 BLE 设备。"""
        self.status_changed.emit(f"扫描 {self.device_identifier} ...")

        # --device 可以是设备名，也可以是 BLE 地址；扫描时同时匹配这两个字段。
        def match(device: BLEDevice, _advertisement_data) -> bool:
            """判断扫描到的设备是否为用户指定目标。"""
            if device.name == self.device_identifier:
                return True
            return device.address == self.device_identifier

        device = await BleakScanner.find_device_by_filter(match, timeout=self.scan_timeout)
        if device is None:
            raise RuntimeError(f"未找到 BLE 设备: {self.device_identifier}")
        return device

    async def _connect(self) -> None:
        """连接 BLE 设备，开启 TX notify，并发送 PING 验证链路。"""
        with self.lock:
            if self.client is not None and self.client.is_connected:
                self.status_changed.emit("已经连接。")
                self.connected_changed.emit(True)
                return

        device = await self._find_device()
        self.status_changed.emit(f"连接 {device.name or device.address} ...")

        client = BleakClient(device, disconnected_callback=self._on_disconnected)
        await client.connect()
        await client.start_notify(NUS_TX_UUID, self._on_notify)

        with self.lock:
            self.client = client

        self.status_changed.emit(f"已连接: {device.name or device.address}")
        self.connected_changed.emit(True)
        await self._send_command("PING")

    async def _disconnect(self) -> None:
        """断开 BLE 前先发送 STOP，随后释放 client 状态。"""
        with self.lock:
            client = self.client
            self.client = None

        if client is not None:
            try:
                if client.is_connected:
                    # 主动断开前先发 STOP，降低手动按住期间断连导致电机继续转动的风险。
                    await client.write_gatt_char(NUS_RX_UUID, b"STOP\n", response=False)
                    await client.disconnect()
            finally:
                self.status_changed.emit("已断开。")
                self.connected_changed.emit(False)
        else:
            self.status_changed.emit("未连接。")
            self.connected_changed.emit(False)

    async def _send_command(self, command: str) -> None:
        """向 RX characteristic 写入一条以换行结尾的命令。"""
        with self.lock:
            client = self.client

        if client is None or not client.is_connected:
            raise RuntimeError("BLE 尚未连接。")

        # 固件按行解析文本命令，所以每条命令都以换行结尾。
        payload = f"{command.strip()}\n".encode("utf-8")
        await client.write_gatt_char(NUS_RX_UUID, payload, response=False)
        self.command_sent.emit(command.strip())

    def _on_notify(self, _sender, data: bytearray) -> None:
        """处理固件通过 TX notify 返回的 ACK/ERR 文本。"""
        text = bytes(data).decode("utf-8", errors="replace").strip()
        if text:
            self.notification_received.emit(text)

    def _on_disconnected(self, _client: BleakClient) -> None:
        """处理 BLE 底层断开事件并同步 UI 状态。"""
        with self.lock:
            self.client = None
        self.status_changed.emit("BLE 已断开。")
        self.connected_changed.emit(False)


class MainWindow(QMainWindow):
    def __init__(self, worker: BleWorker, device_identifier: str) -> None:
        """创建主窗口、按钮、日志区，并绑定 BLE worker 信号。"""
        super().__init__()
        self.worker = worker
        self.device_identifier = device_identifier
        self.connected = False
        self.log_lines: deque[LogLine] = deque(maxlen=300)

        self.setWindowTitle("Neuro Hammer Motor Controller")
        self.resize(720, 520)

        central = QWidget()
        self.setCentralWidget(central)
        main_layout = QVBoxLayout(central)

        top_row = QHBoxLayout()
        self.status_label = QLabel(f"设备: {device_identifier} | 未连接")
        self.status_label.setTextInteractionFlags(Qt.TextSelectableByMouse)
        self.connect_button = QPushButton("连接")
        self.connect_button.clicked.connect(self.toggle_connection)
        top_row.addWidget(self.status_label, 1)
        top_row.addWidget(self.connect_button)
        main_layout.addLayout(top_row)

        gear_grid = QGridLayout()
        self.gear_buttons: list[QPushButton] = []
        for gear in range(1, 6):
            button = QPushButton(f"档位 {gear}")
            button.setMinimumHeight(64)
            button.clicked.connect(lambda _checked=False, g=gear: self.send(f"STRIKE_{g}"))
            self.gear_buttons.append(button)
            gear_grid.addWidget(button, 0, gear - 1)
        main_layout.addLayout(gear_grid)

        manual_row = QHBoxLayout()
        self.forward_button = QPushButton("按住正转")
        self.reverse_button = QPushButton("按住反转")
        self.stop_button = QPushButton("急停")
        for button in (self.forward_button, self.reverse_button, self.stop_button):
            button.setMinimumHeight(72)

        self.forward_button.pressed.connect(lambda: self.send("MOTOR_FWD"))
        # 按住按钮释放时允许在 UI 状态刚变为断开时仍尝试 STOP；错误只进入日志，不阻塞界面。
        self.forward_button.released.connect(lambda: self.send("STOP", allow_disconnected=True))
        self.reverse_button.pressed.connect(lambda: self.send("MOTOR_REV"))
        self.reverse_button.released.connect(lambda: self.send("STOP", allow_disconnected=True))
        self.stop_button.clicked.connect(lambda: self.send("STOP", allow_disconnected=True))

        manual_row.addWidget(self.forward_button)
        manual_row.addWidget(self.reverse_button)
        manual_row.addWidget(self.stop_button)
        main_layout.addLayout(manual_row)

        self.log_view = QPlainTextEdit()
        self.log_view.setReadOnly(True)
        self.log_view.setMaximumBlockCount(350)
        main_layout.addWidget(self.log_view, 1)

        self.worker.status_changed.connect(self.on_status)
        self.worker.connected_changed.connect(self.set_connected)
        self.worker.notification_received.connect(self.on_notification)
        self.worker.command_sent.connect(self.on_command_sent)
        self.worker.error_occurred.connect(self.on_error)

        self.refresh_timer = QTimer(self)
        self.refresh_timer.timeout.connect(self.refresh_log)
        self.refresh_timer.start(200)

        self.set_connected(False)

    def closeEvent(self, event) -> None:
        """窗口关闭时安全释放 BLE worker。"""
        self.worker.close()
        event.accept()

    def toggle_connection(self) -> None:
        """根据当前状态执行连接或断开操作。"""
        if self.connected:
            self.worker.disconnect()
            self.on_status("正在断开 ...")
        else:
            self.worker.connect()
            self.on_status("正在连接 ...")

    def send(self, command: str, allow_disconnected: bool = False) -> None:
        """从 UI 发送命令，并在未连接时给出日志提示。"""
        if not self.connected and not allow_disconnected:
            self.append_log("UI", "尚未连接，命令未发送。")
            return
        self.worker.send_command(command)

    def set_connected(self, connected: bool) -> None:
        """更新连接状态并启用或禁用控制按钮。"""
        self.connected = connected
        self.connect_button.setText("断开" if connected else "连接")
        for button in self.gear_buttons:
            button.setEnabled(connected)
        self.forward_button.setEnabled(connected)
        self.reverse_button.setEnabled(connected)
        self.stop_button.setEnabled(connected)

    def on_status(self, text: str) -> None:
        """显示 BLE worker 的状态文本。"""
        self.status_label.setText(f"设备: {self.device_identifier} | {text}")
        self.append_log("STATUS", text)

    def on_notification(self, text: str) -> None:
        """把固件通知追加到日志。"""
        self.append_log("RX", text)

    def on_command_sent(self, command: str) -> None:
        """把已发送命令追加到日志。"""
        self.append_log("TX", command)

    def on_error(self, text: str) -> None:
        """显示 BLE 错误并追加日志。"""
        self.append_log("ERR", text)
        self.status_label.setText(f"设备: {self.device_identifier} | 错误: {text}")

    def append_log(self, source: str, text: str) -> None:
        """保存一条带时间戳的 UI 日志。"""
        self.log_lines.append(LogLine(time.time(), f"[{source}] {text}"))

    def refresh_log(self) -> None:
        """定时刷新日志文本框，并滚动到最新一行。"""
        current = self.log_view.toPlainText()
        rendered = "\n".join(
            f"{time.strftime('%H:%M:%S', time.localtime(line.ts))} {line.text}"
            for line in self.log_lines
        )
        if rendered != current:
            self.log_view.setPlainText(rendered)
            self.log_view.verticalScrollBar().setValue(self.log_view.verticalScrollBar().maximum())


def main() -> int:
    """创建 Qt 应用、电机 BLE worker 和主窗口。"""
    args = parse_args()

    app = QApplication(sys.argv)
    worker = BleWorker(device_identifier=args.device, scan_timeout=args.scan_timeout)
    window = MainWindow(worker=worker, device_identifier=args.device)
    window.show()

    try:
        return app.exec()
    except Exception as exc:
        QMessageBox.critical(None, "运行失败", str(exc))
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
