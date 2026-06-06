#!/usr/bin/env python3
"""
ESP32-S3 BLE 电机节点上位机控制界面。

默认连接设备名 Neuro_Hammer_BLE，通过 Nordic UART 风格 BLE 服务发送文本命令。
支持动态配置和修改电机叩击参数。
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
    QTableWidget,
    QTableWidgetItem,
    QHeaderView,
    QAbstractItemView
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


class BleWorker(QObject):
    # 保持原有的 BleWorker 逻辑完全不变
    status_changed = Signal(str)
    connected_changed = Signal(bool)
    notification_received = Signal(str)
    command_sent = Signal(str)
    error_occurred = Signal(str)

    def __init__(self, device_identifier: str, scan_timeout: float) -> None:
        super().__init__()
        self.device_identifier = device_identifier
        self.scan_timeout = scan_timeout
        self.loop = asyncio.new_event_loop()
        self.thread = threading.Thread(target=self._run_loop, daemon=True)
        self.client: BleakClient | None = None
        self.lock = threading.Lock()
        self.thread.start()

    def _run_loop(self) -> None:
        asyncio.set_event_loop(self.loop)
        self.loop.run_forever()

    def close(self) -> None:
        try:
            future = asyncio.run_coroutine_threadsafe(self._disconnect(), self.loop)
            future.result(timeout=3.0)
        except Exception:
            pass
        self.loop.call_soon_threadsafe(self.loop.stop)
        self.thread.join(timeout=2.0)

    def connect(self) -> None:
        self._submit(self._connect())

    def disconnect(self) -> None:
        self._submit(self._disconnect())

    def send_command(self, command: str) -> None:
        self._submit(self._send_command(command))

    def _submit(self, coroutine) -> None:
        future = asyncio.run_coroutine_threadsafe(coroutine, self.loop)

        def on_done(done_future) -> None:
            try:
                done_future.result()
            except Exception as exc:
                self.error_occurred.emit(str(exc))
                self.connected_changed.emit(False)

        future.add_done_callback(on_done)

    async def _find_device(self) -> BLEDevice:
        self.status_changed.emit(f"扫描 {self.device_identifier} ...")

        def match(device: BLEDevice, _advertisement_data) -> bool:
            if device.name == self.device_identifier:
                return True
            return device.address == self.device_identifier

        device = await BleakScanner.find_device_by_filter(match, timeout=self.scan_timeout)
        if device is None:
            raise RuntimeError(f"未找到 BLE 设备: {self.device_identifier}")
        return device

    async def _connect(self) -> None:
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
        with self.lock:
            client = self.client
            self.client = None

        if client is not None:
            try:
                if client.is_connected:
                    await client.write_gatt_char(NUS_RX_UUID, b"STOP\n", response=False)
                    await client.disconnect()
            finally:
                self.status_changed.emit("已断开。")
                self.connected_changed.emit(False)
        else:
            self.status_changed.emit("未连接。")
            self.connected_changed.emit(False)

    async def _send_command(self, command: str) -> None:
        with self.lock:
            client = self.client

        if client is None or not client.is_connected:
            raise RuntimeError("BLE 尚未连接。")

        payload = f"{command.strip()}\n".encode("utf-8")
        await client.write_gatt_char(NUS_RX_UUID, payload, response=False)
        self.command_sent.emit(command.strip())

    def _on_notify(self, _sender, data: bytearray) -> None:
        text = bytes(data).decode("utf-8", errors="replace").strip()
        if text:
            self.notification_received.emit(text)

    def _on_disconnected(self, _client: BleakClient) -> None:
        with self.lock:
            self.client = None
        self.status_changed.emit("BLE 已断开。")
        self.connected_changed.emit(False)


class MainWindow(QMainWindow):
    def __init__(self, worker: BleWorker, device_identifier: str) -> None:
        super().__init__()
        self.worker = worker
        self.device_identifier = device_identifier
        self.connected = False
        self.log_lines: deque[LogLine] = deque(maxlen=300)
        self.gear_buttons: list[QPushButton] = []

        # 默认档位缓存（与固件初始化状态保持一致）
        self.gear_profiles = {
            1: {"touch_pwm": 200, "touch_ms": 300, "pull_pwm": 400, "pull_ms": 200, "strike_pwm": 500,  "strike_ms": 200},
            2: {"touch_pwm": 200, "touch_ms": 300, "pull_pwm": 400, "pull_ms": 200, "strike_pwm": 700,  "strike_ms": 200},
            3: {"touch_pwm": 200, "touch_ms": 300, "pull_pwm": 400, "pull_ms": 200, "strike_pwm": 900,  "strike_ms": 200},
            4: {"touch_pwm": 200, "touch_ms": 300, "pull_pwm": 400, "pull_ms": 200, "strike_pwm": 1100, "strike_ms": 200},
            5: {"touch_pwm": 200, "touch_ms": 300, "pull_pwm": 400, "pull_ms": 200, "strike_pwm": 1300, "strike_ms": 200},
        }

        self.setWindowTitle("Neuro Hammer Motor Controller (配置版)")
        self.resize(760, 680)

        central = QWidget()
        self.setCentralWidget(central)
        main_layout = QVBoxLayout(central)

        # ====== 顶部：状态与连接 ======
        top_row = QHBoxLayout()
        self.status_label = QLabel(f"设备: {device_identifier} | 未连接")
        self.status_label.setTextInteractionFlags(Qt.TextSelectableByMouse)
        self.connect_button = QPushButton("连接")
        self.connect_button.clicked.connect(self.toggle_connection)
        top_row.addWidget(self.status_label, 1)
        top_row.addWidget(self.connect_button)
        main_layout.addLayout(top_row)

        # ====== 动态触发按钮区 ======
        self.trigger_layout = QGridLayout()
        main_layout.addLayout(self.trigger_layout)

        # ====== 档位参数表格 ======
        self.table = QTableWidget()
        self.table.setColumnCount(7)  # 从 5 修改为 7
        self.table.setHorizontalHeaderLabels(["档位 ID", "触碰 PWM", "触碰 ms", "回撤 PWM", "回撤 ms", "敲击 PWM", "敲击 ms"])
        self.table.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        self.table.setSelectionBehavior(QAbstractItemView.SelectRows)
        main_layout.addWidget(self.table)

        # ====== 表格操作按钮 ======
        config_row = QHBoxLayout()
        self.btn_sync = QPushButton("⬇️ 同步表格参数到设备")
        self.btn_sync.setMinimumHeight(40)
        self.btn_sync.setStyleSheet("font-weight: bold; color: #005500;")
        self.btn_add = QPushButton("➕ 新增档位")
        self.btn_add.setMinimumHeight(40)
        self.btn_del = QPushButton("❌ 删除选中档位")
        self.btn_del.setMinimumHeight(40)
        
        config_row.addWidget(self.btn_sync)
        config_row.addWidget(self.btn_add)
        config_row.addWidget(self.btn_del)
        main_layout.addLayout(config_row)

        self.btn_sync.clicked.connect(self.sync_table_to_device)
        self.btn_add.clicked.connect(self.add_gear_row)
        self.btn_del.clicked.connect(self.del_selected_gear)

        # ====== 手动控制区 ======
        manual_row = QHBoxLayout()
        self.forward_button = QPushButton("按住正转")
        self.reverse_button = QPushButton("按住反转")
        self.stop_button = QPushButton("急停")
        for button in (self.forward_button, self.reverse_button, self.stop_button):
            button.setMinimumHeight(64)

        self.forward_button.pressed.connect(lambda: self.send("MOTOR_FWD"))
        self.forward_button.released.connect(lambda: self.send("STOP", allow_disconnected=True))
        self.reverse_button.pressed.connect(lambda: self.send("MOTOR_REV"))
        self.reverse_button.released.connect(lambda: self.send("STOP", allow_disconnected=True))
        self.stop_button.clicked.connect(lambda: self.send("STOP", allow_disconnected=True))

        manual_row.addWidget(self.forward_button)
        manual_row.addWidget(self.reverse_button)
        manual_row.addWidget(self.stop_button)
        main_layout.addLayout(manual_row)

        # ====== 日志打印区 ======
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

        # 初始化渲染界面
        self.refresh_ui()
        self.set_connected(False)

    def refresh_ui(self) -> None:
        """根据当前缓存重新渲染表格数据和顶部的触发按钮"""
        # 1. 刷新表格
        self.table.setRowCount(len(self.gear_profiles))
        for row, (gear_id, p) in enumerate(sorted(self.gear_profiles.items())):
            item_id = QTableWidgetItem(str(gear_id))
            item_id.setFlags(item_id.flags() & ~Qt.ItemIsEditable) 
            item_id.setBackground(Qt.lightGray)
            
            self.table.setItem(row, 0, item_id)
            self.table.setItem(row, 1, QTableWidgetItem(str(p["touch_pwm"])))
            self.table.setItem(row, 2, QTableWidgetItem(str(p["touch_ms"])))
            self.table.setItem(row, 3, QTableWidgetItem(str(p["pull_pwm"])))
            self.table.setItem(row, 4, QTableWidgetItem(str(p["pull_ms"])))
            self.table.setItem(row, 5, QTableWidgetItem(str(p["strike_pwm"])))
            self.table.setItem(row, 6, QTableWidgetItem(str(p["strike_ms"])))
        # ... 后面的刷新按钮代码保持不变

        # 2. 刷新顶部触发按钮区
        # 先清空原有按钮
        while self.trigger_layout.count():
            item = self.trigger_layout.takeAt(0)
            widget = item.widget()
            if widget:
                widget.deleteLater()
        self.gear_buttons.clear()

        # 按每行5个自动排布
        col = 0
        row_idx = 0
        for gear_id in sorted(self.gear_profiles.keys()):
            btn = QPushButton(f"⚡ 档位 {gear_id}")
            btn.setMinimumHeight(48)
            btn.setEnabled(self.connected)
            btn.clicked.connect(lambda _checked=False, g=gear_id: self.send(f"STRIKE_{g}"))
            self.trigger_layout.addWidget(btn, row_idx, col)
            self.gear_buttons.append(btn)
            col += 1
            if col > 4:
                col = 0
                row_idx += 1

    def sync_table_to_device(self) -> None:
        """遍历表格，保存到字典并通过 BLE 下发修改命令"""
        if not self.connected:
            QMessageBox.warning(self, "未连接", "请先连接设备后再同步参数。")
            return
            
        success_count = 0
        for row in range(self.table.rowCount()):
            try:
                g_id = int(self.table.item(row, 0).text())
                t_pwm = int(self.table.item(row, 1).text())
                t_ms = int(self.table.item(row, 2).text())
                p_pwm = int(self.table.item(row, 3).text())
                p_ms = int(self.table.item(row, 4).text())
                s_pwm = int(self.table.item(row, 5).text())
                s_ms = int(self.table.item(row, 6).text())
                
                # 存入本地缓存
                self.gear_profiles[g_id] = {
                    "touch_pwm": t_pwm, "touch_ms": t_ms,
                    "pull_pwm": p_pwm, "pull_ms": p_ms,
                    "strike_pwm": s_pwm, "strike_ms": s_ms
                }
                
                # 下发指令到硬件 (现在有 6 个参数，加上档位号共 7 个)
                self.send(f"SET_STRIKE {g_id} {t_pwm} {t_ms} {p_pwm} {p_ms} {s_pwm} {s_ms}")
                success_count += 1
            except (ValueError, AttributeError):
                self.append_log("UI", f"警告: 表格第 {row+1} 行数据格式错误，跳过同步。")

        self.append_log("UI", f"已向硬件同步了 {success_count} 个档位的参数。")

    def add_gear_row(self) -> None:
        """在缓存中追加一个新档位，并刷新 UI"""
        new_id = 1
        if self.gear_profiles:
            new_id = max(self.gear_profiles.keys()) + 1
        
        # 默认参数增加触碰阶段
        self.gear_profiles[new_id] = {
            "touch_pwm": 200, "touch_ms": 100,
            "pull_pwm": 500, "pull_ms": 200, 
            "strike_pwm": 500, "strike_ms": 100
        }
        self.refresh_ui()
        self.table.scrollToBottom()

    def del_selected_gear(self) -> None:
        """删除表格中高亮的行对应的档位"""
        selected_rows = set(item.row() for item in self.table.selectedItems())
        if not selected_rows:
            QMessageBox.information(self, "提示", "请点击表格，选中你要删除的行。")
            return
            
        # 从下往上删避免行号乱序
        for row in sorted(selected_rows, reverse=True):
            g_id = int(self.table.item(row, 0).text())
            if g_id in self.gear_profiles:
                del self.gear_profiles[g_id]
                # 只有连接状态下才发送删除指令
                if self.connected:
                    self.send(f"DEL_STRIKE {g_id}")
        self.refresh_ui()

    def closeEvent(self, event) -> None:
        self.worker.close()
        event.accept()

    def toggle_connection(self) -> None:
        if self.connected:
            self.worker.disconnect()
            self.on_status("正在断开 ...")
        else:
            self.worker.connect()
            self.on_status("正在连接 ...")

    def send(self, command: str, allow_disconnected: bool = False) -> None:
        if not self.connected and not allow_disconnected:
            self.append_log("UI", "尚未连接，命令未发送。")
            return
        self.worker.send_command(command)

    def set_connected(self, connected: bool) -> None:
        self.connected = connected
        self.connect_button.setText("断开" if connected else "连接")
        for button in self.gear_buttons:
            button.setEnabled(connected)
        self.forward_button.setEnabled(connected)
        self.reverse_button.setEnabled(connected)
        self.stop_button.setEnabled(connected)
        self.btn_sync.setEnabled(connected)
        
        if connected:
            self.append_log("UI", "提示：设备刚连接时，建议点击【同步表格参数到设备】下发最新配置。")

    def on_status(self, text: str) -> None:
        self.status_label.setText(f"设备: {self.device_identifier} | {text}")
        self.append_log("STATUS", text)

    def on_notification(self, text: str) -> None:
        self.append_log("RX", text)

    def on_command_sent(self, command: str) -> None:
        self.append_log("TX", command)

    def on_error(self, text: str) -> None:
        self.append_log("ERR", text)
        self.status_label.setText(f"设备: {self.device_identifier} | 错误: {text}")

    def append_log(self, source: str, text: str) -> None:
        self.log_lines.append(LogLine(time.time(), f"[{source}] {text}"))

    def refresh_log(self) -> None:
        current = self.log_view.toPlainText()
        rendered = "\n".join(
            f"{time.strftime('%H:%M:%S', time.localtime(line.ts))} {line.text}"
            for line in self.log_lines
        )
        if rendered != current:
            self.log_view.setPlainText(rendered)
            self.log_view.verticalScrollBar().setValue(self.log_view.verticalScrollBar().maximum())


def main() -> int:
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