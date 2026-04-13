#!/usr/bin/env python3
"""
Minimal EtherCAT HMI — Step 1: Connect + Read Motor Values
"""

import sys
import os
from PyQt6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QGridLayout, QPushButton, QLabel, QTableWidget, QTableWidgetItem,
    QGroupBox, QHeaderView, QMessageBox, QDoubleSpinBox, QCheckBox,
    QTabWidget,
)
from PyQt6.QtCore import QTimer, Qt
from PyQt6.QtGui import QFont

from shm_api import ShmAPI, eRTXState, eCmdSource, eCmdFormat


class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("EtherCAT HMI — 馬達監控")
        self.resize(700, 400)

        self.api = ShmAPI()
        self.connected = False
        self.motor_num = 0

        # --- Config ---
        self._load_config()

        # --- UI ---
        central = QWidget()
        self.setCentralWidget(central)
        layout = QVBoxLayout(central)

        # Status bar
        status_group = QGroupBox("系統狀態")
        status_layout = QHBoxLayout(status_group)
        self.lbl_rtx = QLabel("RTXState: --")
        self.lbl_ecat = QLabel("EcatState: --")
        self.lbl_slaves = QLabel("Slaves: --")
        for lbl in (self.lbl_rtx, self.lbl_ecat, self.lbl_slaves):
            lbl.setFont(QFont("Monospace", 11))
            status_layout.addWidget(lbl)
        layout.addWidget(status_group)

        # Buttons
        btn_layout = QHBoxLayout()
        self.btn_connect = QPushButton("連接 EtherCAT")
        self.btn_connect.setFixedHeight(40)
        self.btn_connect.clicked.connect(self.on_connect)
        btn_layout.addWidget(self.btn_connect)

        self.btn_stop = QPushButton("停止 RTX")
        self.btn_stop.setFixedHeight(40)
        self.btn_stop.setEnabled(False)
        self.btn_stop.clicked.connect(self.on_stop)
        btn_layout.addWidget(self.btn_stop)
        layout.addLayout(btn_layout)

        # Servo ON/OFF buttons
        servo_layout = QHBoxLayout()
        self.btn_servo_on = QPushButton("Servo ON")
        self.btn_servo_on.setFixedHeight(40)
        self.btn_servo_on.setEnabled(False)
        self.btn_servo_on.setStyleSheet("background-color: #4CAF50; color: white; font-weight: bold;")
        self.btn_servo_on.clicked.connect(self.on_servo_on)
        servo_layout.addWidget(self.btn_servo_on)

        self.btn_servo_off = QPushButton("Servo OFF")
        self.btn_servo_off.setFixedHeight(40)
        self.btn_servo_off.setEnabled(False)
        self.btn_servo_off.setStyleSheet("background-color: #f44336; color: white; font-weight: bold;")
        self.btn_servo_off.clicked.connect(self.on_servo_off)
        servo_layout.addWidget(self.btn_servo_off)
        layout.addLayout(servo_layout)

        # --- Jog Control ---
        self._build_jog_panel(layout)

        # Motor data table
        motor_group = QGroupBox("馬達即時數值")
        motor_layout = QVBoxLayout(motor_group)
        self.table = QTableWidget()
        self.table.setColumnCount(7)
        self.table.setHorizontalHeaderLabels([
            "軸", "StatusWord", "角度 (deg)", "速度 (deg/s)", "扭矩", "ServoOn", "Servo 控制"
        ])
        self.table.horizontalHeader().setSectionResizeMode(QHeaderView.ResizeMode.Stretch)
        self.table.setEditTriggers(QTableWidget.EditTrigger.NoEditTriggers)
        motor_layout.addWidget(self.table)
        layout.addWidget(motor_group)

        # Per-axis servo toggle buttons (created after table rows are set up)
        self.servo_toggle_btns = []

        # Polling timer
        self.poll_timer = QTimer()
        self.poll_timer.timeout.connect(self.poll_data)

    def _load_config(self):
        """Load robot config to get motor count."""
        import json
        cfg_path = os.path.join(os.path.dirname(__file__), "robot_config.json")
        with open(cfg_path, "r") as f:
            cfg = json.load(f)
        rob = cfg["robots"][0]
        self.motor_num = rob["motorNum"]

    def on_connect(self):
        """Connect to shared memory, send mapping config, wait for OP."""
        if self.connected:
            return

        # 1) Init shared memory
        if not self.api.init():
            QMessageBox.critical(self, "錯誤", "無法開啟共享記憶體")
            return

        # 2) Check RTXState
        state = self.api.get_rtx_state()
        if state == eRTXState.CLOSE:
            QMessageBox.warning(self, "提示", "ecatApp 尚未啟動，請先執行 ecatApp")
            self.api.close()
            return

        # 3) Get slave info
        motor_num, io_num = self.api.get_slave_info()
        self.lbl_slaves.setText(f"Slaves: motor={motor_num}, io={io_num}")

        # 4) Send robot config (declare + mapping + HMI ready)
        self.api.send_robot_config()

        self.connected = True
        self.btn_connect.setEnabled(False)
        self.btn_stop.setEnabled(True)
        self.btn_servo_on.setEnabled(True)
        self.btn_servo_off.setEnabled(True)
        for btn in self.jog_btns:
            btn.setEnabled(True)

        # Setup table rows + per-axis servo toggle buttons
        self.table.setRowCount(self.motor_num)
        self.servo_toggle_btns.clear()
        for m in range(self.motor_num):
            self.table.setItem(m, 0, QTableWidgetItem(f"M{m}"))
            btn = QPushButton("OFF")
            btn.setStyleSheet("background-color: #f44336; color: white; font-weight: bold;")
            btn.clicked.connect(lambda checked, axis=m: self._toggle_servo(axis))
            self.table.setCellWidget(m, 6, btn)
            self.servo_toggle_btns.append(btn)

        # Start polling at 10 Hz
        self.poll_timer.start(100)

    def on_stop(self):
        """Stop RTX process."""
        if not self.connected:
            return
        # Servo OFF all motors first
        self.api.stop_jog()
        for m in range(self.motor_num):
            self.api.set_servo_on(0, m, False)
        reply = QMessageBox.question(
            self, "確認", "確定要停止 EtherCAT？",
            QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No,
        )
        if reply == QMessageBox.StandardButton.Yes:
            self.poll_timer.stop()
            self.api.stop_rtx_process()
            self.connected = False
            self.btn_connect.setEnabled(True)
            self.btn_stop.setEnabled(False)
            self.btn_servo_on.setEnabled(False)
            self.btn_servo_off.setEnabled(False)
            for btn in self.jog_btns:
                btn.setEnabled(False)
            self.lbl_rtx.setText("RTXState: STOPPED")
            self.api.close()

    def on_servo_on(self):
        """Send Servo ON command for all motors."""
        if not self.connected:
            return
        for m in range(self.motor_num):
            self.api.set_servo_on(0, m, True)

    def on_servo_off(self):
        """Send Servo OFF command for all motors."""
        if not self.connected:
            return
        for m in range(self.motor_num):
            self.api.set_servo_on(0, m, False)

    def _toggle_servo(self, axis: int):
        """Toggle Servo ON/OFF for a single axis."""
        if not self.connected:
            return
        current = self.api.get_is_servo_on(0, axis)
        self.api.set_servo_on(0, axis, not current)

    # ── Jog Panel Builder ───────────────────────────────────────────────
    def _build_jog_panel(self, parent_layout):
        """Build the Jog control UI section."""
        jog_group = QGroupBox("Jog 控制 — 馬達轉動")
        jog_layout = QVBoxLayout(jog_group)

        # Row 1: velocity, acceleration settings
        settings_layout = QGridLayout()

        settings_layout.addWidget(QLabel("速度 (deg/cycle):"), 0, 0)
        self.spin_jog_vel = QDoubleSpinBox()
        self.spin_jog_vel.setRange(0.001, 10.0)
        self.spin_jog_vel.setDecimals(3)
        self.spin_jog_vel.setValue(0.01)
        self.spin_jog_vel.setSingleStep(0.005)
        self.spin_jog_vel.setToolTip("1kHz 下: 0.01 = 10 deg/s, 0.1 = 100 deg/s")
        settings_layout.addWidget(self.spin_jog_vel, 0, 1)

        settings_layout.addWidget(QLabel("加速度:"), 0, 2)
        self.spin_jog_acc = QDoubleSpinBox()
        self.spin_jog_acc.setRange(0.0001, 1.0)
        self.spin_jog_acc.setDecimals(4)
        self.spin_jog_acc.setValue(0.001)
        self.spin_jog_acc.setSingleStep(0.0005)
        settings_layout.addWidget(self.spin_jog_acc, 0, 3)

        self.chk_limit_dist = QCheckBox("限制距離")
        settings_layout.addWidget(self.chk_limit_dist, 0, 4)

        settings_layout.addWidget(QLabel("距離 (deg):"), 0, 5)
        self.spin_jog_dist = QDoubleSpinBox()
        self.spin_jog_dist.setRange(0, 9999)
        self.spin_jog_dist.setDecimals(2)
        self.spin_jog_dist.setValue(10.0)
        settings_layout.addWidget(self.spin_jog_dist, 0, 6)

        jog_layout.addLayout(settings_layout)

        # Row 2: per-axis Jog buttons (press-and-hold)
        axis_layout = QHBoxLayout()
        self.jog_btns = []
        for m in range(self.motor_num):
            axis_layout.addWidget(QLabel(f"M{m}"))
            btn_pos = QPushButton(f"+")
            btn_pos.setFixedSize(50, 40)
            btn_pos.setStyleSheet("font-weight: bold; font-size: 16px;")
            btn_pos.pressed.connect(lambda m=m: self._jog_start(m, 1))
            btn_pos.released.connect(self._jog_stop)
            axis_layout.addWidget(btn_pos)

            btn_neg = QPushButton(f"−")
            btn_neg.setFixedSize(50, 40)
            btn_neg.setStyleSheet("font-weight: bold; font-size: 16px;")
            btn_neg.pressed.connect(lambda m=m: self._jog_start(m, -1))
            btn_neg.released.connect(self._jog_stop)
            axis_layout.addWidget(btn_neg)
            self.jog_btns.extend([btn_pos, btn_neg])

        # Emergency stop
        btn_jog_stop = QPushButton("停止 JOG")
        btn_jog_stop.setFixedHeight(40)
        btn_jog_stop.setStyleSheet(
            "background-color: #cc0000; color: white; font-weight: bold;"
        )
        btn_jog_stop.clicked.connect(self._jog_stop)
        axis_layout.addWidget(btn_jog_stop)

        jog_layout.addLayout(axis_layout)
        parent_layout.addWidget(jog_group)

        # Disable jog buttons initially
        for btn in self.jog_btns:
            btn.setEnabled(False)

    def _jog_start(self, axis: int, direction: int):
        """Start jog: set cmd source/format, then send jog command."""
        if not self.connected:
            return
        vel = self.spin_jog_vel.value() * direction
        acc = self.spin_jog_acc.value()
        dist = self.spin_jog_dist.value()
        limit = self.chk_limit_dist.isChecked()

        self.api.set_cmd_source(eCmdSource.JOG)
        self.api.set_cmd_format(eCmdFormat.AXIS)
        self.api.set_jog_acc(acc)
        self.api.set_jog_cmd_single(0, axis, vel, dist, limit)

    def _jog_stop(self):
        """Stop jog."""
        if not self.connected:
            return
        self.api.stop_jog()

    def poll_data(self):
        """Read motor data from shared memory and update table."""
        if not self.connected:
            return

        # Update status
        rtx = self.api.get_rtx_state()
        ecat = self.api.get_ecat_state()
        rtx_names = {0: "CLOSE", 1: "OPEN", 2: "RUNNING"}
        ecat_names = {0: "--", 1: "INIT", 2: "PREOP", 4: "SAFEOP", 8: "OP"}
        self.lbl_rtx.setText(f"RTXState: {rtx_names.get(rtx, rtx)}")
        self.lbl_ecat.setText(f"EcatState: {ecat_names.get(ecat, ecat)}")

        # Read motor values
        r = 0  # robot index
        axis_deg = self.api.get_axis_deg(r)
        velocity = self.api.get_velocity(r)
        torque = self.api.get_torque(r)

        for m in range(self.motor_num):
            sw = self.api.get_status_word(r, m)
            servo_on = self.api.get_is_servo_on(r, m)

            self._set_cell(m, 1, f"0x{sw:04X}")
            self._set_cell(m, 2, f"{axis_deg[m]:.4f}")
            self._set_cell(m, 3, f"{velocity[m]:.2f}")
            self._set_cell(m, 4, f"{torque[m]:.1f}")
            self._set_cell(m, 5, "ON" if servo_on else "OFF")
            if m < len(self.servo_toggle_btns):
                btn = self.servo_toggle_btns[m]
                if servo_on:
                    btn.setText("ON")
                    btn.setStyleSheet("background-color: #4CAF50; color: white; font-weight: bold;")
                else:
                    btn.setText("OFF")
                    btn.setStyleSheet("background-color: #f44336; color: white; font-weight: bold;")

    def _set_cell(self, row, col, text):
        item = self.table.item(row, col)
        if item is None:
            item = QTableWidgetItem(text)
            item.setTextAlignment(Qt.AlignmentFlag.AlignCenter)
            self.table.setItem(row, col, item)
        else:
            item.setText(text)

    def closeEvent(self, event):
        self.poll_timer.stop()
        if self.connected:
            self.api.close()
        event.accept()


def main():
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
