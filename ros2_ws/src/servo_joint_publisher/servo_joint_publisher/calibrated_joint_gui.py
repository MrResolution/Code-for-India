#!/usr/bin/env python3
"""
ROS 2 Node: calibrated_joint_gui
---------------------------------
An interactive PyQt5 Dashboard with:
  • Live Connection Status Monitor Badge (🟢 Connected / 🔴 Disconnected).
  • Integrated Serial & Wi-Fi Network Console Monitor with manual command input.
  • Integrated RViz 3D Viewport container embedded side-by-side.
  • "⚡ Sim Speed": Adjustable simulation speed slider (0.1x to 5.0x).
  • "🏠 Go Home": Smooth, slow homing movement from any joint position over 2.0s.
  • "Set Low" and "Set High" calibration buttons next to every joint slider.
  • Live Simulation Mode for top 3 joints with automatic limit enforcement.
  • Min/Max numeric spinboxes for fine tuning limit bounds.
  • "Save Config": Persists calibration limits & home pose to servo_calibration.json.
  • Transmits live limit changes over /servo_calibration topic to ESP32.
"""

import sys
import os
import json
import math
import time
import subprocess
import xml.etree.ElementTree as ET
import urllib.request
import urllib.parse
import threading

from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QLabel, QSlider, QDoubleSpinBox, QPushButton, QGroupBox,
    QScrollArea, QMessageBox, QSplitter, QTextEdit, QLineEdit,
    QTabWidget, QGridLayout
)
from PyQt5.QtCore import Qt, QTimer, QProcess, QProcessEnvironment, pyqtSignal
from PyQt5.QtGui import QFont, QWindow, QTextCursor

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String as StringMsg

CALIB_FILE_PATH = "/home/chakradhar/Documents/Hardware/servo_calibration.json"
URDF_PATH = "/home/chakradhar/Documents/Hardware/urdf/unnamed/urdf/unnamed_gazebo.urdf"

# Default Homing Pose requested by user for the 4 URDF arm joints:
DEFAULT_HOME_DEGS = {
    'turntable_link_joint_dup': 90.0,  # Base Turntable Yaw
    'turntable_link_joint': 59.0,      # Shoulder Pitch (Dual Servos: GPIO 19 & 21)
    'turntable_link_joint_dup_1': 153.1, # Elbow 1 Pitch
    'turntable_link_joint_dup_2': 116.0, # Elbow 2 Pitch
    'wrist_twist_joint': 90.0           # Wrist Twist / Roll
}


class CalibratedJointWidget(QGroupBox):
    """Widget row for a single joint with slider, readouts, Set Low/Set High, and Homing support."""

    def __init__(self, joint_name, rad_min, rad_max, calib_data, parent=None):
        super().__init__(parent)
        self.joint_name = joint_name
        self.rad_min = rad_min
        self.rad_max = rad_max
        self.gui_parent = parent

        def_home = DEFAULT_HOME_DEGS.get(joint_name, 90.0)

        self.calib = calib_data.get(joint_name, {
            'servo_min_deg': 0.0,
            'servo_max_deg': 180.0,
            'home_deg': def_home,
            'trim_deg': 0.0,
            'invert': False
        })
        if 'home_deg' not in self.calib:
            self.calib['home_deg'] = def_home

        if joint_name == 'turntable_link_joint':
            self.setTitle(f"🦾 Joint: {joint_name} (Shoulder Pitch — Dual Servos: GPIO 19 & 21)")
        elif joint_name == 'turntable_link_joint_dup':
            self.setTitle(f"🦾 Joint: {joint_name} (Base Turntable Yaw)")
        elif joint_name == 'turntable_link_joint_dup_1':
            self.setTitle(f"🦾 Joint: {joint_name} (Elbow 1 Pitch)")
        elif joint_name == 'turntable_link_joint_dup_2':
            self.setTitle(f"🦾 Joint: {joint_name} (Elbow 2 Pitch)")
        elif joint_name == 'wrist_twist_joint':
            self.setTitle(f"🦾 Joint: {joint_name} (Wrist Twist / Roll)")
        else:
            self.setTitle(f"🦾 Joint: {joint_name}")
        self.setStyleSheet("""
            QGroupBox {
                font-weight: bold;
                font-size: 13px;
                border: 1px solid #444;
                border-radius: 6px;
                margin-top: 8px;
                padding-top: 10px;
                background-color: #2b2b2b;
                color: #e0e0e0;
            }
            QGroupBox::title {
                subcontrol-origin: margin;
                subcontrol-position: top left;
                padding: 0 5px;
                color: #4da6ff;
            }
        """)

        layout = QVBoxLayout()

        # ── Row 1: Joint Name & Readout ──────────────────────────────────
        row1 = QHBoxLayout()
        self.label_name = QLabel(joint_name)
        self.label_name.setFont(QFont("Monospace", 10, QFont.Bold))
        self.label_name.setStyleSheet("color: #ffffff;")

        self.label_val = QLabel("90.0° (0.000 rad)")
        self.label_val.setFont(QFont("Monospace", 10, QFont.Bold))
        self.label_val.setStyleSheet("color: #00ffcc; background: #1e1e1e; padding: 3px 8px; border-radius: 4px;")

        row1.addWidget(self.label_name)
        row1.addStretch()
        row1.addWidget(self.label_val)
        layout.addLayout(row1)

        # ── Row 2: Joint Position Slider (0° to 180°) ─────────────────────
        row2 = QHBoxLayout()
        self.slider = QSlider(Qt.Horizontal)
        self.slider.setRange(0, 1800) # 0.0 to 180.0 deg precision

        init_home_val = int(self.calib['home_deg'] * 10)
        self.slider.setValue(init_home_val)

        self.slider.setStyleSheet("""
            QSlider::groove:horizontal {
                border: 1px solid #555;
                height: 8px;
                background: #3a3a3a;
                border-radius: 4px;
            }
            QSlider::sub-page:horizontal {
                background: #007acc;
                border-radius: 4px;
            }
            QSlider::handle:horizontal {
                background: #ffffff;
                border: 1px solid #777;
                width: 18px;
                margin-top: -5px;
                margin-bottom: -5px;
                border-radius: 9px;
            }
        """)
        self.slider.valueChanged.connect(self.on_slider_changed)
        row2.addWidget(self.slider)
        layout.addLayout(row2)

        # ── Row 3: Limits & "Set Low" / "Set High" Buttons ───────────────
        row3 = QHBoxLayout()

        lbl_min = QLabel("Min:")
        lbl_min.setStyleSheet("color: #aaaaaa;")
        self.spin_min = QDoubleSpinBox()
        self.spin_min.setRange(0.0, 180.0)
        self.spin_min.setValue(self.calib['servo_min_deg'])
        self.spin_min.setSingleStep(1.0)
        self.spin_min.setSuffix("°")
        self.spin_min.setStyleSheet("background: #1e1e1e; color: #ff6666; font-weight: bold;")
        self.spin_min.valueChanged.connect(self.on_limit_changed)

        self.btn_set_low = QPushButton("📍 Set Low")
        self.btn_set_low.setToolTip("Set current slider position as Minimum Safety Limit")
        self.btn_set_low.setStyleSheet("""
            QPushButton { background-color: #8b0000; color: white; font-weight: bold; border-radius: 4px; padding: 4px 8px; }
            QPushButton:hover { background-color: #a52a2a; }
            QPushButton:pressed { background-color: #660000; }
        """)
        self.btn_set_low.clicked.connect(self.set_low_limit)

        lbl_max = QLabel("Max:")
        lbl_max.setStyleSheet("color: #aaaaaa;")
        self.spin_max = QDoubleSpinBox()
        self.spin_max.setRange(0.0, 180.0)
        self.spin_max.setValue(self.calib['servo_max_deg'])
        self.spin_max.setSingleStep(1.0)
        self.spin_max.setSuffix("°")
        self.spin_max.setStyleSheet("background: #1e1e1e; color: #66ff66; font-weight: bold;")
        self.spin_max.valueChanged.connect(self.on_limit_changed)

        self.btn_set_high = QPushButton("📍 Set High")
        self.btn_set_high.setToolTip("Set current slider position as Maximum Safety Limit")
        self.btn_set_high.setStyleSheet("""
            QPushButton { background-color: #006400; color: white; font-weight: bold; border-radius: 4px; padding: 4px 8px; }
            QPushButton:hover { background-color: #2e8b57; }
            QPushButton:pressed { background-color: #004d00; }
        """)
        self.btn_set_high.clicked.connect(self.set_high_limit)

        row3.addWidget(lbl_min)
        row3.addWidget(self.spin_min)
        row3.addWidget(self.btn_set_low)
        row3.addSpacing(15)
        row3.addWidget(lbl_max)
        row3.addWidget(self.spin_max)
        row3.addWidget(self.btn_set_high)
        layout.addLayout(row3)

        self.setLayout(layout)
        self.on_slider_changed(self.slider.value())

    def get_current_deg(self):
        return self.slider.value() / 10.0

    def get_current_rad(self):
        deg = self.get_current_deg()
        ratio = deg / 180.0
        return self.rad_min + ratio * (self.rad_max - self.rad_min)

    def set_deg_programmatically(self, deg):
        deg_clamped = max(self.spin_min.value(), min(self.spin_max.value(), deg))
        self.slider.setValue(int(deg_clamped * 10))

    def set_current_as_home(self):
        curr_d = self.get_current_deg()
        self.calib['home_deg'] = curr_d

    def on_slider_changed(self, value):
        deg = value / 10.0
        min_lim = self.spin_min.value()
        max_lim = self.spin_max.value()

        if deg < min_lim:
            self.slider.blockSignals(True)
            self.slider.setValue(int(min_lim * 10))
            self.slider.blockSignals(False)
            deg = min_lim
        elif deg > max_lim:
            self.slider.blockSignals(True)
            self.slider.setValue(int(max_lim * 10))
            self.slider.blockSignals(False)
            deg = max_lim

        rad = self.get_current_rad()
        self.label_val.setText(f"{deg:.1f}° ({rad:.3f} rad)")

    def set_low_limit(self):
        curr = self.get_current_deg()
        max_lim = self.spin_max.value()
        if curr >= max_lim:
            curr = max_lim - 1.0
        self.spin_min.setValue(curr)
        self.on_limit_changed()

    def set_high_limit(self):
        curr = self.get_current_deg()
        min_lim = self.spin_min.value()
        if curr <= min_lim:
            curr = min_lim + 1.0
        self.spin_max.setValue(curr)
        self.on_limit_changed()

    def on_limit_changed(self):
        min_val = self.spin_min.value()
        max_val = self.spin_max.value()

        if min_val >= max_val:
            self.spin_max.setValue(min_val + 1.0)
            max_val = min_val + 1.0

        self.calib['servo_min_deg'] = min_val
        self.calib['servo_max_deg'] = max_val

        self.on_slider_changed(self.slider.value())

        if self.gui_parent:
            self.gui_parent.on_joint_calib_changed(self.joint_name, min_val, max_val)


class QuardBotWidget(QWidget):
    """Dedicated PyQt5 Tab Widget for Quard Bot Locomotion & Servo Control (NO 3D Viewport)."""
    update_log_signal = pyqtSignal(str)
    update_status_signal = pyqtSignal(bool, str)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.quard_host = "http://sesame-robot.local"
        self.is_online = False
        self.init_ui()
        self.update_log_signal.connect(self.log_to_console)
        self.update_status_signal.connect(self.set_status)

        # Periodic Heartbeat check
        self.ping_timer = QTimer(self)
        self.ping_timer.timeout.connect(self.ping_quard_bot)
        self.ping_timer.start(5000)
        QTimer.singleShot(800, self.ping_quard_bot)

    def init_ui(self):
        main_layout = QHBoxLayout(self)
        main_layout.setContentsMargins(12, 12, 12, 12)
        main_layout.setSpacing(12)

        # ── Left Column (Control Cards): Width 480px ─────────────────────
        left_col = QWidget()
        left_layout = QVBoxLayout(left_col)
        left_layout.setContentsMargins(0, 0, 0, 0)
        left_layout.setSpacing(10)

        # 1. Network Connection Card
        conn_box = QGroupBox("🌐 Quard Bot Network Target")
        conn_box.setStyleSheet("""
            QGroupBox { font-weight: bold; font-size: 13px; border: 1px solid #444; border-radius: 6px; margin-top: 6px; padding-top: 10px; background-color: #2b2b2b; color: #4da6ff; }
            QGroupBox::title { subcontrol-origin: margin; padding: 0 5px; color: #4da6ff; }
        """)
        conn_layout = QHBoxLayout()
        conn_layout.setContentsMargins(8, 8, 8, 8)

        self.input_host = QLineEdit(self.quard_host)
        self.input_host.setFont(QFont("Monospace", 9))
        self.input_host.setStyleSheet("background-color: #18181b; color: #ffffff; border: 1px solid #3f3f46; border-radius: 4px; padding: 4px 8px;")

        self.lbl_status = QLabel("🔴 Offline")
        self.lbl_status.setFont(QFont("SansSerif", 9, QFont.Bold))
        self.lbl_status.setStyleSheet("background-color: #4c0519; color: #f43f5e; padding: 4px 10px; border-radius: 4px;")

        btn_ping = QPushButton("🔄 Ping")
        btn_ping.setStyleSheet("QPushButton { background-color: #0284c7; color: white; font-weight: bold; border-radius: 4px; padding: 4px 10px; } QPushButton:hover { background-color: #0369a1; }")
        btn_ping.clicked.connect(self.ping_quard_bot)

        conn_layout.addWidget(self.input_host, 1)
        conn_layout.addWidget(self.lbl_status)
        conn_layout.addWidget(btn_ping)
        conn_box.setLayout(conn_layout)
        left_layout.addWidget(conn_box)

        # 2. D-Pad Locomotion Controller Card
        dpad_box = QGroupBox("🎮 Locomotion Controller (WASD / Arrow Keys)")
        dpad_box.setStyleSheet("""
            QGroupBox { font-weight: bold; font-size: 13px; border: 1px solid #444; border-radius: 6px; margin-top: 6px; padding-top: 10px; background-color: #2b2b2b; color: #f59e0b; }
            QGroupBox::title { subcontrol-origin: margin; padding: 0 5px; color: #f59e0b; }
        """)
        dpad_grid = QGridLayout()
        dpad_grid.setContentsMargins(10, 10, 10, 10)
        dpad_grid.setSpacing(8)

        btn_fw = QPushButton("▲\nForward (W)")
        btn_bk = QPushButton("▼\nBackward (S)")
        btn_lt = QPushButton("◀\nLeft (A)")
        btn_rt = QPushButton("▶\nRight (D)")
        btn_st = QPushButton("⏹\nStand (Space)")

        dpad_style = """
            QPushButton { background-color: #3f3f46; color: #ffffff; font-weight: bold; border: 1px solid #52525b; border-radius: 8px; min-width: 90px; min-height: 55px; }
            QPushButton:hover { background-color: #0284c7; border-color: #38bdf8; }
            QPushButton:pressed { background-color: #0369a1; }
        """
        btn_st_style = """
            QPushButton { background-color: #7f1d1d; color: #fca5a5; font-weight: bold; border: 1px solid #991b1b; border-radius: 8px; min-width: 90px; min-height: 55px; }
            QPushButton:hover { background-color: #dc2626; color: #ffffff; }
        """
        for b in [btn_fw, btn_bk, btn_lt, btn_rt]:
            b.setStyleSheet(dpad_style)
        btn_st.setStyleSheet(btn_st_style)

        btn_fw.clicked.connect(lambda: self.send_command({'mode': 'forward'}))
        btn_bk.clicked.connect(lambda: self.send_command({'mode': 'backward'}))
        btn_lt.clicked.connect(lambda: self.send_command({'mode': 'left'}))
        btn_rt.clicked.connect(lambda: self.send_command({'mode': 'right'}))
        btn_st.clicked.connect(lambda: self.send_command({'mode': 'stand'}))

        dpad_grid.addWidget(btn_fw, 0, 1)
        dpad_grid.addWidget(btn_lt, 1, 0)
        dpad_grid.addWidget(btn_st, 1, 1)
        dpad_grid.addWidget(btn_rt, 1, 2)
        dpad_grid.addWidget(btn_bk, 2, 1)
        dpad_box.setLayout(dpad_grid)
        left_layout.addWidget(dpad_box)

        # 3. Quick Stances & OLED Face Expression Card
        presets_box = QGroupBox("⚡ Quick Stances & OLED Face Animations")
        presets_box.setStyleSheet("""
            QGroupBox { font-weight: bold; font-size: 13px; border: 1px solid #444; border-radius: 6px; margin-top: 6px; padding-top: 10px; background-color: #2b2b2b; color: #34d399; }
            QGroupBox::title { subcontrol-origin: margin; padding: 0 5px; color: #34d399; }
        """)
        presets_layout = QVBoxLayout()
        presets_layout.setContentsMargins(8, 8, 8, 8)

        stances_row = QHBoxLayout()
        btn_p_stand = QPushButton("🧍 Stand (45°)")
        btn_p_zero  = QPushButton("🎯 Base Zero (0°)")
        btn_p_high  = QPushButton("🦒 High Stance (60°)")
        btn_p_wave  = QPushButton("👋 Wave Animation")

        p_style = "QPushButton { background-color: #18181b; color: #e4e4e7; border: 1px solid #3f3f46; border-radius: 4px; padding: 6px; font-weight: bold; } QPushButton:hover { background-color: #27272a; color: #38bdf8; }"
        for b in [btn_p_stand, btn_p_zero, btn_p_high, btn_p_wave]:
            b.setStyleSheet(p_style)

        btn_p_stand.clicked.connect(lambda: self.send_command({'mode': 'stand'}))
        btn_p_zero.clicked.connect(lambda: self.send_command({'all': 0}))
        btn_p_high.clicked.connect(lambda: self.send_command({'all': 60}))
        btn_p_wave.clicked.connect(lambda: self.send_command({'mode': 'wave'}))

        stances_row.addWidget(btn_p_stand)
        stances_row.addWidget(btn_p_zero)
        stances_row.addWidget(btn_p_high)
        stances_row.addWidget(btn_p_wave)
        presets_layout.addLayout(stances_row)

        faces_row = QHBoxLayout()
        lbl_face = QLabel("OLED Face:")
        lbl_face.setStyleSheet("font-weight: bold; color: #a1a1aa;")
        faces_row.addWidget(lbl_face)

        faces = [("😊 Happy", "happy"), ("🚶 Walk", "walk"), ("👋 Wave", "wave"), ("😴 Sleepy", "sleepy"), ("✨ Cute", "cute")]
        for label_text, face_code in faces:
            b_face = QPushButton(label_text)
            b_face.setStyleSheet("QPushButton { background-color: #27272a; color: #f4f4f5; border: 1px solid #52525b; border-radius: 12px; padding: 4px 8px; font-size: 11px; } QPushButton:hover { background-color: #38bdf8; color: #000000; }")
            b_face.clicked.connect(lambda checked, f=face_code: self.send_command({'face': f}))
            faces_row.addWidget(b_face)

        presets_layout.addLayout(faces_row)
        presets_box.setLayout(presets_layout)
        left_layout.addWidget(presets_box)
        left_layout.addStretch()

        # ── Right Column (PCA9685 Sliders + Console Log) ─────────────────
        right_col = QWidget()
        right_layout = QVBoxLayout(right_col)
        right_layout.setContentsMargins(0, 0, 0, 0)
        right_layout.setSpacing(10)

        # 4. PCA9685 8-Channel Sliders Box
        pca_box = QGroupBox("🦾 PCA9685 8-Channel Live Servo Controls")
        pca_box.setStyleSheet("""
            QGroupBox { font-weight: bold; font-size: 13px; border: 1px solid #444; border-radius: 6px; margin-top: 6px; padding-top: 10px; background-color: #2b2b2b; color: #a855f7; }
            QGroupBox::title { subcontrol-origin: margin; padding: 0 5px; color: #a855f7; }
        """)
        pca_layout = QVBoxLayout()
        pca_layout.setContentsMargins(8, 8, 8, 8)

        all_row = QHBoxLayout()
        lbl_all = QLabel("Set All Servos:")
        lbl_all.setFont(QFont("SansSerif", 9, QFont.Bold))
        lbl_all.setStyleSheet("color: #e4e4e7;")

        self.slider_all = QSlider(Qt.Horizontal)
        self.slider_all.setRange(0, 180)
        self.slider_all.setValue(90)
        self.slider_all.setStyleSheet("QSlider::groove:horizontal { border: 1px solid #444; height: 6px; background: #333; border-radius: 3px; } QSlider::sub-page:horizontal { background: #a855f7; border-radius: 3px; } QSlider::handle:horizontal { background: #ffffff; width: 14px; margin: -4px 0; border-radius: 7px; }")

        self.lbl_all_val = QLabel("90°")
        self.lbl_all_val.setFont(QFont("Monospace", 9, QFont.Bold))
        self.lbl_all_val.setStyleSheet("color: #a855f7; background: #18181b; padding: 2px 6px; border-radius: 4px;")

        self.slider_all.valueChanged.connect(self.on_all_slider_changed)

        all_row.addWidget(lbl_all)
        all_row.addWidget(self.slider_all, 1)
        all_row.addWidget(self.lbl_all_val)
        pca_layout.addLayout(all_row)

        ch_scroll = QScrollArea()
        ch_scroll.setWidgetResizable(True)
        ch_scroll.setStyleSheet("border: none; background: transparent;")
        ch_container = QWidget()
        ch_layout = QVBoxLayout(ch_container)
        ch_layout.setContentsMargins(0, 0, 0, 0)
        ch_layout.setSpacing(6)

        self.channel_sliders = []
        self.channel_readouts = []

        channels_info = [
            (0, "R1 (Right Front Hip)", 90),
            (1, "R2 (Right Rear Hip)", 0),
            (2, "L1 (Left Front Hip)", 0),
            (3, "L2 (Left Rear Hip)", 90),
            (4, "R4 (Right Rear Foot)", 90),
            (5, "R3 (Right Front Foot)", 0),
            (6, "L3 (Left Front Foot)", 90),
            (7, "L4 (Left Rear Foot)", 0)
        ]

        for ch, name, def_val in channels_info:
            row = QWidget()
            row.setStyleSheet("background-color: #18181b; border: 1px solid #27272a; border-radius: 4px; padding: 4px;")
            r_layout = QHBoxLayout(row)
            r_layout.setContentsMargins(6, 4, 6, 4)

            lbl_name = QLabel(f"Ch {ch}: {name}")
            lbl_name.setFont(QFont("SansSerif", 9, QFont.Bold))
            lbl_name.setStyleSheet("color: #e4e4e7; min-width: 180px;")

            slider = QSlider(Qt.Horizontal)
            slider.setRange(0, 180)
            slider.setValue(def_val)
            slider.setStyleSheet("QSlider::groove:horizontal { border: 1px solid #444; height: 5px; background: #27272a; border-radius: 2px; } QSlider::sub-page:horizontal { background: #38bdf8; border-radius: 2px; } QSlider::handle:horizontal { background: #ffffff; width: 12px; margin: -4px 0; border-radius: 6px; }")

            lbl_val = QLabel(f"{def_val}°")
            lbl_val.setFont(QFont("Monospace", 9, QFont.Bold))
            lbl_val.setStyleSheet("color: #38bdf8; min-width: 40px; text-align: right;")

            slider.valueChanged.connect(lambda val, c=ch, l=lbl_val: self.on_channel_slider_changed(c, val, l))

            r_layout.addWidget(lbl_name)
            r_layout.addWidget(slider, 1)
            r_layout.addWidget(lbl_val)

            ch_layout.addWidget(row)
            self.channel_sliders.append(slider)
            self.channel_readouts.append(lbl_val)

        ch_scroll.setWidget(ch_container)
        pca_layout.addWidget(ch_scroll, 1)
        pca_box.setLayout(pca_layout)
        right_layout.addWidget(pca_box, 2)

        # 5. Console Log Box
        log_box = QGroupBox("📟 Quard Bot Console & Output Log")
        log_box.setStyleSheet("""
            QGroupBox { font-weight: bold; font-size: 12px; border: 1px solid #333; border-radius: 6px; margin-top: 6px; padding-top: 8px; background-color: #141414; color: #f59e0b; }
            QGroupBox::title { subcontrol-origin: margin; padding: 0 4px; color: #f59e0b; }
        """)
        log_layout = QVBoxLayout()
        log_layout.setContentsMargins(6, 6, 6, 6)

        self.console_log = QTextEdit()
        self.console_log.setReadOnly(True)
        self.console_log.setFont(QFont("Monospace", 9))
        self.console_log.setStyleSheet("QTextEdit { background-color: #09090b; color: #38bdf8; border: 1px solid #27272a; border-radius: 4px; padding: 4px; }")

        log_layout.addWidget(self.console_log, 1)
        log_box.setLayout(log_layout)
        right_layout.addWidget(log_box, 1)

        main_layout.addWidget(left_col, 0)
        main_layout.addWidget(right_col, 1)

    def on_channel_slider_changed(self, ch, val, lbl_val):
        lbl_val.setText(f"{val}°")
        self.send_command({'ch': ch, 'angle': val})

    def on_all_slider_changed(self, val):
        self.lbl_all_val.setText(f"{val}°")
        for i, s in enumerate(self.channel_sliders):
            s.blockSignals(True)
            s.setValue(val)
            s.blockSignals(False)
            self.channel_readouts[i].setText(f"{val}°")
        self.send_command({'all': val})

    def send_command(self, params):
        # Normalize params for backwards & forward compatibility
        norm_params = dict(params)
        if 'mode' in norm_params:
            m = str(norm_params['mode']).lower()
            mode_map = {
                'forward': 'w', 'walk_forward': 'w', 'w': 'w',
                'backward': 'b', 'walk_backward': 'b', 'b': 'b',
                'left': 'l', 'turn_left': 'l', 'l': 'l',
                'right': 'r', 'turn_right': 'r', 'r': 'r',
                'stand': 's', 'stop': 's', 's': 's',
                'zero': 'z', 'z': 'z',
                'wave': 'hi', 'hi': 'hi'
            }
            norm_params['move'] = mode_map.get(m, m)

        if 'ch' in norm_params and 'angle' in norm_params:
            norm_params['deg'] = norm_params['angle']

        host = self.input_host.text().strip()
        if not host.startswith("http://") and not host.startswith("https://"):
            host = "http://" + host
        url = f"{host.rstrip('/')}/cmd?{urllib.parse.urlencode(norm_params)}"

        def worker():
            try:
                req = urllib.request.Request(url)
                with urllib.request.urlopen(req, timeout=1.5) as resp:
                    reply = resp.read().decode('utf-8', errors='ignore')
                    self.update_status_signal.emit(True, f"Out: {params} | Reply: {reply}")
            except Exception as e:
                self.update_status_signal.emit(False, f"Out: {params} | Error: {e}")

        threading.Thread(target=worker, daemon=True).start()

    def ping_quard_bot(self):
        host = self.input_host.text().strip()
        if not host.startswith("http://") and not host.startswith("https://"):
            host = "http://" + host
        url = f"{host.rstrip('/')}/"

        def worker():
            try:
                req = urllib.request.Request(url)
                with urllib.request.urlopen(req, timeout=1.2) as resp:
                    self.update_status_signal.emit(True, None)
            except Exception as e:
                self.update_status_signal.emit(False, None)

        threading.Thread(target=worker, daemon=True).start()

    def set_status(self, is_online, log_msg):
        self.is_online = is_online
        if is_online:
            self.lbl_status.setText("🟢 Connected")
            self.lbl_status.setStyleSheet("background-color: #064e3b; color: #34d399; padding: 4px 10px; border-radius: 4px;")
        else:
            self.lbl_status.setText("🔴 Offline")
            self.lbl_status.setStyleSheet("background-color: #4c0519; color: #f43f5e; padding: 4px 10px; border-radius: 4px;")

        if log_msg:
            self.log_to_console(log_msg)

    def log_to_console(self, text):
        ts = time.strftime("[%H:%M:%S] ")
        self.console_log.append(ts + text)
        self.console_log.moveCursor(QTextCursor.End)


class CalibratedJointPublisherGUI(QMainWindow):

    def __init__(self, ros_node):
        super().__init__()
        self.ros_node = ros_node
        self.ros_node.gui_ref = self

        self.setWindowTitle("Robot Arm Dashboard — Integrated Control, 3D Viewport & Connection Monitor")
        self.resize(1400, 860)
        self.setStyleSheet("background-color: #1e1e1e; color: #ffffff;")

        self.calib_data = self.load_calibration()
        self.joints_dict = self.parse_urdf_joints(URDF_PATH)
        self.last_conn_state = None

        # ── Main Splitter: Left Control Panel + Right RViz 3D Viewport ─────
        main_splitter = QSplitter(Qt.Horizontal)

        # ── Left Control Panel Widget ──────────────────────────────────────
        control_widget = QWidget()
        control_layout = QVBoxLayout()
        control_layout.setContentsMargins(8, 8, 8, 8)

        # ── Top Control Bar 1: Actions & Home Buttons ─────────────────────
        top_bar1 = QHBoxLayout()

        self.btn_sim = QPushButton("▶️ Start Sim")
        self.btn_sim.setCheckable(True)
        self.btn_sim.setStyleSheet("""
            QPushButton { background-color: #d97706; color: white; font-weight: bold; border-radius: 4px; padding: 6px 10px; }
            QPushButton:hover { background-color: #b45309; }
            QPushButton:checked { background-color: #dc2626; }
        """)
        self.btn_sim.clicked.connect(self.toggle_simulation)

        btn_home = QPushButton("🏠 Go Home")
        btn_home.setToolTip("Smoothly return all joints to home position over 2 seconds")
        btn_home.setStyleSheet("""
            QPushButton { background-color: #0284c7; color: white; font-weight: bold; border-radius: 4px; padding: 6px 10px; }
            QPushButton:hover { background-color: #0369a1; }
        """)
        btn_home.clicked.connect(self.go_to_home_pose_smooth)

        btn_set_home = QPushButton("📌 Set Home")
        btn_set_home.setToolTip("Save current slider positions as default Homing Pose")
        btn_set_home.setStyleSheet("""
            QPushButton { background-color: #7c3aed; color: white; font-weight: bold; border-radius: 4px; padding: 6px 10px; }
            QPushButton:hover { background-color: #6d28d9; }
        """)
        btn_set_home.clicked.connect(self.set_current_as_home_pose)

        btn_center = QPushButton("🎯 Center All")
        btn_center.setStyleSheet("""
            QPushButton { background-color: #475569; color: white; font-weight: bold; border-radius: 4px; padding: 6px 10px; }
            QPushButton:hover { background-color: #334155; }
        """)
        btn_center.clicked.connect(self.center_all)

        self.btn_comm_mode = QPushButton("📡 Comm: WiFi")
        self.btn_comm_mode.setCheckable(True)
        self.btn_comm_mode.setChecked(False) # False = WiFi, True = Serial
        self.btn_comm_mode.setToolTip("Toggle ESP32 control mode between Wi-Fi UDP and USB Serial")
        self.btn_comm_mode.setStyleSheet("""
            QPushButton { background-color: #0284c7; color: white; font-weight: bold; border-radius: 4px; padding: 6px 10px; }
            QPushButton:hover { background-color: #0369a1; }
            QPushButton:checked { background-color: #d97706; }
        """)
        self.btn_comm_mode.clicked.connect(self.toggle_comm_mode)

        btn_save = QPushButton("💾 Save Config")
        btn_save.setStyleSheet("""
            QPushButton { background-color: #16a34a; color: white; font-weight: bold; border-radius: 4px; padding: 6px 10px; }
            QPushButton:hover { background-color: #15803d; }
        """)
        btn_save.clicked.connect(self.save_calibration)

        self.btn_gesture = QPushButton("✋ Gesture Mode")
        self.btn_gesture.setCheckable(True)
        self.btn_gesture.setToolTip("Toggle real-time computer vision body gesture teleoperation (webcam)")
        self.btn_gesture.setStyleSheet("""
            QPushButton { background-color: #8b5cf6; color: white; font-weight: bold; border-radius: 4px; padding: 6px 10px; }
            QPushButton:hover { background-color: #7c3aed; }
            QPushButton:checked { background-color: #10b981; }
        """)
        self.btn_gesture.clicked.connect(self.toggle_gesture_mode)

        top_bar1.addWidget(self.btn_sim)
        top_bar1.addWidget(self.btn_gesture)
        top_bar1.addWidget(btn_home)
        top_bar1.addWidget(btn_set_home)
        top_bar1.addWidget(btn_center)
        top_bar1.addWidget(self.btn_comm_mode)
        top_bar1.addStretch()
        top_bar1.addWidget(btn_save)
        control_layout.addLayout(top_bar1)

        # ── Top Control Bar 2: Simulation Speed Slider ────────────────────
        speed_widget = QWidget()
        speed_widget.setStyleSheet("background-color: #262626; border-radius: 6px;")
        top_bar2 = QHBoxLayout()
        top_bar2.setContentsMargins(10, 6, 10, 6)

        lbl_speed_title = QLabel("⚡ Sim Speed:")
        lbl_speed_title.setFont(QFont("SansSerif", 10, QFont.Bold))
        lbl_speed_title.setStyleSheet("color: #f59e0b;")

        self.slider_speed = QSlider(Qt.Horizontal)
        self.slider_speed.setRange(1, 50) # 0.1x to 5.0x
        self.slider_speed.setValue(10)     # Default 1.0x
        self.slider_speed.setStyleSheet("""
            QSlider::groove:horizontal { border: 1px solid #444; height: 6px; background: #333; border-radius: 3px; }
            QSlider::sub-page:horizontal { background: #f59e0b; border-radius: 3px; }
            QSlider::handle:horizontal { background: #ffffff; width: 14px; margin: -4px 0; border-radius: 7px; }
        """)

        self.lbl_speed_val = QLabel("1.0x")
        self.lbl_speed_val.setFont(QFont("Monospace", 10, QFont.Bold))
        self.lbl_speed_val.setStyleSheet("color: #00ffcc; background: #171717; padding: 2px 8px; border-radius: 4px;")

        self.slider_speed.valueChanged.connect(self.on_speed_changed)

        top_bar2.addWidget(lbl_speed_title)
        top_bar2.addWidget(self.slider_speed)
        top_bar2.addWidget(self.lbl_speed_val)
        speed_widget.setLayout(top_bar2)
        control_layout.addWidget(speed_widget)

        # ── Scroll Area for Joint Rows ───────────────────────────────────
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setStyleSheet("border: none; background-color: #1e1e1e;")

        scroll_content = QWidget()
        scroll_layout = QVBoxLayout()

        self.joint_widgets = {}
        for j_name, (r_min, r_max) in self.joints_dict.items():
            w = CalibratedJointWidget(j_name, r_min, r_max, self.calib_data, self)
            self.joint_widgets[j_name] = w
            scroll_layout.addWidget(w)

        scroll_content.setLayout(scroll_layout)
        scroll.setWidget(scroll_content)
        control_layout.addWidget(scroll, 2)

        # ── Bottom Section: Integrated Serial & Wi-Fi Monitor Console ────
        console_group = QGroupBox("📟 Hardware Serial & Wi-Fi Monitor Console")
        console_group.setStyleSheet("""
            QGroupBox {
                font-weight: bold;
                font-size: 12px;
                border: 1px solid #333;
                border-radius: 6px;
                margin-top: 6px;
                padding-top: 8px;
                background-color: #141414;
                color: #f59e0b;
            }
            QGroupBox::title { subcontrol-origin: margin; padding: 0 4px; color: #f59e0b; }
        """)
        console_layout = QVBoxLayout()
        console_layout.setContentsMargins(6, 6, 6, 6)

        # Live Status Connection Badge Bar (Compact Single-Line Strip)
        status_bar = QHBoxLayout()
        status_bar.setContentsMargins(0, 0, 0, 2)
        self.lbl_conn_badge = QLabel(" 🟡 Initializing ESP32 Wireless Monitor...")
        self.lbl_conn_badge.setFont(QFont("SansSerif", 9, QFont.Bold))
        self.lbl_conn_badge.setFixedHeight(24)
        self.lbl_conn_badge.setStyleSheet("background-color: #332b00; color: #ffcc00; padding: 2px 8px; border-radius: 4px;")

        status_bar.addWidget(self.lbl_conn_badge)
        status_bar.addStretch()
        console_layout.addLayout(status_bar)

        # Terminal Output Box (Stretches to fill 100% of remaining empty vertical space)
        self.console_text = QTextEdit()
        self.console_text.setReadOnly(True)
        self.console_text.setFont(QFont("Monospace", 9))
        self.console_text.setStyleSheet("""
            QTextEdit {
                background-color: #09090b;
                color: #38bdf8;
                border: 1px solid #27272a;
                border-radius: 4px;
                padding: 4px;
            }
        """)
        console_layout.addWidget(self.console_text, 1)

        # Bottom Command Row
        cmd_row = QHBoxLayout()
        self.cmd_input = QLineEdit()
        self.cmd_input.setPlaceholderText("Type raw command (e.g. CMD:turntable_link_joint_dup=90.0) & press Enter...")
        self.cmd_input.setFont(QFont("Monospace", 9))
        self.cmd_input.setStyleSheet("""
            QLineEdit {
                background-color: #18181b;
                color: #ffffff;
                border: 1px solid #3f3f46;
                border-radius: 4px;
                padding: 4px 8px;
            }
        """)
        self.cmd_input.returnPressed.connect(self.send_custom_cmd)

        btn_send = QPushButton("📤 Send")
        btn_send.setStyleSheet("""
            QPushButton { background-color: #2563eb; color: white; font-weight: bold; border-radius: 4px; padding: 4px 10px; }
            QPushButton:hover { background-color: #1d4ed8; }
        """)
        btn_send.clicked.connect(self.send_custom_cmd)

        btn_clear_log = QPushButton("🧹 Clear Log")
        btn_clear_log.setStyleSheet("""
            QPushButton { background-color: #3f3f46; color: white; font-weight: bold; border-radius: 4px; padding: 4px 8px; }
            QPushButton:hover { background-color: #52525b; }
        """)
        btn_clear_log.clicked.connect(self.console_text.clear)

        cmd_row.addWidget(self.cmd_input, 1)
        cmd_row.addWidget(btn_send)
        cmd_row.addWidget(btn_clear_log)
        console_layout.addLayout(cmd_row)

        console_group.setLayout(console_layout)
        control_layout.addWidget(console_group, 1)

        control_widget.setLayout(control_layout)
        main_splitter.addWidget(control_widget)

        # ── Right RViz 3D Viewport Container ──────────────────────────────
        rviz_container = QWidget()
        rviz_container.setStyleSheet("background-color: #121212; border-left: 2px solid #333333;")
        rviz_layout = QVBoxLayout(rviz_container)
        rviz_layout.setContentsMargins(4, 4, 4, 4)
        rviz_layout.setSpacing(4)

        # RViz Header bar
        rviz_header = QLabel(" 🧊 Integrated RViz 3D Viewport")
        rviz_header.setFont(QFont("SansSerif", 10, QFont.Bold))
        rviz_header.setStyleSheet("background-color: #2b2b2b; color: #00ffcc; padding: 4px 10px; border-radius: 4px;")
        rviz_header.setFixedHeight(30)
        rviz_layout.addWidget(rviz_header, 0)

        # Embedded frame
        self.rviz_frame = QWidget()
        self.rviz_frame.setAttribute(Qt.WA_NativeWindow, True)
        self.rviz_frame.setStyleSheet("background-color: #000000; border-radius: 4px;")
        self.rviz_frame_layout = QVBoxLayout(self.rviz_frame)
        self.rviz_frame_layout.setContentsMargins(0, 0, 0, 0)
        rviz_layout.addWidget(self.rviz_frame, 1)

        main_splitter.addWidget(rviz_container)

        # Set Splitter ratios (520px left control panel, 880px right 3D viewport)
        main_splitter.setSizes([520, 880])

        # ── Top-Level QTabWidget Setup ─────────────────────────────────────
        self.tabs = QTabWidget()
        self.tabs.setStyleSheet("""
            QTabWidget::pane {
                border: 1px solid #333333;
                background-color: #1e1e1e;
            }
            QTabBar::tab {
                background-color: #2b2b2b;
                color: #aaaaaa;
                font-weight: bold;
                font-size: 13px;
                padding: 8px 24px;
                border-top-left-radius: 6px;
                border-top-right-radius: 6px;
                margin-right: 4px;
            }
            QTabBar::tab:hover {
                background-color: #383838;
                color: #ffffff;
            }
            QTabBar::tab:selected {
                background-color: #0284c7;
                color: #ffffff;
            }
        """)

        arm_dashboard_widget = QWidget()
        arm_layout = QVBoxLayout(arm_dashboard_widget)
        arm_layout.setContentsMargins(0, 0, 0, 0)
        arm_layout.addWidget(main_splitter)

        self.quard_bot_widget = QuardBotWidget(self)

        self.tabs.addTab(arm_dashboard_widget, "🦾 Robot Arm Dashboard")
        self.tabs.addTab(self.quard_bot_widget, "🤖 Quard Bot (Quadruped)")

        self.setCentralWidget(self.tabs)

        # RViz embedding process state
        self.rviz_process = None
        self.embedded_rviz_widget = None
        self.init_rviz_embed()

        # Gesture Mode Process State
        self.gesture_process = None
        self.is_gesture_mode = False

        # Simulation Mode State
        self.is_simulating = False
        self.sim_time_accumulator = 0.0
        self.last_sim_tick = time.time()

        # Smooth Homing State
        self.is_homing = False
        self.homing_start_time = 0.0
        self.homing_duration = 2.0
        self.homing_start_degs = {}
        self.homing_target_degs = {}

        # Sinusoidal motion parameters
        self.sim_configs = [
            ('turntable_link_joint_dup',   0.15, 0.0),
            ('turntable_link_joint',       0.20, math.pi / 3),
            ('turntable_link_joint_dup_1', 0.25, math.pi / 1.5)
        ]

        self.log_to_console("🚀 Dashboard Hardware & Wi-Fi Monitor Initialized.")

    def update_connection_status(self, status_str):
        """Update live status badge and output notification message when ESP32 connects/disconnects."""
        if status_str.startswith("STATUS:CONNECTED:"):
            info_text = status_str[len("STATUS:CONNECTED:"):]
            self.lbl_conn_badge.setText(f" {info_text}")
            self.lbl_conn_badge.setStyleSheet("background-color: #064e3b; color: #34d399; padding: 2px 8px; border-radius: 4px;")

            if self.last_conn_state != True:
                self.last_conn_state = True
                self.log_to_console("🟢 [SYSTEM] ✅ Connection Established between Dashboard and ESP32!")

        elif status_str.startswith("STATUS:DISCONNECTED:"):
            info_text = status_str[len("STATUS:DISCONNECTED:"):]
            self.lbl_conn_badge.setText(f" {info_text}")
            self.lbl_conn_badge.setStyleSheet("background-color: #4c0519; color: #f43f5e; padding: 2px 8px; border-radius: 4px;")

            if self.last_conn_state != False:
                self.last_conn_state = False
                self.log_to_console("🔴 [SYSTEM] ⚠️ Connection Lost to ESP32 / Searching on Wi-Fi...")

    def log_to_console(self, text):
        """Append log message to Serial/Wi-Fi Monitor Console with timestamp."""
        ts = time.strftime("[%H:%M:%S] ")
        self.console_text.append(ts + text)
        self.console_text.moveCursor(QTextCursor.End)

    def send_custom_cmd(self):
        """Send custom manual string entered by user in the Serial Monitor input."""
        cmd_text = self.cmd_input.text().strip()
        if cmd_text:
            self.log_to_console(f"📤 Outgoing Command: {cmd_text}")
            self.ros_node.publish_calibration(cmd_text)
            self.cmd_input.clear()

    def init_rviz_embed(self):
        """Spawn RViz2 as child process and schedule embedding."""
        rviz_config = "/home/chakradhar/Documents/Hardware/urdf/unnamed/robot.rviz"
        self.rviz_process = QProcess(self)

        env = QProcessEnvironment.systemEnvironment()
        env.insert("QT_QPA_PLATFORM", "xcb")
        env.insert("GDK_BACKEND", "x11")
        self.rviz_process.setProcessEnvironment(env)

        self.rviz_process.start("ros2", ["run", "rviz2", "rviz2", "-d", rviz_config])

        self.already_docked = False
        QTimer.singleShot(1500, self.embed_rviz_window)
        QTimer.singleShot(3000, self.embed_rviz_window)

    def embed_rviz_window(self):
        """Locate X11 Window ID of spawned RViz process and reparent into container."""
        if hasattr(self, 'already_docked') and self.already_docked:
            return

        if not self.isVisible() or not self.winId():
            QTimer.singleShot(500, self.embed_rviz_window)
            return

        main_win_id = str(int(self.winId()))

        search_queries = [
            ["xdotool", "search", "--onlyvisible", "--class", "rviz2"],
            ["xdotool", "search", "--onlyvisible", "--class", "rviz"],
            ["xdotool", "search", "--onlyvisible", "--name", "robot.rviz"],
            ["xdotool", "search", "--onlyvisible", "--name", "RViz"]
        ]

        rviz_win_id = None
        for cmd in search_queries:
            try:
                out = subprocess.check_output(cmd).decode().strip()
                if out:
                    ids = [i for i in out.split('\n') if i.strip()]
                    valid_ids = [i for i in ids if i != main_win_id]
                    if valid_ids:
                        rviz_win_id = int(valid_ids[-1])
                        break
            except Exception:
                continue

        if rviz_win_id is not None:
            try:
                container_win_id = str(int(self.rviz_frame.winId()))

                for wid in valid_ids:
                    if int(wid) != rviz_win_id:
                        subprocess.call(["xdotool", "windowunmap", str(wid)])

                subprocess.call(["xdotool", "windowreparent", str(rviz_win_id), container_win_id])

                qwin = QWindow.fromWinId(rviz_win_id)
                qwin.setFlags(Qt.SubWindow | Qt.FramelessWindowHint)

                self.embedded_rviz_widget = QWidget.createWindowContainer(qwin, self.rviz_frame)
                self.rviz_frame_layout.addWidget(self.embedded_rviz_widget)

                self.already_docked = True
                self.log_to_console("✅ Integrated RViz 3D Viewport docked successfully.")
            except Exception as e:
                print(f"[Dashboard] RViz window embedding notice: {e}")

    def closeEvent(self, event):
        """Cleanly terminate child processes when dashboard window is closed."""
        if self.rviz_process and self.rviz_process.state() != QProcess.NotRunning:
            self.rviz_process.terminate()
            self.rviz_process.waitForFinished(1000)
        event.accept()

    def keyPressEvent(self, event):
        """Capture keyboard events for Quard Bot locomotion when Quard Bot tab is active."""
        if hasattr(self, 'tabs') and self.tabs.currentIndex() == 1:
            key = event.key()
            if key in (Qt.Key_W, Qt.Key_Up):
                self.quard_bot_widget.send_command({'mode': 'forward'})
            elif key in (Qt.Key_S, Qt.Key_Down):
                self.quard_bot_widget.send_command({'mode': 'backward'})
            elif key in (Qt.Key_A, Qt.Key_Left):
                self.quard_bot_widget.send_command({'mode': 'left'})
            elif key in (Qt.Key_D, Qt.Key_Right):
                self.quard_bot_widget.send_command({'mode': 'right'})
            elif key == Qt.Key_Space:
                self.quard_bot_widget.send_command({'mode': 'stand'})
            else:
                super().keyPressEvent(event)
        else:
            super().keyPressEvent(event)

    def on_speed_changed(self, value):
        speed_factor = value / 10.0
        self.lbl_speed_val.setText(f"{speed_factor:.1f}x")

    def parse_urdf_joints(self, path):
        joints = {}
        try:
            tree = ET.parse(path)
            root = tree.getroot()
            for j in root.findall('joint'):
                jname = j.get('name')
                jtype = j.get('type')
                if jtype == 'revolute':
                    limit = j.find('limit')
                    r_min = float(limit.get('lower', -3.14)) if limit is not None else -3.14
                    r_max = float(limit.get('upper', 3.14)) if limit is not None else 3.14
                    joints[jname] = (r_min, r_max)
        except Exception as e:
            print(f"Error parsing URDF: {e}")
            joints = {
                'turntable_link_joint_dup': (-3.0, 3.0),
                'turntable_link_joint': (-2.0, 2.0),
                'turntable_link_joint_dup_1': (-2.0, 2.0),
                'turntable_link_joint_dup_2': (-2.0, 2.0)
            }
        if 'wrist_twist_joint' not in joints:
            joints['wrist_twist_joint'] = (-3.14159, 3.14159)
        return joints

    def load_calibration(self):
        if os.path.exists(CALIB_FILE_PATH):
            try:
                with open(CALIB_FILE_PATH, 'r') as f:
                    return json.load(f)
            except Exception as e:
                print(f"Error loading calib file: {e}")
        return {}

    def save_calibration(self):
        calib_out = {}
        for j_name, w in self.joint_widgets.items():
            calib_out[j_name] = w.calib

        try:
            with open(CALIB_FILE_PATH, 'w') as f:
                json.dump(calib_out, f, indent=2)
            self.log_to_console(f"💾 Homing & Safety Config saved to {CALIB_FILE_PATH}")
            QMessageBox.information(self, "Success", f"Homing & Safety Calibration saved to:\n{CALIB_FILE_PATH}")
        except Exception as e:
            QMessageBox.critical(self, "Error", f"Failed to save calibration:\n{e}")

    def toggle_comm_mode(self, checked):
        mode_str = "SERIAL" if checked else "WIFI"
        if checked:
            self.btn_comm_mode.setText("📟 Comm: Serial")
        else:
            self.btn_comm_mode.setText("📡 Comm: WiFi")

        cmd = f"MODE:{mode_str}"
        self.ros_node.publish_calibration(cmd)
        self.log_to_console(f"📡 Transmitted {cmd} to ESP32 via ROS topic")

    def update_sliders_from_ros(self, msg: JointState):
        """Update GUI joint sliders visually when gesture teleop is active."""
        if not self.is_gesture_mode:
            return
        for j_name, w in self.joint_widgets.items():
            if j_name in msg.name:
                idx = msg.name.index(j_name)
                if idx < len(msg.position):
                    rad_val = msg.position[idx]
                    r_min, r_max = w.rad_min, w.rad_max
                    ratio = (rad_val - r_min) / (r_max - r_min) if r_max != r_min else 0.5
                    deg = max(0.0, min(180.0, ratio * 180.0))
                    w.slider.blockSignals(True)
                    w.slider.setValue(int(deg * 10))
                    w.label_val.setText(f"{deg:.1f}° ({rad_val:.3f} rad)")
                    w.slider.blockSignals(False)

    def toggle_gesture_mode(self, checked):
        """Toggle between Manual Joint Sliders and Computer Vision Body Gesture Mode."""
        self.is_gesture_mode = checked
        if checked:
            if self.is_simulating:
                self.btn_sim.setChecked(False)
                self.toggle_simulation(False)

            self.btn_gesture.setText("✋ Gesture ACTIVE")
            self.log_to_console("🖐️ [MODE SWITCH] Activated Body Gesture Teleoperation Mode (Webcam /dev/video0).")
            self.lbl_conn_badge.setText(" 🖐️ Mode: Body Gesture Teleoperation (Webcam)")
            self.lbl_conn_badge.setStyleSheet("background-color: #064e3b; color: #34d399; padding: 2px 8px; border-radius: 4px;")

            if self.gesture_process is None:
                self.gesture_process = QProcess(self)
            self.gesture_process.start("ros2", ["run", "servo_joint_publisher", "gesture_teleop_node"])
        else:
            self.btn_gesture.setText("✋ Gesture Mode")
            self.log_to_console("🕹️ [MODE SWITCH] Returned to Manual Slider Control Mode.")
            self.lbl_conn_badge.setText(" 🕹️ Mode: Manual Joint Sliders")
            self.lbl_conn_badge.setStyleSheet("background-color: #1e3a8a; color: #60a5fa; padding: 2px 8px; border-radius: 4px;")

            if self.gesture_process and self.gesture_process.state() != QProcess.NotRunning:
                self.gesture_process.kill()
                self.gesture_process.waitForFinished(1000)

    def closeEvent(self, event):
        """Clean up gesture teleop child process on dashboard window exit."""
        if self.gesture_process and self.gesture_process.state() != QProcess.NotRunning:
            self.gesture_process.kill()
            self.gesture_process.waitForFinished(1000)
        super().closeEvent(event)

    def go_to_home_pose_smooth(self):
        """Initiate smooth, slow homing transition from any joint position over 2.0 seconds."""
        if self.is_simulating:
            self.btn_sim.setChecked(False)
            self.toggle_simulation(False)

        self.homing_start_degs = {}
        self.homing_target_degs = {}

        for j_name, w in self.joint_widgets.items():
            self.homing_start_degs[j_name] = w.get_current_deg()
            self.homing_target_degs[j_name] = w.calib.get('home_deg', 90.0)
            w.slider.setEnabled(False)

        self.is_homing = True
        self.homing_start_time = time.time()
        self.log_to_console("🏠 Homing Sequence Started (2.0s smooth transition)...")

    def step_homing(self):
        """Interpolate joint angles smoothly towards home position (ease-in-out)."""
        if not self.is_homing:
            return

        elapsed = time.time() - self.homing_start_time
        progress = elapsed / self.homing_duration

        if progress >= 1.0:
            progress = 1.0
            self.is_homing = False
            for w in self.joint_widgets.values():
                w.slider.setEnabled(True)
            self.log_to_console("✅ Robot Arm Reached Home Pose.")

        t_smooth = 0.5 - 0.5 * math.cos(progress * math.pi)

        for j_name, w in self.joint_widgets.items():
            start_d = self.homing_start_degs.get(j_name, 90.0)
            target_d = self.homing_target_degs.get(j_name, 90.0)
            interp_deg = start_d + (target_d - start_d) * t_smooth
            w.set_deg_programmatically(interp_deg)

    def set_current_as_home_pose(self):
        for w in self.joint_widgets.values():
            w.set_current_as_home()
        self.save_calibration()
        self.log_to_console("📌 Current Pose set as default Homing Position.")

    def toggle_simulation(self, checked):
        self.is_simulating = checked
        if checked:
            if self.is_homing:
                self.is_homing = False

            self.sim_time_accumulator = 0.0
            self.last_sim_tick = time.time()
            self.btn_sim.setText("⏸️ Stop Sim")
            for w in self.joint_widgets.values():
                w.slider.setEnabled(False)
            self.log_to_console("▶️ Motion Simulation Mode Enabled.")
        else:
            self.btn_sim.setText("▶️ Start Sim")
            for w in self.joint_widgets.values():
                w.slider.setEnabled(True)
            self.log_to_console("⏸️ Motion Simulation Mode Paused.")

    def step_simulation(self):
        if not self.is_simulating:
            return

        now = time.time()
        dt = now - self.last_sim_tick
        self.last_sim_tick = now

        speed_factor = self.slider_speed.value() / 10.0
        self.sim_time_accumulator += dt * speed_factor

        for j_name, freq, phase in self.sim_configs:
            if j_name in self.joint_widgets:
                w = self.joint_widgets[j_name]
                min_lim = w.spin_min.value()
                max_lim = w.spin_max.value()

                center = (min_lim + max_lim) / 2.0
                amplitude = (max_lim - min_lim) / 2.0

                target_deg = center + amplitude * math.sin(2.0 * math.pi * freq * self.sim_time_accumulator + phase)
                w.set_deg_programmatically(target_deg)

    def center_all(self):
        if self.is_simulating:
            self.btn_sim.setChecked(False)
            self.toggle_simulation(False)
        if self.is_homing:
            self.is_homing = False
        for w in self.joint_widgets.values():
            w.slider.setValue(900)
        self.log_to_console("🎯 All Joints Centered to 90.0°.")

    def reset_all_limits(self):
        for w in self.joint_widgets.values():
            w.spin_min.setValue(0.0)
            w.spin_max.setValue(180.0)
        self.log_to_console("🔄 Safety Limits Reset to Default (0° to 180°).")

    def on_joint_calib_changed(self, joint_name, min_deg, max_deg):
        calib_str = f"CALIB:{joint_name}={min_deg:.1f},{max_deg:.1f}"
        self.ros_node.publish_calibration(calib_str)
        self.log_to_console(f"⚙️ Limit Broadcast: {calib_str}")

    def publish_joint_states(self):
        if self.is_gesture_mode:
            return  # Yield /joint_states control to gesture_teleop_node to prevent RViz glitching

        if self.is_homing:
            self.step_homing()
        elif self.is_simulating:
            self.step_simulation()

        msg = JointState()
        msg.header.stamp = self.ros_node.get_clock().now().to_msg()

        names = []
        positions = []

        for j_name, w in self.joint_widgets.items():
            names.append(j_name)
            positions.append(w.get_current_rad())

        msg.name = names
        msg.position = positions

        self.ros_node.joint_pub.publish(msg)


class CalibratedGUINode(Node):

    def __init__(self):
        super().__init__('calibrated_joint_gui')
        self.gui_ref = None
        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.calib_pub = self.create_publisher(StringMsg, '/servo_calibration', 10)

        # Subscribe to ESP32 connection status messages
        self.status_sub = self.create_subscription(
            StringMsg,
            '/esp32_connection_status',
            self.status_callback,
            10
        )
        # Subscribe to /joint_states to mirror active gesture pose on GUI sliders
        self.joint_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_states_callback,
            10
        )
        self.get_logger().info("Calibrated Joint Publisher GUI Node initialized.")

    def joint_states_callback(self, msg: JointState):
        if self.gui_ref and self.gui_ref.is_gesture_mode:
            self.gui_ref.update_sliders_from_ros(msg)

    def status_callback(self, msg: StringMsg):
        if self.gui_ref:
            self.gui_ref.update_connection_status(msg.data)

    def publish_calibration(self, calib_str):
        msg = StringMsg()
        msg.data = calib_str
        self.calib_pub.publish(msg)
        self.get_logger().info(f"Broadcasted Calibration: {calib_str}")


def main(args=None):
    rclpy.init(args=args)
    ros_node = CalibratedGUINode()

    app = QApplication(sys.argv)
    gui = CalibratedJointPublisherGUI(ros_node)

    if '--sim' in sys.argv or '-s' in sys.argv:
        gui.btn_sim.setChecked(True)
        gui.toggle_simulation(True)

    gui.show()
    gui.publish_joint_states()

    timer = QTimer()
    timer.timeout.connect(lambda: (
        rclpy.spin_once(ros_node, timeout_sec=0.001),
        gui.publish_joint_states()
    ))
    timer.start(20) # 20 ms = 50 Hz

    exit_code = app.exec_()

    if gui.gesture_process and gui.gesture_process.state() != QProcess.NotRunning:
        gui.gesture_process.kill()
        gui.gesture_process.waitForFinished(1000)

    ros_node.destroy_node()
    rclpy.shutdown()
    sys.exit(exit_code)


if __name__ == '__main__':
    main()
