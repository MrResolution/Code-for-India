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
import re
import random

from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QLabel, QSlider, QDoubleSpinBox, QPushButton, QGroupBox,
    QScrollArea, QMessageBox, QSplitter, QTextEdit, QLineEdit,
    QTabWidget, QGridLayout, QComboBox
)
from PyQt5.QtCore import Qt, QTimer, QProcess, QProcessEnvironment, pyqtSignal
from PyQt5.QtGui import QFont, QWindow, QTextCursor, QPainter, QPen, QBrush, QColor, QPainterPath, QRadialGradient, QLinearGradient

try:
    from .visual_slam_widget import VisualSLAMWidget
except ImportError:
    try:
        from visual_slam_widget import VisualSLAMWidget
    except ImportError:
        VisualSLAMWidget = None

if "QT_QPA_PLATFORM_PLUGIN_PATH" in os.environ and "cv2" in os.environ["QT_QPA_PLATFORM_PLUGIN_PATH"]:
    del os.environ["QT_QPA_PLATFORM_PLUGIN_PATH"]

try:
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import JointState
    from std_msgs.msg import String as StringMsg
    ROS2_AVAILABLE = True
except ImportError:
    ROS2_AVAILABLE = False
    JointState = object

CALIB_FILE_PATH = "/home/sabo/Documents/learn_/Hardware/servo_calibration.json"
URDF_PATH = "/home/sabo/Documents/learn_/Hardware/urdf/arm/urdf/arm.urdf"

# Default Homing Pose requested by user for the 5 URDF arm joints:
DEFAULT_HOME_DEGS = {
    'turntable_link_joint_dup': 90.0,    # Base Turntable Yaw
    'turntable_link_joint': 59.0,        # Shoulder Pitch (Dual Servos: GPIO 19 & 21)
    'turntable_link_joint_dup_1': 153.1, # Elbow 1 Pitch
    'turntable_link_joint_dup_2': 116.0, # Elbow 2 Pitch
    'turntable_link_joint_dup_3': 90.0,  # Wrist Twist / Roll (GPIO 27)
    'wrist_twist_joint': 90.0,           # Wrist Twist / Roll
    'grip': 90.0                         # End-Effector Gripper (GPIO 26)
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
            'invert': False,
            'sync_offset_deg': 0.0
        })
        if 'home_deg' not in self.calib:
            self.calib['home_deg'] = def_home
        self.sync_offset_deg = float(self.calib.get('sync_offset_deg', 0.0))

        if joint_name == 'turntable_link_joint':
            self.setTitle(f"🦾 Joint: {joint_name} (Shoulder Pitch — Dual Servos: GPIO 19 & 21)")
        elif joint_name == 'turntable_link_joint_dup':
            self.setTitle(f"🦾 Joint: {joint_name} (Base Turntable Yaw)")
        elif joint_name == 'turntable_link_joint_dup_1':
            self.setTitle(f"🦾 Joint: {joint_name} (Elbow 1 Pitch)")
        elif joint_name == 'turntable_link_joint_dup_2':
            self.setTitle(f"🦾 Joint: {joint_name} (Elbow 2 Pitch)")
        elif joint_name in ('wrist_twist_joint', 'turntable_link_joint_dup_3'):
            self.setTitle(f"🦾 Joint: {joint_name} (Wrist Twist / Roll — GPIO 27)")
        elif joint_name in ('grip', 'gripper'):
            self.setTitle(f"🦾 Joint: {joint_name} (End-Effector Gripper — GPIO 26)")
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

        # ── Row 4: Twin Sync & Alignment Offset ───────────────────────────
        row4 = QHBoxLayout()
        lbl_sync = QLabel("🎯 Sync:")
        lbl_sync.setStyleSheet("color: #38bdf8; font-weight: bold; font-size: 11px;")
        lbl_sync.setToolTip("Fine-tune 3D virtual orientation offset relative to physical servo")

        self.spin_sync = QDoubleSpinBox()
        self.spin_sync.setRange(-180.0, 180.0)
        self.spin_sync.setValue(self.sync_offset_deg)
        self.spin_sync.setSingleStep(0.5)
        self.spin_sync.setSuffix("°")
        self.spin_sync.setStyleSheet("background: #0f172a; color: #38bdf8; font-weight: bold; border: 1px solid #0284c7; border-radius: 3px; padding: 2px 4px;")
        self.spin_sync.valueChanged.connect(self.on_sync_offset_changed)

        btn_nudge_m5 = QPushButton("-5°")
        btn_nudge_m1 = QPushButton("-1°")
        btn_nudge_p1 = QPushButton("+1°")
        btn_nudge_p5 = QPushButton("+5°")
        btn_sync_zero = QPushButton("0°")

        nudge_style = """
            QPushButton { background-color: #1e293b; color: #e2e8f0; font-weight: bold; border: 1px solid #475569; border-radius: 3px; padding: 2px 5px; font-size: 10px; }
            QPushButton:hover { background-color: #334155; }
            QPushButton:pressed { background-color: #0f172a; }
        """
        for b in [btn_nudge_m5, btn_nudge_m1, btn_nudge_p1, btn_nudge_p5, btn_sync_zero]:
            b.setStyleSheet(nudge_style)

        btn_nudge_m5.clicked.connect(lambda: self.spin_sync.setValue(self.spin_sync.value() - 5.0))
        btn_nudge_m1.clicked.connect(lambda: self.spin_sync.setValue(self.spin_sync.value() - 1.0))
        btn_nudge_p1.clicked.connect(lambda: self.spin_sync.setValue(self.spin_sync.value() + 1.0))
        btn_nudge_p5.clicked.connect(lambda: self.spin_sync.setValue(self.spin_sync.value() + 5.0))
        btn_sync_zero.clicked.connect(lambda: self.spin_sync.setValue(0.0))

        row4.addWidget(lbl_sync)
        row4.addWidget(self.spin_sync)
        row4.addWidget(btn_nudge_m5)
        row4.addWidget(btn_nudge_m1)
        row4.addWidget(btn_nudge_p1)
        row4.addWidget(btn_nudge_p5)
        row4.addWidget(btn_sync_zero)
        layout.addLayout(row4)

        self.setLayout(layout)
        self.on_slider_changed(self.slider.value())

    def get_current_deg(self):
        return self.slider.value() / 10.0

    def get_current_rad(self):
        deg = self.get_current_deg() + self.sync_offset_deg
        ratio = deg / 180.0
        return self.rad_min + ratio * (self.rad_max - self.rad_min)

    def on_sync_offset_changed(self, val):
        self.sync_offset_deg = val
        self.calib['sync_offset_deg'] = val
        rad = self.get_current_rad()
        deg = self.get_current_deg()
        self.label_val.setText(f"{deg:.1f}° ({rad:.3f} rad)")
        if self.gui_parent and hasattr(self.gui_parent, 'on_sync_offset_changed'):
            self.gui_parent.on_sync_offset_changed(self.joint_name, val)

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


class CuteOLEDDisplayWidget(QWidget):
    """Simulated SSD1306 128x64 / 16:9 OLED Display with live animated faces."""
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setFixedSize(170, 92)
        self.current_face = "happy"
        self.blink_state = False
        self.eye_offset = 0
        self.step_counter = 0

        # Animation timer for blinking & walking eye movements
        self.anim_timer = QTimer(self)
        self.anim_timer.timeout.connect(self._on_anim_tick)
        self.anim_timer.start(250)
        self.tick_count = 0

    def set_face(self, face_name):
        self.current_face = face_name.lower()
        self.blink_state = False
        self.update()

    def set_walk_phase(self, phase):
        self.step_counter = phase
        self.eye_offset = 4 if (phase % 2 == 0) else -4
        self.update()

    def _on_anim_tick(self):
        self.tick_count += 1
        # Random natural blink every ~3.5 seconds when happy or cute
        if self.current_face in ["happy", "cute"]:
            if self.tick_count % 14 == 0:
                self.blink_state = True
                self.update()
            elif self.blink_state:
                self.blink_state = False
                self.update()
        elif self.current_face == "walk":
            self.eye_offset = 4 if (self.tick_count % 2 == 0) else -4
            self.update()
        elif self.current_face == "sleepy":
            self.update()

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)

        w = self.width()
        h = self.height()

        # Outer bezel casing
        painter.setBrush(QBrush(QColor("#111318")))
        painter.setPen(QPen(QColor("#2d3342"), 2))
        painter.drawRoundedRect(1, 1, w - 2, h - 2, 8, 8)

        # OLED Screen panel
        margin = 6
        sw = w - 2 * margin
        sh = h - 2 * margin
        painter.setBrush(QBrush(QColor("#030712")))
        painter.setPen(QPen(QColor("#1e293b"), 1))
        painter.drawRoundedRect(margin, margin, sw, sh, 5, 5)

        cyan = QColor("#38bdf8")
        glow_pen = QPen(cyan, 3)
        glow_pen.setCapStyle(Qt.RoundCap)
        glow_pen.setJoinStyle(Qt.RoundJoin)
        pink = QColor("#fb7185")

        cx = w / 2.0
        cy = h / 2.0

        if self.blink_state:
            # Closed blinking eyes
            painter.setPen(glow_pen)
            painter.drawLine(int(cx - 38), int(cy - 2), int(cx - 14), int(cy - 2))
            painter.drawLine(int(cx + 14), int(cy - 2), int(cx + 38), int(cy - 2))
            return

        if self.current_face == "happy":
            painter.setPen(glow_pen)
            path_l = QPainterPath()
            path_l.moveTo(cx - 40, cy + 2)
            path_l.quadTo(cx - 27, cy - 18, cx - 14, cy + 2)
            painter.drawPath(path_l)

            path_r = QPainterPath()
            path_r.moveTo(cx + 14, cy + 2)
            path_r.quadTo(cx + 27, cy - 18, cx + 40, cy + 2)
            painter.drawPath(path_r)

            # Rosy cheeks
            painter.setPen(Qt.NoPen)
            painter.setBrush(QBrush(pink))
            painter.drawEllipse(int(cx - 44), int(cy + 10), 10, 5)
            painter.drawEllipse(int(cx + 34), int(cy + 10), 10, 5)

            # Smile
            painter.setPen(QPen(cyan, 2))
            path_m = QPainterPath()
            path_m.moveTo(cx - 8, cy + 12)
            path_m.quadTo(cx, cy + 20, cx + 8, cy + 12)
            painter.drawPath(path_m)

        elif self.current_face == "walk":
            # Oval eyes with tracking pupils
            painter.setPen(glow_pen)
            painter.setBrush(Qt.NoBrush)
            painter.drawRoundedRect(int(cx - 40), int(cy - 16), 24, 26, 8, 8)
            painter.drawRoundedRect(int(cx + 16), int(cy - 16), 24, 26, 8, 8)

            painter.setBrush(QBrush(cyan))
            ox = self.eye_offset
            painter.drawEllipse(int(cx - 32 + ox), int(cy - 8), 10, 10)
            painter.drawEllipse(int(cx + 24 + ox), int(cy - 8), 10, 10)

            painter.setPen(QPen(cyan, 2))
            painter.drawLine(int(cx - 6), int(cy + 18), int(cx + 6), int(cy + 18))

        elif self.current_face == "wave":
            painter.setPen(glow_pen)
            path_wink = QPainterPath()
            path_wink.moveTo(cx - 38, cy + 2)
            path_wink.quadTo(cx - 26, cy - 16, cx - 14, cy + 2)
            painter.drawPath(path_wink)

            painter.setBrush(QBrush(cyan))
            painter.drawRoundedRect(int(cx + 16), int(cy - 16), 24, 24, 8, 8)
            painter.setBrush(QBrush(QColor("#ffffff")))
            painter.drawEllipse(int(cx + 28), int(cy - 12), 6, 6)

            painter.setPen(Qt.NoPen)
            painter.setBrush(QBrush(pink))
            painter.drawEllipse(int(cx - 42), int(cy + 10), 10, 5)
            painter.drawEllipse(int(cx + 34), int(cy + 10), 10, 5)

            painter.setPen(QPen(cyan, 2))
            path_m = QPainterPath()
            path_m.moveTo(cx - 7, cy + 12)
            path_m.quadTo(cx, cy + 22, cx + 7, cy + 12)
            painter.drawPath(path_m)

        elif self.current_face == "sleepy":
            painter.setPen(glow_pen)
            painter.drawLine(int(cx - 38), int(cy - 2), int(cx - 14), int(cy + 2))
            painter.drawLine(int(cx + 14), int(cy + 2), int(cx + 38), int(cy - 2))

            painter.setPen(QPen(cyan, 1))
            painter.setFont(QFont("Monospace", 9, QFont.Bold))
            painter.drawText(int(cx + 28), int(cy - 12), "z")
            painter.setFont(QFont("Monospace", 12, QFont.Bold))
            painter.drawText(int(cx + 38), int(cy - 22), "Z")

            painter.drawEllipse(int(cx - 4), int(cy + 12), 8, 8)

        elif self.current_face == "cute":
            painter.setPen(glow_pen)
            painter.setBrush(QBrush(cyan))
            painter.drawEllipse(int(cx - 40), int(cy - 16), 26, 26)
            painter.drawEllipse(int(cx + 14), int(cy - 16), 26, 26)

            painter.setBrush(QBrush(QColor("#ffffff")))
            painter.setPen(Qt.NoPen)
            painter.drawEllipse(int(cx - 32), int(cy - 12), 9, 9)
            painter.drawEllipse(int(cx + 22), int(cy - 12), 9, 9)
            painter.drawEllipse(int(cx - 24), int(cy - 2), 4, 4)
            painter.drawEllipse(int(cx + 30), int(cy - 2), 4, 4)

            painter.setBrush(QBrush(pink))
            painter.drawEllipse(int(cx - 44), int(cy + 12), 10, 5)
            painter.drawEllipse(int(cx + 34), int(cy + 12), 10, 5)

            painter.setPen(QPen(cyan, 2))
            path_cat = QPainterPath()
            path_cat.moveTo(cx - 8, cy + 12)
            path_cat.quadTo(cx - 4, cy + 17, cx, cy + 14)
            path_cat.quadTo(cx + 4, cy + 17, cx + 8, cy + 12)
            painter.drawPath(path_cat)


class QuardBotWidget(QWidget):
    """Dedicated PyQt5 Tab Widget for Quard Bot Locomotion, Stances, OLED Faces & PCA9685 Controls."""
    update_log_signal = pyqtSignal(str)
    update_status_signal = pyqtSignal(bool, dict)

    GAIT_PATTERNS = {
        'forward': [
            # Phase 0: Lift Diag A (feet 5, 7) & swing hips (0, 3) forward
            {5: 25, 7: 65, 0: 70, 3: 70, 4: 45, 6: 45, 1: 45, 2: 45},
            # Phase 1: Ground Diag A to neutral stand
            {0: 45, 1: 45, 2: 45, 3: 45, 4: 45, 5: 45, 6: 45, 7: 45},
            # Phase 2: Lift Diag B (feet 4, 6) & swing hips (1, 2) forward
            {4: 65, 6: 25, 1: 20, 2: 20, 5: 45, 7: 45, 0: 45, 3: 45},
            # Phase 3: Ground Diag B to neutral stand
            {0: 45, 1: 45, 2: 45, 3: 45, 4: 45, 5: 45, 6: 45, 7: 45}
        ],
        'backward': [
            # Phase 0: Lift Diag B & swing hips backward
            {4: 25, 6: 65, 1: 70, 2: 70, 5: 45, 7: 45, 0: 45, 3: 45},
            # Phase 1: Ground Diag B
            {0: 45, 1: 45, 2: 45, 3: 45, 4: 45, 5: 45, 6: 45, 7: 45},
            # Phase 2: Lift Diag A & swing hips backward
            {5: 65, 7: 25, 0: 20, 3: 20, 4: 45, 6: 45, 1: 45, 2: 45},
            # Phase 3: Ground Diag A
            {0: 45, 1: 45, 2: 45, 3: 45, 4: 45, 5: 45, 6: 45, 7: 45}
        ],
        'left': [
            # Phase 0: Right side swing, lift feet
            {5: 30, 4: 60, 0: 70, 1: 70, 6: 45, 7: 45, 2: 45, 3: 45},
            # Phase 1: Ground
            {0: 45, 1: 45, 2: 45, 3: 45, 4: 45, 5: 45, 6: 45, 7: 45},
            # Phase 2: Left side swing, lift feet
            {6: 60, 7: 30, 2: 20, 3: 20, 5: 45, 4: 45, 0: 45, 1: 45},
            # Phase 3: Ground
            {0: 45, 1: 45, 2: 45, 3: 45, 4: 45, 5: 45, 6: 45, 7: 45}
        ],
        'right': [
            # Phase 0: Left side swing, lift feet
            {6: 30, 7: 60, 2: 70, 3: 70, 5: 45, 4: 45, 0: 45, 1: 45},
            # Phase 1: Ground
            {0: 45, 1: 45, 2: 45, 3: 45, 4: 45, 5: 45, 6: 45, 7: 45},
            # Phase 2: Right side swing, lift feet
            {5: 60, 4: 30, 0: 20, 1: 20, 6: 45, 7: 45, 2: 45, 3: 45},
            # Phase 3: Ground
            {0: 45, 1: 45, 2: 45, 3: 45, 4: 45, 5: 45, 6: 45, 7: 45}
        ]
    }

    def __init__(self, parent=None):
        super().__init__(parent)
        self.main_gui = parent
        self.quard_host = "http://sesame-robot.local"
        self.is_online = False
        self.step_delay = 180
        self.active_gait = None
        self.gait_phase = 0
        self.quard_offsets = {str(i): 0.0 for i in range(8)}
        self.load_quard_offsets()

        # Gait generation timer
        self.gait_timer = QTimer(self)
        self.gait_timer.timeout.connect(self._on_gait_timer_tick)

        # Waving animation state
        self.wave_timer = QTimer(self)
        self.wave_step = 0
        self.wave_timer.timeout.connect(self._on_wave_timer_tick)

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

        # ── Left Column (Control Cards): Fixed width 490px ─────────────────────
        left_col = QWidget()
        left_layout = QVBoxLayout(left_col)
        left_layout.setContentsMargins(0, 0, 0, 0)
        left_layout.setSpacing(8)

        # 1. Network Connection Card with Telemetry
        conn_box = QGroupBox("🌐 Quard Bot Network Target & Live Telemetry")
        conn_box.setStyleSheet("""
            QGroupBox { font-weight: bold; font-size: 13px; border: 1px solid #444; border-radius: 6px; margin-top: 6px; padding-top: 10px; background-color: #2b2b2b; color: #4da6ff; }
            QGroupBox::title { subcontrol-origin: margin; padding: 0 5px; color: #4da6ff; }
        """)
        conn_layout = QVBoxLayout()
        conn_layout.setContentsMargins(8, 8, 8, 8)
        conn_layout.setSpacing(6)

        c_row1 = QHBoxLayout()
        self.input_host = QLineEdit(self.quard_host)
        self.input_host.setFont(QFont("Monospace", 9))
        self.input_host.setStyleSheet("background-color: #18181b; color: #ffffff; border: 1px solid #3f3f46; border-radius: 4px; padding: 4px 8px;")

        self.lbl_status = QLabel("🔴 Offline")
        self.lbl_status.setFont(QFont("SansSerif", 9, QFont.Bold))
        self.lbl_status.setStyleSheet("background-color: #4c0519; color: #f43f5e; padding: 4px 10px; border-radius: 4px;")

        btn_ping = QPushButton("🔄 Ping")
        btn_ping.setStyleSheet("QPushButton { background-color: #0284c7; color: white; font-weight: bold; border-radius: 4px; padding: 4px 10px; } QPushButton:hover { background-color: #0369a1; }")
        btn_ping.clicked.connect(self.ping_quard_bot)

        c_row1.addWidget(self.input_host, 1)
        c_row1.addWidget(self.lbl_status)
        c_row1.addWidget(btn_ping)
        conn_layout.addLayout(c_row1)

        # Telemetry readouts row
        self.lbl_telemetry = QLabel("📡 Telemetry: Waiting for ping...")
        self.lbl_telemetry.setFont(QFont("Monospace", 8))
        self.lbl_telemetry.setStyleSheet("color: #94a3b8; background-color: #18181b; padding: 3px 6px; border-radius: 3px;")
        conn_layout.addWidget(self.lbl_telemetry)

        conn_box.setLayout(conn_layout)
        left_layout.addWidget(conn_box)

        # OLED widget removed from UI; keep a None reference for guarded calls
        self.oled_widget = None

        # 3. D-Pad Locomotion Controller Card with Gait Speed & Status
        dpad_box = QGroupBox("🎮 Locomotion Controller (WASD / Space)")
        dpad_box.setStyleSheet("""
            QGroupBox { font-weight: bold; font-size: 13px; border: 1px solid #444; border-radius: 6px; margin-top: 6px; padding-top: 10px; background-color: #2b2b2b; color: #f59e0b; }
            QGroupBox::title { subcontrol-origin: margin; padding: 0 5px; color: #f59e0b; }
        """)
        dpad_main_layout = QVBoxLayout()
        dpad_main_layout.setContentsMargins(8, 8, 8, 8)
        dpad_main_layout.setSpacing(8)

        # Gait Status readout badge
        self.lbl_gait_status = QLabel("Status: ⏹ Stand (Idle)")
        self.lbl_gait_status.setFont(QFont("SansSerif", 9, QFont.Bold))
        self.lbl_gait_status.setStyleSheet("background-color: #1c1917; color: #f59e0b; border: 1px solid #44403c; border-radius: 4px; padding: 4px 8px;")
        dpad_main_layout.addWidget(self.lbl_gait_status)

        # D-Pad Grid
        dpad_grid = QGridLayout()
        dpad_grid.setContentsMargins(4, 4, 4, 4)
        dpad_grid.setSpacing(6)

        btn_fw = QPushButton("▲\nForward (W)")
        btn_bk = QPushButton("▼\nBackward (S)")
        btn_lt = QPushButton("◀\nLeft (A)")
        btn_rt = QPushButton("▶\nRight (D)")
        btn_st = QPushButton("⏹\nStand (Space)")

        dpad_style = """
            QPushButton { background-color: #3f3f46; color: #ffffff; font-weight: bold; border: 1px solid #52525b; border-radius: 8px; min-width: 85px; min-height: 50px; font-size: 11px; }
            QPushButton:hover { background-color: #0284c7; border-color: #38bdf8; }
            QPushButton:pressed { background-color: #0369a1; }
        """
        btn_st_style = """
            QPushButton { background-color: #7f1d1d; color: #fca5a5; font-weight: bold; border: 1px solid #991b1b; border-radius: 8px; min-width: 85px; min-height: 50px; font-size: 11px; }
            QPushButton:hover { background-color: #dc2626; color: #ffffff; }
        """
        for b in [btn_fw, btn_bk, btn_lt, btn_rt]:
            b.setStyleSheet(dpad_style)
        btn_st.setStyleSheet(btn_st_style)

        btn_fw.clicked.connect(lambda: self.start_gait('forward'))
        btn_bk.clicked.connect(lambda: self.start_gait('backward'))
        btn_lt.clicked.connect(lambda: self.start_gait('left'))
        btn_rt.clicked.connect(lambda: self.start_gait('right'))
        btn_st.clicked.connect(self.stand_pose)

        dpad_grid.addWidget(btn_fw, 0, 1)
        dpad_grid.addWidget(btn_lt, 1, 0)
        dpad_grid.addWidget(btn_st, 1, 1)
        dpad_grid.addWidget(btn_rt, 1, 2)
        dpad_grid.addWidget(btn_bk, 2, 1)
        dpad_main_layout.addLayout(dpad_grid)

        # Gait Speed Slider
        speed_row = QHBoxLayout()
        lbl_speed = QLabel("Gait Step Delay:")
        lbl_speed.setFont(QFont("SansSerif", 9, QFont.Bold))
        lbl_speed.setStyleSheet("color: #e2e8f0;")

        self.slider_gait_speed = QSlider(Qt.Horizontal)
        self.slider_gait_speed.setRange(100, 400)
        self.slider_gait_speed.setValue(self.step_delay)
        self.slider_gait_speed.setStyleSheet("QSlider::groove:horizontal { border: 1px solid #444; height: 5px; background: #333; border-radius: 2px; } QSlider::sub-page:horizontal { background: #f59e0b; border-radius: 2px; } QSlider::handle:horizontal { background: #ffffff; width: 12px; margin: -4px 0; border-radius: 6px; }")

        self.lbl_speed_val = QLabel(f"{self.step_delay} ms")
        self.lbl_speed_val.setFont(QFont("Monospace", 9, QFont.Bold))
        self.lbl_speed_val.setStyleSheet("color: #f59e0b; background: #18181b; padding: 2px 6px; border-radius: 4px; min-width: 50px; text-align: center;")

        self.slider_gait_speed.valueChanged.connect(self.on_gait_speed_changed)

        speed_row.addWidget(lbl_speed)
        speed_row.addWidget(self.slider_gait_speed, 1)
        speed_row.addWidget(self.lbl_speed_val)
        dpad_main_layout.addLayout(speed_row)

        dpad_box.setLayout(dpad_main_layout)
        left_layout.addWidget(dpad_box)

        # 4. Quick Stances & Trims Card
        presets_box = QGroupBox("⚡ Stance Presets & Zero-Tare Trims")
        presets_box.setStyleSheet("""
            QGroupBox { font-weight: bold; font-size: 13px; border: 1px solid #444; border-radius: 6px; margin-top: 6px; padding-top: 10px; background-color: #2b2b2b; color: #34d399; }
            QGroupBox::title { subcontrol-origin: margin; padding: 0 5px; color: #34d399; }
        """)
        presets_layout = QVBoxLayout()
        presets_layout.setContentsMargins(8, 8, 8, 8)
        presets_layout.setSpacing(6)

        stances_row1 = QHBoxLayout()
        btn_p_stand  = QPushButton("🧍 Stand (45°)")
        btn_p_zero   = QPushButton("🎯 Zero (0°)")
        btn_p_crouch = QPushButton("🦆 Crouch (30°)")
        btn_p_high   = QPushButton("🦒 High (60°)")
        btn_p_wave   = QPushButton("👋 Wave")

        p_style = "QPushButton { background-color: #18181b; color: #e4e4e7; border: 1px solid #3f3f46; border-radius: 4px; padding: 6px 4px; font-weight: bold; font-size: 11px; } QPushButton:hover { background-color: #27272a; color: #38bdf8; }"
        for b in [btn_p_stand, btn_p_zero, btn_p_crouch, btn_p_high, btn_p_wave]:
            b.setStyleSheet(p_style)

        btn_p_stand.clicked.connect(self.stand_pose)
        btn_p_zero.clicked.connect(self.zero_pose)
        btn_p_crouch.clicked.connect(self.crouch_pose)
        btn_p_high.clicked.connect(self.high_pose)
        btn_p_wave.clicked.connect(self.play_wave_animation)

        stances_row1.addWidget(btn_p_stand)
        stances_row1.addWidget(btn_p_zero)
        stances_row1.addWidget(btn_p_crouch)
        stances_row1.addWidget(btn_p_high)
        stances_row1.addWidget(btn_p_wave)
        presets_layout.addLayout(stances_row1)

        trims_row = QHBoxLayout()
        btn_p_sync = QPushButton("🎯 Sync Current Stance (Tare Offsets)")
        btn_p_sync.setToolTip("Store current slider deviations as trim offsets for 45° CAD alignment")
        btn_p_sync.setStyleSheet("QPushButton { background-color: #0c4a6e; color: #38bdf8; border: 1px solid #0284c7; border-radius: 4px; padding: 6px; font-weight: bold; font-size: 11px; } QPushButton:hover { background-color: #0284c7; color: #ffffff; }")
        btn_p_sync.clicked.connect(self.sync_current_stance)

        btn_p_reset = QPushButton("🔄 Reset Trims")
        btn_p_reset.setToolTip("Reset all 8 servo trim offsets back to 0.0°")
        btn_p_reset.setStyleSheet("QPushButton { background-color: #27272a; color: #f43f5e; border: 1px solid #4c0519; border-radius: 4px; padding: 6px; font-weight: bold; font-size: 11px; } QPushButton:hover { background-color: #4c0519; color: #ffffff; }")
        btn_p_reset.clicked.connect(self.reset_trims)

        trims_row.addWidget(btn_p_sync, 2)
        trims_row.addWidget(btn_p_reset, 1)
        presets_layout.addLayout(trims_row)

        presets_box.setLayout(presets_layout)
        left_layout.addWidget(presets_box)
        left_layout.addStretch()

        # ── Right Column (PCA9685 Sliders + Console Log) ─────────────────
        right_col = QWidget()
        right_layout = QVBoxLayout(right_col)
        right_layout.setContentsMargins(0, 0, 0, 0)
        right_layout.setSpacing(8)

        # 5. PCA9685 8-Channel Sliders Box
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
        self.slider_all.setValue(45)
        self.slider_all.setStyleSheet("QSlider::groove:horizontal { border: 1px solid #444; height: 6px; background: #333; border-radius: 3px; } QSlider::sub-page:horizontal { background: #a855f7; border-radius: 3px; } QSlider::handle:horizontal { background: #ffffff; width: 14px; margin: -4px 0; border-radius: 7px; }")

        self.lbl_all_val = QLabel("45°")
        self.lbl_all_val.setFont(QFont("Monospace", 9, QFont.Bold))
        self.lbl_all_val.setStyleSheet("color: #a855f7; background: #18181b; padding: 2px 6px; border-radius: 4px; min-width: 40px; text-align: center;")

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
        ch_layout.setSpacing(5)

        self.channel_sliders = []
        self.channel_readouts = []
        self.channel_trim_labels = []

        channels_info = [
            (0, "R1 (Right Front Hip)", 45),
            (1, "R2 (Right Rear Hip)", 45),
            (2, "L1 (Left Front Hip)", 45),
            (3, "L2 (Left Rear Hip)", 45),
            (4, "R4 (Right Rear Foot)", 45),
            (5, "R3 (Right Front Foot)", 45),
            (6, "L3 (Left Front Foot)", 45),
            (7, "L4 (Left Rear Foot)", 45)
        ]

        for ch, name, def_val in channels_info:
            row = QWidget()
            row.setStyleSheet("background-color: #18181b; border: 1px solid #27272a; border-radius: 4px; padding: 2px 4px;")
            r_layout = QHBoxLayout(row)
            r_layout.setContentsMargins(6, 3, 6, 3)

            lbl_name = QLabel(f"Ch {ch}: {name}")
            lbl_name.setFont(QFont("SansSerif", 9, QFont.Bold))
            lbl_name.setStyleSheet("color: #e4e4e7; min-width: 175px;")

            slider = QSlider(Qt.Horizontal)
            slider.setRange(0, 180)
            slider.setValue(def_val)
            slider.setStyleSheet("QSlider::groove:horizontal { border: 1px solid #444; height: 5px; background: #27272a; border-radius: 2px; } QSlider::sub-page:horizontal { background: #38bdf8; border-radius: 2px; } QSlider::handle:horizontal { background: #ffffff; width: 12px; margin: -4px 0; border-radius: 6px; }")

            lbl_val = QLabel(f"{def_val}°")
            lbl_val.setFont(QFont("Monospace", 9, QFont.Bold))
            lbl_val.setStyleSheet("color: #38bdf8; min-width: 40px; text-align: right;")

            offset_val = self.quard_offsets.get(str(ch), 0.0)
            lbl_trim = QLabel(f"Δ{offset_val:+.1f}°")
            lbl_trim.setFont(QFont("Monospace", 8))
            lbl_trim.setStyleSheet("color: #a1a1aa; background-color: #27272a; padding: 1px 4px; border-radius: 3px; min-width: 45px;")

            slider.valueChanged.connect(lambda val, c=ch, l=lbl_val: self.on_channel_slider_changed(c, val, l))

            r_layout.addWidget(lbl_name)
            r_layout.addWidget(slider, 1)
            r_layout.addWidget(lbl_val)
            r_layout.addWidget(lbl_trim)

            ch_layout.addWidget(row)
            self.channel_sliders.append(slider)
            self.channel_readouts.append(lbl_val)
            self.channel_trim_labels.append(lbl_trim)

        ch_scroll.setWidget(ch_container)
        pca_layout.addWidget(ch_scroll, 1)
        pca_box.setLayout(pca_layout)
        right_layout.addWidget(pca_box, 2)

        # 6. Console Log Box
        log_box = QGroupBox("📟 Quard Bot Console & Communication Log")
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

    def set_face(self, face_name):
        if self.oled_widget:
            self.oled_widget.set_face(face_name)
        self.send_command({'face': face_name})
        self.log_to_console(f"😊 OLED Face set to: {face_name}")

    def on_gait_speed_changed(self, val):
        self.step_delay = val
        self.lbl_speed_val.setText(f"{val} ms")
        if self.gait_timer.isActive():
            self.gait_timer.setInterval(self.step_delay)
        self.send_command({'speed': val})

    def on_channel_slider_changed(self, ch, val, lbl_val):
        lbl_val.setText(f"{val}°")
        # Only transmit individual servo updates if not actively running a gait pattern
        if self.active_gait is None:
            self.send_command({'ch': ch, 'angle': val})

    def on_all_slider_changed(self, val):
        self.lbl_all_val.setText(f"{val}°")
        for i, s in enumerate(self.channel_sliders):
            s.blockSignals(True)
            s.setValue(val)
            s.blockSignals(False)
            self.channel_readouts[i].setText(f"{val}°")
        if self.active_gait is None:
            self.send_command({'all': val})

    def start_gait(self, mode):
        if mode not in self.GAIT_PATTERNS:
            return
        self.active_gait = mode
        self.gait_phase = 0
        if self.oled_widget:
            self.oled_widget.set_face('walk')
        self.lbl_gait_status.setText(f"Status: 🚶 {mode.capitalize()} (Active)")
        self.lbl_gait_status.setStyleSheet("background-color: #064e3b; color: #34d399; border: 1px solid #059669; border-radius: 4px; padding: 4px 8px;")

        phase0 = self.GAIT_PATTERNS[mode][0]
        target_map = {ch: deg for ch, deg in phase0.items()}

        def _begin_gait_loop():
            if not self.gait_timer.isActive():
                self.gait_timer.start(self.step_delay)
            self._on_gait_timer_tick()
            self.send_command({'mode': mode, 'speed': self.step_delay})

        self.animate_to_target_angles(target_map, 150, on_finished=_begin_gait_loop)
        self.log_to_console(f"▶️ Smooth Transition into Gait: {mode.upper()} (Step Delay: {self.step_delay}ms)")

    def _on_gait_timer_tick(self):
        if not self.active_gait or self.active_gait not in self.GAIT_PATTERNS:
            return

        pattern = self.GAIT_PATTERNS[self.active_gait]
        phase_data = pattern[self.gait_phase]

        # Apply target angles to sliders without triggering individual HTTP spam
        for ch, deg in phase_data.items():
            if ch < len(self.channel_sliders):
                s = self.channel_sliders[ch]
                s.blockSignals(True)
                s.setValue(deg)
                s.blockSignals(False)
                self.channel_readouts[ch].setText(f"{deg}°")

        if self.oled_widget:
            self.oled_widget.set_walk_phase(self.gait_phase)
        self.lbl_gait_status.setText(f"Status: 🚶 {self.active_gait.capitalize()} [Phase {self.gait_phase + 1}/4]")

        # Alternate delay for lift vs ground phases matching firmware
        next_interval = self.step_delay if (self.gait_phase in [0, 2]) else (self.step_delay // 2)
        self.gait_timer.setInterval(next_interval)

        self.gait_phase = (self.gait_phase + 1) % len(pattern)

    def animate_to_target_angles(self, target_map, duration_ms=300, on_finished=None):
        """Smoothly interpolates all channel sliders to target_map {ch: angle} over duration_ms using cosine easing."""
        if hasattr(self, 'smooth_timer') and self.smooth_timer.isActive():
            self.smooth_timer.stop()

        self.smooth_start_angles = {i: s.value() for i, s in enumerate(self.channel_sliders)}
        self.smooth_target_map = {i: target_map.get(i, self.smooth_start_angles.get(i, 45)) for i in range(len(self.channel_sliders))}
        self.smooth_duration_ms = max(50, duration_ms)
        self.smooth_start_time = time.time()
        self.smooth_on_finished = on_finished

        if not hasattr(self, 'smooth_timer'):
            self.smooth_timer = QTimer(self)
            self.smooth_timer.timeout.connect(self._on_smooth_timer_tick)
        
        self.smooth_timer.start(16)

    def _on_smooth_timer_tick(self):
        elapsed = (time.time() - self.smooth_start_time) * 1000.0
        progress = elapsed / self.smooth_duration_ms
        if progress >= 1.0:
            progress = 1.0
            self.smooth_timer.stop()

        # Cosine easing for organic, silky-smooth motion transition
        t_smooth = 0.5 - 0.5 * math.cos(progress * math.pi)

        for i, s in enumerate(self.channel_sliders):
            start_a = self.smooth_start_angles.get(i, 45)
            target_a = self.smooth_target_map.get(i, 45)
            interp_a = int(round(start_a + (target_a - start_a) * t_smooth))
            s.blockSignals(True)
            s.setValue(interp_a)
            s.blockSignals(False)
            self.channel_readouts[i].setText(f"{interp_a}°")

        # Update Master All Slider
        if len(self.channel_sliders) > 0:
            avg_val = int(round(sum(s.value() for s in self.channel_sliders) / len(self.channel_sliders)))
            self.slider_all.blockSignals(True)
            self.slider_all.setValue(avg_val)
            self.slider_all.blockSignals(False)
            self.lbl_all_val.setText(f"{avg_val}°")

        if progress >= 1.0 and self.smooth_on_finished:
            cb = self.smooth_on_finished
            self.smooth_on_finished = None
            cb()

    def stand_pose(self):
        if self.gait_timer.isActive():
            self.gait_timer.stop()
        if self.wave_timer.isActive():
            self.wave_timer.stop()
        self.active_gait = None

        if self.oled_widget:
            self.oled_widget.set_face('happy')
        self.lbl_gait_status.setText("Status: ⏹ Stand (45° Neutral)")
        self.lbl_gait_status.setStyleSheet("background-color: #1c1917; color: #f59e0b; border: 1px solid #44403c; border-radius: 4px; padding: 4px 8px;")
        
        target_map = {i: 45 for i in range(8)}
        self.animate_to_target_angles(target_map, 300, on_finished=lambda: self.send_command({'mode': 'stand'}))
        self.log_to_console("🧍 Quard Bot: Smooth Transition to Stand Pose (45° Neutral).")

    def zero_pose(self):
        if self.gait_timer.isActive():
            self.gait_timer.stop()
        if self.wave_timer.isActive():
            self.wave_timer.stop()
        self.active_gait = None

        if self.oled_widget:
            self.oled_widget.set_face('sleepy')
        self.lbl_gait_status.setText("Status: 🎯 Base Zero (0°)")
        self.lbl_gait_status.setStyleSheet("background-color: #1c1917; color: #f59e0b; border: 1px solid #44403c; border-radius: 4px; padding: 4px 8px;")
        
        target_map = {i: 0 for i in range(8)}
        self.animate_to_target_angles(target_map, 300, on_finished=lambda: self.send_command({'all': 0}))
        self.log_to_console("🎯 Quard Bot: Smooth Transition to Base Zero Pose (0°).")

    def crouch_pose(self):
        if self.gait_timer.isActive():
            self.gait_timer.stop()
        if self.wave_timer.isActive():
            self.wave_timer.stop()
        self.active_gait = None

        if self.oled_widget:
            self.oled_widget.set_face('cute')
        self.lbl_gait_status.setText("Status: 🦆 Crouch (30° Stance)")
        self.lbl_gait_status.setStyleSheet("background-color: #1c1917; color: #38bdf8; border: 1px solid #0284c7; border-radius: 4px; padding: 4px 8px;")
        
        target_map = {i: 30 for i in range(8)}
        self.animate_to_target_angles(target_map, 300, on_finished=lambda: self.send_command({'all': 30}))
        self.log_to_console("🦆 Quard Bot: Smooth Transition to Crouch Stance (30°).")

    def high_pose(self):
        if self.gait_timer.isActive():
            self.gait_timer.stop()
        if self.wave_timer.isActive():
            self.wave_timer.stop()
        self.active_gait = None

        if self.oled_widget:
            self.oled_widget.set_face('happy')
        self.lbl_gait_status.setText("Status: 🦒 High Stance (60°)")
        self.lbl_gait_status.setStyleSheet("background-color: #1c1917; color: #34d399; border: 1px solid #059669; border-radius: 4px; padding: 4px 8px;")
        
        target_map = {i: 60 for i in range(8)}
        self.animate_to_target_angles(target_map, 300, on_finished=lambda: self.send_command({'all': 60}))
        self.log_to_console("🦒 Quard Bot: Smooth Transition to High Stance (60°).")

    def play_wave_animation(self):
        if self.gait_timer.isActive():
            self.gait_timer.stop()
        if self.wave_timer.isActive():
            self.wave_timer.stop()
        self.active_gait = 'wave'
        if self.oled_widget:
            self.oled_widget.set_face('wave')
        self.lbl_gait_status.setText("Status: 👋 Wave Animation")
        self.lbl_gait_status.setStyleSheet("background-color: #1c1917; color: #38bdf8; border: 1px solid #0284c7; border-radius: 4px; padding: 4px 8px;")

        # Smoothly transition to 60° high stance first before waving right front foot
        target_map = {i: 60 for i in range(8)}
        def _start_waving():
            self.wave_step = 0
            self.wave_timer.start(200)
            self.send_command({'mode': 'wave'})

        self.animate_to_target_angles(target_map, 250, on_finished=_start_waving)
        self.log_to_console("👋 Quard Bot: Smoothly Transitioning & Playing Wave Animation.")

    def _on_wave_timer_tick(self):
        # Oscillate Ch 5 (Right Front Foot) between 0° and 35°
        wave_angles = [0, 35, 0, 35, 0, 35, 45]
        if self.wave_step < len(wave_angles):
            angle = wave_angles[self.wave_step]
            if len(self.channel_sliders) > 5:
                s = self.channel_sliders[5]
                s.blockSignals(True)
                s.setValue(angle)
                s.blockSignals(False)
                self.channel_readouts[5].setText(f"{angle}°")
            self.wave_step += 1
        else:
            self.wave_timer.stop()
            self.stand_pose()

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
                    try:
                        self.update_status_signal.emit(True, {"type": "cmd", "params": params, "reply": reply})
                    except (RuntimeError, Exception):
                        pass
            except Exception as e:
                try:
                    self.update_status_signal.emit(False, {"type": "cmd", "params": params, "error": str(e)})
                except (RuntimeError, Exception):
                    pass

        threading.Thread(target=worker, daemon=True).start()

    def ping_quard_bot(self):
        host = self.input_host.text().strip()
        if not host.startswith("http://") and not host.startswith("https://"):
            host = "http://" + host
        url_status = f"{host.rstrip('/')}/status"

        def worker():
            try:
                req = urllib.request.Request(url_status)
                with urllib.request.urlopen(req, timeout=1.5) as resp:
                    raw_data = resp.read().decode('utf-8', errors='ignore')
                    try:
                        telemetry = json.loads(raw_data)
                    except Exception:
                        telemetry = {"status": "ok", "raw": raw_data}
                    try:
                        self.update_status_signal.emit(True, {"type": "ping", "telemetry": telemetry})
                    except (RuntimeError, Exception):
                        pass
            except Exception:
                # Fallback to root /
                try:
                    url_root = f"{host.rstrip('/')}/"
                    req = urllib.request.Request(url_root)
                    with urllib.request.urlopen(req, timeout=1.2) as resp:
                        try:
                            self.update_status_signal.emit(True, {"type": "ping", "telemetry": {"status": "online"}})
                        except (RuntimeError, Exception):
                            pass
                except Exception as e2:
                    try:
                        self.update_status_signal.emit(False, {"type": "ping", "error": str(e2)})
                    except (RuntimeError, Exception):
                        pass

        threading.Thread(target=worker, daemon=True).start()

    def ping_robot(self):
        """Compatibility alias for ping_quard_bot."""
        self.ping_quard_bot()

    def set_status(self, is_online, info_dict):
        self.is_online = is_online
        msg_type = info_dict.get("type", "")

        if is_online:
            telemetry = info_dict.get("telemetry", {})
            rssi = telemetry.get("rssi", None)
            rssi_str = f" ({rssi} dBm)" if rssi is not None else ""
            self.lbl_status.setText(f"🟢 Connected{rssi_str}")
            self.lbl_status.setStyleSheet("background-color: #064e3b; color: #34d399; padding: 4px 10px; border-radius: 4px;")

            if telemetry:
                ip = telemetry.get("ip", "sesame-robot.local")
                mode = telemetry.get("mode", "Stand")
                face = telemetry.get("face", "happy")
                speed = telemetry.get("step_delay", self.step_delay)
                self.lbl_telemetry.setText(f"📡 IP: {ip} | RSSI: {rssi or '--'} dBm | Mode: {mode} | Speed: {speed}ms")
        else:
            self.lbl_status.setText("🔴 Offline")
            self.lbl_status.setStyleSheet("background-color: #4c0519; color: #f43f5e; padding: 4px 10px; border-radius: 4px;")
            self.lbl_telemetry.setText("📡 Telemetry: Offline (check Wi-Fi / AP connection)")

        if msg_type == "cmd":
            if "reply" in info_dict:
                self.log_to_console(f"Out: {info_dict['params']} | Reply: {info_dict['reply']}")
            elif "error" in info_dict:
                self.log_to_console(f"⚠️ Out: {info_dict['params']} | Error: {info_dict['error']}")

    def log_to_console(self, text):
        ts = time.strftime("[%H:%M:%S] ")
        self.console_log.append(ts + text)
        self.console_log.moveCursor(QTextCursor.End)

    def get_joint_radians(self):
        rads = {}
        if not hasattr(self, 'channel_sliders') or len(self.channel_sliders) < 8:
            return rads
        mapping = [
            (0, 'joint_r1_hip'),
            (1, 'joint_r2_hip'),
            (2, 'joint_l1_hip'),
            (3, 'joint_l2_hip'),
            (4, 'joint_r4_foot'),
            (5, 'joint_r3_foot'),
            (6, 'joint_l3_foot'),
            (7, 'joint_l4_foot'),
        ]
        # Hip signs for outward diagonal stance splay:
        # Ch 0 (R1 Right Front): +1.0
        # Ch 1 (R2 Right Rear):  -1.0
        # Ch 2 (L1 Left Front):  -1.0
        # Ch 3 (L2 Left Rear):   +1.0
        # Foot signs for downward pitch onto ground:
        # Ch 4 (R4 Right Rear):  +1.0
        # Ch 5 (R3 Right Front): -1.0
        # Ch 6 (L3 Left Front):  +1.0
        # Ch 7 (L4 Left Rear):   -1.0
        sign_map = {
            0: +1.0,
            1: -1.0,
            2: -1.0,
            3: +1.0,
            4: -1.0,
            5: +1.0,
            6: -1.0,
            7: +1.0,
        }
        for ch, j_name in mapping:
            offset = self.quard_offsets.get(str(ch), 0.0) if hasattr(self, 'quard_offsets') else 0.0
            val = self.channel_sliders[ch].value() + offset
            sign = sign_map.get(ch, 1.0)
            delta = val * math.pi / 180.0 * sign
            rads[j_name] = max(-1.2, min(1.2, delta))
        return rads

    def sync_current_stance(self):
        """Zero-tare all 8 quadruped servo channels relative to reference pose."""
        if not hasattr(self, 'channel_sliders') or len(self.channel_sliders) < 8:
            return
        for ch in range(8):
            curr_deg = self.channel_sliders[ch].value()
            ref = 0.0 if curr_deg < 22.0 else 45.0
            self.quard_offsets[str(ch)] = round(curr_deg - ref, 1)
            if hasattr(self, 'channel_trim_labels') and ch < len(self.channel_trim_labels):
                self.channel_trim_labels[ch].setText(f"Δ{self.quard_offsets[str(ch)]:+.1f}°")
        self.save_quard_offsets()
        self.log_to_console(f"🎯 Quard Bot Stance Synced: {self.quard_offsets}")

    def reset_trims(self):
        """Reset all 8 trim offsets back to 0.0°."""
        self.quard_offsets = {str(i): 0.0 for i in range(8)}
        if hasattr(self, 'channel_trim_labels'):
            for lbl in self.channel_trim_labels:
                lbl.setText("Δ+0.0°")
        self.save_quard_offsets()
        self.log_to_console("🔄 All Quard Bot trim offsets reset to 0.0°.")

    def load_quard_offsets(self):
        if os.path.exists(CALIB_FILE_PATH):
            try:
                with open(CALIB_FILE_PATH, 'r') as f:
                    data = json.load(f)
                    if "quard_offsets" in data and isinstance(data["quard_offsets"], dict):
                        self.quard_offsets = {str(k): float(v) for k, v in data["quard_offsets"].items()}
            except Exception as e:
                print(f"Error loading quard_offsets: {e}")

    def save_quard_offsets(self):
        p = self.parent()
        while p is not None and not hasattr(p, 'save_calibration'):
            p = p.parent()
        if p and hasattr(p, 'save_calibration'):
            p.save_calibration()
        else:
            data = {}
            if os.path.exists(CALIB_FILE_PATH):
                try:
                    with open(CALIB_FILE_PATH, 'r') as f:
                        data = json.load(f)
                except Exception:
                    pass
            data["quard_offsets"] = self.quard_offsets
            try:
                with open(CALIB_FILE_PATH, 'w') as f:
                    json.dump(data, f, indent=2)
            except Exception as e:
                print(f"Error saving quard_offsets: {e}")


class ESP32SensorNodeWidget(QWidget):
    update_data_signal = pyqtSignal(dict)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.main_gui = parent

        self.sensor_state = {
            "temp_dht": 24.5,
            "humidity": 48.0,
            "temp_bmp": 24.8,
            "pressure": 1013.25,
            "temperature": 24.5,
            "dht_error": False,
            "bmp_error": False,
            "gas": 415,
            "water": 120,
            "flame": 1,
            "ax": 120, "ay": -50, "az": 16384,
            "gx": 10, "gy": 0, "gz": -20,
            "accel": {"x": 0.12, "y": -0.05, "z": 9.81},
            "gyro": {"x": 0.01, "y": 0.00, "z": -0.02},
            "pitch": -0.3,
            "roll": 0.7,
            "raw_pitch": -0.3,
            "raw_roll": 0.7,
            "lat": 17.385044,
            "lng": 78.486671,
            "gps_valid": False,
            "gps_sats": 0,
            "alert": "OK",
            "sd_ok": True,
            "uptime_ms": 0,
            "air_quality": "Clean",
            "connected": False,
            "wifi_connected": False,
            "wifi_ip": "192.168.1.100",
            "simulating": True,
            "port": "/dev/ttyUSB0",
            "baudrate": 115200,
            "raw_log": []
        }

        self.serial_thread = None
        self.wifi_thread = None
        self.imu_offsets = {"pitch": 0.0, "roll": 0.0, "yaw": 0.0}
        self.load_imu_offsets()
        self.update_data_signal.connect(self.on_data_updated)

        self.init_ui()

        # Timer to drive simulation when hardware is not connected
        self.sim_timer = QTimer(self)
        self.sim_timer.timeout.connect(self.sim_tick)
        self.sim_timer.start(1000)

    def init_ui(self):
        main_layout = QVBoxLayout(self)
        main_layout.setContentsMargins(8, 8, 8, 8)
        main_layout.setSpacing(8)

        # ── 0. Emergency Fire Alert Banner (Hidden by default) ────────────
        self.lbl_fire_banner = QLabel("🚨 EMERGENCY: FIRE DETECTED ON ROVER NODE! 🚨")
        self.lbl_fire_banner.setAlignment(Qt.AlignCenter)
        self.lbl_fire_banner.setFont(QFont("Arial", 13, QFont.Bold))
        self.lbl_fire_banner.setStyleSheet("""
            background-color: #b91c1c;
            color: #ffffff;
            padding: 8px 16px;
            border-radius: 6px;
            border: 2px solid #ef4444;
        """)
        self.lbl_fire_banner.setVisible(False)
        main_layout.addWidget(self.lbl_fire_banner)

        # ── 1. Dual Connection Box: Wi-Fi (HTTP /data) & USB Serial ────────
        conn_box = QGroupBox("🔌 Rover Wi-Fi & Serial Connection")
        conn_box.setStyleSheet("""
            QGroupBox { font-weight: bold; font-size: 13px; border: 1px solid #444; border-radius: 6px; margin-top: 6px; padding-top: 10px; background-color: #242424; color: #38bdf8; }
            QGroupBox::title { subcontrol-origin: margin; padding: 0 6px; color: #38bdf8; }
        """)
        conn_layout = QVBoxLayout(conn_box)
        conn_layout.setSpacing(6)

        # Row 1: Wi-Fi Connection
        wifi_row = QHBoxLayout()
        lbl_wifi = QLabel("🌐 Wi-Fi IP / URL:")
        lbl_wifi.setStyleSheet("color: #cccccc; font-weight: bold; font-size: 12px;")
        self.txt_wifi_ip = QLineEdit("192.168.1.100")
        self.txt_wifi_ip.setPlaceholderText("e.g. 192.168.1.50 or http://192.168.1.50")
        self.txt_wifi_ip.setStyleSheet("background-color: #18181b; color: #38bdf8; padding: 4px 8px; border: 1px solid #3f3f46; border-radius: 4px; font-weight: bold;")

        self.btn_wifi_connect = QPushButton("🌐 Connect Wi-Fi (/data)")
        self.btn_wifi_connect.setStyleSheet("""
            QPushButton { background-color: #0284c7; color: white; font-weight: bold; padding: 5px 12px; border-radius: 4px; }
            QPushButton:hover { background-color: #0369a1; }
        """)
        self.btn_wifi_connect.clicked.connect(self.toggle_wifi_connection)

        self.lbl_wifi_status = QLabel("Wi-Fi: Offline")
        self.lbl_wifi_status.setStyleSheet("background-color: #3f3f46; color: #d4d4d8; font-weight: bold; padding: 4px 8px; border-radius: 4px;")

        wifi_row.addWidget(lbl_wifi)
        wifi_row.addWidget(self.txt_wifi_ip, 2)
        wifi_row.addWidget(self.btn_wifi_connect)
        wifi_row.addWidget(self.lbl_wifi_status)
        wifi_row.addStretch()
        conn_layout.addLayout(wifi_row)

        # Row 2: Serial Connection & Sim Mode
        serial_row = QHBoxLayout()
        lbl_port = QLabel("🔌 Port:")
        lbl_port.setStyleSheet("color: #aaaaaa; font-weight: bold; font-size: 12px;")
        self.combo_port = QComboBox()
        self.combo_port.addItems(["/dev/ttyUSB0", "/dev/ttyUSB1", "/dev/ttyACM0", "/dev/ttyACM1"])
        self.combo_port.setStyleSheet("background-color: #18181b; color: #ffffff; padding: 3px 6px; border: 1px solid #3f3f46; border-radius: 4px;")

        lbl_baud = QLabel("Baud:")
        lbl_baud.setStyleSheet("color: #aaaaaa; font-weight: bold; font-size: 12px;")
        self.combo_baud = QComboBox()
        self.combo_baud.addItems(["115200", "9600", "57600", "230400"])
        self.combo_baud.setStyleSheet("background-color: #18181b; color: #ffffff; padding: 3px 6px; border: 1px solid #3f3f46; border-radius: 4px;")

        self.btn_connect = QPushButton("Connect Serial")
        self.btn_connect.setStyleSheet("""
            QPushButton { background-color: #2563eb; color: white; font-weight: bold; padding: 5px 10px; border-radius: 4px; }
            QPushButton:hover { background-color: #1d4ed8; }
        """)
        self.btn_connect.clicked.connect(self.toggle_serial_connection)

        self.btn_sim = QPushButton("▶️ Simulating Data")
        self.btn_sim.setStyleSheet("""
            QPushButton { background-color: #d97706; color: white; font-weight: bold; padding: 5px 10px; border-radius: 4px; }
            QPushButton:hover { background-color: #b45309; }
        """)
        self.btn_sim.clicked.connect(self.toggle_sim)

        self.lbl_status = QLabel("Simulating")
        self.lbl_status.setStyleSheet("background-color: #064e3b; color: #34d399; font-weight: bold; padding: 4px 8px; border-radius: 4px;")

        serial_row.addWidget(lbl_port)
        serial_row.addWidget(self.combo_port)
        serial_row.addWidget(lbl_baud)
        serial_row.addWidget(self.combo_baud)
        serial_row.addWidget(self.btn_connect)
        serial_row.addWidget(self.btn_sim)
        serial_row.addStretch()
        serial_row.addWidget(self.lbl_status)
        conn_layout.addLayout(serial_row)

        main_layout.addWidget(conn_box)

        # ── 2. Scrollable Sensor Cards Area ────────────────────────────────
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setStyleSheet("QScrollArea { border: none; background-color: transparent; }")

        container = QWidget()
        grid_layout = QGridLayout(container)
        grid_layout.setContentsMargins(0, 0, 0, 0)
        grid_layout.setSpacing(8)

        # ── Card A: Climate (DHT11 & BMP-280) ──────────────────────────────
        climate_box = QGroupBox("🌡️ Climate & Atmospheric (DHT11 & BMP-280)")
        climate_box.setStyleSheet("""
            QGroupBox { font-weight: bold; font-size: 12px; border: 1px solid #3f3f46; border-radius: 6px; margin-top: 6px; padding-top: 10px; background-color: #27272a; color: #38bdf8; }
            QGroupBox::title { subcontrol-origin: margin; padding: 0 4px; color: #38bdf8; }
        """)
        c_layout = QGridLayout(climate_box)

        c_layout.addWidget(QLabel("DHT11 Temp:"), 0, 0)
        self.lbl_temp_val = QLabel("24.50 °C (76.10 °F)")
        self.lbl_temp_val.setFont(QFont("Monospace", 12, QFont.Bold))
        self.lbl_temp_val.setStyleSheet("color: #38bdf8; background-color: #18181b; padding: 4px 8px; border-radius: 4px;")
        c_layout.addWidget(self.lbl_temp_val, 0, 1)

        c_layout.addWidget(QLabel("DHT11 Humidity:"), 1, 0)
        self.lbl_hum_val = QLabel("48.00 %")
        self.lbl_hum_val.setFont(QFont("Monospace", 12, QFont.Bold))
        self.lbl_hum_val.setStyleSheet("color: #34d399; background-color: #18181b; padding: 4px 8px; border-radius: 4px;")
        c_layout.addWidget(self.lbl_hum_val, 1, 1)

        c_layout.addWidget(QLabel("BMP280 Temp:"), 2, 0)
        self.lbl_bmp_temp_val = QLabel("24.80 °C")
        self.lbl_bmp_temp_val.setFont(QFont("Monospace", 12, QFont.Bold))
        self.lbl_bmp_temp_val.setStyleSheet("color: #f59e0b; background-color: #18181b; padding: 4px 8px; border-radius: 4px;")
        c_layout.addWidget(self.lbl_bmp_temp_val, 2, 1)

        c_layout.addWidget(QLabel("Barometric Pressure:"), 3, 0)
        self.lbl_pressure_val = QLabel("1013.25 hPa")
        self.lbl_pressure_val.setFont(QFont("Monospace", 12, QFont.Bold))
        self.lbl_pressure_val.setStyleSheet("color: #e0e7ff; background-color: #18181b; padding: 4px 8px; border-radius: 4px;")
        c_layout.addWidget(self.lbl_pressure_val, 3, 1)

        self.lbl_dht_status = QLabel("DHT11: OK | BMP280: OK")
        self.lbl_dht_status.setStyleSheet("background-color: #064e3b; color: #34d399; font-weight: bold; padding: 3px 6px; border-radius: 4px;")
        c_layout.addWidget(self.lbl_dht_status, 4, 0, 1, 2)

        grid_layout.addWidget(climate_box, 0, 0)

        # ── Card B: Hazards (Gas, Water, Flame) ────────────────────────────
        hazard_box = QGroupBox("⚠️ Environmental Hazards (MQ Gas, Water, Flame)")
        hazard_box.setStyleSheet("""
            QGroupBox { font-weight: bold; font-size: 12px; border: 1px solid #3f3f46; border-radius: 6px; margin-top: 6px; padding-top: 10px; background-color: #27272a; color: #fbbf24; }
            QGroupBox::title { subcontrol-origin: margin; padding: 0 4px; color: #fbbf24; }
        """)
        h_layout = QGridLayout(hazard_box)

        h_layout.addWidget(QLabel("MQ Gas Reading:"), 0, 0)
        self.lbl_gas_val = QLabel("415 / 4095")
        self.lbl_gas_val.setFont(QFont("Monospace", 12, QFont.Bold))
        self.lbl_gas_val.setStyleSheet("color: #fbbf24; background-color: #18181b; padding: 4px 8px; border-radius: 4px;")
        h_layout.addWidget(self.lbl_gas_val, 0, 1)

        self.lbl_gas_badge = QLabel("🟢 Clean Air")
        self.lbl_gas_badge.setStyleSheet("background-color: #064e3b; color: #34d399; font-weight: bold; padding: 3px 6px; border-radius: 4px;")
        h_layout.addWidget(self.lbl_gas_badge, 0, 2)

        h_layout.addWidget(QLabel("Water Level SIG:"), 1, 0)
        self.lbl_water_val = QLabel("120 / 4095")
        self.lbl_water_val.setFont(QFont("Monospace", 12, QFont.Bold))
        self.lbl_water_val.setStyleSheet("color: #60a5fa; background-color: #18181b; padding: 4px 8px; border-radius: 4px;")
        h_layout.addWidget(self.lbl_water_val, 1, 1)

        self.lbl_water_badge = QLabel("🟢 Dry")
        self.lbl_water_badge.setStyleSheet("background-color: #064e3b; color: #34d399; font-weight: bold; padding: 3px 6px; border-radius: 4px;")
        h_layout.addWidget(self.lbl_water_badge, 1, 2)

        h_layout.addWidget(QLabel("Flame Sensor DO:"), 2, 0)
        self.lbl_flame_val = QLabel("HIGH (Clear)")
        self.lbl_flame_val.setFont(QFont("Monospace", 12, QFont.Bold))
        self.lbl_flame_val.setStyleSheet("color: #a7f3d0; background-color: #18181b; padding: 4px 8px; border-radius: 4px;")
        h_layout.addWidget(self.lbl_flame_val, 2, 1)

        self.lbl_flame_badge = QLabel("✅ Clear")
        self.lbl_flame_badge.setStyleSheet("background-color: #064e3b; color: #34d399; font-weight: bold; padding: 3px 6px; border-radius: 4px;")
        h_layout.addWidget(self.lbl_flame_badge, 2, 2)

        grid_layout.addWidget(hazard_box, 0, 1)

        # ── Card C: MPU-6050 6-DOF IMU ────────────────────────────────────
        mpu_box = QGroupBox("🧭 MPU6050 6-DOF IMU (Motion & Tilt)")
        mpu_box.setStyleSheet("""
            QGroupBox { font-weight: bold; font-size: 12px; border: 1px solid #3f3f46; border-radius: 6px; margin-top: 6px; padding-top: 10px; background-color: #27272a; color: #a78bfa; }
            QGroupBox::title { subcontrol-origin: margin; padding: 0 4px; color: #a78bfa; }
        """)
        m_layout = QGridLayout(mpu_box)

        m_layout.addWidget(QLabel("Accel (m/s²):"), 0, 0)
        self.lbl_accel_val = QLabel("X: 0.12  |  Y: -0.05  |  Z: 9.81")
        self.lbl_accel_val.setFont(QFont("Monospace", 11, QFont.Bold))
        self.lbl_accel_val.setStyleSheet("color: #a78bfa; background-color: #18181b; padding: 4px 8px; border-radius: 4px;")
        m_layout.addWidget(self.lbl_accel_val, 0, 1)

        m_layout.addWidget(QLabel("Gyro (rad/s):"), 1, 0)
        self.lbl_gyro_val = QLabel("X: 0.010 |  Y: 0.000  |  Z: -0.020")
        self.lbl_gyro_val.setFont(QFont("Monospace", 11, QFont.Bold))
        self.lbl_gyro_val.setStyleSheet("color: #a78bfa; background-color: #18181b; padding: 4px 8px; border-radius: 4px;")
        m_layout.addWidget(self.lbl_gyro_val, 1, 1)

        m_layout.addWidget(QLabel("Estimated Tilt:"), 2, 0)
        self.lbl_tilt_val = QLabel("Pitch: -0.3°   |   Roll: 0.7°")
        self.lbl_tilt_val.setFont(QFont("Monospace", 11, QFont.Bold))
        self.lbl_tilt_val.setStyleSheet("color: #38bdf8; background-color: #0c4a6e; padding: 4px 8px; border-radius: 4px;")
        m_layout.addWidget(self.lbl_tilt_val, 2, 1)

        self.btn_tare_imu = QPushButton("🎯 Tare Horizon")
        self.btn_tare_imu.setStyleSheet("""
            QPushButton { background-color: #4f46e5; color: white; font-weight: bold; border-radius: 4px; padding: 4px 8px; }
            QPushButton:hover { background-color: #4338ca; }
        """)
        self.btn_tare_imu.clicked.connect(self.tare_imu_horizon)
        m_layout.addWidget(self.btn_tare_imu, 3, 0, 1, 2)

        grid_layout.addWidget(mpu_box, 1, 0)

        # ── Card D: GPS NEO-6M Navigation & System ────────────────────────
        gps_box = QGroupBox("🛰️ GPS NEO-6M Navigation & Node Status")
        gps_box.setStyleSheet("""
            QGroupBox { font-weight: bold; font-size: 12px; border: 1px solid #3f3f46; border-radius: 6px; margin-top: 6px; padding-top: 10px; background-color: #27272a; color: #4ade80; }
            QGroupBox::title { subcontrol-origin: margin; padding: 0 4px; color: #4ade80; }
        """)
        g_layout = QGridLayout(gps_box)

        g_layout.addWidget(QLabel("GPS Fix:"), 0, 0)
        self.lbl_gps_status = QLabel("🔴 Searching / No Fix")
        self.lbl_gps_status.setStyleSheet("background-color: #451a1a; color: #f87171; font-weight: bold; padding: 3px 6px; border-radius: 4px;")
        g_layout.addWidget(self.lbl_gps_status, 0, 1)

        g_layout.addWidget(QLabel("Satellites:"), 0, 2)
        self.lbl_gps_sats = QLabel("0 sats")
        self.lbl_gps_sats.setFont(QFont("Monospace", 11, QFont.Bold))
        self.lbl_gps_sats.setStyleSheet("color: #d1d5db; background-color: #18181b; padding: 3px 6px; border-radius: 4px;")
        g_layout.addWidget(self.lbl_gps_sats, 0, 3)

        g_layout.addWidget(QLabel("Coordinates:"), 1, 0)
        self.lbl_gps_coords = QLabel("Lat: 0.000000 | Lng: 0.000000")
        self.lbl_gps_coords.setFont(QFont("Monospace", 11, QFont.Bold))
        self.lbl_gps_coords.setStyleSheet("color: #4ade80; background-color: #18181b; padding: 4px 8px; border-radius: 4px;")
        g_layout.addWidget(self.lbl_gps_coords, 1, 1, 1, 3)

        g_layout.addWidget(QLabel("microSD Storage:"), 2, 0)
        self.lbl_sd_status = QLabel("💾 SD: Ready (/log.csv)")
        self.lbl_sd_status.setStyleSheet("background-color: #064e3b; color: #34d399; font-weight: bold; padding: 3px 6px; border-radius: 4px;")
        g_layout.addWidget(self.lbl_sd_status, 2, 1)

        g_layout.addWidget(QLabel("Alert / Uptime:"), 2, 2)
        self.lbl_system_status = QLabel("Alert: OK | Uptime: 0s")
        self.lbl_system_status.setFont(QFont("Monospace", 10, QFont.Bold))
        self.lbl_system_status.setStyleSheet("color: #38bdf8; background-color: #18181b; padding: 3px 6px; border-radius: 4px;")
        g_layout.addWidget(self.lbl_system_status, 2, 3)

        grid_layout.addWidget(gps_box, 1, 1)

        scroll.setWidget(container)
        main_layout.addWidget(scroll, 2)

        # ── 3. Rover Telemetry Console Log ─────────────────────────────────
        log_box = QGroupBox("📜 Rover Telemetry Stream Console (Serial / HTTP JSON)")
        log_box.setStyleSheet("""
            QGroupBox { font-weight: bold; font-size: 11px; border: 1px solid #333; border-radius: 6px; margin-top: 4px; padding-top: 6px; background-color: #141414; color: #38bdf8; }
            QGroupBox::title { subcontrol-origin: margin; padding: 0 4px; color: #38bdf8; }
        """)
        log_layout = QVBoxLayout(log_box)

        self.log_console = QTextEdit()
        self.log_console.setReadOnly(True)
        self.log_console.setMaximumHeight(140)
        self.log_console.setFont(QFont("Monospace", 9))
        self.log_console.setStyleSheet("QTextEdit { background-color: #09090b; color: #38bdf8; border: 1px solid #27272a; border-radius: 4px; padding: 4px; }")

        btn_clear = QPushButton("Clear Console")
        btn_clear.setStyleSheet("background-color: #27272a; color: #a1a1aa; font-size: 11px; padding: 2px 8px; border-radius: 3px;")
        btn_clear.clicked.connect(self.log_console.clear)

        log_layout.addWidget(self.log_console)
        log_layout.addWidget(btn_clear, 0, Qt.AlignRight)
        main_layout.addWidget(log_box, 1)

    def toggle_wifi_connection(self):
        if self.sensor_state.get("wifi_connected", False):
            self.sensor_state["wifi_connected"] = False
            self.btn_wifi_connect.setText("🌐 Connect Wi-Fi (/data)")
            self.btn_wifi_connect.setStyleSheet("QPushButton { background-color: #0284c7; color: white; font-weight: bold; padding: 5px 12px; border-radius: 4px; }")
            self.lbl_wifi_status.setText("Wi-Fi: Disconnected")
            self.lbl_wifi_status.setStyleSheet("background-color: #3f3f46; color: #d4d4d8; font-weight: bold; padding: 4px 8px; border-radius: 4px;")
        else:
            ip_text = self.txt_wifi_ip.text().strip()
            if not ip_text:
                ip_text = "192.168.1.100"
                self.txt_wifi_ip.setText(ip_text)

            self.sensor_state["wifi_connected"] = True
            self.sensor_state["simulating"] = False
            self.btn_wifi_connect.setText("⏹ Disconnect Wi-Fi")
            self.btn_wifi_connect.setStyleSheet("QPushButton { background-color: #dc2626; color: white; font-weight: bold; padding: 5px 12px; border-radius: 4px; }")
            self.lbl_wifi_status.setText("Wi-Fi: Connecting...")
            self.lbl_wifi_status.setStyleSheet("background-color: #78350f; color: #fde047; font-weight: bold; padding: 4px 8px; border-radius: 4px;")
            self.btn_sim.setText("▶️ Simulating Data")
            self.lbl_status.setText(f"Live Wi-Fi ({ip_text})")
            self.lbl_status.setStyleSheet("background-color: #064e3b; color: #34d399; font-weight: bold; padding: 4px 8px; border-radius: 4px;")

            def wifi_polling_worker():
                while self.sensor_state.get("wifi_connected", False):
                    cur_ip = self.txt_wifi_ip.text().strip()
                    if not cur_ip.startswith("http://") and not cur_ip.startswith("https://"):
                        url = f"http://{cur_ip}/data"
                    elif cur_ip.endswith("/data"):
                        url = cur_ip
                    else:
                        url = f"{cur_ip.rstrip('/')}/data"

                    try:
                        req = urllib.request.Request(url, headers={"User-Agent": "RobotArm-GUI/1.0"})
                        with urllib.request.urlopen(req, timeout=1.5) as resp:
                            if resp.status == 200:
                                raw = resp.read().decode('utf-8', errors='ignore')
                                self.parse_sensor_json(raw)
                                self.sensor_state["wifi_connected"] = True
                                self.update_data_signal.emit(self.sensor_state)
                    except Exception as err:
                        self.sensor_state["raw_log"].append(f"[Wi-Fi Error] {err}")
                        if len(self.sensor_state["raw_log"]) > 100:
                            self.sensor_state["raw_log"].pop(0)
                        self.update_data_signal.emit(self.sensor_state)
                    time.sleep(0.5)

            threading.Thread(target=wifi_polling_worker, daemon=True).start()

    def parse_sensor_json(self, raw_str):
        try:
            d = json.loads(raw_str)
        except Exception:
            return

        self.sensor_state["raw_log"].append(f"[JSON] {raw_str[:90]}...")
        if len(self.sensor_state["raw_log"]) > 100:
            self.sensor_state["raw_log"].pop(0)

        # Climate
        t_dht = float(d.get("temp_dht", self.sensor_state["temp_dht"]))
        hum = float(d.get("humidity", self.sensor_state["humidity"]))
        t_bmp = float(d.get("temp_bmp", self.sensor_state["temp_bmp"]))
        pres = float(d.get("pressure", self.sensor_state["pressure"]))
        self.sensor_state["temp_dht"] = t_dht
        self.sensor_state["temperature"] = t_dht
        self.sensor_state["humidity"] = hum
        self.sensor_state["temp_bmp"] = t_bmp
        self.sensor_state["pressure"] = pres

        # Hazards
        gas_val = int(d.get("air", d.get("gas", self.sensor_state["gas"])))
        water_val = int(d.get("water", self.sensor_state["water"]))
        flame_val = int(d.get("flame", self.sensor_state["flame"]))
        self.sensor_state["gas"] = gas_val
        self.sensor_state["water"] = water_val
        self.sensor_state["flame"] = flame_val

        # Gas Quality
        if gas_val < 600:
            self.sensor_state["air_quality"] = "Clean"
        elif gas_val < 1500:
            self.sensor_state["air_quality"] = "Moderate"
        else:
            self.sensor_state["air_quality"] = "Danger"

        # IMU Accel & Gyro
        ax = float(d.get("ax", 0))
        ay = float(d.get("ay", 0))
        az = float(d.get("az", 16384))
        gx = float(d.get("gx", 0))
        gy = float(d.get("gy", 0))
        gz = float(d.get("gz", 0))

        # Convert MPU6050 raw LSB (16384 LSB/g) to m/s² if large integer
        if abs(az) > 50 or abs(ax) > 50 or abs(ay) > 50:
            ax_mps2 = (ax / 16384.0) * 9.80665
            ay_mps2 = (ay / 16384.0) * 9.80665
            az_mps2 = (az / 16384.0) * 9.80665
            gx_rad = (gx / 131.0) * (math.pi / 180.0)
            gy_rad = (gy / 131.0) * (math.pi / 180.0)
            gz_rad = (gz / 131.0) * (math.pi / 180.0)
        else:
            ax_mps2, ay_mps2, az_mps2 = ax, ay, az
            gx_rad, gy_rad, gz_rad = gx, gy, gz

        self.sensor_state["accel"] = {"x": round(ax_mps2, 2), "y": round(ay_mps2, 2), "z": round(az_mps2, 2)}
        self.sensor_state["gyro"] = {"x": round(gx_rad, 3), "y": round(gy_rad, 3), "z": round(gz_rad, 3)}

        # Pitch & Roll
        if az_mps2 != 0 or ay_mps2 != 0 or ax_mps2 != 0:
            pitch_rad = math.atan2(ay_mps2, math.sqrt(ax_mps2**2 + az_mps2**2))
            roll_rad = math.atan2(-ax_mps2, az_mps2)
            raw_p = round(pitch_rad * (180.0 / math.pi), 1)
            raw_r = round(roll_rad * (180.0 / math.pi), 1)
            self.sensor_state["raw_pitch"] = raw_p
            self.sensor_state["raw_roll"] = raw_r
            p_off = float(self.imu_offsets.get("pitch", 0.0))
            r_off = float(self.imu_offsets.get("roll", 0.0))
            self.sensor_state["pitch"] = round(raw_p - p_off, 1)
            self.sensor_state["roll"] = round(raw_r - r_off, 1)

        # GPS & System
        self.sensor_state["lat"] = float(d.get("lat", self.sensor_state["lat"]))
        self.sensor_state["lng"] = float(d.get("lng", self.sensor_state["lng"]))
        self.sensor_state["gps_valid"] = bool(d.get("gps_valid", False))
        self.sensor_state["gps_sats"] = int(d.get("gps_sats", 0))
        self.sensor_state["alert"] = str(d.get("alert", "OK"))
        self.sensor_state["sd_ok"] = bool(d.get("sd_ok", True))
        self.sensor_state["uptime_ms"] = int(d.get("uptime_ms", 0))

    def sim_tick(self):
        if not self.sensor_state["simulating"]:
            return

        t = time.time()
        temp_d = round(24.5 + 1.2 * math.sin(0.2 * t), 2)
        hum = round(48.0 + 3.5 * math.cos(0.15 * t), 2)
        temp_b = round(24.8 + 0.9 * math.sin(0.25 * t), 2)
        pres = round(1013.25 + 2.5 * math.sin(0.1 * t), 2)

        ax = round(0.4 * math.sin(0.5 * t), 2)
        ay = round(0.3 * math.cos(0.4 * t), 2)
        az = round(9.81 + 0.1 * math.sin(0.8 * t), 2)
        gx = round(0.02 * math.sin(0.6 * t), 3)
        gy = round(0.01 * math.cos(0.5 * t), 3)
        gz = round(-0.01 * math.sin(0.3 * t), 3)

        gas = int(410 + 25 * math.sin(0.1 * t) + random.randint(-4, 4))
        water = int(120 + 15 * math.sin(0.05 * t) + random.randint(-2, 2))
        uptime = int(t * 1000) % 10000000

        block = [
            "-----------------------------------",
            f"DHT11   temp: {temp_d:.1f} C   hum: {hum:.1f} %",
            f"BMP280  temp: {temp_b:.1f} C   pres: {pres:.1f} hPa",
            f"Gas:    {gas}   Water: {water}",
            "Flame:  clear",
            f"Accel:  X={int(ax*1671)} Y={int(ay*1671)} Z={int(az*1671)}",
            f"Gyro:   X={int(gx*7500)} Y={int(gy*7500)} Z={int(gz*7500)}",
            "GPS:    17.38504, 78.48667   sats: 7",
            "Alert:  OK",
            "-----------------------------------"
        ]

        self.sensor_state["temp_dht"] = temp_d
        self.sensor_state["temperature"] = temp_d
        self.sensor_state["humidity"] = hum
        self.sensor_state["temp_bmp"] = temp_b
        self.sensor_state["pressure"] = pres
        self.sensor_state["gas"] = gas
        self.sensor_state["water"] = water
        self.sensor_state["flame"] = 1
        self.sensor_state["alert"] = "OK"
        self.sensor_state["accel"] = {"x": ax, "y": ay, "z": az}
        self.sensor_state["gyro"] = {"x": gx, "y": gy, "z": gz}
        self.sensor_state["lat"] = 17.385044
        self.sensor_state["lng"] = 78.486671
        self.sensor_state["gps_valid"] = True
        self.sensor_state["gps_sats"] = 7
        self.sensor_state["sd_ok"] = True
        self.sensor_state["uptime_ms"] = uptime

        pitch_rad = math.atan2(ay, math.sqrt(ax**2 + az**2))
        roll_rad = math.atan2(-ax, az)
        self.sensor_state["raw_pitch"] = round(pitch_rad * (180.0 / math.pi), 1)
        self.sensor_state["raw_roll"] = round(roll_rad * (180.0 / math.pi), 1)
        p_off = float(self.imu_offsets.get("pitch", 0.0))
        r_off = float(self.imu_offsets.get("roll", 0.0))
        self.sensor_state["pitch"] = round(self.sensor_state["raw_pitch"] - p_off, 1)
        self.sensor_state["roll"] = round(self.sensor_state["raw_roll"] - r_off, 1)

        for line in block:
            self.sensor_state["raw_log"].append(line)
        if len(self.sensor_state["raw_log"]) > 100:
            self.sensor_state["raw_log"] = self.sensor_state["raw_log"][-100:]

        self.update_data_signal.emit(self.sensor_state)

    def parse_sensor_line(self, line):
        line = line.strip()
        if not line:
            return

        self.sensor_state["raw_log"].append(line)
        if len(self.sensor_state["raw_log"]) > 100:
            self.sensor_state["raw_log"].pop(0)

        # 1. DHT11
        if "DHT11: ERROR" in line:
            self.sensor_state["dht_error"] = True
        else:
            m_dht = re.search(r"DHT11\s+temp:\s*([-\d\.]+)\s*C\s+hum:\s*([-\d\.]+)", line, re.I)
            if m_dht:
                self.sensor_state["temp_dht"] = float(m_dht.group(1))
                self.sensor_state["temperature"] = float(m_dht.group(1))
                self.sensor_state["humidity"] = float(m_dht.group(2))
                self.sensor_state["dht_error"] = False

        # 2. BMP280
        m_bmp = re.search(r"BMP280\s+temp:\s*([-\d\.]+)\s*C\s+pres:\s*([-\d\.]+)", line, re.I)
        if m_bmp:
            self.sensor_state["temp_bmp"] = float(m_bmp.group(1))
            self.sensor_state["pressure"] = float(m_bmp.group(2))

        # 3. Gas & Water
        m_gw = re.search(r"Gas:\s*(\d+)\s+Water:\s*(\d+)", line, re.I)
        if m_gw:
            self.sensor_state["gas"] = int(m_gw.group(1))
            self.sensor_state["water"] = int(m_gw.group(2))

        # 4. Flame
        if "Flame:" in line:
            if "*** FIRE ***" in line or "FIRE" in line:
                self.sensor_state["flame"] = 0
                self.sensor_state["alert"] = "FIRE"
            else:
                self.sensor_state["flame"] = 1

        # 5. IMU Accel & Gyro
        m_acc = re.search(r"Accel:\s*X=([-\d\.]+)\s+Y=([-\d\.]+)\s+Z=([-\d\.]+)", line, re.I)
        if m_acc:
            ax = float(m_acc.group(1))
            ay = float(m_acc.group(2))
            az = float(m_acc.group(3))
            if abs(az) > 50 or abs(ax) > 50 or abs(ay) > 50:
                ax_m = (ax / 16384.0) * 9.80665
                ay_m = (ay / 16384.0) * 9.80665
                az_m = (az / 16384.0) * 9.80665
            else:
                ax_m, ay_m, az_m = ax, ay, az
            self.sensor_state["accel"] = {"x": round(ax_m, 2), "y": round(ay_m, 2), "z": round(az_m, 2)}

            if az_m != 0 or ay_m != 0 or ax_m != 0:
                pitch_rad = math.atan2(ay_m, math.sqrt(ax_m**2 + az_m**2))
                roll_rad = math.atan2(-ax_m, az_m)
                raw_p = round(pitch_rad * (180.0 / math.pi), 1)
                raw_r = round(roll_rad * (180.0 / math.pi), 1)
                self.sensor_state["raw_pitch"] = raw_p
                self.sensor_state["raw_roll"] = raw_r
                p_off = float(self.imu_offsets.get("pitch", 0.0))
                r_off = float(self.imu_offsets.get("roll", 0.0))
                self.sensor_state["pitch"] = round(raw_p - p_off, 1)
                self.sensor_state["roll"] = round(raw_r - r_off, 1)

        m_gyr = re.search(r"Gyro:\s*X=([-\d\.]+)\s+Y=([-\d\.]+)\s+Z=([-\d\.]+)", line, re.I)
        if m_gyr:
            gx = float(m_gyr.group(1))
            gy = float(m_gyr.group(2))
            gz = float(m_gyr.group(3))
            if abs(gx) > 20 or abs(gy) > 20 or abs(gz) > 20:
                gx_r = (gx / 131.0) * (math.pi / 180.0)
                gy_r = (gy / 131.0) * (math.pi / 180.0)
                gz_r = (gz / 131.0) * (math.pi / 180.0)
            else:
                gx_r, gy_r, gz_r = gx, gy, gz
            self.sensor_state["gyro"] = {"x": round(gx_r, 3), "y": round(gy_r, 3), "z": round(gz_r, 3)}

        # 6. GPS
        m_gps = re.search(r"GPS:\s*([-\d\.]+),\s*([-\d\.]+)\s+sats:\s*(\d+)", line, re.I)
        if m_gps:
            self.sensor_state["lat"] = float(m_gps.group(1))
            self.sensor_state["lng"] = float(m_gps.group(2))
            self.sensor_state["gps_sats"] = int(m_gps.group(3))
            self.sensor_state["gps_valid"] = True
        elif "GPS:    no fix" in line or "GPS: no fix" in line:
            self.sensor_state["gps_valid"] = False

        # 7. Alert
        m_al = re.search(r"Alert:\s*(\w+)", line, re.I)
        if m_al:
            self.sensor_state["alert"] = m_al.group(1).upper()

    def on_data_updated(self, data):
        # 1. Fire Alert Banner & Flame Status
        flame_val = data.get("flame", 1)
        alert_state = str(data.get("alert", "OK")).upper()
        is_fire = (flame_val == 0 or alert_state == "FIRE")

        self.lbl_fire_banner.setVisible(is_fire)
        if is_fire:
            self.lbl_flame_val.setText("LOW (🔥 FIRE!)")
            self.lbl_flame_val.setStyleSheet("color: #f87171; background-color: #450a0a; padding: 4px 8px; border-radius: 4px;")
            self.lbl_flame_badge.setText("🚨 FIRE ALERT!")
            self.lbl_flame_badge.setStyleSheet("background-color: #991b1b; color: #ffffff; font-weight: bold; padding: 3px 6px; border-radius: 4px;")
        else:
            self.lbl_flame_val.setText("HIGH (Clear)")
            self.lbl_flame_val.setStyleSheet("color: #a7f3d0; background-color: #18181b; padding: 4px 8px; border-radius: 4px;")
            self.lbl_flame_badge.setText("✅ Clear")
            self.lbl_flame_badge.setStyleSheet("background-color: #064e3b; color: #34d399; font-weight: bold; padding: 3px 6px; border-radius: 4px;")

        # 2. Climate (DHT11 & BMP-280)
        tempC = data.get("temp_dht", data.get("temperature", 24.5))
        tempF = (tempC * 9/5) + 32
        hum = data.get("humidity", 48.0)
        tempBMP = data.get("temp_bmp", 24.8)
        press = data.get("pressure", 1013.25)

        self.lbl_temp_val.setText(f"{tempC:.1f} °C ({tempF:.1f} °F)")
        self.lbl_hum_val.setText(f"{hum:.1f} %")
        self.lbl_bmp_temp_val.setText(f"{tempBMP:.1f} °C")
        self.lbl_pressure_val.setText(f"{press:.1f} hPa")

        # 3. Hazards (MQ Gas, Water)
        gas_val = data.get("gas", 415)
        self.lbl_gas_val.setText(f"{gas_val} / 4095")
        if gas_val > 2500:
            self.lbl_gas_badge.setText("🔴 HAZARD!")
            self.lbl_gas_badge.setStyleSheet("background-color: #991b1b; color: #ffffff; font-weight: bold; padding: 3px 6px; border-radius: 4px;")
        elif gas_val > 1500:
            self.lbl_gas_badge.setText("🟡 Moderate")
            self.lbl_gas_badge.setStyleSheet("background-color: #78350f; color: #fde047; font-weight: bold; padding: 3px 6px; border-radius: 4px;")
        else:
            self.lbl_gas_badge.setText("🟢 Clean Air")
            self.lbl_gas_badge.setStyleSheet("background-color: #064e3b; color: #34d399; font-weight: bold; padding: 3px 6px; border-radius: 4px;")

        water_val = data.get("water", 120)
        self.lbl_water_val.setText(f"{water_val} / 4095")
        if water_val > 2000:
            self.lbl_water_badge.setText("🌊 Submerged")
            self.lbl_water_badge.setStyleSheet("background-color: #1e3a8a; color: #93c5fd; font-weight: bold; padding: 3px 6px; border-radius: 4px;")
        elif water_val > 600:
            self.lbl_water_badge.setText("💧 Moisture")
            self.lbl_water_badge.setStyleSheet("background-color: #0c4a6e; color: #38bdf8; font-weight: bold; padding: 3px 6px; border-radius: 4px;")
        else:
            self.lbl_water_badge.setText("🟢 Dry")
            self.lbl_water_badge.setStyleSheet("background-color: #064e3b; color: #34d399; font-weight: bold; padding: 3px 6px; border-radius: 4px;")

        # 4. IMU Accel & Gyro
        acc = data.get("accel", {"x": 0.12, "y": -0.05, "z": 9.81})
        gy = data.get("gyro", {"x": 0.01, "y": 0.0, "z": -0.02})
        pitch = data.get("pitch", 0.0)
        roll = data.get("roll", 0.0)

        self.lbl_accel_val.setText(f"X: {acc['x']:.2f}  |  Y: {acc['y']:.2f}  |  Z: {acc['z']:.2f}")
        self.lbl_gyro_val.setText(f"X: {gy['x']:.3f}  |  Y: {gy['y']:.3f}  |  Z: {gy['z']:.3f}")
        self.lbl_tilt_val.setText(f"Pitch: {pitch:.1f}°   |   Roll: {roll:.1f}°")

        # 5. GPS Navigation & Status
        gps_valid = data.get("gps_valid", False)
        sats = data.get("gps_sats", 0)
        lat = data.get("lat", 0.0)
        lng = data.get("lng", 0.0)

        if gps_valid:
            self.lbl_gps_status.setText("🟢 3D Satellite Fix")
            self.lbl_gps_status.setStyleSheet("background-color: #064e3b; color: #34d399; font-weight: bold; padding: 3px 6px; border-radius: 4px;")
            self.lbl_gps_coords.setText(f"Lat: {lat:.6f} | Lng: {lng:.6f}")
        else:
            self.lbl_gps_status.setText("🔴 Searching / No Fix")
            self.lbl_gps_status.setStyleSheet("background-color: #451a1a; color: #f87171; font-weight: bold; padding: 3px 6px; border-radius: 4px;")
            self.lbl_gps_coords.setText("Lat: --.------ | Lng: --.------")

        self.lbl_gps_sats.setText(f"{sats} sats")

        # 6. SD & System Status
        sd_ok = data.get("sd_ok", True)
        if sd_ok:
            self.lbl_sd_status.setText("💾 SD: Ready (/log.csv)")
            self.lbl_sd_status.setStyleSheet("background-color: #064e3b; color: #34d399; font-weight: bold; padding: 3px 6px; border-radius: 4px;")
        else:
            self.lbl_sd_status.setText("❌ No SD Card")
            self.lbl_sd_status.setStyleSheet("background-color: #451a1a; color: #f87171; font-weight: bold; padding: 3px 6px; border-radius: 4px;")

        uptime_sec = int(data.get("uptime_ms", 0) / 1000)
        mins = uptime_sec // 60
        secs = uptime_sec % 60
        self.lbl_system_status.setText(f"Alert: {alert_state} | Up: {mins:02d}m {secs:02d}s")

        if data.get("wifi_connected", False):
            self.lbl_wifi_status.setText("Wi-Fi: Connected")
            self.lbl_wifi_status.setStyleSheet("background-color: #064e3b; color: #34d399; font-weight: bold; padding: 4px 8px; border-radius: 4px;")

        # Console
        if data.get("raw_log"):
            self.log_console.setPlainText("\n".join(data["raw_log"][-40:]))
            self.log_console.moveCursor(QTextCursor.End)

    def toggle_sim(self):
        self.sensor_state["simulating"] = not self.sensor_state["simulating"]
        if self.sensor_state["simulating"]:
            self.btn_sim.setText("▶️ Simulating Data")
            self.lbl_status.setText("Simulating")
            self.lbl_status.setStyleSheet("background-color: #064e3b; color: #34d399; font-weight: bold; padding: 4px 10px; border-radius: 4px;")
        else:
            self.btn_sim.setText("⏸️ Simulation Paused")
            self.lbl_status.setText("Paused")
            self.lbl_status.setStyleSheet("background-color: #4c0519; color: #f43f5e; font-weight: bold; padding: 4px 10px; border-radius: 4px;")

    def toggle_serial_connection(self):
        port = self.combo_port.currentText()
        baud = int(self.combo_baud.currentText())

        if self.sensor_state["connected"]:
            self.sensor_state["connected"] = False
            self.btn_connect.setText("Connect Serial")
            self.lbl_status.setText("Disconnected")
            self.lbl_status.setStyleSheet("background-color: #4c0519; color: #f43f5e; font-weight: bold; padding: 4px 10px; border-radius: 4px;")
        else:
            self.sensor_state["connected"] = True
            self.sensor_state["simulating"] = False
            self.sensor_state["wifi_connected"] = False
            self.btn_connect.setText("Disconnect Serial")
            self.btn_sim.setText("▶️ Simulating Data")
            self.lbl_status.setText(f"Hardware Serial ({port})")
            self.lbl_status.setStyleSheet("background-color: #064e3b; color: #34d399; font-weight: bold; padding: 4px 10px; border-radius: 4px;")

            def serial_reader_loop():
                try:
                    import serial
                    with serial.Serial(port, baud, timeout=1.0) as s:
                        while self.sensor_state["connected"]:
                            raw_bytes = s.readline()
                            if raw_bytes:
                                line = raw_bytes.decode('utf-8', errors='ignore')
                                self.parse_sensor_line(line)
                                self.update_data_signal.emit(self.sensor_state)
                except Exception:
                    self.sensor_state["connected"] = False
                    self.update_data_signal.emit(self.sensor_state)

            threading.Thread(target=serial_reader_loop, daemon=True).start()


    def tare_imu_horizon(self):
        """Record current accelerometer-derived tilt as zero-horizon reference."""
        raw_p = self.sensor_state.get("raw_pitch", self.sensor_state.get("pitch", 0.0))
        raw_r = self.sensor_state.get("raw_roll", self.sensor_state.get("roll", 0.0))
        self.imu_offsets["pitch"] = round(raw_p, 1)
        self.imu_offsets["roll"] = round(raw_r, 1)
        self.imu_offsets["yaw"] = 0.0
        self.sensor_state["pitch"] = 0.0
        self.sensor_state["roll"] = 0.0
        self.lbl_tilt_val.setText("Pitch: 0.0°   |   Roll: 0.0°")
        self.save_imu_offsets()
        if self.main_gui and hasattr(self.main_gui, 'log_to_console'):
            self.main_gui.log_to_console(f"🎯 MPU6050 Horizon Tared: Pitch offset={raw_p:.1f}°, Roll offset={raw_r:.1f}°")

    def load_imu_offsets(self):
        if os.path.exists(CALIB_FILE_PATH):
            try:
                with open(CALIB_FILE_PATH, 'r') as f:
                    data = json.load(f)
                    if "imu_offsets" in data and isinstance(data["imu_offsets"], dict):
                        self.imu_offsets = {
                            "pitch": float(data["imu_offsets"].get("pitch", 0.0)),
                            "roll": float(data["imu_offsets"].get("roll", 0.0)),
                            "yaw": float(data["imu_offsets"].get("yaw", 0.0))
                        }
            except Exception as e:
                print(f"Error loading imu_offsets: {e}")

    def save_imu_offsets(self):
        p = self.parent()
        while p is not None and not hasattr(p, 'save_calibration'):
            p = p.parent()
        if p and hasattr(p, 'save_calibration'):
            p.save_calibration()
        else:
            data = {}
            if os.path.exists(CALIB_FILE_PATH):
                try:
                    with open(CALIB_FILE_PATH, 'r') as f:
                        data = json.load(f)
                except Exception:
                    pass
            data["imu_offsets"] = self.imu_offsets
            try:
                with open(CALIB_FILE_PATH, 'w') as f:
                    json.dump(data, f, indent=2)
            except Exception as e:
                print(f"Error saving imu_offsets: {e}")


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

        btn_sync_phys = QPushButton("🎯 Sync to Physical")
        btn_sync_phys.setToolTip("Tare/Sync virtual arm to physical position: sets sync offset so 3D model matches physical pose")
        btn_sync_phys.setStyleSheet("""
            QPushButton { background-color: #0284c7; color: white; font-weight: bold; border-radius: 4px; padding: 6px 10px; }
            QPushButton:hover { background-color: #0369a1; }
        """)
        btn_sync_phys.clicked.connect(self.sync_all_joints_to_physical)

        top_bar1.addWidget(self.btn_sim)
        top_bar1.addWidget(self.btn_gesture)
        top_bar1.addWidget(btn_home)
        top_bar1.addWidget(btn_set_home)
        top_bar1.addWidget(btn_center)
        top_bar1.addWidget(self.btn_comm_mode)
        top_bar1.addWidget(btn_sync_phys)
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

        # ── Left Control Tabs ──────────────────────────────────────────────
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
                padding: 8px 20px;
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

        self.quard_bot_widget = QuardBotWidget(self)
        self.sensor_node_widget = ESP32SensorNodeWidget(self)
        self.slam_widget = VisualSLAMWidget(self) if VisualSLAMWidget else None

        self.tabs.addTab(control_widget, "🦾 Robot Arm")
        self.tabs.addTab(self.quard_bot_widget, "🤖 Quard Bot")
        self.tabs.addTab(self.sensor_node_widget, "🚀 Rover")
        if self.slam_widget:
            self.tabs.addTab(self.slam_widget, "👁️ Visual SLAM")

        self.tabs.currentChanged.connect(self.on_tab_changed)

        main_splitter.addWidget(self.tabs)

        # ── Right RViz 3D Viewport Container ──────────────────────────────
        rviz_container = QWidget()
        rviz_container.setStyleSheet("background-color: #121212; border-left: 2px solid #333333;")
        rviz_layout = QVBoxLayout(rviz_container)
        rviz_layout.setContentsMargins(4, 4, 4, 4)
        rviz_layout.setSpacing(4)

        # RViz Header bar with Status Badge
        rviz_header_widget = QWidget()
        rviz_header_widget.setStyleSheet("background-color: #2b2b2b; border-radius: 4px;")
        rviz_header_layout = QHBoxLayout(rviz_header_widget)
        rviz_header_layout.setContentsMargins(8, 4, 8, 4)

        self.rviz_header_label = QLabel("🧊 Integrated RViz 3D Viewport")
        self.rviz_header_label.setFont(QFont("SansSerif", 10, QFont.Bold))
        self.rviz_header_label.setStyleSheet("color: #00ffcc;")
        rviz_header_layout.addWidget(self.rviz_header_label)

        rviz_header_layout.addStretch(1)

        self.lbl_rviz_target = QLabel("Target: 🦾 Arm")
        self.lbl_rviz_target.setFont(QFont("SansSerif", 9, QFont.Bold))
        self.lbl_rviz_target.setStyleSheet("background-color: #0284c7; color: #ffffff; padding: 2px 8px; border-radius: 10px;")
        rviz_header_layout.addWidget(self.lbl_rviz_target)

        rviz_layout.addWidget(rviz_header_widget, 0)

        # Embedded frame
        self.rviz_frame = QWidget()
        self.rviz_frame.setAttribute(Qt.WA_NativeWindow, True)
        self.rviz_frame.setStyleSheet("background-color: #000000; border-radius: 4px;")
        self.rviz_frame_layout = QVBoxLayout(self.rviz_frame)
        self.rviz_frame_layout.setContentsMargins(0, 0, 0, 0)
        rviz_layout.addWidget(self.rviz_frame, 1)

        main_splitter.addWidget(rviz_container)

        # Set Splitter ratios (540px left control tabs, 860px right 3D viewport)
        main_splitter.setSizes([540, 860])

        self.setCentralWidget(main_splitter)

        target_robot = os.environ.get("ROBOT_TARGET", "arm").lower()
        if target_robot == "quard":
            self.tabs.setCurrentWidget(self.quard_bot_widget)
            self.lbl_rviz_target.setText("Target: 🤖 Quard")
            self.lbl_rviz_target.setStyleSheet("background-color: #10b981; color: #ffffff; padding: 2px 8px; border-radius: 10px;")
        elif target_robot == "dual":
            self.lbl_rviz_target.setText("Target: ⚡ Dual Bots")
            self.lbl_rviz_target.setStyleSheet("background-color: #8b5cf6; color: #ffffff; padding: 2px 8px; border-radius: 10px;")

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
        self.sim_blend_start_time = 0.0
        self.sim_blend_duration = 2.0
        self.sim_blend_start_degs = {}

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
            ('turntable_link_joint_dup_1', 0.25, math.pi / 1.5),
            ('turntable_link_joint_dup_2', 0.35, math.pi / 3),
            ('turntable_link_joint_dup_3', 0.20, math.pi / 2),
            ('wrist_twist_joint',          0.20, math.pi / 2),
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

    def on_tab_changed(self, index):
        """Handle tab transitions and update status badges."""
        target_robot = os.environ.get("ROBOT_TARGET", "arm").lower()
        if target_robot == "dual":
            # In dual mode, RViz always displays both bots
            prefix = "Tab: "
        else:
            prefix = "Active: "

        if index == 0:
            self.lbl_rviz_target.setText(f"{prefix}🦾 Arm")
            self.lbl_rviz_target.setStyleSheet("background-color: #0284c7; color: #ffffff; padding: 2px 8px; border-radius: 10px;")
            self.log_to_console("📑 Switched to 🦾 Robotic Arm Dashboard.")
        elif index == 1:
            self.lbl_rviz_target.setText(f"{prefix}🤖 Quard")
            self.lbl_rviz_target.setStyleSheet("background-color: #10b981; color: #ffffff; padding: 2px 8px; border-radius: 10px;")
            self.log_to_console("📑 Switched to 🤖 Quard Bot Quadruped Dashboard.")
            if hasattr(self, 'quard_bot_widget'):
                if hasattr(self.quard_bot_widget, 'ping_quard_bot'):
                    self.quard_bot_widget.ping_quard_bot()
                elif hasattr(self.quard_bot_widget, 'ping_robot'):
                    self.quard_bot_widget.ping_robot()
        elif index == 2:
            self.lbl_rviz_target.setText(f"{prefix}🚀 Rover")
            self.lbl_rviz_target.setStyleSheet("background-color: #f59e0b; color: #ffffff; padding: 2px 8px; border-radius: 10px;")
            self.log_to_console("📑 Switched to 🚀 Rover Sensor Dashboard.")
        elif index == 3:
            self.lbl_rviz_target.setText(f"{prefix}👁️ SLAM")
            self.lbl_rviz_target.setStyleSheet("background-color: #06b6d4; color: #ffffff; padding: 2px 8px; border-radius: 10px;")
            self.log_to_console("📑 Switched to 👁️ Visual SLAM (ESP32-CAM & ToF) Dashboard.")

        # Ensure embedded RViz window is properly resized
        QTimer.singleShot(50, self._resize_embedded_rviz)
        QTimer.singleShot(250, self._resize_embedded_rviz)

    def init_rviz_embed(self):
        """Spawn RViz2 as child process and schedule embedding."""
        import sys as sys
        print("[Dashboard] init_rviz_embed: Starting RViz2 process...", file=sys.stderr, flush=True)

        target_robot = os.environ.get("ROBOT_TARGET", "arm").lower()
        if target_robot == "quard":
            default_rviz = "/home/sabo/Documents/learn_/Hardware/urdf/quard_bot/rviz/quard_bot.rviz"
        elif target_robot == "dual":
            default_rviz = "/home/sabo/Documents/learn_/Hardware/urdf/dual_robots.rviz"
        else:
            default_rviz = "/home/sabo/Documents/learn_/Hardware/urdf/arm/rviz/arm.rviz"
        rviz_config = os.environ.get("RVIZ_CONFIG", default_rviz)
        self.rviz_process = QProcess(self)

        env = QProcessEnvironment.systemEnvironment()
        env.insert("QT_QPA_PLATFORM", "xcb")
        env.insert("GDK_BACKEND", "x11")
        self.rviz_process.setProcessEnvironment(env)

        self.rviz_process.start("ros2", ["run", "rviz2", "rviz2", "-d", rviz_config])

        self.already_docked = False
        self._embed_attempt = 0
        # Schedule multiple retry attempts with increasing delays
        for delay_ms in [3000, 5000, 7000, 10000, 14000, 18000, 23000, 28000]:
            QTimer.singleShot(delay_ms, self.embed_rviz_window)
        print(f"[Dashboard] init_rviz_embed: Scheduled 8 embed attempts", file=sys.stderr, flush=True)

    def _find_rviz_x11_window(self):
        """Find the main RViz2 X11 window ID using xdotool (without --onlyvisible for XWayland compat)."""
        import sys as sys
        try:
            main_win_id = str(int(self.winId()))
        except Exception as e:
            print(f"[Dashboard] _find_rviz: Error getting main winId: {e}", file=sys.stderr, flush=True)
            return None, []

        print(f"[Dashboard] _find_rviz: main_win_id={main_win_id}", file=sys.stderr, flush=True)

        # Search without --onlyvisible since it doesn't work under XWayland on Wayland compositors
        search_queries = [
            ["xdotool", "search", "--name", "RViz"],
            ["xdotool", "search", "--name", "robot.rviz"],
            ["xdotool", "search", "--class", "rviz2"],
            ["xdotool", "search", "--class", "rviz"],
        ]

        for cmd in search_queries:
            try:
                out = subprocess.check_output(cmd, stderr=subprocess.DEVNULL, timeout=5).decode().strip()
                if out:
                    ids = [i.strip() for i in out.split('\n') if i.strip()]
                    valid_ids = [i for i in ids if i != main_win_id]
                    print(f"[Dashboard] _find_rviz: cmd={cmd[-1]}, found ids={ids}, valid={valid_ids}", file=sys.stderr, flush=True)
                    if not valid_ids:
                        continue

                    # Pick the window whose WM_NAME contains "RViz" (the main viewport window)
                    for wid in reversed(valid_ids):
                        try:
                            props = subprocess.check_output(
                                ["xprop", "-id", wid, "WM_NAME"],
                                stderr=subprocess.DEVNULL,
                                timeout=3
                            ).decode()
                            if "RViz" in props or "robot.rviz" in props:
                                print(f"[Dashboard] _find_rviz: Found RViz window {wid}: {props.strip()}", file=sys.stderr, flush=True)
                                return int(wid), valid_ids
                        except Exception:
                            continue

                    # Fallback: return last valid id
                    print(f"[Dashboard] _find_rviz: Fallback to window {valid_ids[-1]}", file=sys.stderr, flush=True)
                    return int(valid_ids[-1]), valid_ids
            except subprocess.TimeoutExpired:
                print(f"[Dashboard] _find_rviz: Timeout on {cmd}", file=sys.stderr, flush=True)
                continue
            except subprocess.CalledProcessError:
                continue
            except Exception as e:
                print(f"[Dashboard] _find_rviz: Error on {cmd}: {e}", file=sys.stderr, flush=True)
                continue

        print(f"[Dashboard] _find_rviz: No RViz window found", file=sys.stderr, flush=True)
        return None, []

    def embed_rviz_window(self):
        """Locate X11 Window ID of spawned RViz process and embed into container."""
        import sys

        try:
            if hasattr(self, 'already_docked') and self.already_docked:
                return

            self._embed_attempt = getattr(self, '_embed_attempt', 0) + 1

            if not self.isVisible():
                QTimer.singleShot(1000, self.embed_rviz_window)
                return

            rviz_win_id, valid_ids = self._find_rviz_x11_window()

            if rviz_win_id is not None:
                try:
                    # 1. Hide auxiliary RViz windows (toolbars, selection-owner, etc.)
                    for wid in valid_ids:
                        if int(wid) != rviz_win_id:
                            try:
                                subprocess.call(["xdotool", "windowunmap", str(wid)],
                                                stderr=subprocess.DEVNULL, timeout=2)
                            except Exception:
                                pass

                    # 2. Strip window decorations & set window type to utility so compositor allows embedding
                    try:
                        subprocess.call([
                            "xprop", "-id", str(rviz_win_id), "-f", "_MOTIF_WM_HINTS", "32c",
                            "-set", "_MOTIF_WM_HINTS", "2, 0, 0, 0, 0"
                        ], stderr=subprocess.DEVNULL, timeout=2)
                        subprocess.call([
                            "xprop", "-id", str(rviz_win_id), "-f", "_NET_WM_WINDOW_TYPE", "32a",
                            "-set", "_NET_WM_WINDOW_TYPE", "_NET_WM_WINDOW_TYPE_UTILITY"
                        ], stderr=subprocess.DEVNULL, timeout=2)
                    except Exception:
                        pass

                    container_win_id = str(int(self.rviz_frame.winId()))
                    self.rviz_win_id = rviz_win_id

                    # 3. Perform native X11 reparent into container widget
                    subprocess.call(["xdotool", "windowreparent", str(rviz_win_id), container_win_id], stderr=subprocess.DEVNULL, timeout=2)
                    subprocess.call(["xdotool", "windowmap", str(rviz_win_id)], stderr=subprocess.DEVNULL, timeout=2)

                    # 4. Wrap with Qt window container for layout management
                    qwin = QWindow.fromWinId(rviz_win_id)
                    qwin.setFlags(Qt.SubWindow | Qt.FramelessWindowHint)

                    self.embedded_rviz_widget = QWidget.createWindowContainer(qwin, self.rviz_frame)
                    self.rviz_frame_layout.addWidget(self.embedded_rviz_widget)

                    self.already_docked = True
                    self.log_to_console("✅ Integrated RViz 3D Viewport docked successfully.")

                    # 5. Sync size & position
                    QTimer.singleShot(100, self._resize_embedded_rviz)
                    QTimer.singleShot(500, self._resize_embedded_rviz)
                    QTimer.singleShot(1200, self._resize_embedded_rviz)

                    # 6. Hide RViz menu bar and toolbars for clean 3D-only view
                    QTimer.singleShot(800, self._hide_rviz_chrome)
                    QTimer.singleShot(2000, self._hide_rviz_chrome)
                except Exception as e:
                    print(f"[Dashboard] embed_rviz_window: ERROR: {e}", file=sys.stderr, flush=True)
            else:
                if self._embed_attempt <= 10:
                    QTimer.singleShot(1000, self.embed_rviz_window)
        except Exception as e:
            print(f"[Dashboard] embed_rviz_window: UNHANDLED ERROR: {e}", file=sys.stderr, flush=True)

    def _resize_embedded_rviz(self):
        """Force embedded RViz container widget to fit container geometry without disturbing X11 window coordinates."""
        import sys
        try:
            if self.embedded_rviz_widget and self.rviz_frame:
                size = self.rviz_frame.size()
                w, h = size.width(), size.height()
                if w > 10 and h > 10:
                    self.embedded_rviz_widget.resize(w, h)
                    self.embedded_rviz_widget.updateGeometry()
                    if hasattr(self, 'rviz_frame_layout') and self.rviz_frame_layout:
                        self.rviz_frame_layout.update()
                print(f"[Dashboard] _resize_embedded_rviz: Refreshed container geometry ({w}x{h})", file=sys.stderr, flush=True)
        except Exception as e:
            print(f"[Dashboard] _resize_embedded_rviz: ERROR: {e}", file=sys.stderr, flush=True)

    def _hide_rviz_chrome(self):
        """Hide RViz menu bar, toolbars, and status bar for a clean 3D-only viewport."""
        import sys
        if not hasattr(self, 'rviz_win_id') or not self.rviz_win_id:
            return
        if getattr(self, '_rviz_chrome_hidden', False):
            return
        try:
            wid = str(self.rviz_win_id)
            # Focus the RViz window and use Panels menu to hide Displays/Views
            subprocess.call(["xdotool", "windowfocus", "--sync", wid], stderr=subprocess.DEVNULL, timeout=3)
            import time
            time.sleep(0.1)

            # Use xdotool to find and unmap toolbar/menubar child windows
            # Get all child windows of the RViz main window
            try:
                children_out = subprocess.check_output(
                    ["xdotool", "search", "--pid", str(self.rviz_process.processId())],
                    stderr=subprocess.DEVNULL, timeout=5
                ).decode().strip()
                if children_out:
                    child_ids = [c.strip() for c in children_out.split('\n') if c.strip()]
                    main_wid = int(wid)
                    for cid in child_ids:
                        cid_int = int(cid)
                        if cid_int != main_wid:
                            # Unmap auxiliary windows (toolbars, panels, etc.)
                            subprocess.call(["xdotool", "windowunmap", cid],
                                            stderr=subprocess.DEVNULL, timeout=2)
            except Exception:
                pass

            # Restore focus back to the dashboard
            try:
                dashboard_wid = str(int(self.winId()))
                subprocess.call(["xdotool", "windowfocus", dashboard_wid],
                                stderr=subprocess.DEVNULL, timeout=2)
            except Exception:
                pass

            self._rviz_chrome_hidden = True
            self._resize_embedded_rviz()
            print(f"[Dashboard] _hide_rviz_chrome: Chrome hidden for clean 3D view", file=sys.stderr, flush=True)
        except Exception as e:
            print(f"[Dashboard] _hide_rviz_chrome: ERROR: {e}", file=sys.stderr, flush=True)


    def closeEvent(self, event):
        """Cleanly terminate child processes when dashboard window is closed."""
        if self.rviz_process and self.rviz_process.state() != QProcess.NotRunning:
            self.rviz_process.terminate()
            self.rviz_process.waitForFinished(1000)
        event.accept()

    def keyPressEvent(self, event):
        """Capture keyboard events for Quard Bot locomotion when Quard Bot tab is active."""
        if hasattr(self, 'tabs') and self.tabs.currentIndex() == 1 and hasattr(self, 'quard_bot_widget'):
            key = event.key()
            if key in (Qt.Key_W, Qt.Key_Up):
                self.quard_bot_widget.start_gait('forward')
            elif key in (Qt.Key_S, Qt.Key_Down):
                self.quard_bot_widget.start_gait('backward')
            elif key in (Qt.Key_A, Qt.Key_Left):
                self.quard_bot_widget.start_gait('left')
            elif key in (Qt.Key_D, Qt.Key_Right):
                self.quard_bot_widget.start_gait('right')
            elif key == Qt.Key_Space:
                self.quard_bot_widget.stand_pose()
            elif key == Qt.Key_C:
                self.quard_bot_widget.crouch_pose()
            elif key == Qt.Key_H:
                self.quard_bot_widget.high_pose()
            elif key == Qt.Key_1:
                self.quard_bot_widget.set_face('happy')
            elif key == Qt.Key_2:
                self.quard_bot_widget.set_face('walk')
            elif key == Qt.Key_3:
                self.quard_bot_widget.set_face('wave')
            elif key == Qt.Key_4:
                self.quard_bot_widget.set_face('sleepy')
            elif key == Qt.Key_5:
                self.quard_bot_widget.set_face('cute')
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
        if 'wrist_twist_joint' not in joints and 'turntable_link_joint_dup_3' not in joints:
            joints['wrist_twist_joint'] = (-3.14159, 3.14159)
        if 'grip' not in joints and 'gripper' not in joints:
            joints['grip'] = (0.0, 3.14159)
        return joints

    def load_calibration(self):
        if os.path.exists(CALIB_FILE_PATH):
            try:
                with open(CALIB_FILE_PATH, 'r') as f:
                    return json.load(f)
            except Exception as e:
                print(f"Error loading calib file: {e}")
        return {}

    def sync_all_joints_to_physical(self):
        """Sets sync offset for each joint so the current position matches the home pose."""
        for j_name, w in self.joint_widgets.items():
            curr_deg = w.get_current_deg()
            home_deg = w.calib.get('home_deg', 90.0)
            offset = round(curr_deg - home_deg, 1)
            w.spin_sync.setValue(offset)
            w.on_sync_offset_changed(offset)
        self.save_calibration()
        self.log_to_console("🎯 Synced all joints to physical reference pose and saved calibration.")

    def on_sync_offset_changed(self, j_name, val):
        cmd = f"SYNC:{j_name}={val:.1f}"
        self.ros_node.publish_calibration(cmd)
        self.log_to_console(f"🎯 Twin Sync offset updated for {j_name}: {val:+.1f}°")

    def save_calibration(self):
        calib_out = {}
        if os.path.exists(CALIB_FILE_PATH):
            try:
                with open(CALIB_FILE_PATH, 'r') as f:
                    calib_out = json.load(f)
            except Exception as e:
                print(f"Warning reading calib file: {e}")

        for j_name, w in self.joint_widgets.items():
            if j_name not in calib_out or not isinstance(calib_out[j_name], dict):
                calib_out[j_name] = {}
            calib_out[j_name].update(w.calib)
            calib_out[j_name]['sync_offset_deg'] = w.sync_offset_deg

        if hasattr(self, 'sensor_node_widget') and hasattr(self.sensor_node_widget, 'imu_offsets'):
            calib_out['imu_offsets'] = self.sensor_node_widget.imu_offsets
        if hasattr(self, 'quard_bot_widget') and hasattr(self.quard_bot_widget, 'quard_offsets'):
            calib_out['quard_offsets'] = self.quard_bot_widget.quard_offsets

        try:
            with open(CALIB_FILE_PATH, 'w') as f:
                json.dump(calib_out, f, indent=2)
            self.log_to_console(f"💾 Calibration Config saved to {CALIB_FILE_PATH}")
            QMessageBox.information(self, "Success", f"Homing, Safety & Sync Calibration saved to:\n{CALIB_FILE_PATH}")
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
        """Clean up gesture teleop, SLAM manager, and child processes on dashboard window exit."""
        if hasattr(self, 'slam_widget') and self.slam_widget:
            try:
                self.slam_widget.cleanup()
            except Exception:
                pass
        if hasattr(self, 'rviz_process') and self.rviz_process and self.rviz_process.state() != QProcess.NotRunning:
            try:
                self.rviz_process.terminate()
                self.rviz_process.waitForFinished(1000)
            except Exception:
                pass
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
            self.sim_blend_start_time = time.time()
            self.sim_blend_duration = 2.0
            self.sim_blend_start_degs = {
                j_name: w.get_current_deg() for j_name, w in self.joint_widgets.items()
            }
            self.btn_sim.setText("⏸️ Stop Sim")
            for w in self.joint_widgets.values():
                w.slider.setEnabled(False)
            if hasattr(self, 'quard_bot_widget') and self.quard_bot_widget:
                self.quard_bot_widget.start_gait('forward')
            self.log_to_console("▶️ Motion Simulation Mode Enabled (smooth 2.0s ease-in).")
        else:
            self.btn_sim.setText("▶️ Start Sim")
            for w in self.joint_widgets.values():
                w.slider.setEnabled(True)
            if hasattr(self, 'quard_bot_widget') and self.quard_bot_widget:
                self.quard_bot_widget.stand_pose()
            self.log_to_console("⏸️ Motion Simulation Mode Paused.")

    def step_simulation(self):
        if not self.is_simulating:
            return

        now = time.time()
        dt = now - self.last_sim_tick
        self.last_sim_tick = now

        speed_factor = self.slider_speed.value() / 10.0
        self.sim_time_accumulator += dt * speed_factor

        # Cosine ease-in S-curve blend (C1-continuous)
        elapsed_blend = now - self.sim_blend_start_time
        blend_progress = min(1.0, max(0.0, elapsed_blend / self.sim_blend_duration))
        weight = 0.5 - 0.5 * math.cos(blend_progress * math.pi)

        for j_name, freq, phase in self.sim_configs:
            if j_name in self.joint_widgets:
                w = self.joint_widgets[j_name]
                min_lim = w.spin_min.value()
                max_lim = w.spin_max.value()

                center = (min_lim + max_lim) / 2.0
                amplitude = (max_lim - min_lim) / 2.0

                sine_deg = center + amplitude * math.sin(2.0 * math.pi * freq * self.sim_time_accumulator + phase)
                start_deg = self.sim_blend_start_degs.get(j_name, center)
                target_deg = (1.0 - weight) * start_deg + weight * sine_deg
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

        # Ensure all arm kinematic links are fully resolved in robot_state_publisher
        arm_extra_joints = []
        for extra_j in arm_extra_joints:
            if extra_j not in names:
                names.append(extra_j)
                positions.append(0.0)

        if hasattr(self, 'quard_bot_widget') and self.quard_bot_widget:
            for q_name, rad_val in self.quard_bot_widget.get_joint_radians().items():
                names.append(q_name)
                positions.append(rad_val)

        msg.name = names
        msg.position = positions

        self.ros_node.joint_pub.publish(msg)


if ROS2_AVAILABLE:
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
else:
    class CalibratedGUINode:
        def __init__(self):
            self.gui_ref = None
        def publish_calibration(self, calib_str):
            pass


def main(args=None):
    if "QT_QPA_PLATFORM_PLUGIN_PATH" in os.environ and "cv2" in os.environ["QT_QPA_PLATFORM_PLUGIN_PATH"]:
        del os.environ["QT_QPA_PLATFORM_PLUGIN_PATH"]
    rclpy.init(args=args)
    ros_node = CalibratedGUINode()

    app = QApplication(sys.argv)
    gui = CalibratedJointPublisherGUI(ros_node)

    if '--sim' in sys.argv or '-s' in sys.argv:
        gui.btn_sim.setChecked(True)
        gui.toggle_simulation(True)

    gui.show()
    gui.publish_joint_states()

    def _on_tick():
        if rclpy.ok():
            try:
                rclpy.spin_once(ros_node, timeout_sec=0.001)
            except Exception:
                pass
        gui.publish_joint_states()

    timer = QTimer()
    timer.timeout.connect(_on_tick)
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
