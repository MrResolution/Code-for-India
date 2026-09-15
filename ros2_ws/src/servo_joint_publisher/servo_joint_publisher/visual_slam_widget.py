"""PyQt5 Visual SLAM & ToF Sensor Widget for Desktop Robotics Dashboard.
----------------------------------------------------------------------
Embeds the real-time ESP32-CAM + VL53L1X Visual SLAM pipeline into the
PyQt5 calibrated_joint_gui desktop application.

Features:
- Live Undistorted Video Feed with telemetry HUD overlay.
- Real-time 2D Top-Down (Bird's-Eye) SLAM Map plot with metric grid.
- View switcher: Camera HUD, 2D Top-Down Map, or Side-by-Side Dual View.
- Pipeline Telemetry: Ingestion FPS, VO Processing FPS, ToF Distance,
  ORB Features, Inlier Matches, 3D Landmarks, and 6-DOF Robot Pose.
- Interactive controls: Start/Stop Engine, Toggle Mock Streamer, Reset Map.
"""

import os
import sys
import time
from typing import Optional
import cv2
import numpy as np

# Prevent OpenCV from hijacking Qt's platform plugin search path
if "QT_QPA_PLATFORM_PLUGIN_PATH" in os.environ and "cv2" in os.environ["QT_QPA_PLATFORM_PLUGIN_PATH"]:
    del os.environ["QT_QPA_PLATFORM_PLUGIN_PATH"]

from PyQt5.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QLabel, QPushButton,
    QGroupBox, QGridLayout, QScrollArea, QProgressBar, QFrame, QLineEdit
)
from PyQt5.QtCore import Qt, QTimer, pyqtSignal
from PyQt5.QtGui import QFont, QImage, QPixmap

# Locate and import SLAMManager from web_dashboard or lidr_slam
current_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(current_dir, "web_dashboard"))

try:
    from slam_service import SLAMManager
    SLAM_AVAILABLE = True
except Exception as e:
    print(f"[VisualSLAMWidget] Error importing SLAMManager: {e}", file=sys.stderr)
    SLAMManager = None
    SLAM_AVAILABLE = False


class VisualSLAMWidget(QWidget):
    """PyQt5 Dashboard panel for ESP32-CAM Visual SLAM & ToF Mapping."""

    def __init__(self, parent=None, udp_port: int = 5000):
        super().__init__(parent)
        self.main_gui = parent
        self.udp_port = udp_port

        # SLAM Manager singleton / service instance
        self.slam_manager = None
        if SLAM_AVAILABLE and SLAMManager:
            try:
                self.slam_manager = SLAMManager(udp_port=self.udp_port)
                self.slam_manager.start()
            except Exception as ex:
                print(f"[VisualSLAMWidget] Failed to initialize SLAMManager: {ex}", file=sys.stderr)

        self.view_mode = "cam"  # "cam" | "map" | "dual"

        # GUI update timer (20 Hz = 50ms)
        self.gui_fps_counter = 0
        self.gui_fps_timer = time.time()
        self.gui_fps = 0.0

        self.init_ui()

        self.update_timer = QTimer(self)
        self.update_timer.timeout.connect(self.update_tick)
        self.update_timer.start(50)

    def init_ui(self):
        root_layout = QVBoxLayout(self)
        root_layout.setContentsMargins(6, 6, 6, 6)
        root_layout.setSpacing(6)

        scroll = QScrollArea(self)
        scroll.setWidgetResizable(True)
        scroll.setStyleSheet("""
            QScrollArea { border: none; background-color: #1a1a1a; }
            QScrollBar:vertical { background-color: #2b2b2b; width: 10px; border-radius: 5px; }
            QScrollBar::handle:vertical { background-color: #444444; border-radius: 5px; min-height: 20px; }
            QScrollBar::handle:vertical:hover { background-color: #0284c7; }
        """)

        container = QWidget()
        container.setStyleSheet("background-color: #1a1a1a;")
        layout = QVBoxLayout(container)
        layout.setContentsMargins(8, 8, 8, 8)
        layout.setSpacing(10)

        # ── 1. Connection & Engine Status Header Box ───────────────────────
        header_box = QGroupBox("📡 Stream Target & Visual SLAM Engine")
        header_box.setStyleSheet("""
            QGroupBox {
                font-weight: bold; font-size: 13px;
                border: 1px solid #333333; border-radius: 6px;
                margin-top: 6px; padding-top: 10px;
                background-color: #242424; color: #38bdf8;
            }
            QGroupBox::title { subcontrol-origin: margin; padding: 0 6px; color: #38bdf8; }
        """)
        header_layout = QHBoxLayout(header_box)
        header_layout.setContentsMargins(8, 6, 8, 6)
        header_layout.setSpacing(8)

        lbl_ip_tag = QLabel("Target:")
        lbl_ip_tag.setStyleSheet("color: #cbd5e1; font-weight: bold;")
        header_layout.addWidget(lbl_ip_tag)

        self.input_ip = QLineEdit("10.88.106.30")
        self.input_ip.setPlaceholderText("IP or /dev/ttyUSB0")
        self.input_ip.setFixedWidth(120)
        self.input_ip.setStyleSheet("""
            QLineEdit {
                background-color: #0f172a;
                color: #38bdf8;
                border: 1px solid #334155;
                border-radius: 4px;
                padding: 4px 8px;
                font-weight: bold;
                font-size: 11px;
            }
            QLineEdit:focus {
                border: 1px solid #0284c7;
            }
        """)
        self.input_ip.returnPressed.connect(self.handle_connect)
        header_layout.addWidget(self.input_ip)

        self.btn_connect = QPushButton("🔗 Connect")
        self.btn_connect.setStyleSheet("""
            QPushButton {
                background-color: #0284c7;
                color: white;
                font-weight: bold;
                padding: 5px 10px;
                border-radius: 4px;
            }
            QPushButton:hover {
                background-color: #0369a1;
            }
        """)
        self.btn_connect.clicked.connect(self.handle_connect)
        header_layout.addWidget(self.btn_connect)

        # Quick preset buttons
        self.btn_quick_wifi = QPushButton("🌐 Wi-Fi")
        self.btn_quick_wifi.setToolTip("Set target to default ESP32 IP 10.88.106.30")
        self.btn_quick_wifi.setStyleSheet("QPushButton { background-color: #1e293b; color: #38bdf8; border: 1px solid #334155; font-weight: bold; padding: 5px 8px; border-radius: 4px; font-size: 11px; } QPushButton:hover { background-color: #334155; }")
        self.btn_quick_wifi.clicked.connect(lambda: self._set_target_and_connect("10.88.106.30"))
        header_layout.addWidget(self.btn_quick_wifi)

        self.btn_quick_serial = QPushButton("🔌 USB Serial")
        self.btn_quick_serial.setToolTip("Connect directly to ESP32 via /dev/ttyUSB0")
        self.btn_quick_serial.setStyleSheet("QPushButton { background-color: #1e293b; color: #a7f3d0; border: 1px solid #334155; font-weight: bold; padding: 5px 8px; border-radius: 4px; font-size: 11px; } QPushButton:hover { background-color: #334155; }")
        self.btn_quick_serial.clicked.connect(lambda: self._set_target_and_connect("/dev/ttyUSB0"))
        header_layout.addWidget(self.btn_quick_serial)

        self.btn_quick_auto = QPushButton("🔍 Auto-Detect")
        self.btn_quick_auto.setToolTip("Scan serial ports and network to find ESP32")
        self.btn_quick_auto.setStyleSheet("QPushButton { background-color: #1e293b; color: #fde047; border: 1px solid #334155; font-weight: bold; padding: 5px 8px; border-radius: 4px; font-size: 11px; } QPushButton:hover { background-color: #334155; }")
        self.btn_quick_auto.clicked.connect(lambda: self._set_target_and_connect("auto"))
        header_layout.addWidget(self.btn_quick_auto)

        # Ingest IP Pill
        local_ip_str = getattr(self.slam_manager, "local_ip", "10.35.234.59") if self.slam_manager else "10.35.234.59"
        self.lbl_local_ingest = QLabel(f"📥 Ingest: {local_ip_str}:5000")
        self.lbl_local_ingest.setStyleSheet("background-color: #0f172a; color: #94a3b8; border: 1px solid #334155; font-weight: bold; padding: 4px 8px; border-radius: 4px; font-size: 11px;")
        header_layout.addWidget(self.lbl_local_ingest)

        self.lbl_engine_status = QLabel("Listening")
        self.lbl_engine_status.setStyleSheet("background-color: #064e3b; color: #34d399; font-weight: bold; padding: 4px 10px; border-radius: 4px;")
        header_layout.addWidget(self.lbl_engine_status)

        self.btn_toggle_engine = QPushButton("⏹️ Stop Engine")
        self.btn_toggle_engine.setStyleSheet("QPushButton { background-color: #334155; color: white; font-weight: bold; padding: 5px 10px; border-radius: 4px; } QPushButton:hover { background-color: #475569; }")
        self.btn_toggle_engine.clicked.connect(self.toggle_engine)
        header_layout.addWidget(self.btn_toggle_engine)

        self.btn_reset = QPushButton("🔄 Reset Map")
        self.btn_reset.setStyleSheet("QPushButton { background-color: #991b1b; color: white; font-weight: bold; padding: 5px 10px; border-radius: 4px; } QPushButton:hover { background-color: #7f1d1d; }")
        self.btn_reset.clicked.connect(self.reset_map)
        header_layout.addWidget(self.btn_reset)

        header_layout.addStretch(1)
        layout.addWidget(header_box)

        # ── 2. Live Video & 2D Map Feeds Card ──────────────────────────────
        feeds_box = QGroupBox("📹 Visual Mapping Feeds")
        feeds_box.setStyleSheet("""
            QGroupBox {
                font-weight: bold; font-size: 13px;
                border: 1px solid #333333; border-radius: 6px;
                margin-top: 6px; padding-top: 10px;
                background-color: #242424; color: #38bdf8;
            }
            QGroupBox::title { subcontrol-origin: margin; padding: 0 6px; color: #38bdf8; }
        """)
        feeds_layout = QVBoxLayout(feeds_box)
        feeds_layout.setContentsMargins(8, 6, 8, 6)
        feeds_layout.setSpacing(6)

        # View Mode Toggle Sub-bar
        view_bar = QHBoxLayout()
        view_bar.addWidget(QLabel("Display Feed:"))

        self.btn_view_cam = QPushButton("📹 Camera HUD")
        self.btn_view_cam.setStyleSheet("background-color: #0284c7; color: white; font-weight: bold; padding: 3px 10px; border-radius: 3px;")
        self.btn_view_cam.clicked.connect(lambda: self.set_view_mode("cam"))
        view_bar.addWidget(self.btn_view_cam)

        self.btn_view_map = QPushButton("🗺️ 2D Top-Down Map")
        self.btn_view_map.setStyleSheet("background-color: #334155; color: #cbd5e1; font-weight: bold; padding: 3px 10px; border-radius: 3px;")
        self.btn_view_map.clicked.connect(lambda: self.set_view_mode("map"))
        view_bar.addWidget(self.btn_view_map)

        self.btn_view_dual = QPushButton("⚡ Dual View")
        self.btn_view_dual.setStyleSheet("background-color: #334155; color: #cbd5e1; font-weight: bold; padding: 3px 10px; border-radius: 3px;")
        self.btn_view_dual.clicked.connect(lambda: self.set_view_mode("dual"))
        view_bar.addWidget(self.btn_view_dual)

        view_bar.addStretch(1)
        feeds_layout.addLayout(view_bar)

        # Video Frame Containers
        self.display_container = QHBoxLayout()
        self.display_container.setSpacing(6)

        # Camera Frame Label
        self.cam_frame_widget = QFrame()
        self.cam_frame_widget.setStyleSheet("background-color: #0b0f19; border: 1px solid #334155; border-radius: 6px;")
        cam_frame_layout = QVBoxLayout(self.cam_frame_widget)
        cam_frame_layout.setContentsMargins(2, 2, 2, 2)
        cam_frame_layout.setSpacing(2)

        self.lbl_cam_feed = QLabel()
        self.lbl_cam_feed.setAlignment(Qt.AlignCenter)
        self.lbl_cam_feed.setMinimumSize(320, 240)
        self.lbl_cam_feed.setStyleSheet("background-color: #0b0f19;")
        cam_frame_layout.addWidget(self.lbl_cam_feed)

        lbl_cam_tag = QLabel("📹 Camera Feed & Telemetry HUD")
        lbl_cam_tag.setStyleSheet("color: #38bdf8; font-size: 10px; font-weight: bold; padding: 2px 4px;")
        lbl_cam_tag.setAlignment(Qt.AlignCenter)
        cam_frame_layout.addWidget(lbl_cam_tag)

        # 2D Map Frame Label
        self.map_frame_widget = QFrame()
        self.map_frame_widget.setStyleSheet("background-color: #0b0f19; border: 1px solid #334155; border-radius: 6px;")
        map_frame_layout = QVBoxLayout(self.map_frame_widget)
        map_frame_layout.setContentsMargins(2, 2, 2, 2)
        map_frame_layout.setSpacing(2)

        self.lbl_map_feed = QLabel()
        self.lbl_map_feed.setAlignment(Qt.AlignCenter)
        self.lbl_map_feed.setMinimumSize(320, 240)
        self.lbl_map_feed.setStyleSheet("background-color: #0b0f19;")
        map_frame_layout.addWidget(self.lbl_map_feed)

        lbl_map_tag = QLabel("🗺️ 2D Top-Down SLAM Map (Metric Grid)")
        lbl_map_tag.setStyleSheet("color: #00ffcc; font-size: 10px; font-weight: bold; padding: 2px 4px;")
        lbl_map_tag.setAlignment(Qt.AlignCenter)
        map_frame_layout.addWidget(lbl_map_tag)

        self.display_container.addWidget(self.cam_frame_widget, 1)
        self.display_container.addWidget(self.map_frame_widget, 1)
        self.map_frame_widget.setVisible(False)

        feeds_layout.addLayout(self.display_container)
        layout.addWidget(feeds_box)

        # ── 3. Performance & Rates Grid ────────────────────────────────────
        rates_box = QGroupBox("⚡ Real-Time Pipeline Rates")
        rates_box.setStyleSheet("""
            QGroupBox {
                font-weight: bold; font-size: 12px;
                border: 1px solid #333333; border-radius: 6px;
                margin-top: 4px; padding-top: 8px;
                background-color: #242424; color: #facc15;
            }
            QGroupBox::title { subcontrol-origin: margin; padding: 0 6px; color: #facc15; }
        """)
        rates_layout = QGridLayout(rates_box)
        rates_layout.setContentsMargins(8, 6, 8, 6)
        rates_layout.setSpacing(8)

        self.lbl_udp_fps = QLabel("0.0 FPS")
        self.lbl_udp_fps.setFont(QFont("Monospace", 12, QFont.Bold))
        self.lbl_udp_fps.setStyleSheet("color: #38bdf8;")

        self.lbl_vo_fps = QLabel("0.0 FPS")
        self.lbl_vo_fps.setFont(QFont("Monospace", 12, QFont.Bold))
        self.lbl_vo_fps.setStyleSheet("color: #4ade80;")

        self.lbl_gui_fps = QLabel("0.0 FPS")
        self.lbl_gui_fps.setFont(QFont("Monospace", 12, QFont.Bold))
        self.lbl_gui_fps.setStyleSheet("color: #f472b6;")

        rates_layout.addWidget(QLabel("UDP Ingest Rate:"), 0, 0)
        rates_layout.addWidget(self.lbl_udp_fps, 0, 1)
        rates_layout.addWidget(QLabel("VO Processing Rate:"), 0, 2)
        rates_layout.addWidget(self.lbl_vo_fps, 0, 3)
        rates_layout.addWidget(QLabel("GUI Display Rate:"), 0, 4)
        rates_layout.addWidget(self.lbl_gui_fps, 0, 5)

        layout.addWidget(rates_box)

        # ── 4. VL53L1X Time-of-Flight Distance Box ─────────────────────────
        tof_box = QGroupBox("📏 VL53L1X Time-of-Flight Laser Distance")
        tof_box.setStyleSheet("""
            QGroupBox {
                font-weight: bold; font-size: 12px;
                border: 1px solid #333333; border-radius: 6px;
                margin-top: 4px; padding-top: 8px;
                background-color: #242424; color: #10b981;
            }
            QGroupBox::title { subcontrol-origin: margin; padding: 0 6px; color: #10b981; }
        """)
        tof_layout = QVBoxLayout(tof_box)
        tof_layout.setContentsMargins(8, 6, 8, 6)
        tof_layout.setSpacing(6)

        tof_info_row = QHBoxLayout()
        tof_info_row.addWidget(QLabel("Front Target Range:"))

        self.lbl_tof_val = QLabel("0.00 m (0 mm)")
        self.lbl_tof_val.setFont(QFont("Monospace", 12, QFont.Bold))
        self.lbl_tof_val.setStyleSheet("color: #ffffff;")
        tof_info_row.addWidget(self.lbl_tof_val)

        tof_info_row.addStretch(1)

        self.lbl_tof_badge = QLabel("Awaiting")
        self.lbl_tof_badge.setStyleSheet("background-color: #334155; color: #94a3b8; font-weight: bold; padding: 2px 8px; border-radius: 4px;")
        tof_info_row.addWidget(self.lbl_tof_badge)

        tof_layout.addLayout(tof_info_row)

        self.bar_tof = QProgressBar()
        self.bar_tof.setRange(0, 4000)
        self.bar_tof.setValue(0)
        self.bar_tof.setTextVisible(False)
        self.bar_tof.setFixedHeight(8)
        self.bar_tof.setStyleSheet("""
            QProgressBar { border: 1px solid #333; border-radius: 4px; background-color: #1e1e1e; }
            QProgressBar::chunk { background: qlineargradient(x1:0, y1:0, x2:1, y2:0, stop:0 #10b981, stop:1 #06b6d4); border-radius: 3px; }
        """)
        tof_layout.addWidget(self.bar_tof)

        layout.addWidget(tof_box)

        # ── 5. Visual Odometry Tracking & Pose Grid ────────────────────────
        vo_box = QGroupBox("🧭 Visual Odometry & 6-DOF Robot Pose")
        vo_box.setStyleSheet("""
            QGroupBox {
                font-weight: bold; font-size: 12px;
                border: 1px solid #333333; border-radius: 6px;
                margin-top: 4px; padding-top: 8px;
                background-color: #242424; color: #a855f7;
            }
            QGroupBox::title { subcontrol-origin: margin; padding: 0 6px; color: #a855f7; }
        """)
        vo_layout = QGridLayout(vo_box)
        vo_layout.setContentsMargins(8, 6, 8, 6)
        vo_layout.setSpacing(8)

        self.lbl_features = QLabel("0")
        self.lbl_features.setFont(QFont("Monospace", 11, QFont.Bold))
        self.lbl_features.setStyleSheet("color: #ffffff;")

        self.lbl_matches = QLabel("0")
        self.lbl_matches.setFont(QFont("Monospace", 11, QFont.Bold))
        self.lbl_matches.setStyleSheet("color: #ffffff;")

        self.lbl_scale = QLabel("0.050 m")
        self.lbl_scale.setFont(QFont("Monospace", 11, QFont.Bold))
        self.lbl_scale.setStyleSheet("color: #38bdf8;")

        self.lbl_points = QLabel("0")
        self.lbl_points.setFont(QFont("Monospace", 11, QFont.Bold))
        self.lbl_points.setStyleSheet("color: #facc15;")

        self.lbl_pos_x = QLabel("+0.000 m")
        self.lbl_pos_x.setFont(QFont("Monospace", 11, QFont.Bold))
        self.lbl_pos_x.setStyleSheet("color: #4ade80;")

        self.lbl_pos_y = QLabel("+0.000 m")
        self.lbl_pos_y.setFont(QFont("Monospace", 11, QFont.Bold))
        self.lbl_pos_y.setStyleSheet("color: #4ade80;")

        self.lbl_pos_z = QLabel("+0.000 m")
        self.lbl_pos_z.setFont(QFont("Monospace", 11, QFont.Bold))
        self.lbl_pos_z.setStyleSheet("color: #4ade80;")

        self.lbl_heading = QLabel("0.0°")
        self.lbl_heading.setFont(QFont("Monospace", 11, QFont.Bold))
        self.lbl_heading.setStyleSheet("color: #f43f5e;")

        # Row 0: VO Features & Matches
        vo_layout.addWidget(QLabel("ORB Keypoints:"), 0, 0)
        vo_layout.addWidget(self.lbl_features, 0, 1)
        vo_layout.addWidget(QLabel("Inlier Matches:"), 0, 2)
        vo_layout.addWidget(self.lbl_matches, 0, 3)

        # Row 1: Scale & Landmarks
        vo_layout.addWidget(QLabel("Metric Scale:"), 1, 0)
        vo_layout.addWidget(self.lbl_scale, 1, 1)
        vo_layout.addWidget(QLabel("3D Map Landmarks:"), 1, 2)
        vo_layout.addWidget(self.lbl_points, 1, 3)

        # Row 2: 3D Position X, Y, Z
        vo_layout.addWidget(QLabel("Position [X, Y, Z]:"), 2, 0)
        pos_row = QHBoxLayout()
        pos_row.addWidget(QLabel("X:"))
        pos_row.addWidget(self.lbl_pos_x)
        pos_row.addWidget(QLabel("Y:"))
        pos_row.addWidget(self.lbl_pos_y)
        pos_row.addWidget(QLabel("Z:"))
        pos_row.addWidget(self.lbl_pos_z)
        vo_layout.addLayout(pos_row, 2, 1, 1, 3)

        # Row 3: Heading / Yaw
        vo_layout.addWidget(QLabel("Heading (Yaw):"), 3, 0)
        vo_layout.addWidget(self.lbl_heading, 3, 1)

        layout.addWidget(vo_box)

        layout.addStretch(1)
        scroll.setWidget(container)
        root_layout.addWidget(scroll)

    def set_view_mode(self, mode: str):
        """Switches between Camera HUD, 2D Map, or Dual Side-by-Side."""
        self.view_mode = mode
        self.btn_view_cam.setStyleSheet("background-color: #334155; color: #cbd5e1; font-weight: bold; padding: 3px 10px; border-radius: 3px;")
        self.btn_view_map.setStyleSheet("background-color: #334155; color: #cbd5e1; font-weight: bold; padding: 3px 10px; border-radius: 3px;")
        self.btn_view_dual.setStyleSheet("background-color: #334155; color: #cbd5e1; font-weight: bold; padding: 3px 10px; border-radius: 3px;")

        if mode == "dual":
            self.btn_view_dual.setStyleSheet("background-color: #0284c7; color: white; font-weight: bold; padding: 3px 10px; border-radius: 3px;")
            self.cam_frame_widget.setVisible(True)
            self.map_frame_widget.setVisible(True)
        elif mode == "map":
            self.btn_view_map.setStyleSheet("background-color: #0284c7; color: white; font-weight: bold; padding: 3px 10px; border-radius: 3px;")
            self.cam_frame_widget.setVisible(False)
            self.map_frame_widget.setVisible(True)
        else:
            self.btn_view_cam.setStyleSheet("background-color: #0284c7; color: white; font-weight: bold; padding: 3px 10px; border-radius: 3px;")
            self.cam_frame_widget.setVisible(True)
            self.map_frame_widget.setVisible(False)

    def toggle_engine(self):
        """Starts or stops the SLAM background service."""
        if not self.slam_manager:
            return
        if self.slam_manager.running:
            self.slam_manager.stop()
            self.btn_toggle_engine.setText("▶️ Start Engine")
            self.btn_toggle_engine.setStyleSheet("QPushButton { background-color: #0284c7; color: white; font-weight: bold; padding: 5px 10px; border-radius: 4px; }")
        else:
            self.slam_manager.start()
            self.btn_toggle_engine.setText("⏹️ Stop Engine")
            self.btn_toggle_engine.setStyleSheet("QPushButton { background-color: #334155; color: white; font-weight: bold; padding: 5px 10px; border-radius: 4px; }")

    def reset_map(self):
        """Resets the 3D map landmarks, trajectory, and camera pose."""
        if self.slam_manager:
            self.slam_manager.reset_map()

    def _set_target_and_connect(self, target: str):
        """Sets target input text and triggers connection."""
        self.input_ip.setText(target)
        self.handle_connect()

    def handle_connect(self):
        """Connects to the specified IP or Stream Target."""
        if not self.slam_manager:
            return
        target = self.input_ip.text().strip()
        if not target:
            target = "10.88.106.30"
            self.input_ip.setText(target)
        self.slam_manager.connect_to(target)
        self.lbl_engine_status.setText(f"Connecting ({target})")
        self.lbl_engine_status.setStyleSheet("background-color: #1e3a8a; color: #93c5fd; font-weight: bold; padding: 4px 10px; border-radius: 4px;")

    def update_tick(self):
        """Periodic 20 Hz GUI refresh loop."""
        if not self.slam_manager:
            return

        # 1. Update Display FPS
        self.gui_fps_counter += 1
        now = time.time()
        if now - self.gui_fps_timer >= 1.0:
            self.gui_fps = self.gui_fps_counter / (now - self.gui_fps_timer)
            self.gui_fps_counter = 0
            self.gui_fps_timer = now
            self.lbl_gui_fps.setText(f"{self.gui_fps:.1f} FPS")

        # 2. Update Video & Map Feeds
        if self.view_mode in ("cam", "dual") and self.cam_frame_widget.isVisible():
            cam_bytes = self.slam_manager.get_latest_hud_jpeg()
            if cam_bytes:
                qimg = QImage.fromData(cam_bytes)
                if not qimg.isNull():
                    target_w = max(320, min(640, self.cam_frame_widget.width() - 8))
                    pix = QPixmap.fromImage(qimg).scaledToWidth(target_w, Qt.SmoothTransformation)
                    self.lbl_cam_feed.setPixmap(pix)

        if self.view_mode in ("map", "dual") and self.map_frame_widget.isVisible():
            map_bytes = self.slam_manager.get_latest_map2d_jpeg()
            if map_bytes:
                qimg_map = QImage.fromData(map_bytes)
                if not qimg_map.isNull():
                    target_w = max(320, min(520, self.map_frame_widget.width() - 8))
                    pix_map = QPixmap.fromImage(qimg_map).scaledToWidth(target_w, Qt.SmoothTransformation)
                    self.lbl_map_feed.setPixmap(pix_map)

        # 3. Update Telemetry State
        state = self.slam_manager.get_state()

        # Rates
        self.lbl_udp_fps.setText(f"{state.get('udp_fps', 0.0):.1f} FPS")
        self.lbl_vo_fps.setText(f"{state.get('vo_fps', 0.0):.1f} FPS")

        # Dynamic Ingest IP Pill
        local_ip = state.get("local_ip", "10.35.234.59")
        udp_port = state.get("udp_port", 5000)
        if hasattr(self, "lbl_local_ingest"):
            self.lbl_local_ingest.setText(f"📥 Ingest: {local_ip}:{udp_port}")

        # Engine Status with error awareness
        status = state.get("status", "Standby")
        source_mode = state.get("source_mode", "Auto")
        connected = state.get("connected", False)
        is_error = state.get("is_error", False)

        if is_error or "⚠️" in status:
            self.lbl_engine_status.setText(status[:35])
            self.lbl_engine_status.setStyleSheet("background-color: #7f1d1d; color: #fca5a5; font-weight: bold; padding: 4px 10px; border-radius: 4px;")
        elif status == "Tracking":
            self.lbl_engine_status.setText(f"Tracking [{source_mode}]")
            self.lbl_engine_status.setStyleSheet("background-color: #064e3b; color: #34d399; font-weight: bold; padding: 4px 10px; border-radius: 4px;")
        elif connected:
            self.lbl_engine_status.setText(f"Connected [{source_mode}]")
            self.lbl_engine_status.setStyleSheet("background-color: #064e3b; color: #34d399; font-weight: bold; padding: 4px 10px; border-radius: 4px;")
        elif state.get("active", False):
            self.lbl_engine_status.setText(f"{status[:35]}")
            self.lbl_engine_status.setStyleSheet("background-color: #1e3a8a; color: #93c5fd; font-weight: bold; padding: 4px 10px; border-radius: 4px;")
        else:
            self.lbl_engine_status.setText("Standby")
            self.lbl_engine_status.setStyleSheet("background-color: #334155; color: #94a3b8; font-weight: bold; padding: 4px 10px; border-radius: 4px;")

        # ToF Distance
        dist_mm = state.get("distance_mm", 0)
        dist_m = state.get("distance_m", 0.0)
        self.lbl_tof_val.setText(f"{dist_m:.2f} m ({dist_mm} mm)")
        self.bar_tof.setValue(min(4000, dist_mm))

        if state.get("is_tof_valid", False):
            self.lbl_tof_badge.setText("VALID")
            self.lbl_tof_badge.setStyleSheet("background-color: #064e3b; color: #34d399; font-weight: bold; padding: 2px 8px; border-radius: 4px;")
        else:
            self.lbl_tof_badge.setText("OUT OF RANGE")
            self.lbl_tof_badge.setStyleSheet("background-color: #4c0519; color: #f43f5e; font-weight: bold; padding: 2px 8px; border-radius: 4px;")

        # VO Features & Scale
        self.lbl_features.setText(str(state.get("num_features", 0)))
        self.lbl_matches.setText(str(state.get("num_matches", 0)))
        self.lbl_scale.setText(f"{state.get('metric_scale', 0.05):.3f} m")
        self.lbl_points.setText(str(state.get("point_count", 0)))

        # 6-DOF Pose & Heading
        curr_pos = state.get("curr_pos", [0.0, 0.0, 0.0])
        self.lbl_pos_x.setText(f"{curr_pos[0]:+.3f}")
        self.lbl_pos_y.setText(f"{curr_pos[1]:+.3f}")
        self.lbl_pos_z.setText(f"{curr_pos[2]:+.3f}")
        self.lbl_heading.setText(f"{state.get('heading_deg', 0.0):.1f}°")

    def cleanup(self):
        """Stops background threads and timers on app exit."""
        self.update_timer.stop()
        if self.slam_manager:
            self.slam_manager.stop()
            print("[VisualSLAMWidget] Cleaned up SLAM service.", file=sys.stderr)
