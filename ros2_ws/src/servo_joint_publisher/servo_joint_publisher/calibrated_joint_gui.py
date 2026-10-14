#!/usr/bin/env python3
"""
ROS 2 Node: calibrated_joint_gui
---------------------------------
An interactive PyQt5 Dashboard with:
  • Connection card per robot: live state, latency, last reply, reconnect,
    port / host settings (arm state comes from servo_serial_commander's
    /esp32_link_state; Quard Bot over HTTP; rover over Wi-Fi or serial).
  • Shared console in every tab: colour-coded, filter, find, pause, copy,
    save, command input with history.
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
import shutil
import xml.etree.ElementTree as ET
import urllib.request
import urllib.parse
import threading
import re
import random
import html
import glob
import socket
import collections
import urllib.error

from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QLabel, QSlider, QDoubleSpinBox, QPushButton, QGroupBox,
    QScrollArea, QMessageBox, QSplitter, QTextEdit, QLineEdit,
    QTabWidget, QGridLayout, QComboBox, QShortcut, QPlainTextEdit,
    QFileDialog, QCheckBox
)
from PyQt5.QtCore import Qt, QTimer, QProcess, QProcessEnvironment, pyqtSignal, QCoreApplication, QUrl, QSettings
from PyQt5.QtGui import QFont, QWindow, QTextCursor, QPainter, QPen, QBrush, QColor, QPainterPath, QRadialGradient, QLinearGradient, QPalette, QKeySequence, QFontDatabase

try:
    QCoreApplication.setAttribute(Qt.AA_ShareOpenGLContexts)
    from PyQt5.QtWebEngineWidgets import QWebEngineView
    WEBENGINE_AVAILABLE = True
except Exception:
    WEBENGINE_AVAILABLE = False

if "QT_QPA_PLATFORM_PLUGIN_PATH" in os.environ and "cv2" in os.environ["QT_QPA_PLATFORM_PLUGIN_PATH"]:
    del os.environ["QT_QPA_PLATFORM_PLUGIN_PATH"]

# Auto-discover and source ROS 2 Jazzy environment if available
if os.path.exists("/opt/ros/jazzy"):
    if "/opt/ros/jazzy/bin" not in os.environ.get("PATH", ""):
        os.environ["PATH"] = f"/opt/ros/jazzy/bin:{os.environ.get('PATH', '')}"
    if "/opt/ros/jazzy/lib/python3.12/site-packages" not in sys.path:
        sys.path.insert(0, "/opt/ros/jazzy/lib/python3.12/site-packages")
    if "AMENT_PREFIX_PATH" not in os.environ:
        os.environ["AMENT_PREFIX_PATH"] = "/opt/ros/jazzy"
    if "PYTHONPATH" not in os.environ:
        os.environ["PYTHONPATH"] = "/opt/ros/jazzy/lib/python3.12/site-packages"
    if "LD_LIBRARY_PATH" not in os.environ or "/opt/ros/jazzy" not in os.environ.get("LD_LIBRARY_PATH", ""):
        ros_ld = "/opt/ros/jazzy/opt/rviz_ogre_vendor/lib:/opt/ros/jazzy/lib/x86_64-linux-gnu:/opt/ros/jazzy/lib"
        os.environ["LD_LIBRARY_PATH"] = f"{ros_ld}:{os.environ.get('LD_LIBRARY_PATH', '')}"

try:
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import JointState
    from std_msgs.msg import String as StringMsg
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
    # Must match servo_serial_commander: latched link state, volatile console.
    LINK_STATE_QOS = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                durability=DurabilityPolicy.TRANSIENT_LOCAL,
                                history=HistoryPolicy.KEEP_LAST)
    CONSOLE_QOS = QoSProfile(depth=50, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.VOLATILE,
                             history=HistoryPolicy.KEEP_LAST)
    ROS2_AVAILABLE = True
except ImportError:
    ROS2_AVAILABLE = False
    class JointState:
        def __init__(self):
            self.header = self
            self.stamp = self
            self.name = []
            self.position = []
            self.velocity = []
            self.effort = []
        def to_msg(self):
            return self
    class StringMsg:
        def __init__(self):
            self.data = ""

def find_project_dir():
    """Repository root (holds servo_calibration.json and urdf/).

    Honours $ROBOT_PROJECT_DIR, otherwise walks up from this file, which works
    from the source tree and from a colcon install inside ros2_ws/.
    """
    env_dir = os.environ.get("ROBOT_PROJECT_DIR")
    if env_dir:
        return os.path.abspath(env_dir)
    d = os.path.dirname(os.path.abspath(__file__))
    while True:
        if os.path.isfile(os.path.join(d, "servo_calibration.json")) and os.path.isdir(os.path.join(d, "urdf")):
            return d
        parent = os.path.dirname(d)
        if parent == d:
            return os.getcwd()
        d = parent


PROJECT_DIR = find_project_dir()
CALIB_FILE_PATH = os.path.join(PROJECT_DIR, "servo_calibration.json")
URDF_PATH = os.path.join(PROJECT_DIR, "urdf", "arm", "urdf", "arm.urdf")

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


# ── Dashboard theme ───────────────────────────────────────────────────────
# Shared colours for the whole window. Many widgets still carry their own
# inline styles; these set the defaults (labels, scroll bars, tool tips, status
# bar, splitters) and the toolbar buttons so everything reads as one app.
THEME = {
    'window': '#1b1d22',
    'surface': '#24272e',
    'sunken': '#15171b',
    'border': '#343842',
    'text': '#e6e8ec',
    'muted': '#9aa1ad',
    'accent': '#0ea5e9',
    'accent_hover': '#0284c7',
}

APP_QSS = f"""
    QLabel {{ background: transparent; }}
    QToolTip {{
        background-color: {THEME['surface']}; color: {THEME['text']};
        border: 1px solid {THEME['border']}; padding: 4px 6px;
    }}
    QScrollBar:vertical {{ background: transparent; width: 10px; margin: 0; }}
    QScrollBar:horizontal {{ background: transparent; height: 10px; margin: 0; }}
    QScrollBar::handle:vertical {{ background: #3a3f4b; border-radius: 5px; min-height: 28px; }}
    QScrollBar::handle:horizontal {{ background: #3a3f4b; border-radius: 5px; min-width: 28px; }}
    QScrollBar::handle:hover {{ background: #4b5263; }}
    QScrollBar::add-line, QScrollBar::sub-line {{ width: 0; height: 0; }}
    QScrollBar::add-page, QScrollBar::sub-page {{ background: transparent; }}
    QSplitter::handle {{ background: {THEME['border']}; }}
    QSplitter::handle:hover {{ background: {THEME['accent']}; }}
    QStatusBar {{ background: {THEME['sunken']}; color: {THEME['muted']}; border-top: 1px solid {THEME['border']}; }}
    QStatusBar QLabel {{ padding: 0 8px; }}
    QDoubleSpinBox, QSpinBox, QLineEdit, QComboBox, QTextEdit {{
        selection-background-color: {THEME['accent']}; selection-color: #ffffff;
    }}
    QLineEdit {{
        background-color: #0f1115; color: {THEME['text']};
        border: 1px solid {THEME['border']}; border-radius: 4px; padding: 3px 6px;
    }}
    QLineEdit:focus {{ border-color: {THEME['accent']}; }}
    QComboBox QLineEdit {{ border: none; background: transparent; padding: 0; }}
    QCheckBox {{ spacing: 6px; color: {THEME['text']}; }}
    QCheckBox::indicator {{
        width: 14px; height: 14px; border: 1px solid #5b6273; border-radius: 3px; background: #0f1115;
    }}
    QCheckBox::indicator:checked {{ background: {THEME['accent']}; border-color: {THEME['accent']}; }}
    QCheckBox:disabled {{ color: #6b7280; }}
"""


def apply_app_theme(app):
    """Fusion style + dark palette + base stylesheet. Safe to call twice."""
    if app is None or app.property("robot_theme_applied"):
        return
    app.setStyle("Fusion")
    pal = QPalette()
    pal.setColor(QPalette.Window, QColor(THEME['window']))
    pal.setColor(QPalette.WindowText, QColor(THEME['text']))
    pal.setColor(QPalette.Base, QColor(THEME['sunken']))
    pal.setColor(QPalette.AlternateBase, QColor(THEME['surface']))
    pal.setColor(QPalette.Text, QColor(THEME['text']))
    pal.setColor(QPalette.Button, QColor(THEME['surface']))
    pal.setColor(QPalette.ButtonText, QColor(THEME['text']))
    pal.setColor(QPalette.ToolTipBase, QColor(THEME['surface']))
    pal.setColor(QPalette.ToolTipText, QColor(THEME['text']))
    pal.setColor(QPalette.Highlight, QColor(THEME['accent']))
    pal.setColor(QPalette.HighlightedText, QColor('#ffffff'))
    pal.setColor(QPalette.PlaceholderText, QColor(THEME['muted']))
    for role in (QPalette.WindowText, QPalette.Text, QPalette.ButtonText):
        pal.setColor(QPalette.Disabled, role, QColor('#6b7280'))
    app.setPalette(pal)
    app.setStyleSheet(APP_QSS)
    app.setProperty("robot_theme_applied", True)


# (background, hover, text, border) per button role
_BUTTON_KINDS = {
    'primary': ('#0284c7', '#0369a1', '#ffffff', '#0284c7'),
    'success': ('#16a34a', '#15803d', '#ffffff', '#16a34a'),
    'warning': ('#d97706', '#b45309', '#ffffff', '#d97706'),
    'danger':  ('#dc2626', '#b91c1c', '#ffffff', '#dc2626'),
    'purple':  ('#7c3aed', '#6d28d9', '#ffffff', '#7c3aed'),
    'neutral': ('#2c3039', '#363b46', THEME['text'], THEME['border']),
}


def button_qss(kind='neutral', checked=None, padding='6px 12px'):
    """Stylesheet for a toolbar button; `checked` styles a checkable button's on state."""
    bg, hover, fg, border = _BUTTON_KINDS[kind]
    qss = (f"QPushButton {{ background-color: {bg}; color: {fg}; border: 1px solid {border}; "
           f"border-radius: 6px; padding: {padding}; font-weight: bold; }} "
           f"QPushButton:hover {{ background-color: {hover}; }} "
           f"QPushButton:disabled {{ background-color: #23262d; color: #6b7280; border-color: #2c3039; }}")
    if checked:
        cbg, chover, cfg, cborder = _BUTTON_KINDS[checked]
        qss += (f" QPushButton:checked {{ background-color: {cbg}; color: {cfg}; border-color: {cborder}; }}"
                f" QPushButton:checked:hover {{ background-color: {chover}; }}")
    return qss


def section_label(text):
    """Small upper-case caption used to group toolbar rows."""
    lbl = QLabel(text.upper())
    lbl.setStyleSheet(f"color: {THEME['muted']}; font-size: 10px; font-weight: bold; letter-spacing: 1px;")
    lbl.setFixedWidth(92)
    return lbl


def group_box_qss(accent):
    """Dark group box with a coloured title (console / connection cards)."""
    return (f"QGroupBox {{ font-weight: bold; font-size: 12px; border: 1px solid {THEME['border']}; "
            f"border-radius: 8px; margin-top: 8px; padding-top: 10px; background-color: {THEME['sunken']}; "
            f"color: {accent}; }} "
            f"QGroupBox::title {{ subcontrol-origin: margin; left: 8px; padding: 0 4px; color: {accent}; }}")


def muted_label(text=""):
    lbl = QLabel(text)
    lbl.setStyleSheet(f"color: {THEME['muted']}; font-size: 11px;")
    lbl.setWordWrap(True)
    return lbl


def _ui_settings():
    """The INI file the main window keeps its layout in (also console history, hosts)."""
    return QSettings(QSettings.IniFormat, QSettings.UserScope, "CodeForIndia", "RobotDashboard")


def list_serial_ports():
    """[(device, description)] for the serial ports present right now."""
    try:
        from serial.tools import list_ports
        found = [(p.device, p.description if p.description and p.description != 'n/a' else '')
                 for p in list_ports.comports()]
    except Exception:
        found = [(d, '') for pattern in ('/dev/ttyUSB*', '/dev/ttyACM*') for d in sorted(glob.glob(pattern))]
    return sorted(found)


def explain_serial_error(err):
    """Short reason plus the usual fix for a serial port failure."""
    text = str(err)
    errno_ = getattr(err, 'errno', None)
    if errno_ == 13 or 'Permission denied' in text or 'PermissionError' in text:
        return "permission denied: add your user to the 'dialout' group, then log out and back in"
    if errno_ == 2 or 'No such file' in text or 'FileNotFoundError' in text:
        return "port not found: is the board plugged in? Rescan the port list"
    if errno_ == 16 or 'busy' in text.lower():
        return "port busy: another program has it open"
    if 'readiness to read but returned no data' in text:
        return "device disconnected (unplugged or reset)"
    return text


def describe_net_error(err):
    """Plain-language reason for an HTTP request failure."""
    if isinstance(err, urllib.error.HTTPError):
        return f"HTTP {err.code} {err.reason}"
    reason = getattr(err, 'reason', err)
    if isinstance(reason, socket.gaierror):
        return "host name not found (a .local name needs mDNS; try the IP address)"
    if isinstance(reason, (socket.timeout, TimeoutError)) or 'timed out' in str(reason):
        return "no reply (timed out)"
    if isinstance(reason, ConnectionRefusedError):
        return "connection refused (board up, but its web server isn't running?)"
    if isinstance(reason, OSError) and getattr(reason, 'errno', None) in (101, 113, 10051, 10065):
        return "network unreachable (is this computer on the robot's Wi-Fi?)"
    return str(reason)


def infer_console_kind(text):
    """Console kind for older log calls that only carry an emoji."""
    t = text.lstrip()
    if t.startswith(('🔴', '❌', '⛔', '✗')):
        return 'error'
    if t.startswith(('⚠', '🟡')):
        return 'warn'
    if t.startswith(('🟢', '✅', '💾', '✓')):
        return 'ok'
    if t.startswith(('📤', '→')):
        return 'tx'
    return 'info'


# (background, text, dot) per state
_PILL_COLORS = {
    'ok':    ('#0b3b2c', '#6ee7b7', '#10b981'),
    'warn':  ('#3d2a07', '#fcd34d', '#f59e0b'),
    'error': ('#3f0d16', '#fda4af', '#f43f5e'),
    'busy':  ('#0b2f45', '#7dd3fc', '#0ea5e9'),
    'idle':  ('#2c3039', '#cbd5e1', '#64748b'),
}


class StatusPill(QLabel):
    """Rounded "● Label" badge for a link state."""

    def __init__(self, text="", kind='idle', parent=None):
        super().__init__(parent)
        self.kind = None
        self.setTextFormat(Qt.RichText)
        self.set_state(kind, text)

    def set_state(self, kind, text):
        bg, fg, dot = _PILL_COLORS.get(kind, _PILL_COLORS['idle'])
        self.kind = kind
        self.plain_text = text
        self.setText(f'<span style="color:{dot}">●</span>&nbsp;{html.escape(text)}')
        self.setStyleSheet(f"QLabel {{ background-color: {bg}; color: {fg}; border: 1px solid {dot}; "
                           f"border-radius: 10px; padding: 2px 10px; font-weight: bold; font-size: 12px; }}")


class HistoryLineEdit(QLineEdit):
    """Line edit with shell-style Up/Down history, kept between runs."""

    def __init__(self, history_key=None, parent=None):
        super().__init__(parent)
        self.history_key = history_key
        self.history = []
        if history_key:
            saved = _ui_settings().value(history_key, [])
            if isinstance(saved, str):
                saved = [saved]
            self.history = [h for h in (saved or []) if isinstance(h, str)][-50:]
        self.hist_idx = len(self.history)

    def remember(self, text):
        if not text:
            return
        if text in self.history:
            self.history.remove(text)
        self.history.append(text)
        self.history = self.history[-50:]
        self.hist_idx = len(self.history)
        if self.history_key:
            _ui_settings().setValue(self.history_key, self.history)

    def keyPressEvent(self, event):
        if event.key() == Qt.Key_Up and self.history:
            self.hist_idx = max(0, self.hist_idx - 1)
            self.setText(self.history[self.hist_idx])
            return
        if event.key() == Qt.Key_Down and self.history:
            self.hist_idx = min(len(self.history), self.hist_idx + 1)
            self.setText(self.history[self.hist_idx] if self.hist_idx < len(self.history) else "")
            return
        super().keyPressEvent(event)


class _ConsoleEntry:
    __slots__ = ('ts', 'kind', 'text', 'key', 'count', 'rendered')

    def __init__(self, ts, kind, text, key):
        self.ts, self.kind, self.text, self.key = ts, kind, text, key
        self.count = 1
        self.rendered = False


class ConsoleWidget(QWidget):
    """Log view shared by the Arm, Quard Bot and Rover tabs.

    log() is safe to call from worker threads. Identical consecutive lines
    collapse into one with a repeat count; consecutive entries with the same
    `key` replace each other (a slider drag shows only its latest angle).
    """
    submitted = pyqtSignal(str)
    _incoming = pyqtSignal(str, str, object, float)

    KINDS = {
        'tx':    ('→', '#7dd3fc'),
        'rx':    ('←', '#5eead4'),
        'data':  (' ', '#94a3b8'),
        'info':  ('•', '#cbd5e1'),
        'ok':    ('✓', '#86efac'),
        'warn':  ('!', '#fcd34d'),
        'error': ('✗', '#fca5a5'),
    }
    FILTERS = (("All", None), ("Sent", {'tx'}), ("Received", {'rx', 'data'}),
               ("Events", {'info', 'ok', 'warn', 'error'}), ("Problems", {'warn', 'error'}))

    def __init__(self, name, input_placeholder=None, quick_commands=(), max_entries=3000, parent=None):
        super().__init__(parent)
        self.name = name
        self.entries = collections.deque(maxlen=max_entries)
        self.paused = False
        self.pending = 0
        self.hidden = 0
        self.kind_filter = None
        self.text_filter = ""
        self._incoming.connect(self._add)

        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(4)

        small = button_qss('neutral', padding='3px 8px')
        bar = QHBoxLayout()
        bar.setSpacing(4)
        self.filter_combo = QComboBox()
        for label, _ in self.FILTERS:
            self.filter_combo.addItem(label)
        self.filter_combo.setToolTip("Show only some kinds of line")
        self.filter_combo.currentIndexChanged.connect(self._on_filter_changed)
        self.search = QLineEdit()
        self.search.setPlaceholderText("Find…")
        self.search.setClearButtonEnabled(True)
        self.search.setMaximumWidth(170)
        self._search_timer = QTimer(self)
        self._search_timer.setSingleShot(True)
        self._search_timer.setInterval(150)
        self._search_timer.timeout.connect(self._on_filter_changed)
        self.search.textChanged.connect(lambda _t: self._search_timer.start())
        self.extra_bar = QHBoxLayout()
        self.extra_bar.setSpacing(6)
        self.lbl_count = QLabel()
        self.lbl_count.setStyleSheet(f"color: {THEME['muted']}; font-size: 11px;")
        self.btn_pause = QPushButton("⏸ Pause")
        self.btn_pause.setCheckable(True)
        self.btn_pause.setToolTip("Freeze the view to read it; new lines are kept")
        self.btn_pause.setStyleSheet(button_qss('neutral', checked='warning', padding='3px 8px'))
        self.btn_pause.toggled.connect(self.set_paused)
        btn_copy = QPushButton("Copy")
        btn_copy.setToolTip("Copy the lines shown to the clipboard")
        btn_copy.clicked.connect(self.copy_visible)
        btn_save = QPushButton("Save…")
        btn_save.setToolTip("Save every line to a .log file")
        btn_save.clicked.connect(self.save_to_file)
        btn_clear = QPushButton("Clear")
        btn_clear.clicked.connect(self.clear)
        for b in (btn_copy, btn_save, btn_clear):
            b.setStyleSheet(small)
        bar.addWidget(self.filter_combo)
        bar.addWidget(self.search)
        bar.addLayout(self.extra_bar)
        bar.addStretch(1)
        bar.addWidget(self.lbl_count)
        bar.addWidget(self.btn_pause)
        bar.addWidget(btn_copy)
        bar.addWidget(btn_save)
        bar.addWidget(btn_clear)
        layout.addLayout(bar)

        self.view = QPlainTextEdit()
        self.view.setReadOnly(True)
        self.view.setUndoRedoEnabled(False)
        self.view.setMaximumBlockCount(max_entries)
        mono = QFontDatabase.systemFont(QFontDatabase.FixedFont)
        mono.setPointSize(9)
        self.view.setFont(mono)
        self.view.setStyleSheet(f"QPlainTextEdit {{ background-color: #0b0d10; color: {THEME['text']}; "
                                f"border: 1px solid {THEME['border']}; border-radius: 6px; padding: 4px; }}")
        layout.addWidget(self.view, 1)

        self.input = None
        if input_placeholder:
            row = QHBoxLayout()
            row.setSpacing(4)
            self.input = HistoryLineEdit(f"console/{name}/history")
            self.input.setPlaceholderText(input_placeholder)
            self.input.setFont(mono)
            self.input.returnPressed.connect(self._submit)
            btn_send = QPushButton("Send")
            btn_send.setStyleSheet(button_qss('primary', padding='4px 12px'))
            btn_send.clicked.connect(self._submit)
            row.addWidget(self.input, 1)
            row.addWidget(btn_send)
            for label, command, tip in quick_commands:
                b = QPushButton(label)
                b.setToolTip(f"{tip}  [{command}]")
                b.setStyleSheet(small)
                b.clicked.connect(lambda _=False, c=command: self.submitted.emit(c))
                row.addWidget(b)
            layout.addLayout(row)
        self._update_count()

    # ── public API ──
    def log(self, text, kind='info', key=None):
        """Add a line (thread-safe)."""
        self._incoming.emit(str(text), kind if kind in self.KINDS else 'info', key, time.time())

    def add_toolbar_widget(self, widget):
        self.extra_bar.addWidget(widget)

    def plain_lines(self, visible_only=True):
        return [self._plain(e) for e in self.entries if not visible_only or self._matches(e)]

    def set_paused(self, on):
        self.paused = bool(on)
        if self.btn_pause.isChecked() != self.paused:
            self.btn_pause.setChecked(self.paused)
        self.btn_pause.setText("▶ Resume" if self.paused else "⏸ Pause")
        if not self.paused and self.pending:
            self._render_all()
        self._update_count()

    def clear(self):
        self.entries.clear()
        self.view.clear()
        self.pending = self.hidden = 0
        self._update_count()

    def copy_visible(self):
        lines = self.plain_lines()
        QApplication.clipboard().setText("\n".join(lines))
        self.lbl_count.setText(f"Copied {len(lines)} lines")
        QTimer.singleShot(1500, self._update_count)

    def save_to_file(self):
        default = os.path.join(os.path.expanduser("~"), f"{self.name}-console-{time.strftime('%Y%m%d-%H%M%S')}.log")
        path, _ = QFileDialog.getSaveFileName(self, "Save console log", default, "Log files (*.log *.txt);;All files (*)")
        if not path:
            return
        try:
            with open(path, 'w', encoding='utf-8') as f:
                f.write("\n".join(self.plain_lines(visible_only=False)) + "\n")
            self.log(f"Saved {len(self.entries)} lines to {path}", 'ok')
        except OSError as e:
            self.log(f"Could not save {path}: {e}", 'error')

    # ── internals ──
    def _matches(self, e):
        if self.kind_filter and e.kind not in self.kind_filter:
            return False
        if self.text_filter and self.text_filter not in e.text.lower():
            return False
        return True

    @staticmethod
    def _stamp(ts):
        return time.strftime('%H:%M:%S', time.localtime(ts)) + f".{int((ts % 1) * 1000):03d}"

    def _html(self, e):
        sym, color = self.KINDS[e.kind]
        text = html.escape(e.text).replace('  ', ' &nbsp;')
        count = f'<span style="color:{THEME["muted"]}">  ×{e.count}</span>' if e.count > 1 else ''
        return (f'<span style="color:#6b7280">{self._stamp(e.ts)}</span> '
                f'<span style="color:{color}">{sym} {text}</span>{count}')

    def _plain(self, e):
        count = f"  (x{e.count})" if e.count > 1 else ""
        return f"{self._stamp(e.ts)} {e.kind.upper():5} {e.text}{count}"

    def _add(self, text, kind, key, ts):
        last = self.entries[-1] if self.entries else None
        same_key = last is not None and key is not None and last.key == key
        same_line = (last is not None and key is None and last.key is None
                     and last.kind == kind and last.text == text)
        if same_key or same_line:
            if same_key:
                last.text, last.kind = text, kind
            else:
                last.count += 1
            last.ts = ts
            if not self.paused and last.rendered and self._matches(last):
                cursor = QTextCursor(self.view.document().lastBlock())
                cursor.movePosition(QTextCursor.StartOfBlock)
                cursor.movePosition(QTextCursor.EndOfBlock, QTextCursor.KeepAnchor)
                cursor.insertHtml(self._html(last))
            self._update_count()
            return
        e = _ConsoleEntry(ts, kind, text, key)
        self.entries.append(e)
        if self.paused:
            self.pending += 1
        elif self._matches(e):
            self.view.appendHtml(self._html(e))
            e.rendered = True
        else:
            self.hidden += 1
        self._update_count()

    def _render_all(self):
        self.view.setUpdatesEnabled(False)
        self.view.clear()
        self.hidden = 0
        for e in self.entries:
            if self._matches(e):
                self.view.appendHtml(self._html(e))
                e.rendered = True
            else:
                e.rendered = False
                self.hidden += 1
        self.view.setUpdatesEnabled(True)
        sb = self.view.verticalScrollBar()
        sb.setValue(sb.maximum())
        self.pending = 0
        self._update_count()

    def _on_filter_changed(self, *_):
        self.kind_filter = self.FILTERS[self.filter_combo.currentIndex()][1]
        self.text_filter = self.search.text().strip().lower()
        self._render_all()

    def _update_count(self):
        txt = f"{len(self.entries)} lines"
        if self.hidden and (self.kind_filter or self.text_filter):
            txt += f" · {self.hidden} hidden"
        if self.paused:
            txt += f" · paused, {self.pending} new"
        self.lbl_count.setText(txt)

    def _submit(self):
        text = self.input.text().strip()
        if not text:
            return
        self.input.remember(text)
        self.input.clear()
        self.submitted.emit(text)


# Readable name and wiring hint for each arm joint (URDF names are opaque).
JOINT_LABELS = {
    'turntable_link_joint_dup':   ("Base Turntable Yaw", ""),
    'turntable_link_joint':       ("Shoulder Pitch", "dual servos · GPIO 19 & 21"),
    'turntable_link_joint_dup_1': ("Elbow 1 Pitch", ""),
    'turntable_link_joint_dup_2': ("Elbow 2 Pitch", ""),
    'turntable_link_joint_dup_3': ("Wrist Twist / Roll", "GPIO 27"),
    'wrist_twist_joint':          ("Wrist Twist / Roll", "GPIO 27"),
    'grip':                       ("End-Effector Gripper", "GPIO 26"),
    'gripper':                    ("End-Effector Gripper", "GPIO 26"),
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

        friendly, hint = JOINT_LABELS.get(joint_name, (joint_name, ""))
        self.setTitle(f"🦾 {friendly}")
        self.setStyleSheet(f"""
            QGroupBox {{
                font-weight: bold;
                font-size: 13px;
                border: 1px solid {THEME['border']};
                border-radius: 8px;
                margin-top: 10px;
                padding-top: 10px;
                background-color: {THEME['surface']};
                color: {THEME['text']};
            }}
            QGroupBox::title {{
                subcontrol-origin: margin;
                subcontrol-position: top left;
                left: 8px;
                padding: 0 4px;
                color: #7dd3fc;
            }}
        """)

        layout = QVBoxLayout()
        layout = QVBoxLayout()

        # ── Row 1: Joint Name & Readout ──────────────────────────────────
        row1 = QHBoxLayout()
        # URDF name (and wiring hint) as a quiet caption; the title carries the readable name
        self.label_name = QLabel(f"{joint_name}  ·  {hint}" if hint else joint_name)
        self.label_name.setFont(QFont("Monospace", 9))
        self.label_name.setStyleSheet(f"color: {THEME['muted']};")
        self.label_name.setToolTip(f"URDF joint: {joint_name}")

        self.label_val = QLabel("90.0° (0.000 rad)")
        self.label_val.setFont(QFont("Monospace", 10, QFont.Bold))
        self.label_val.setStyleSheet(f"color: #5eead4; background: {THEME['sunken']}; border: 1px solid {THEME['border']}; padding: 3px 8px; border-radius: 4px;")

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
                border: none;
                height: 6px;
                background: #3a3f4b;
                border-radius: 3px;
            }
            QSlider::sub-page:horizontal {
                background: #0ea5e9;
                border-radius: 3px;
            }
            QSlider::handle:horizontal {
                background: #ffffff;
                border: 2px solid #0ea5e9;
                width: 14px;
                margin-top: -6px;
                margin-bottom: -6px;
                border-radius: 9px;
            }
            QSlider::handle:horizontal:disabled { border-color: #6b7280; }
            QSlider::sub-page:horizontal:disabled { background: #4b5263; }
        """)
        self.slider.valueChanged.connect(self.on_slider_changed)
        row2.addWidget(self.slider)
        layout.addLayout(row2)

        # ── Row 3: Limits & "Set Low" / "Set High" Buttons ───────────────
        row3 = QHBoxLayout()

        lbl_min = QLabel("Min")
        lbl_min.setStyleSheet(f"color: {THEME['muted']}; font-weight: bold;")
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
            QPushButton { background-color: #3b1d22; color: #fca5a5; font-weight: bold; border: 1px solid #7f1d1d; border-radius: 5px; padding: 4px 10px; }
            QPushButton:hover { background-color: #4c1d24; }
            QPushButton:pressed { background-color: #2a1418; }
        """)
        self.btn_set_low.clicked.connect(self.set_low_limit)

        lbl_max = QLabel("Max")
        lbl_max.setStyleSheet(f"color: {THEME['muted']}; font-weight: bold;")
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
            QPushButton { background-color: #13301f; color: #86efac; font-weight: bold; border: 1px solid #166534; border-radius: 5px; padding: 4px 10px; }
            QPushButton:hover { background-color: #173d27; }
            QPushButton:pressed { background-color: #0d2316; }
        """)
        self.btn_set_high.clicked.connect(self.set_high_limit)

        row3.addWidget(lbl_min)
        row3.addWidget(self.spin_min)
        row3.addWidget(self.btn_set_low)
        row3.addStretch(1)
        row3.addWidget(lbl_max)
        row3.addWidget(self.spin_max)
        row3.addWidget(self.btn_set_high)
        layout.addLayout(row3)

        # ── Row 4: Twin Sync & Alignment Offset ───────────────────────────
        row4 = QHBoxLayout()
        lbl_sync = QLabel("🎯 Twin offset")
        lbl_sync.setStyleSheet(f"color: {THEME['muted']}; font-weight: bold; font-size: 11px;")
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
            QPushButton { background-color: #2c3039; color: #e6e8ec; font-weight: bold; border: 1px solid #3a3f4b; border-radius: 4px; padding: 3px 8px; font-size: 11px; }
            QPushButton:hover { background-color: #363b46; }
            QPushButton:pressed { background-color: #1f2228; }
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
        row4.addStretch(1)
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
        # Connection bookkeeping (GUI thread only)
        self.link = {'online': None, 'rtt_ms': None, 'last_seen': 0.0, 'sent': 0,
                     'failed': 0, 'last_error': '', 'rssi': None}
        self._ping_in_flight = False
        self._ping_gen = 0             # results of superseded pings are dropped
        self._resolved = {}            # hostname -> IP from the last good ping
        self._ping_failures = 0
        self._cmd_failures = 0
        self._next_ping_at = None

        # Gait generation timer
        self.gait_timer = QTimer(self)
        self.gait_timer.timeout.connect(self._on_gait_timer_tick)

        # Waving animation state
        self.wave_timer = QTimer(self)
        self.wave_step = 0
        self.wave_timer.timeout.connect(self._on_wave_timer_tick)

        # Single ordered sender thread for /cmd requests (see send_command).
        # Created before init_ui(), which may already send commands.
        self._cmd_cv = threading.Condition()
        self._cmd_queue = []

        self.init_ui()
        self.update_log_signal.connect(self.log_to_console)
        self.update_status_signal.connect(self.set_status)
        threading.Thread(target=self._command_sender_loop, daemon=True).start()

        # Heartbeat: every 3 s while online, backing off to 10 s while offline
        self.ping_timer = QTimer(self)
        self.ping_timer.setSingleShot(True)
        self.ping_timer.timeout.connect(self.ping_quard_bot)
        QTimer.singleShot(800, self.ping_quard_bot)
        self.link_age_timer = QTimer(self)
        self.link_age_timer.timeout.connect(self._refresh_link_detail)
        self.link_age_timer.start(1000)

        # The firmware stands the robot up after 2 s without a /cmd request
        # (/status pings don't count), so re-send the active gait every second.
        # Repeating the same mode does not restart the gait on the robot.
        self.gait_keepalive_timer = QTimer(self)
        self.gait_keepalive_timer.timeout.connect(self._send_gait_keepalive)
        self.gait_keepalive_timer.start(1000)

    def _send_gait_keepalive(self):
        if self.active_gait in self.GAIT_PATTERNS:
            self.send_command({'mode': self.active_gait}, quiet=True)

    def init_ui(self):
        main_layout = QHBoxLayout(self)
        main_layout.setContentsMargins(12, 12, 12, 12)
        main_layout.setSpacing(12)

        # ── Left Column (Control Cards): Fixed width 490px ─────────────────────
        left_col = QWidget()
        left_layout = QVBoxLayout(left_col)
        left_layout.setContentsMargins(0, 0, 0, 0)
        left_layout.setSpacing(8)

        # 1. Connection card
        conn_box = QGroupBox("🌐 Quard Bot Connection")
        conn_box.setStyleSheet("""
            QGroupBox { font-weight: bold; font-size: 13px; border: 1px solid #444; border-radius: 6px; margin-top: 6px; padding-top: 10px; background-color: #2b2b2b; color: #4da6ff; }
            QGroupBox::title { subcontrol-origin: margin; padding: 0 5px; color: #4da6ff; }
        """)
        conn_layout = QVBoxLayout()
        conn_layout.setContentsMargins(8, 8, 8, 8)
        conn_layout.setSpacing(6)

        c_row1 = QHBoxLayout()
        self.combo_host = QComboBox()
        self.combo_host.setEditable(True)
        self.combo_host.setInsertPolicy(QComboBox.NoInsert)
        self.combo_host.setToolTip("Quard Bot address: http://sesame-robot.local, or its IP (shown on its OLED / router)")
        saved_hosts = _ui_settings().value("quard/hosts", [])
        if isinstance(saved_hosts, str):
            saved_hosts = [saved_hosts]
        hosts = [h for h in (saved_hosts or []) if isinstance(h, str) and h] or [self.quard_host]
        self.combo_host.addItems(hosts)
        self.combo_host.setCurrentText(hosts[0])
        self.input_host = self.combo_host.lineEdit()
        self.input_host.setFont(QFont("Monospace", 9))
        self.input_host.returnPressed.connect(lambda: self.ping_quard_bot(manual=True))
        self.combo_host.activated.connect(lambda _i: self.ping_quard_bot(manual=True))
        self._host_edit_timer = QTimer(self)
        self._host_edit_timer.setSingleShot(True)
        self._host_edit_timer.setInterval(900)
        self._host_edit_timer.timeout.connect(self.ping_quard_bot)
        self.input_host.textEdited.connect(self._on_host_edited)

        self.btn_ping = QPushButton("🔄 Ping")
        self.btn_ping.setToolTip("Check the connection now (GET /status)")
        self.btn_ping.setStyleSheet("QPushButton { background-color: #0284c7; color: white; font-weight: bold; border-radius: 4px; padding: 4px 10px; } QPushButton:hover { background-color: #0369a1; }")
        self.btn_ping.clicked.connect(lambda: self.ping_quard_bot(manual=True))
        self.chk_auto_ping = QCheckBox("Auto")
        self.chk_auto_ping.setChecked(True)
        self.chk_auto_ping.setToolTip("Keep checking: every 3 s while online, backing off to 10 s while unreachable")
        self.chk_auto_ping.toggled.connect(self._on_auto_ping_toggled)

        c_row1.addWidget(self.combo_host, 1)
        c_row1.addWidget(self.btn_ping)
        c_row1.addWidget(self.chk_auto_ping)
        conn_layout.addLayout(c_row1)

        c_row2 = QHBoxLayout()
        self.lbl_status = StatusPill("Checking…", 'busy')
        self.lbl_link_detail = muted_label()
        c_row2.addWidget(self.lbl_status)
        c_row2.addWidget(self.lbl_link_detail, 1)
        conn_layout.addLayout(c_row2)

        # Telemetry readouts row
        self.lbl_telemetry = QLabel("📡 Telemetry: waiting for the first reply…")
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
        presets_box = QGroupBox("⚡ Stance Presets && Zero-Tare Trims")
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
        ch_scroll.setStyleSheet("QScrollArea { border: none; background: transparent; }")
        ch_scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
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
            row.setObjectName("quardChannelRow")
            row.setAttribute(Qt.WA_StyledBackground, True)
            row.setStyleSheet("QWidget#quardChannelRow { background-color: #18181b; border: 1px solid #27272a; border-radius: 6px; }")
            r_layout = QHBoxLayout(row)
            r_layout.setContentsMargins(6, 3, 6, 3)

            lbl_name = QLabel(f"Ch {ch}: {name}")
            lbl_name.setFont(QFont("SansSerif", 9, QFont.Bold))
            lbl_name.setStyleSheet("color: #e4e4e7; min-width: 150px;")

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

        # 6. Console
        log_box = QGroupBox("📟 Quard Bot Console")
        log_box.setStyleSheet(group_box_qss('#f59e0b'))
        log_layout = QVBoxLayout()
        log_layout.setContentsMargins(6, 6, 6, 6)
        self.console = ConsoleWidget(
            "quard",
            input_placeholder="Quard command: move=w · ch=3&deg=70 · all=45 · speed=180 · status  (↑/↓ history)",
            quick_commands=[("Status", "status", "GET /status and show the reply"),
                            ("Stand", "move=s", "Stand still")])
        self.console.submitted.connect(self.on_console_submit)
        log_layout.addWidget(self.console, 1)
        log_box.setLayout(log_layout)
        right_layout.addWidget(log_box, 1)

        # Locked by E-STOP; the connection card and console stay usable
        self.motion_boxes = [dpad_box, presets_box, pca_box]

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

    def send_command(self, params, quiet=False):
        """Queue a /cmd request. Requests go out one at a time, in order; while
        one is in flight, newer angles for the same channel replace older ones
        so a slider drag cannot leave the servo at a stale angle."""
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
            del norm_params['mode']   # the firmware reads 'move' first

        if 'ch' in norm_params and 'angle' in norm_params:
            norm_params['deg'] = norm_params.pop('angle')

        query = urllib.parse.urlencode(norm_params)
        url = f"{self.request_base_url()}/cmd?{query}"
        move = str(norm_params.get('move', '')).lower()
        entry = {"url": url, "params": params, "quiet": quiet, "query": query,
                 "t": time.monotonic(),
                 # Stand is always delivered, however late; motion is not.
                 "always": move in ('s', 'stand', 'stop')}

        with self._cmd_cv:
            last = self._cmd_queue[-1] if self._cmd_queue else None
            if last and 'ch' in params and last["params"].get('ch') == params.get('ch'):
                self._cmd_queue[-1] = entry
            elif quiet and any(e["quiet"] for e in self._cmd_queue):
                pass  # a keepalive is already waiting
            else:
                self._cmd_queue.append(entry)
            self._cmd_cv.notify()

    def _command_sender_loop(self):
        while True:
            with self._cmd_cv:
                while not self._cmd_queue:
                    self._cmd_cv.wait()
                entry = self._cmd_queue.pop(0)
            info = {"type": "keepalive" if entry["quiet"] else "cmd",
                    "params": entry["params"], "query": entry.get("query", "")}
            waited = time.monotonic() - entry.get("t", time.monotonic())
            if waited > 2.0 and not entry.get("always"):
                # Queued behind a slow request: replaying motion this late
                # would move the robot unexpectedly, so skip it.
                if not entry["quiet"]:
                    info["type"] = "dropped"
                    info["waited"] = waited
                    try:
                        self.update_status_signal.emit(False, info)
                    except RuntimeError:
                        return
                continue
            t0 = time.monotonic()
            try:
                req = urllib.request.Request(entry["url"])
                with urllib.request.urlopen(req, timeout=1.5) as resp:
                    info["reply"] = resp.read().decode('utf-8', errors='ignore').strip()
                ok = True
            except urllib.error.HTTPError as e:
                # The robot answered (e.g. 400 "Bad Args"): online, command refused.
                try:
                    body = e.read().decode('utf-8', errors='ignore').strip()
                except Exception:
                    body = ""
                info["rejected"] = e.code
                info["reply"] = body or str(e.reason)
                ok = True
            except Exception as e:
                info["error"] = describe_net_error(e)
                ok = False
            info["rtt_ms"] = (time.monotonic() - t0) * 1000.0
            try:
                self.update_status_signal.emit(ok, info)
            except RuntimeError:
                return  # widget destroyed

    def set_motion_enabled(self, on):
        for box in getattr(self, 'motion_boxes', []):
            box.setEnabled(on)

    def current_host_url(self):
        host = self.input_host.text().strip() or self.quard_host
        if not host.startswith("http://") and not host.startswith("https://"):
            host = "http://" + host
        return host.rstrip('/')

    def request_base_url(self):
        """current_host_url() with the host name swapped for the IP the last
        good ping resolved, so commands skip slow .local lookups."""
        base = self.current_host_url()
        parts = urllib.parse.urlsplit(base)
        ip = self._resolved.get(parts.hostname or "")
        if not ip:
            return base
        netloc = ip + (f":{parts.port}" if parts.port else "")
        return urllib.parse.urlunsplit((parts.scheme, netloc, parts.path, "", "")).rstrip('/')

    def _on_host_edited(self, _text):
        """The address changed: commands queued for the old one are dropped."""
        with self._cmd_cv:
            dropped = sum(1 for e in self._cmd_queue if not e["quiet"])
            self._cmd_queue.clear()
        if dropped:
            self.console.log(f"Address changed: dropped {dropped} queued command(s)", 'warn')
        self.link.update(online=None, rtt_ms=None, last_error='')
        self._resolved = {}
        self._ping_gen += 1          # ignore a ping still running for the old address
        self._ping_in_flight = False
        self._refresh_link_detail()
        if self.chk_auto_ping.isChecked():
            self._host_edit_timer.start()

    def ping_quard_bot(self, manual=False):
        """GET /status in the background. Automatic pings wait for the previous
        one; a manual ping replaces it (e.g. the old host is still resolving)."""
        if self._ping_in_flight and not manual:
            return
        self.ping_timer.stop()
        self._ping_in_flight = True
        self._ping_gen += 1
        gen = self._ping_gen
        host = self.current_host_url()
        if manual:
            self.lbl_status.set_state('busy', "Checking…")
            self.console.log(f"GET {host}/status", 'tx')

        def worker():
            info = {"type": "ping", "manual": manual, "host": host, "gen": gen}
            t0 = time.monotonic()
            try:
                parts = urllib.parse.urlsplit(host)
                name = parts.hostname or ""
                ip = socket.getaddrinfo(name, parts.port or 80, socket.AF_INET, socket.SOCK_STREAM)[0][4][0]
                if ip != name:
                    info["resolved"] = (name, ip)
                t0 = time.monotonic()   # RTT excludes the name lookup
                netloc = ip + (f":{parts.port}" if parts.port else "")
                url = urllib.parse.urlunsplit((parts.scheme, netloc, "/status", "", ""))
                with urllib.request.urlopen(urllib.request.Request(url), timeout=2.0) as resp:
                    raw = resp.read().decode('utf-8', errors='ignore')
                try:
                    info["telemetry"] = json.loads(raw)
                except ValueError:
                    info["telemetry"] = {}
                    info["note"] = raw.strip()[:120]
                ok = True
            except urllib.error.HTTPError as e:
                # It answered, so it's online; this firmware just has no /status.
                info["telemetry"] = {}
                info["note"] = f"online (HTTP {e.code} on /status)"
                ok = True
            except Exception as e:
                info["error"] = describe_net_error(e)
                ok = False
            info["rtt_ms"] = (time.monotonic() - t0) * 1000.0
            try:
                self.update_status_signal.emit(ok, info)
            except RuntimeError:
                pass

        threading.Thread(target=worker, daemon=True).start()

    def ping_robot(self):
        """Compatibility alias for ping_quard_bot."""
        self.ping_quard_bot()

    def _on_auto_ping_toggled(self, on):
        if on:
            self.ping_quard_bot()
        else:
            self.ping_timer.stop()
            self._next_ping_at = None
        self._refresh_link_detail()

    def _schedule_next_ping(self):
        if not self.chk_auto_ping.isChecked():
            self._next_ping_at = None
            return
        if self.link['online']:
            delay = 3.0
        else:
            delay = min(10.0, 2.0 * (2 ** max(0, self._ping_failures - 1)))
        self._next_ping_at = time.time() + delay
        self.ping_timer.start(int(delay * 1000))

    def _remember_host(self):
        text = self.input_host.text().strip()
        if not text:
            return
        hosts = [self.combo_host.itemText(i) for i in range(self.combo_host.count())]
        if hosts and hosts[0] == text:
            return
        hosts = [text] + [h for h in hosts if h != text][:5]
        self.combo_host.blockSignals(True)
        self.combo_host.clear()
        self.combo_host.addItems(hosts)
        self.combo_host.setCurrentText(text)
        self.combo_host.blockSignals(False)
        _ui_settings().setValue("quard/hosts", hosts)

    def set_status(self, is_online, info):
        """Result of a ping or command (runs in the GUI thread)."""
        t = info.get("type")
        if t == 'ping' and info.get('gen') != self._ping_gen:
            return   # superseded by a newer ping
        was_online = self.link['online']
        now = time.time()

        if t == 'dropped':
            self.console.log(f"/cmd?{info.get('query', '')}  not sent: waited {info.get('waited', 0):.1f} s "
                             "behind a slow request (motion is never replayed late)", 'warn')
            return

        if t == 'ping' and info.get('resolved'):
            name, ip = info['resolved']
            if is_online:
                self._resolved = {name: ip}
        elif t == 'ping' and not is_online:
            self._resolved = {}

        if is_online:
            self.link['online'] = True
            self.link['last_seen'] = now
            self.link['rtt_ms'] = info.get('rtt_ms')
            self.link['last_error'] = ''
            self._cmd_failures = 0
            if t == 'ping':
                self._ping_failures = 0
        else:
            self.link['last_error'] = info.get('error', 'no reply')
            if t == 'ping':
                self._ping_failures += 1
                self.link['online'] = False
            else:
                # One lost command isn't an outage; two in a row is.
                self._cmd_failures += 1
                if self._cmd_failures >= 2:
                    self.link['online'] = False

        if t in ('cmd', 'keepalive'):
            self.link['sent'] += 1
            if not is_online or info.get('rejected'):
                self.link['failed'] += 1

        if t == 'ping':
            if is_online:
                tel = info.get('telemetry') or {}
                self.link['rssi'] = tel.get('rssi')
                if tel:
                    up = int(tel.get('uptime_ms', 0) / 1000)
                    self.lbl_telemetry.setText(
                        f"📡 IP: {tel.get('ip', '?')} | RSSI: {tel.get('rssi', '--')} dBm | Mode: {tel.get('mode', '?')} | "
                        f"Step: {tel.get('step_delay', self.step_delay)} ms | Up: {up // 60}m {up % 60:02d}s")
                if info.get('manual'):
                    self.console.log(json.dumps(tel) if tel else (info.get('note') or "online"), 'rx')
                self._remember_host()
            elif info.get('manual'):
                self.console.log(f"No answer from {info.get('host')}: {info.get('error')}", 'error')

        if self.link['online'] is not None and self.link['online'] != was_online:
            if self.link['online']:
                rtt = f" (ping {self.link['rtt_ms']:.0f} ms)" if self.link['rtt_ms'] is not None else ""
                self.console.log(f"Quard Bot online at {self.current_host_url()}{rtt}", 'ok')
            else:
                self.console.log(f"Quard Bot unreachable: {self.link['last_error']}", 'error')
                self.lbl_telemetry.setText("📡 Telemetry: no reply (is the robot on, and on the same Wi-Fi?)")

        if t == 'cmd':
            q = info.get('query', '')
            params = info.get('params') or {}
            key = f"ch{params['ch']}" if 'ch' in params else None
            ms = info.get('rtt_ms', 0.0)
            if info.get('rejected'):
                self.console.log(f"/cmd?{q}  ←  refused: HTTP {info['rejected']} {info.get('reply', '')}", 'warn', key=key)
            elif is_online:
                self.console.log(f"/cmd?{q}  ←  {info.get('reply') or 'OK'}  ({ms:.0f} ms)", 'tx', key=key)
            else:
                self.console.log(f"/cmd?{q}  ✗  {info.get('error')}", 'error', key=key)
        elif t == 'keepalive' and not is_online:
            self.console.log(f"Gait keep-alive failed: {info.get('error')}", 'warn', key='keepalive')

        self.is_online = bool(self.link['online'])
        if t == 'ping':
            self._ping_in_flight = False
            self._schedule_next_ping()
        self._refresh_link_detail()

    def _refresh_link_detail(self):
        if not hasattr(self, 'lbl_link_detail'):
            return
        l = self.link
        now = time.time()
        parts = []
        if l['online'] is None:
            self.lbl_status.set_state('busy', "Checking…")
            parts.append(self.current_host_url())
        elif l['online']:
            rtt = f" · {l['rtt_ms']:.0f} ms" if l['rtt_ms'] is not None else ""
            self.lbl_status.set_state('ok', "Online" + rtt)
            parts.append(f"seen {now - l['last_seen']:.0f} s ago")
            if l['rssi'] not in (None, 0):
                parts.append(f"RSSI {l['rssi']} dBm")
        else:
            self.lbl_status.set_state('error', "Unreachable")
            parts.append(l['last_error'] or "no reply")
            if l['last_seen']:
                parts.append(f"last seen {now - l['last_seen']:.0f} s ago")
        if self._next_ping_at and not l['online'] and not self._ping_in_flight:
            parts.append(f"retry in {max(0, self._next_ping_at - now):.0f} s")
        if l['sent']:
            parts.append(f"sent {l['sent']}" + (f", failed {l['failed']}" if l['failed'] else ""))
        self.lbl_link_detail.setText(" · ".join(parts))

    def on_console_submit(self, text):
        """Console input: 'status', or a /cmd query like move=w or ch=3&deg=70."""
        t = text.strip()
        if t.lower() in ('status', '/status', 'ping'):
            self.ping_quard_bot(manual=True)
            return
        q = t
        for prefix in ('/cmd?', 'cmd?', '?'):
            if q.lower().startswith(prefix):
                q = q[len(prefix):]
                break
        params = dict(urllib.parse.parse_qsl(q))
        if not params:
            self.console.log(f"Not understood: {t!r}. Use key=value pairs such as move=w or ch=3&deg=70", 'warn')
            return
        move = str(params.get('move', params.get('mode', params.get('cmd', '')))).lower()
        is_stand = set(params) <= {'move', 'mode', 'cmd'} and move in ('s', 'stand', 'stop')
        if getattr(self.main_gui, 'estop_engaged', False) and not is_stand:
            self.console.log(f"Blocked by E-STOP: {t}", 'warn')
            return
        self.send_command(params)

    def log_to_console(self, text, kind=None):
        self.console.log(text, kind or infer_console_kind(text))

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
            # get_joint_radians() renders (slider + offset); choose the offset
            # that makes the twin show the reference pose right now.
            self.quard_offsets[str(ch)] = round(ref - curr_deg, 1)
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
    link_signal = pyqtSignal(int, str, str)   # generation, state, detail (from worker threads)

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
        self.link_signal.connect(self._on_link_signal)

        # Data source: 'sim', 'wifi', 'serial' or 'off'. Bumping _gen tells the
        # previous source's worker thread to stop.
        self.source = 'sim'
        self._gen = 0
        self.active_url = None
        self.active_port = None
        self.active_baud = None
        self.link_phase = 'sim'
        self.link_error = ''
        self.last_rx = 0.0
        self.phase_since = time.time()
        self.rx_times = collections.deque(maxlen=200)
        self._stale_logged = False
        self._btn_key = None

        self.init_ui()
        self.refresh_port_list()

        # Timer to drive simulation when hardware is not connected
        self.sim_timer = QTimer(self)
        self.sim_timer.timeout.connect(self.sim_tick)
        self.sim_timer.start(1000)
        self.link_timer = QTimer(self)
        self.link_timer.timeout.connect(self._refresh_link)
        self.link_timer.start(1000)
        self.set_source('sim')

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

        # ── 1. Connection card: simulated, Wi-Fi (HTTP /data) or USB serial ─
        conn_box = QGroupBox("🔌 Rover Connection")
        conn_box.setStyleSheet("""
            QGroupBox { font-weight: bold; font-size: 13px; border: 1px solid #444; border-radius: 6px; margin-top: 6px; padding-top: 10px; background-color: #242424; color: #38bdf8; }
            QGroupBox::title { subcontrol-origin: margin; padding: 0 6px; color: #38bdf8; }
        """)
        conn_layout = QVBoxLayout(conn_box)
        conn_layout.setSpacing(6)
        field_qss = "background-color: #18181b; color: #e6e8ec; padding: 4px 8px; border: 1px solid #3f3f46; border-radius: 4px;"

        wifi_row = QHBoxLayout()
        lbl_wifi = QLabel("🌐 Wi-Fi")
        lbl_wifi.setStyleSheet("color: #cccccc; font-weight: bold; font-size: 12px;")
        lbl_wifi.setFixedWidth(70)
        self.txt_wifi_ip = QLineEdit(str(_ui_settings().value("rover/wifi_host", "192.168.1.100")))
        self.txt_wifi_ip.setPlaceholderText("Rover IP, e.g. 192.168.1.50 (reads http://<ip>/data)")
        self.txt_wifi_ip.setStyleSheet(field_qss)
        self.txt_wifi_ip.returnPressed.connect(lambda: self.set_source('wifi'))
        self.btn_wifi_connect = QPushButton("🌐 Connect Wi-Fi")
        self.btn_wifi_connect.clicked.connect(self.toggle_wifi_connection)
        wifi_row.addWidget(lbl_wifi)
        wifi_row.addWidget(self.txt_wifi_ip, 1)
        wifi_row.addWidget(self.btn_wifi_connect)
        conn_layout.addLayout(wifi_row)

        serial_row = QHBoxLayout()
        lbl_port = QLabel("🔌 Serial")
        lbl_port.setStyleSheet("color: #cccccc; font-weight: bold; font-size: 12px;")
        lbl_port.setFixedWidth(70)
        self.combo_port = QComboBox()
        self.combo_port.setEditable(True)
        self.combo_port.setMinimumWidth(150)
        self.combo_port.setToolTip("Serial port the rover's ESP32 is on")
        btn_rescan = QPushButton("⟳")
        btn_rescan.setToolTip("Rescan serial ports")
        btn_rescan.setFixedWidth(32)
        btn_rescan.setStyleSheet(button_qss('neutral', padding='3px 6px'))
        btn_rescan.clicked.connect(lambda: self.refresh_port_list(report=True))
        self.combo_baud = QComboBox()
        self.combo_baud.addItems(["115200", "9600", "57600", "230400"])
        self.btn_connect = QPushButton("🔌 Connect Serial")
        self.btn_connect.clicked.connect(self.toggle_serial_connection)
        self.btn_sim = QPushButton("▶ Simulate")
        self.btn_sim.setToolTip("Show demo values instead of rover data")
        self.btn_sim.clicked.connect(self.toggle_sim)
        serial_row.addWidget(lbl_port)
        serial_row.addWidget(self.combo_port, 1)
        serial_row.addWidget(btn_rescan)
        serial_row.addWidget(self.combo_baud)
        serial_row.addWidget(self.btn_connect)
        serial_row.addWidget(self.btn_sim)
        conn_layout.addLayout(serial_row)

        status_row = QHBoxLayout()
        self.lbl_status = StatusPill("Simulated", 'busy')
        self.lbl_link_detail = muted_label()
        status_row.addWidget(self.lbl_status)
        status_row.addWidget(self.lbl_link_detail, 1)
        conn_layout.addLayout(status_row)

        main_layout.addWidget(conn_box)

        # ── 2. Scrollable Sensor Cards Area ────────────────────────────────
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        scroll.setStyleSheet("QScrollArea { border: none; background-color: transparent; }")

        container = QWidget()
        grid_layout = QGridLayout(container)
        grid_layout.setContentsMargins(0, 0, 0, 0)
        grid_layout.setSpacing(8)

        # ── Card A: Climate (DHT11 & BMP-280) ──────────────────────────────
        climate_box = QGroupBox("🌡️ Climate && Atmospheric (DHT11 && BMP-280)")
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
        mpu_box = QGroupBox("🧭 MPU6050 6-DOF IMU (Motion && Tilt)")
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
        gps_box = QGroupBox("🛰️ GPS NEO-6M Navigation && Node Status")
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

        # ── 3. Console ─────────────────────────────────────────────────────
        log_box = QGroupBox("📜 Rover Console")
        log_box.setStyleSheet(group_box_qss('#38bdf8'))
        log_layout = QVBoxLayout(log_box)
        log_layout.setContentsMargins(6, 6, 6, 6)
        self.console = ConsoleWidget("rover")
        log_layout.addWidget(self.console)
        main_layout.addWidget(log_box, 1)

    # ── Data source / connection ──────────────────────────────────────────
    @staticmethod
    def _wifi_url(text):
        cur = (text or "").strip() or "192.168.1.100"
        if not cur.startswith(("http://", "https://")):
            cur = "http://" + cur
        cur = cur.rstrip('/')
        return cur if cur.endswith('/data') else cur + "/data"

    def source_label(self):
        if self.source == 'wifi':
            return f"Wi-Fi {self.active_url}"
        if self.source == 'serial':
            return f"{self.active_port} @ {self.active_baud}"
        return "simulated data" if self.source == 'sim' else "nothing"

    def refresh_port_list(self, report=False):
        """Fill the port list; preselect a port the arm commander isn't using."""
        current = self.combo_port.currentText().strip()
        ports = list_serial_ports()
        arm = getattr(self.main_gui, 'arm_link', None) or {}
        arm_port = arm.get('port') if arm.get('transport') == 'serial' else None
        self.combo_port.clear()
        for dev, desc in ports:
            label = dev + ("   (arm)" if dev == arm_port else "")
            self.combo_port.addItem(label, dev)
            if desc:
                self.combo_port.setItemData(self.combo_port.count() - 1, desc, Qt.ToolTipRole)
        free = [d for d, _ in ports if d != arm_port]
        self.combo_port.setEditText(current or (free[0] if free else (ports[0][0] if ports else "/dev/ttyUSB1")))
        if report:
            found = ", ".join(f"{d} ({desc})" if desc else d for d, desc in ports) or "none"
            self.console.log(f"Serial ports found: {found}", 'info')

    def selected_port(self):
        text = self.combo_port.currentText().strip()
        idx = self.combo_port.findText(text)
        data = self.combo_port.itemData(idx) if idx >= 0 else None
        return data or text.replace("(arm)", "").strip()

    def set_source(self, src):
        self._gen += 1
        gen = self._gen
        self.source = src
        self.link_phase = 'connecting' if src in ('wifi', 'serial') else src
        self.link_error = ''
        self.rx_times.clear()
        self.last_rx = 0.0
        self.phase_since = time.time()
        self._stale_logged = False
        st = self.sensor_state
        st['simulating'] = src == 'sim'
        st['wifi_connected'] = src == 'wifi'
        st['connected'] = src == 'serial'
        if src == 'wifi':
            self.active_url = self._wifi_url(self.txt_wifi_ip.text())
            _ui_settings().setValue("rover/wifi_host", self.txt_wifi_ip.text().strip())
            self.console.log(f"Connecting to {self.active_url} …", 'info')
            threading.Thread(target=self._wifi_worker, args=(gen, self.active_url), daemon=True).start()
        elif src == 'serial':
            self.console.log(f"Opening {self.active_port} @ {self.active_baud} …", 'info')
            threading.Thread(target=self._serial_worker, args=(gen, self.active_port, self.active_baud), daemon=True).start()
        elif src == 'sim':
            self.console.log("Showing simulated data (not from the rover)", 'info')
        else:
            self.console.log("Rover data source off", 'info')
        self._refresh_link()

    def toggle_wifi_connection(self):
        self.set_source('off' if self.source == 'wifi' else 'wifi')

    def toggle_sim(self):
        self.set_source('off' if self.source == 'sim' else 'sim')

    def toggle_serial_connection(self):
        if self.source == 'serial':
            self.set_source('off')
            return
        port = self.selected_port()
        try:
            baud = int(self.combo_baud.currentText())
        except ValueError:
            self.console.log("Baud rate must be a number", 'error')
            return
        if not port:
            self.console.log("Pick a serial port first (⟳ rescans)", 'warn')
            return
        gui = self.main_gui
        arm = getattr(gui, 'arm_link', None) or {}
        alive = gui.commander_alive() if hasattr(gui, 'commander_alive') else False
        if alive and arm.get('transport') == 'serial' and arm.get('port') == port:
            answer = QMessageBox.question(
                self, "Port used by the arm",
                f"{port} is the arm ESP32's port: servo_serial_commander has it open.\n\n"
                "Opening it here as well resets that ESP32 and takes its replies. Connect anyway?",
                QMessageBox.Yes | QMessageBox.No, QMessageBox.No)
            if answer != QMessageBox.Yes:
                return
        self.active_port, self.active_baud = port, baud
        self.sensor_state['port'], self.sensor_state['baudrate'] = port, baud
        self.set_source('serial')

    def _wifi_worker(self, gen, url):
        failures = 0
        while gen == self._gen:
            try:
                req = urllib.request.Request(url, headers={"User-Agent": "RobotArm-GUI/1.0"})
                with urllib.request.urlopen(req, timeout=2.0) as resp:
                    raw = resp.read().decode('utf-8', errors='ignore')
                if gen != self._gen:
                    return
                if not self.parse_sensor_json(raw):
                    raise ValueError(f"reply from {url} is not JSON: {raw.strip()[:60]!r}")
                failures = 0
                self.console.log(raw.strip()[:240], 'data')
                self.link_signal.emit(gen, 'live', '')
                self.update_data_signal.emit(self.sensor_state)
                time.sleep(0.5)
            except Exception as e:
                if gen != self._gen:
                    return
                failures += 1
                if isinstance(e, urllib.error.HTTPError) and e.code == 404:
                    detail = f"HTTP 404: no /data page at {url}"
                elif isinstance(e, ValueError):
                    detail = str(e)
                else:
                    detail = describe_net_error(e)
                self.link_signal.emit(gen, 'error', detail)
                time.sleep(min(5.0, 0.5 * (2 ** min(failures, 4))))

    def _serial_worker(self, gen, port, baud):
        try:
            import serial
        except ImportError:
            self.link_signal.emit(gen, 'fatal', "pyserial is not installed (sudo apt install python3-serial)")
            return
        while gen == self._gen:
            try:
                with serial.Serial(port, baud, timeout=0.5) as ser:
                    self.link_signal.emit(gen, 'opened', f"{port} @ {baud}")
                    while gen == self._gen:
                        raw_bytes = ser.readline()
                        if not raw_bytes:
                            continue
                        line = raw_bytes.decode('utf-8', errors='ignore').strip()
                        if not line:
                            continue
                        self.parse_sensor_line(line)
                        self.console.log(line, 'data')
                        self.link_signal.emit(gen, 'live', '')
                        self.update_data_signal.emit(self.sensor_state)
            except Exception as e:
                if gen != self._gen:
                    return
                self.link_signal.emit(gen, 'error', explain_serial_error(e))
                time.sleep(2.0)

    def _on_link_signal(self, gen, state, detail):
        """Worker report (GUI thread). Ignores reports from replaced sources."""
        if gen != self._gen:
            return
        before = self.link_phase
        if state == 'live':
            now = time.time()
            self.last_rx = now
            self.rx_times.append(now)
            if before != 'live' or self._stale_logged:
                self.console.log(f"Receiving data from {self.source_label()}", 'ok')
            self._stale_logged = False
            self.link_phase, self.link_error = 'live', ''
        elif state == 'opened':
            self.console.log(f"Opened {detail}; waiting for data…", 'info')
            self.link_phase, self.link_error = 'waiting', ''
            self.phase_since = time.time()
        elif state in ('error', 'fatal'):
            if detail != self.link_error:
                self.console.log(f"{self.source_label()}: {detail}", 'error')
            self.link_phase, self.link_error = 'error', detail
            if state == 'fatal':
                self.set_source('off')
                return
        if self.link_phase != before:
            self._refresh_link()

    def _refresh_link(self):
        if not hasattr(self, 'lbl_link_detail'):
            return
        now = time.time()
        src = self.source
        if src == 'off':
            kind, label, detail = 'idle', "Off", "No data source. Choose Wi-Fi, Serial or Simulate."
        elif src == 'sim':
            kind, label, detail = 'busy', "Simulated", "Demo values, not from the rover"
        else:
            target = self.source_label()
            phase = self.link_phase
            if phase == 'live' and now - self.last_rx < 5.0:
                rate = sum(1 for t in self.rx_times if now - t < 5.0) / 5.0
                unit = "updates/s" if src == 'wifi' else "lines/s"
                kind, label = 'ok', "Live"
                detail = f"{target} · {rate:.1f} {unit} · last {now - self.last_rx:.1f} s ago"
            elif phase == 'live':
                kind, label = 'warn', "No data"
                detail = f"{target} · nothing for {now - self.last_rx:.0f} s"
                if not self._stale_logged:
                    self._stale_logged = True
                    self.console.log(f"No data from {target} for 5 s", 'warn')
            elif phase == 'waiting':
                kind, label = 'busy', "Waiting for data"
                detail = target
                if now - self.phase_since > 5.0:
                    detail += " · port open but silent: wrong baud rate, or the board isn't sending?"
            elif phase == 'error':
                kind, label = 'error', "Error"
                detail = f"{target} · {self.link_error} · retrying"
            else:
                kind, label, detail = 'busy', "Connecting…", target
        self.lbl_status.set_state(kind, label)
        self.lbl_link_detail.setText(detail)
        self.lbl_link_detail.setToolTip(detail)
        if src != self._btn_key:
            self._btn_key = src
            wifi_on, ser_on, sim_on = src == 'wifi', src == 'serial', src == 'sim'
            self.btn_wifi_connect.setText("⏹ Disconnect Wi-Fi" if wifi_on else "🌐 Connect Wi-Fi")
            self.btn_wifi_connect.setStyleSheet(button_qss('danger' if wifi_on else 'primary', padding='5px 12px'))
            self.btn_connect.setText("⏹ Disconnect Serial" if ser_on else "🔌 Connect Serial")
            self.btn_connect.setStyleSheet(button_qss('danger' if ser_on else 'primary', padding='5px 12px'))
            self.btn_sim.setText("⏸ Stop Simulation" if sim_on else "▶ Simulate")
            self.btn_sim.setStyleSheet(button_qss('warning' if sim_on else 'neutral', padding='5px 12px'))

    def parse_sensor_json(self, raw_str):
        """Apply a /data JSON reply. Returns False if it isn't a JSON object."""
        try:
            d = json.loads(raw_str)
        except Exception:
            return False
        if not isinstance(d, dict):
            return False

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
        return True

    def sim_tick(self):
        if self.source != 'sim':
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
            self.console.log(line, 'data')
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
        self.console.log(f"MPU6050 horizon tared: pitch offset {raw_p:.1f}°, roll offset {raw_r:.1f}°", 'ok')

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

        self.setWindowTitle("Robotics Control Center — Arm · Quard Bot · Rover")
        self.resize(1400, 860)
        self.setMinimumSize(1000, 640)
        # Theme lives on the QApplication (a selector-less window stylesheet
        # painted a dark box behind every label).
        apply_app_theme(QApplication.instance())
        self.estop_engaged = False
        # Arm link, as reported by servo_serial_commander (/esp32_link_state)
        self.arm_link = None
        self.arm_link_rx_t = 0.0
        self.arm_legacy = None          # (connected, text, time) from /esp32_connection_status
        self.arm_link_last_state = None
        self._gui_started = time.time()
        self._comm_mode_set_at = 0.0
        self._arm_fix_mode = 'SERIAL'
        self._raw_cmd_hint_shown = False
        # INI everywhere (~/.config/CodeForIndia/RobotDashboard.conf on Linux)
        self.settings = QSettings(QSettings.IniFormat, QSettings.UserScope, "CodeForIndia", "RobotDashboard")

        self.calib_data = self.load_calibration()
        self.joints_dict = self.parse_urdf_joints(URDF_PATH)
        self.last_conn_state = None

        # ── Main Splitter: Left Control Panel + Right RViz 3D Viewport ─────
        main_splitter = QSplitter(Qt.Horizontal)

        # ── Left Control Panel Widget ──────────────────────────────────────
        control_widget = QWidget()
        control_layout = QVBoxLayout()
        control_layout.setContentsMargins(8, 8, 8, 8)

        # ── Toolbar: motion row + calibration row ─────────────────────────
        # Colour marks what moves the robot or saves; everything else is neutral.
        self.btn_sim = QPushButton("▶️ Start Sim")
        self.btn_sim.setCheckable(True)
        self.btn_sim.setToolTip("Sweep every joint through its limits (sinusoidal demo motion)")
        self.btn_sim.setStyleSheet(button_qss('warning', checked='danger'))
        self.btn_sim.clicked.connect(self.toggle_simulation)

        btn_home = QPushButton("🏠 Go Home")
        btn_home.setToolTip("Smoothly return all joints to home position over 2 seconds")
        btn_home.setStyleSheet(button_qss('primary'))
        btn_home.clicked.connect(self.go_to_home_pose_smooth)

        btn_center = QPushButton("⊕ Center All")
        btn_center.setToolTip("Move every joint to 90°")
        btn_center.setStyleSheet(button_qss('neutral'))
        btn_center.clicked.connect(self.center_all)

        self.btn_gesture = QPushButton("✋ Gesture Mode")
        self.btn_gesture.setCheckable(True)
        self.btn_gesture.setToolTip("Toggle real-time computer vision body gesture teleoperation (webcam)")
        self.btn_gesture.setStyleSheet(button_qss('neutral', checked='success'))
        self.btn_gesture.clicked.connect(self.toggle_gesture_mode)

        btn_set_home = QPushButton("📌 Set Home")
        btn_set_home.setToolTip("Save current slider positions as default Homing Pose")
        btn_set_home.setStyleSheet(button_qss('neutral'))
        btn_set_home.clicked.connect(self.set_current_as_home_pose)

        btn_sync_phys = QPushButton("🎯 Sync to Physical")
        btn_sync_phys.setToolTip("Tare/Sync virtual arm to physical position: sets sync offset so 3D model matches physical pose")
        btn_sync_phys.setStyleSheet(button_qss('neutral'))
        btn_sync_phys.clicked.connect(self.sync_all_joints_to_physical)

        self.btn_comm_mode = QPushButton("📡 Comm: WiFi")
        self.btn_comm_mode.setCheckable(True)
        self.btn_comm_mode.setChecked(False) # False = WiFi, True = Serial
        self.btn_comm_mode.setToolTip("Toggle ESP32 control mode between Wi-Fi UDP and USB Serial")
        self.btn_comm_mode.setStyleSheet(button_qss('neutral', checked='neutral'))
        self.btn_comm_mode.clicked.connect(self.toggle_comm_mode)

        btn_save = QPushButton("💾 Save Config")
        btn_save.setToolTip("Save limits, home pose and sync offsets (Ctrl+S)")
        btn_save.setStyleSheet(button_qss('success'))
        btn_save.clicked.connect(self.save_calibration)

        # Locked while E-STOP is engaged
        self.motion_buttons = [self.btn_sim, btn_home, btn_center, self.btn_gesture]

        motion_row = QHBoxLayout()
        motion_row.setSpacing(6)
        motion_row.addWidget(section_label("Motion"))
        for b in (self.btn_sim, btn_home, btn_center, self.btn_gesture):
            motion_row.addWidget(b)
        motion_row.addStretch()
        control_layout.addLayout(motion_row)

        calib_row = QHBoxLayout()
        calib_row.setSpacing(6)
        calib_row.addWidget(section_label("Calibration"))
        for b in (btn_set_home, btn_sync_phys, self.btn_comm_mode):
            calib_row.addWidget(b)
        calib_row.addStretch()
        calib_row.addWidget(btn_save)
        control_layout.addLayout(calib_row)

        # ── Top Control Bar 2: Simulation Speed Slider ────────────────────
        speed_widget = QWidget()
        speed_widget.setObjectName("speedBar")
        speed_widget.setAttribute(Qt.WA_StyledBackground, True)
        speed_widget.setStyleSheet(f"QWidget#speedBar {{ background-color: {THEME['surface']}; border: 1px solid {THEME['border']}; border-radius: 6px; }}")
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
        scroll.setStyleSheet("QScrollArea { border: none; background: transparent; }")

        scroll_content = QWidget()
        scroll_layout = QVBoxLayout()

        self.joint_widgets = {}
        for j_name, (r_min, r_max) in self.joints_dict.items():
            w = CalibratedJointWidget(j_name, r_min, r_max, self.calib_data, self)
            self.joint_widgets[j_name] = w
            scroll_layout.addWidget(w)

        scroll_layout.addStretch(1)
        scroll_content.setLayout(scroll_layout)
        scroll.setWidget(scroll_content)

        # ── Bottom Section: arm ESP32 link card + console ─────────────────
        console_group = QGroupBox("📟 Arm ESP32 Link && Console")
        console_group.setStyleSheet(group_box_qss('#fbbf24'))
        console_layout = QVBoxLayout()
        console_layout.setContentsMargins(8, 8, 8, 8)
        console_layout.setSpacing(6)

        link_row = QHBoxLayout()
        self.arm_link_pill = StatusPill("Waiting for commander…", 'busy')
        self.lbl_arm_link_detail = muted_label()
        self.btn_arm_reconnect = QPushButton("⟳ Reconnect")
        self.btn_arm_reconnect.setToolTip("Close and reopen the commander's serial port / UDP socket now")
        self.btn_arm_reconnect.setStyleSheet(button_qss('neutral', padding='4px 10px'))
        self.btn_arm_reconnect.clicked.connect(lambda: self.send_link_command("RECONNECT"))
        self.btn_arm_link_settings = QPushButton("⚙ Link")
        self.btn_arm_link_settings.setCheckable(True)
        self.btn_arm_link_settings.setToolTip("Change the arm's serial port, baud rate or Wi-Fi address")
        self.btn_arm_link_settings.setStyleSheet(button_qss('neutral', checked='primary', padding='4px 10px'))
        link_row.addWidget(self.arm_link_pill)
        link_row.addWidget(self.lbl_arm_link_detail, 1)
        link_row.addWidget(self.btn_arm_reconnect)
        link_row.addWidget(self.btn_arm_link_settings)
        console_layout.addLayout(link_row)

        # Shown when the ESP32 replies but only obeys the other link
        self.arm_mismatch_bar = QWidget()
        self.arm_mismatch_bar.setObjectName("armMismatch")
        self.arm_mismatch_bar.setAttribute(Qt.WA_StyledBackground, True)
        self.arm_mismatch_bar.setStyleSheet("QWidget#armMismatch { background-color: #3d2a07; border: 1px solid #f59e0b; border-radius: 6px; }")
        mm_layout = QHBoxLayout(self.arm_mismatch_bar)
        mm_layout.setContentsMargins(8, 4, 8, 4)
        self.lbl_arm_mismatch = QLabel("")
        self.lbl_arm_mismatch.setStyleSheet("color: #fde68a;")
        self.lbl_arm_mismatch.setWordWrap(True)
        self.btn_arm_fix_mode = QPushButton("Switch ESP32")
        self.btn_arm_fix_mode.setStyleSheet(button_qss('warning', padding='4px 10px'))
        self.btn_arm_fix_mode.clicked.connect(self.fix_arm_mode_mismatch)
        mm_layout.addWidget(self.lbl_arm_mismatch, 1)
        mm_layout.addWidget(self.btn_arm_fix_mode)
        self.arm_mismatch_bar.hide()
        console_layout.addWidget(self.arm_mismatch_bar)

        # Link settings: hidden until "⚙ Link" (opens itself on a port error)
        self.arm_link_settings = QWidget()
        ls = QHBoxLayout(self.arm_link_settings)
        ls.setContentsMargins(0, 0, 0, 0)
        ls.setSpacing(6)
        ls.addWidget(QLabel("Port"))
        self.combo_arm_port = QComboBox()
        self.combo_arm_port.setEditable(True)
        self.combo_arm_port.setMinimumWidth(150)
        self.combo_arm_port.setToolTip("Serial port servo_serial_commander opens (pick one or type a path)")
        btn_rescan = QPushButton("⟳")
        btn_rescan.setToolTip("Rescan serial ports")
        btn_rescan.setFixedWidth(32)
        btn_rescan.setStyleSheet(button_qss('neutral', padding='3px 6px'))
        btn_rescan.clicked.connect(lambda: self.refresh_arm_port_list(report=True))
        ls.addWidget(self.combo_arm_port)
        ls.addWidget(btn_rescan)
        ls.addWidget(QLabel("Baud"))
        self.combo_arm_baud = QComboBox()
        self.combo_arm_baud.addItems(["115200", "57600", "230400", "9600"])
        ls.addWidget(self.combo_arm_baud)
        btn_apply_serial = QPushButton("Use port")
        btn_apply_serial.setToolTip("Tell the commander to reopen on this port / baud rate")
        btn_apply_serial.setStyleSheet(button_qss('primary', padding='4px 10px'))
        btn_apply_serial.clicked.connect(self.apply_arm_serial_settings)
        ls.addWidget(btn_apply_serial)
        ls.addSpacing(10)
        ls.addWidget(QLabel("Wi-Fi"))
        self.txt_arm_ip = QLineEdit()
        self.txt_arm_ip.setPlaceholderText("robot-arm.local or 192.168.1.50")
        self.txt_arm_ip.setMinimumWidth(130)
        self.txt_arm_ip.returnPressed.connect(self.apply_arm_ip)
        ls.addWidget(self.txt_arm_ip, 1)
        btn_apply_ip = QPushButton("Use IP")
        btn_apply_ip.setToolTip("Tell the commander which address to send UDP commands to")
        btn_apply_ip.setStyleSheet(button_qss('primary', padding='4px 10px'))
        btn_apply_ip.clicked.connect(self.apply_arm_ip)
        ls.addWidget(btn_apply_ip)
        self.arm_link_settings.hide()
        self.btn_arm_link_settings.toggled.connect(self.arm_link_settings.setVisible)
        console_layout.addWidget(self.arm_link_settings)

        self.arm_console = ConsoleWidget(
            "arm",
            input_placeholder="Command for the arm ESP32: PING, MODE:SERIAL, CMD:turntable_link_joint=90 …  (↑/↓ history)",
            quick_commands=[("PING", "PING", "Ask the ESP32 to reply (measures the round trip)"),
                            ("Status", "LINK:STATUS", "Ask the commander to publish its link state now")])
        self.arm_console.submitted.connect(self.on_arm_console_submit)
        self.chk_arm_verbose = QCheckBox("Joint traffic")
        self.chk_arm_verbose.setToolTip("Also show every joint command and ACK (busy while sliders move)")
        self.chk_arm_verbose.toggled.connect(lambda on: self.send_link_command(f"VERBOSE={1 if on else 0}"))
        self.arm_console.add_toolbar_widget(self.chk_arm_verbose)
        console_layout.addWidget(self.arm_console, 1)

        console_group.setLayout(console_layout)
        console_group.setMinimumHeight(230)

        # Joints and console share a draggable divider (the console used to
        # take a third of the height and leave room for only ~3 joints).
        self.arm_splitter = QSplitter(Qt.Vertical)
        self.arm_splitter.addWidget(scroll)
        self.arm_splitter.addWidget(console_group)
        self.arm_splitter.setChildrenCollapsible(False)
        self.arm_splitter.setStretchFactor(0, 3)
        self.arm_splitter.setStretchFactor(1, 1)
        self.arm_splitter.setSizes([320, 300])
        control_layout.addWidget(self.arm_splitter, 1)

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

        self.tabs.addTab(control_widget, "🦾 Robot Arm")
        self.tabs.addTab(self.quard_bot_widget, "🤖 Quard Bot")
        self.tabs.addTab(self.sensor_node_widget, "🚀 Rover")

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

        # Set Splitter ratios (left control tabs, right 3D viewport)
        main_splitter.setSizes([860, 540])
        main_splitter.setChildrenCollapsible(False)
        self.main_splitter = main_splitter

        # ── Header: title, ROS status, E-STOP ─────────────────────────────
        header = QWidget()
        header.setObjectName("appHeader")
        header.setAttribute(Qt.WA_StyledBackground, True)
        header.setStyleSheet(f"QWidget#appHeader {{ background-color: {THEME['sunken']}; border-bottom: 1px solid {THEME['border']}; }}")
        header_layout = QHBoxLayout(header)
        header_layout.setContentsMargins(12, 6, 12, 6)
        lbl_brand = QLabel("🦾 Robotics Control Center")
        lbl_brand.setStyleSheet(f"color: {THEME['text']}; font-size: 15px; font-weight: bold;")
        header_layout.addWidget(lbl_brand)
        lbl_ros = QLabel("ROS 2 Jazzy · publishing /joint_states" if ROS2_AVAILABLE else "⚠️ ROS 2 not found: UI only, nothing reaches the robots")
        lbl_ros.setStyleSheet(
            "background-color: #0c4a6e; color: #7dd3fc; padding: 3px 10px; border-radius: 10px; font-weight: bold; font-size: 11px;"
            if ROS2_AVAILABLE else
            "background-color: #78350f; color: #fde68a; padding: 3px 10px; border-radius: 10px; font-weight: bold; font-size: 11px;")
        header_layout.addWidget(lbl_ros)
        header_layout.addStretch(1)
        lbl_keys = QLabel("Esc = E-STOP · Ctrl+S = save")
        lbl_keys.setStyleSheet(f"color: {THEME['muted']}; font-size: 11px;")
        header_layout.addWidget(lbl_keys)
        self.btn_estop = QPushButton("⛔ E-STOP")
        self.btn_estop.setToolTip("Stop all motion and lock the motion controls (Esc). Software stop only: it does not cut servo power.")
        self.btn_estop.setCursor(Qt.PointingHandCursor)
        self.btn_estop.setStyleSheet(button_qss('danger', padding='7px 18px') + " QPushButton { font-size: 13px; }")
        self.btn_estop.clicked.connect(lambda: self.release_estop() if self.estop_engaged else self.engage_estop())
        header_layout.addWidget(self.btn_estop)

        self.lbl_estop_banner = QLabel("⛔  E-STOP ENGAGED — sim, homing and gesture control stopped, Quard Bot told to stand, controls locked. Press Release to continue.")
        self.lbl_estop_banner.setAlignment(Qt.AlignCenter)
        self.lbl_estop_banner.setStyleSheet("background-color: #b91c1c; color: #ffffff; font-weight: bold; padding: 6px;")
        self.lbl_estop_banner.hide()

        central = QWidget()
        central_layout = QVBoxLayout(central)
        central_layout.setContentsMargins(0, 0, 0, 0)
        central_layout.setSpacing(0)
        central_layout.addWidget(header)
        central_layout.addWidget(self.lbl_estop_banner)
        central_layout.addWidget(main_splitter, 1)
        self.setCentralWidget(central)

        self.statusBar().showMessage("Ready", 3000)
        QShortcut(QKeySequence("Ctrl+S"), self, activated=self.save_calibration)
        esc = QShortcut(QKeySequence(Qt.Key_Escape), self, activated=self.engage_estop)
        esc.setContext(Qt.ApplicationShortcut)

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

        self.restore_ui_state()
        self.refresh_arm_port_list()
        self.arm_link_timer = QTimer(self)
        self.arm_link_timer.timeout.connect(self.refresh_arm_link)
        self.arm_link_timer.start(1000)
        self.refresh_arm_link()
        self.log_to_console("Dashboard started" + ("" if ROS2_AVAILABLE else " without ROS 2: nothing will reach the robots"),
                            'info' if ROS2_AVAILABLE else 'warn')

    def update_connection_status(self, status_str):
        """Legacy /esp32_connection_status. Only drives the card when no
        /esp32_link_state arrives (e.g. unified_robot_commander)."""
        if status_str.startswith("STATUS:CONNECTED:"):
            self.arm_legacy = (True, status_str[len("STATUS:CONNECTED:"):].strip(), time.time())
        elif status_str.startswith("STATUS:DISCONNECTED:"):
            self.arm_legacy = (False, status_str[len("STATUS:DISCONNECTED:"):].strip(), time.time())
        else:
            return
        if not self.commander_alive():
            if self.last_conn_state != self.arm_legacy[0]:
                self.last_conn_state = self.arm_legacy[0]
                self.arm_console.log(self.arm_legacy[1], 'ok' if self.arm_legacy[0] else 'error')
            self.refresh_arm_link()

    def log_to_console(self, text, kind=None):
        """Add a line to the arm console (kind inferred from a leading emoji)."""
        if not hasattr(self, 'arm_console'):
            print(text)
            return
        self.arm_console.log(text, kind or infer_console_kind(text))

    # ── Arm link (servo_serial_commander) ─────────────────────────────────
    def commander_alive(self):
        return self.arm_link is not None and time.time() - self.arm_link_rx_t < 3.5

    def on_arm_link_state(self, data):
        """New /esp32_link_state message (JSON already decoded)."""
        if not isinstance(data, dict):
            return
        if self.arm_link_last_state == 'missing':
            self.arm_console.log(f"servo_serial_commander is running ({data.get('target', '?')})", 'ok')
        self.arm_link = data
        self.arm_link_rx_t = time.time()
        self.refresh_arm_link()

    def on_arm_console_line(self, text):
        """New /esp32_console line: RX:/TX:/SYS:/ERR:<text>."""
        kinds = {'RX': 'rx', 'TX': 'tx', 'SYS': 'info', 'ERR': 'error'}
        prefix, sep, rest = text.partition(':')
        if sep and prefix in kinds:
            self.arm_console.log(rest, kinds[prefix])
        else:
            self.arm_console.log(text, 'rx')

    @staticmethod
    def _age_text(age):
        if age is None:
            return "never"
        return f"{age:.1f} s ago" if age < 60 else f"{age / 60:.0f} min ago"

    def refresh_arm_link(self):
        """Redraw the arm link card (also runs once a second so ages stay current)."""
        if not hasattr(self, 'arm_link_pill'):
            return
        now = time.time()
        mismatch_text = None
        if not ROS2_AVAILABLE:
            kind, label = 'error', "ROS 2 not available"
            detail = "Nothing reaches the arm. Start the dashboard with ./launch_dashboard.sh so ROS 2 is sourced."
        elif self.commander_alive():
            d = self.arm_link
            state = d.get('state')
            serial_link = d.get('transport') != 'wifi'
            parts = [d.get('target', '?')]
            if state == 'online':
                kind, label = 'ok', "Online"
                if d.get('rtt_ms') is not None:
                    parts.append(f"ping {d['rtt_ms']:.0f} ms")
                parts.append(f"last reply {self._age_text(d.get('last_rx_age'))}")
            elif state == 'mismatch':
                kind, label = 'warn', "Mode mismatch"
                listen = 'Wi-Fi' if d.get('mismatch_mode') == 'WIFI' else 'serial'
                ours = 'serial' if serial_link else 'Wi-Fi'
                mismatch_text = (f"The ESP32 replies, but it is listening on {listen}, so commands "
                                 f"sent over {ours} are ignored.")
                self._arm_fix_mode = 'SERIAL' if serial_link else 'WIFI'
                parts.append(f"last reply {self._age_text(d.get('last_rx_age'))}")
            elif state == 'port_error':
                kind, label = 'error', "Port error" if serial_link else "Socket error"
                parts.append(d.get('last_error') or "cannot open the link")
            else:
                kind, label = 'warn', "No reply"
                parts.append(f"last reply {self._age_text(d.get('last_rx_age'))}")
                parts.append("is the ESP32 powered and on this link?")
            parts.append(f"RX {d.get('rx', 0)} · TX {d.get('tx', 0)}")
            if d.get('esp32_ip'):
                parts.append(f"ESP32 IP {d['esp32_ip']}")
            detail = " · ".join(str(x) for x in parts)
            self.sync_comm_button(d.get('transport'))
            self._fill_arm_settings_from_state(d)
            if state == 'port_error' and self.arm_link_last_state != 'port_error':
                self.btn_arm_link_settings.setChecked(True)   # the fix is usually another port
            self.arm_link_last_state = state
        elif self.arm_legacy and now - self.arm_legacy[2] < 5.0:
            ok, text, _ = self.arm_legacy
            kind, label = ('ok', "Online") if ok else ('warn', "Disconnected")
            detail = f"{text} (basic status from an older commander)"
        elif self.arm_link is None and now - self._gui_started < 4.0:
            kind, label = 'busy', "Waiting for commander…"
            detail = "Looking for servo_serial_commander on /esp32_link_state"
        else:
            kind, label = 'error', "Commander not running"
            since = f" (last heard {now - self.arm_link_rx_t:.0f} s ago)" if self.arm_link_rx_t else ""
            detail = (f"No link state from servo_serial_commander{since}. "
                      "Start it: ros2 run servo_joint_publisher servo_serial_commander")
            if self.arm_link_last_state not in ('missing', None):
                self.arm_console.log("Lost contact with servo_serial_commander", 'error')
            self.arm_link_last_state = 'missing'
        self.arm_link_pill.set_state(kind, label)
        self.lbl_arm_link_detail.setText(detail)
        self.lbl_arm_link_detail.setToolTip(detail)
        self.arm_mismatch_bar.setVisible(mismatch_text is not None)
        if mismatch_text:
            self.lbl_arm_mismatch.setText("⚠ " + mismatch_text)
            self.btn_arm_fix_mode.setText(f"Switch ESP32 to {'Wi-Fi' if self._arm_fix_mode == 'WIFI' else 'serial'}")
        self.btn_arm_reconnect.setEnabled(ROS2_AVAILABLE and self.commander_alive())

    def sync_comm_button(self, transport):
        """Keep the Comm toggle showing the link the commander really uses."""
        if transport not in ('serial', 'wifi') or time.time() - self._comm_mode_set_at < 2.5:
            return
        serial_link = transport == 'serial'
        if self.btn_comm_mode.isChecked() != serial_link:
            self.btn_comm_mode.blockSignals(True)
            self.btn_comm_mode.setChecked(serial_link)
            self.btn_comm_mode.blockSignals(False)
        self.btn_comm_mode.setText("📟 Comm: Serial" if serial_link else "📡 Comm: WiFi")

    def _fill_arm_settings_from_state(self, d):
        key = (d.get('port'), d.get('baudrate'), d.get('ip'))
        if key == getattr(self, '_arm_settings_key', None):
            return
        self._arm_settings_key = key
        if d.get('port'):
            self.combo_arm_port.setEditText(d['port'])
        if d.get('baudrate'):
            baud = str(d['baudrate'])
            if self.combo_arm_baud.findText(baud) < 0:
                self.combo_arm_baud.addItem(baud)
            self.combo_arm_baud.setCurrentText(baud)
        if d.get('ip'):
            self.txt_arm_ip.setText(d['ip'])

    def refresh_arm_port_list(self, report=False):
        current = self.combo_arm_port.currentText()
        ports = list_serial_ports()
        self.combo_arm_port.blockSignals(True)
        self.combo_arm_port.clear()
        for dev, desc in ports:
            self.combo_arm_port.addItem(dev)
            if desc:
                self.combo_arm_port.setItemData(self.combo_arm_port.count() - 1, desc, Qt.ToolTipRole)
        self.combo_arm_port.setEditText(current or (ports[0][0] if ports else "/dev/ttyUSB0"))
        self.combo_arm_port.blockSignals(False)
        if report:
            found = ", ".join(f"{d} ({desc})" if desc else d for d, desc in ports) or "none"
            self.arm_console.log(f"Serial ports found: {found}", 'info')

    def send_link_command(self, cmd):
        """LINK:<cmd> to servo_serial_commander (it reports back on the console)."""
        if not ROS2_AVAILABLE:
            self.arm_console.log(f"Not sent (ROS 2 not available): LINK:{cmd}", 'error')
            return
        self.ros_node.publish_calibration(f"LINK:{cmd}")
        if not self.commander_alive():
            self.arm_console.log(f"LINK:{cmd} sent, but servo_serial_commander isn't running", 'warn')

    def apply_arm_serial_settings(self):
        port = self.combo_arm_port.currentText().strip()
        baud = self.combo_arm_baud.currentText().strip()
        d = self.arm_link or {}
        rover = getattr(self, 'sensor_node_widget', None)
        if rover is not None and rover.source == 'serial' and rover.active_port == port:
            self.arm_console.log(f"{port} is open in the Rover tab; disconnect it there first", 'error')
            return
        changed = False
        if baud and str(d.get('baudrate')) != baud:
            self.send_link_command(f"BAUD={baud}")
            changed = True
        if port and d.get('port') != port:
            self.send_link_command(f"PORT={port}")
            changed = True
        if not changed:
            self.send_link_command("RECONNECT")
        if d.get('transport') == 'wifi':
            self.arm_console.log("Port saved. The commander is on Wi-Fi: switch Comm to Serial to use it.", 'info')

    def apply_arm_ip(self):
        host = self.txt_arm_ip.text().strip()
        if not host:
            return
        self.send_link_command(f"IP={host}")
        if (self.arm_link or {}).get('transport') == 'serial':
            self.arm_console.log("Address saved. The commander is on serial: switch Comm to Wi-Fi to use it.", 'info')

    def fix_arm_mode_mismatch(self):
        mode = self._arm_fix_mode
        self.ros_node.publish_calibration(f"MODE:{mode}")
        self.arm_console.log(f"Asked the ESP32 to listen on {'Wi-Fi' if mode == 'WIFI' else 'serial'}", 'info')

    def on_arm_console_submit(self, text):
        """Console input → /servo_calibration (unknown text is sent as RAW:)."""
        t = text.strip()
        if not t:
            return
        upper = t.upper()
        known = ('MODE:', 'CMD:', 'CALIB:', 'SYNC:', 'LINK:', 'RAW:')
        motion = upper.startswith('CMD:') or upper.startswith('RAW:CMD:')
        if motion and self.estop_engaged:
            self.arm_console.log(f"Blocked by E-STOP: {t}", 'warn')
            return
        if not ROS2_AVAILABLE:
            self.arm_console.log(f"Not sent (ROS 2 not available): {t}", 'error')
            return
        prefix = next((k for k in known if upper.startswith(k)), None)
        payload = (prefix + t[len(prefix):]) if prefix else "RAW:" + t
        self.ros_node.publish_calibration(payload)
        if not self.commander_alive():
            self.arm_console.log(f"{t}: published, but servo_serial_commander isn't running", 'warn')
        if motion and not self._raw_cmd_hint_shown:
            self._raw_cmd_hint_shown = True
            self.arm_console.log("Joint commands typed here are clamped to the joint limits; the next slider move overrides them.", 'info')

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
        elif index == 1:
            self.lbl_rviz_target.setText(f"{prefix}🤖 Quard")
            self.lbl_rviz_target.setStyleSheet("background-color: #10b981; color: #ffffff; padding: 2px 8px; border-radius: 10px;")
            if hasattr(self, 'quard_bot_widget'):
                if hasattr(self.quard_bot_widget, 'ping_quard_bot'):
                    self.quard_bot_widget.ping_quard_bot()
                elif hasattr(self.quard_bot_widget, 'ping_robot'):
                    self.quard_bot_widget.ping_robot()
        elif index == 2:
            self.lbl_rviz_target.setText(f"{prefix}🚀 Rover")
            self.lbl_rviz_target.setStyleSheet("background-color: #f59e0b; color: #ffffff; padding: 2px 8px; border-radius: 10px;")

        # Ensure embedded RViz window is properly resized
        QTimer.singleShot(50, self._resize_embedded_rviz)
        QTimer.singleShot(250, self._resize_embedded_rviz)

    def init_rviz_embed(self):
        """Spawn RViz2 as child process or embed WebGL twin if ROS 2 is not present."""
        import sys as sys
        rviz_bin = shutil.which("rviz2") or ("/opt/ros/jazzy/bin/rviz2" if os.path.exists("/opt/ros/jazzy/bin/rviz2") else None)
        if not rviz_bin:
            print("[Dashboard] ROS 2 / RViz2 not found on system. Loading WebGL 3D Twin...", file=sys.stderr, flush=True)
            self._load_webgl_twin()
            return

        print(f"[Dashboard] init_rviz_embed: Starting RViz2 process ({rviz_bin})...", file=sys.stderr, flush=True)

        target_robot = os.environ.get("ROBOT_TARGET", "arm").lower()
        if target_robot == "quard":
            default_rviz = os.path.join(PROJECT_DIR, "urdf", "quard_bot", "rviz", "quard_bot.rviz")
        elif target_robot == "dual":
            default_rviz = os.path.join(PROJECT_DIR, "urdf", "dual_robots.rviz")
        else:
            default_rviz = os.path.join(PROJECT_DIR, "urdf", "arm", "rviz", "arm.rviz")
        rviz_config = os.environ.get("RVIZ_CONFIG", default_rviz)
        self.rviz_process = QProcess(self)

        env = QProcessEnvironment.systemEnvironment()
        env.insert("QT_QPA_PLATFORM", "xcb")
        env.insert("GDK_BACKEND", "x11")
        if os.path.exists("/opt/ros/jazzy"):
            env.insert("AMENT_PREFIX_PATH", os.environ.get("AMENT_PREFIX_PATH", "/opt/ros/jazzy"))
            env.insert("PATH", os.environ.get("PATH", f"/opt/ros/jazzy/bin:{os.environ.get('PATH', '')}"))
            env.insert("PYTHONPATH", os.environ.get("PYTHONPATH", "/opt/ros/jazzy/lib/python3.12/site-packages"))
            ros_ld = "/opt/ros/jazzy/opt/rviz_ogre_vendor/lib:/opt/ros/jazzy/lib/x86_64-linux-gnu:/opt/ros/jazzy/lib"
            curr_ld = os.environ.get("LD_LIBRARY_PATH", "")
            env.insert("LD_LIBRARY_PATH", f"{ros_ld}:{curr_ld}")
        self.rviz_process.setProcessEnvironment(env)

        self.rviz_process.start(rviz_bin, ["-d", rviz_config])

        self.already_docked = False
        self._embed_attempt = 0
        # Schedule multiple retry attempts with increasing delays
        for delay_ms in [3000, 5000, 7000, 10000, 14000, 18000, 23000, 28000]:
            QTimer.singleShot(delay_ms, self.embed_rviz_window)
        print(f"[Dashboard] init_rviz_embed: Scheduled 8 embed attempts", file=sys.stderr, flush=True)

    def _load_webgl_twin(self):
        """Embeds the WebGL 3D Robot Digital Twin into the native application frame."""
        import urllib.request
        server_running = False
        try:
            with urllib.request.urlopen("http://localhost:8000/api/state", timeout=0.5) as resp:
                if resp.status == 200:
                    server_running = True
        except Exception:
            pass

        if not server_running:
            server_script = os.path.join(os.path.dirname(os.path.abspath(__file__)), "web_dashboard", "server.py")
            if os.path.exists(server_script):
                subprocess.Popen([sys.executable, server_script, "--port", "8000"],
                                 stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
                time.sleep(1.2)

        if WEBENGINE_AVAILABLE:
            try:
                self.web_view = QWebEngineView(self.rviz_frame)
                self.web_view.load(QUrl("http://localhost:8000"))
                self.rviz_frame_layout.addWidget(self.web_view)
                self.rviz_header_label.setText("🌐 Integrated 3D Digital Twin (WebGL)")
                self.rviz_header_label.setStyleSheet("color: #38bdf8;")
                self.already_docked = True
                print("[Dashboard] Successfully docked WebGL 3D digital twin via QtWebEngine.", file=sys.stderr, flush=True)
                return
            except Exception as e:
                print(f"[Dashboard] QWebEngineView error: {e}", file=sys.stderr, flush=True)

        lbl = QLabel("3D Viewport Active at: http://localhost:8000")
        lbl.setAlignment(Qt.AlignCenter)
        lbl.setStyleSheet("color: #38bdf8; font-size: 14px; font-weight: bold;")
        self.rviz_frame_layout.addWidget(lbl)

    def _find_rviz_x11_window(self):
        """Find the main RViz2 X11 window ID using PID and WM_CLASS validation."""
        import sys as sys
        try:
            main_win_id = str(int(self.winId()))
        except Exception:
            return None

        rviz_pid = None
        if self.rviz_process and self.rviz_process.state() != QProcess.NotRunning:
            rviz_pid = self.rviz_process.processId()

        candidate_ids = []
        # Search by RViz PID first (most specific and safe)
        if rviz_pid:
            try:
                out = subprocess.check_output(
                    ["xdotool", "search", "--pid", str(rviz_pid)],
                    stderr=subprocess.DEVNULL, timeout=2
                ).decode().strip()
                if out:
                    candidate_ids.extend([i.strip() for i in out.split('\n') if i.strip()])
            except Exception:
                pass

        # If not found by PID, search strictly by rviz2 WM_CLASS
        if not candidate_ids:
            for cls_name in ["rviz2", "rviz"]:
                try:
                    out = subprocess.check_output(
                        ["xdotool", "search", "--class", cls_name],
                        stderr=subprocess.DEVNULL, timeout=2
                    ).decode().strip()
                    if out:
                        candidate_ids.extend([i.strip() for i in out.split('\n') if i.strip()])
                        break
                except Exception:
                    pass

        if not candidate_ids:
            return None

        for wid in reversed(candidate_ids):
            if wid == main_win_id:
                continue
            try:
                # Strictly verify WM_CLASS belongs to RViz
                cls_prop = subprocess.check_output(
                    ["xprop", "-id", wid, "WM_CLASS"],
                    stderr=subprocess.DEVNULL, timeout=2
                ).decode().lower()

                # Guard: NEVER touch Antigravity or other IDE/browser windows
                if any(x in cls_prop for x in ["antigravity", "code", "chrome", "firefox", "terminal"]):
                    continue
                if "rviz" not in cls_prop:
                    continue

                # Check if this is the main RViz window with a title
                name_prop = subprocess.check_output(
                    ["xprop", "-id", wid, "WM_NAME"],
                    stderr=subprocess.DEVNULL, timeout=2
                ).decode()

                if "RViz" in name_prop or "rviz" in name_prop.lower():
                    print(f"[Dashboard] _find_rviz: Found verified RViz window {wid}: {name_prop.strip()}", file=sys.stderr, flush=True)
                    return int(wid)
            except Exception:
                continue

        return None

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

            rviz_win_id = self._find_rviz_x11_window()

            if rviz_win_id is not None:
                try:
                    # Strip window decorations & set window type to utility so compositor allows embedding
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

                    # Perform native X11 reparent into container widget
                    subprocess.call(["xdotool", "windowreparent", str(rviz_win_id), container_win_id], stderr=subprocess.DEVNULL, timeout=2)
                    subprocess.call(["xdotool", "windowmap", str(rviz_win_id)], stderr=subprocess.DEVNULL, timeout=2)

                    # Wrap with Qt window container for layout management
                    qwin = QWindow.fromWinId(rviz_win_id)
                    qwin.setFlags(Qt.SubWindow | Qt.FramelessWindowHint)

                    self.embedded_rviz_widget = QWidget.createWindowContainer(qwin, self.rviz_frame)
                    self.rviz_frame_layout.addWidget(self.embedded_rviz_widget)

                    self.already_docked = True
                    self.log_to_console("✅ Integrated RViz 3D Viewport docked successfully.")

                    # Sync size & position
                    QTimer.singleShot(100, self._resize_embedded_rviz)
                    QTimer.singleShot(500, self._resize_embedded_rviz)
                    QTimer.singleShot(1200, self._resize_embedded_rviz)
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
        """Safe placeholder for viewport adjustments without touching unrelated desktop windows."""
        pass


    def closeEvent(self, event):
        """Cleanly terminate child processes when dashboard window is closed."""
        if self.rviz_process and self.rviz_process.state() != QProcess.NotRunning:
            self.rviz_process.terminate()
            self.rviz_process.waitForFinished(1000)
        event.accept()

    def keyPressEvent(self, event):
        """Capture keyboard events for Quard Bot locomotion when Quard Bot tab is active."""
        if getattr(self, 'estop_engaged', False):
            super().keyPressEvent(event)
            return
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
            # The twin shows (slider + sync); make it show the home pose now.
            offset = round(home_deg - curr_deg, 1)
            w.spin_sync.setValue(offset)  # valueChanged -> on_sync_offset_changed
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
            self.statusBar().showMessage(f"💾 Saved limits, home pose and sync offsets to {CALIB_FILE_PATH}", 6000)
        except Exception as e:
            QMessageBox.critical(self, "Error", f"Failed to save calibration:\n{e}")

    def toggle_comm_mode(self, checked):
        mode_str = "SERIAL" if checked else "WIFI"
        if checked:
            self.btn_comm_mode.setText("📟 Comm: Serial")
        else:
            self.btn_comm_mode.setText("📡 Comm: WiFi")

        cmd = f"MODE:{mode_str}"
        self._comm_mode_set_at = time.time()
        self.ros_node.publish_calibration(cmd)
        if not self.commander_alive():
            self.log_to_console(f"{cmd} published, but servo_serial_commander isn't running", 'warn')

    def update_sliders_from_ros(self, msg: JointState):
        """Update GUI joint sliders visually when gesture teleop is active."""
        if not self.is_gesture_mode:
            return
        for j_name, w in self.joint_widgets.items():
            if j_name in msg.name:
                idx = msg.name.index(j_name)
                if idx < len(msg.position):
                    rad_val = msg.position[idx]
                    if not math.isfinite(rad_val):
                        continue
                    r_min, r_max = w.rad_min, w.rad_max
                    ratio = (rad_val - r_min) / (r_max - r_min) if r_max != r_min else 0.5
                    # Published angles are twin poses (slider + sync), the inverse of
                    # get_current_rad(); drop the offset and respect the slider limits
                    # so leaving gesture mode does not make the arm jump.
                    deg = ratio * 180.0 - w.sync_offset_deg
                    deg = max(w.spin_min.value(), min(w.spin_max.value(), deg))
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

            if self.gesture_process is None:
                self.gesture_process = QProcess(self)
            self.gesture_process.start("ros2", ["run", "servo_joint_publisher", "gesture_teleop_node"])
        else:
            self.btn_gesture.setText("✋ Gesture Mode")
            self.log_to_console("🕹️ [MODE SWITCH] Returned to Manual Slider Control Mode.")

            if self.gesture_process and self.gesture_process.state() != QProcess.NotRunning:
                self.gesture_process.kill()
                self.gesture_process.waitForFinished(1000)

    def engage_estop(self):
        """Software E-STOP: stop every motion source and lock the motion controls.

        Joint states keep publishing, so the arm holds its current pose. This
        does not cut servo power; that needs a hardware switch.
        """
        if self.estop_engaged:
            return
        self.estop_engaged = True

        if self.is_simulating:
            self.btn_sim.setChecked(False)
            self.toggle_simulation(False)
        if self.is_homing:
            self.is_homing = False
        if self.is_gesture_mode:
            self.btn_gesture.setChecked(False)
            self.toggle_gesture_mode(False)

        q = getattr(self, 'quard_bot_widget', None)
        if q:
            for t in ('gait_timer', 'wave_timer', 'smooth_timer'):
                timer = getattr(q, t, None)
                if timer is not None and timer.isActive():
                    timer.stop()
            q.active_gait = None
            q.send_command({'mode': 'stand'})  # now, not after the stand animation
            q.stand_pose()
            q.set_motion_enabled(False)   # connection + console stay usable

        for w in self.joint_widgets.values():
            w.setEnabled(False)
        for b in self.motion_buttons:
            b.setEnabled(False)

        self.lbl_estop_banner.show()
        self.btn_estop.setText("✅ Release E-STOP")
        self.btn_estop.setStyleSheet(button_qss('neutral', padding='7px 18px') + " QPushButton { font-size: 13px; color: #fca5a5; border-color: #dc2626; }")
        self.log_to_console("⛔ E-STOP engaged: all motion stopped, controls locked.")
        self.statusBar().showMessage("⛔ E-STOP engaged", 0)

    def release_estop(self):
        if not self.estop_engaged:
            return
        self.estop_engaged = False
        for w in self.joint_widgets.values():
            w.setEnabled(True)
        for b in self.motion_buttons:
            b.setEnabled(True)
        if getattr(self, 'quard_bot_widget', None):
            self.quard_bot_widget.set_motion_enabled(True)
        self.lbl_estop_banner.hide()
        self.btn_estop.setText("⛔ E-STOP")
        self.btn_estop.setStyleSheet(button_qss('danger', padding='7px 18px') + " QPushButton { font-size: 13px; }")
        self.log_to_console("✅ E-STOP released: controls unlocked.")
        self.statusBar().showMessage("E-STOP released", 4000)

    def restore_ui_state(self):
        """Window size, splitter positions and (unless ROBOT_TARGET picks one) the last tab."""
        geom = self.settings.value("geometry")
        if geom is not None:
            self.restoreGeometry(geom)
        for key, splitter in (("main_splitter", self.main_splitter), ("arm_splitter", self.arm_splitter)):
            state = self.settings.value(key)
            if state is not None:
                splitter.restoreState(state)
        if "ROBOT_TARGET" not in os.environ:
            try:
                idx = int(self.settings.value("tab", 0))
            except (TypeError, ValueError):
                idx = 0
            if 0 <= idx < self.tabs.count():
                self.tabs.setCurrentIndex(idx)

    def save_ui_state(self):
        self.settings.setValue("geometry", self.saveGeometry())
        self.settings.setValue("main_splitter", self.main_splitter.saveState())
        self.settings.setValue("arm_splitter", self.arm_splitter.saveState())
        self.settings.setValue("tab", self.tabs.currentIndex())

    def closeEvent(self, event):
        """Clean up gesture teleop and child processes on dashboard window exit."""
        try:
            self.save_ui_state()
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

        try:
            self.ros_node.joint_pub.publish(msg)
        except Exception:
            pass


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
            # Structured link state + ESP32 console from servo_serial_commander
            self.link_sub = self.create_subscription(
                StringMsg, '/esp32_link_state', self.link_state_callback, LINK_STATE_QOS)
            self.console_sub = self.create_subscription(
                StringMsg, '/esp32_console', self.console_callback, CONSOLE_QOS)
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

        def link_state_callback(self, msg: StringMsg):
            if not self.gui_ref:
                return
            try:
                data = json.loads(msg.data)
            except ValueError:
                return
            self.gui_ref.on_arm_link_state(data)

        def console_callback(self, msg: StringMsg):
            if self.gui_ref:
                self.gui_ref.on_arm_console_line(msg.data)

        def publish_calibration(self, calib_str):
            msg = StringMsg()
            msg.data = calib_str
            self.calib_pub.publish(msg)
            self.get_logger().info(f"Broadcasted Calibration: {calib_str}")
else:
    class DummyClock:
        def now(self):
            return self
        def to_msg(self):
            return self

    class DummyPub:
        def publish(self, msg):
            pass

    class DummyLogger:
        def info(self, msg):
            print(f"[GUI Info] {msg}")
        def warn(self, msg):
            print(f"[GUI Warn] {msg}")
        def error(self, msg):
            print(f"[GUI Error] {msg}")

    class CalibratedGUINode:
        def __init__(self):
            self.gui_ref = None
            self._clock = DummyClock()
            self._logger = DummyLogger()
            self.joint_pub = DummyPub()
            self.calib_pub = DummyPub()
        def get_clock(self):
            return self._clock
        def get_logger(self):
            return self._logger
        def publish_calibration(self, calib_str):
            pass
        def destroy_node(self):
            pass


def main(args=None):
    if "QT_QPA_PLATFORM_PLUGIN_PATH" in os.environ and "cv2" in os.environ["QT_QPA_PLATFORM_PLUGIN_PATH"]:
        del os.environ["QT_QPA_PLATFORM_PLUGIN_PATH"]
    if ROS2_AVAILABLE:
        rclpy.init(args=args)
    ros_node = CalibratedGUINode()

    app = QApplication(sys.argv)
    apply_app_theme(app)
    gui = CalibratedJointPublisherGUI(ros_node)

    if '--sim' in sys.argv or '-s' in sys.argv:
        gui.btn_sim.setChecked(True)
        gui.toggle_simulation(True)

    gui.show()
    gui.publish_joint_states()

    def _on_tick():
        if ROS2_AVAILABLE and rclpy.ok():
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
    if ROS2_AVAILABLE:
        rclpy.shutdown()
    sys.exit(exit_code)


if __name__ == '__main__':
    main()
