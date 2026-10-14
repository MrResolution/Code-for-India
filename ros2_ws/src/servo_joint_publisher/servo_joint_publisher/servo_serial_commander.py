#!/usr/bin/env python3
"""
ROS 2 Node: servo_serial_commander (5 Physical Servos Architecture)
-------------------------------------------------------------------
Subscribes to /joint_states from calibrated_joint_gui, maps the 4 URDF arm joints
to 5 physical servos on ESP32, applies live joint calibration (min/max limits, trim),
and streams commands wirelessly over Wi-Fi UDP to the ESP32.

Physical Servo to Joint Pin Mapping:
  1. turntable_link_joint_dup   → ESP32 GPIO 18 (Base Turntable Yaw)
  2. turntable_link_joint       → ESP32 GPIO 19 (Shoulder Pitch Primary)
     └─ Mirrored Slave Servo    → ESP32 GPIO 21 (Shoulder Pitch Opposing Slave)
  3. turntable_link_joint_dup_1 → ESP32 GPIO 22 (Elbow 1 Pitch)
  4. turntable_link_joint_dup_2 → ESP32 GPIO 23 (Elbow 2 Pitch)

Link monitoring (consumed by calibrated_joint_gui and the web dashboard):
  /esp32_link_state       std_msgs/String, JSON, 1 Hz + on change; reliable,
                          transient local (late subscribers get the last state).
                          Consumers must still treat it as stale after ~3 s.
  /esp32_console          std_msgs/String, "RX:<line>", "TX:<line>", "SYS:<text>",
                          "ERR:<text>"; reliable, volatile (no replay). Repeated
                          identical RX lines are throttled.
  /esp32_connection_status  legacy "STATUS:CONNECTED:..." strings (kept).

Extra commands accepted on /servo_calibration:
  RAW:<line>              send <line> to the ESP32 as is (CMD:<joint>=<deg> is
                          clamped to that joint's limits first)
  LINK:RECONNECT | LINK:STATUS | LINK:VERBOSE=0/1
  LINK:PORT=/dev/ttyACM0 | LINK:BAUD=115200 | LINK:IP=192.168.1.50
"""

import json
import math
import os
import re
import socket
import threading
import time
import serial
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import String as StringMsg

LINK_STATE_QOS = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                            durability=DurabilityPolicy.TRANSIENT_LOCAL,
                            history=HistoryPolicy.KEEP_LAST)
# Volatile: console lines carry no timestamp, so replaying old ones to a late
# subscriber would show stale events as current.
CONSOLE_QOS = QoSProfile(depth=50, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.VOLATILE,
                         history=HistoryPolicy.KEEP_LAST)

# Commanded link must reply within this long to count as connected. The
# firmware answers every PING (1 Hz) and prints a heartbeat every 2 s when idle.
LINK_TIMEOUT_S = 4.0
ALLOWED_BAUDS = (9600, 19200, 38400, 57600, 115200, 230400, 460800, 921600)
_PORT_RE = re.compile(r'^(/dev/[A-Za-z0-9_./+-]+|COM\d+)$')
_HOST_RE = re.compile(r'^[A-Za-z0-9.-]{1,253}$')
_JOINT_CMD_RE = re.compile(r'^CMD:([A-Za-z0-9_]+)=(-?\d+(?:\.\d+)?)$')
# Firmware replies when a command arrives on the link it is not listening to.
_MISMATCH_ERRORS = {'ERR:WIFI_MODE_ACTIVE': 'WIFI', 'ERR:SERIAL_MODE_ACTIVE': 'SERIAL'}

def _find_calib_file():
    """servo_calibration.json at the repo root: $ROBOT_PROJECT_DIR, else the first
    parent of this file that has it (works from source and from ros2_ws/install)."""
    env_dir = os.environ.get("ROBOT_PROJECT_DIR")
    if env_dir:
        return os.path.join(env_dir, "servo_calibration.json")
    d = os.path.dirname(os.path.abspath(__file__))
    while True:
        candidate = os.path.join(d, "servo_calibration.json")
        if os.path.isfile(candidate):
            return candidate
        parent = os.path.dirname(d)
        if parent == d:
            return os.path.join(os.getcwd(), "servo_calibration.json")
        d = parent


CALIB_FILE_PATH = _find_calib_file()


class ServoSerialCommander(Node):

    def __init__(self):
        super().__init__('servo_serial_commander')

        # ── ROS 2 Node Parameters ─────────────────────────────────────────
        self.declare_parameter('use_wifi', False)
        # Attempt to dynamically resolve robot-arm.local, fallback to known IP
        resolved_ip = '10.35.234.59'
        try:
            resolved_ip = socket.gethostbyname('robot-arm.local')
        except Exception:
            pass
        self.declare_parameter('esp32_ip', resolved_ip)
        self.declare_parameter('udp_port', 8888)
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('max_cmd_rate', 50.0) # Hz

        self.use_wifi = self.get_parameter('use_wifi').get_parameter_value().bool_value
        self.esp32_ip = self.get_parameter('esp32_ip').get_parameter_value().string_value
        self.udp_port = self.get_parameter('udp_port').get_parameter_value().integer_value
        self.port = self.get_parameter('port').get_parameter_value().string_value
        self.baudrate = self.get_parameter('baudrate').get_parameter_value().integer_value
        max_cmd_rate = self.get_parameter('max_cmd_rate').get_parameter_value().double_value

        # Connection status tracking:
        self.is_connected = False
        self.last_rx_time = 0.0
        self.status_pub = self.create_publisher(StringMsg, '/esp32_connection_status', 10)
        self.link_state_pub = self.create_publisher(StringMsg, '/esp32_link_state', LINK_STATE_QOS)
        self.console_pub = self.create_publisher(StringMsg, '/esp32_console', CONSOLE_QOS)
        self.rx_count = 0
        self.tx_count = 0
        self.err_count = 0
        self.last_error = ""
        self.ping_sent_at = None      # monotonic time of the outstanding PING
        self.rtt_ms = None
        self.esp32_mode = None        # from HEARTBEAT / ACK:MODE=
        self.esp32_ip_reported = None  # ESP32's own IP, from HEARTBEAT
        self.mismatch_mode = None     # ESP32 listens on this link instead of ours
        self.mismatch_at = 0.0
        self.verbose = False          # also forward joint CMD/ACK traffic to /esp32_console
        self._rx_throttle = {}        # line -> [last_forward_time, suppressed_count]
        self._last_state_json = None

        # Per-joint calibration dictionary for the 4 URDF arm joints:
        self.joint_calib = {
            'turntable_link_joint_dup': {
                'rad_min': -3.0, 'rad_max': 3.0,
                'servo_min_deg': 0.0, 'servo_max_deg': 180.0,
                'trim_deg': 0.0, 'sync_offset_deg': 0.0, 'invert': False, 'pin': 18
            },
            'turntable_link_joint': {
                'rad_min': -2.0, 'rad_max': 2.0,
                'servo_min_deg': 0.0, 'servo_max_deg': 180.0,
                'trim_deg': 0.0, 'sync_offset_deg': 0.0, 'invert': False, 'pin': 19 # Drives GPIO 19 Primary & GPIO 21 Slave
            },
            'turntable_link_joint_dup_1': {
                'rad_min': -2.0, 'rad_max': 2.0,
                'servo_min_deg': 0.0, 'servo_max_deg': 180.0,
                'trim_deg': 0.0, 'sync_offset_deg': 0.0, 'invert': False, 'pin': 22
            },
            'turntable_link_joint_dup_2': {
                'rad_min': -2.0, 'rad_max': 2.0,
                'servo_min_deg': 0.0, 'servo_max_deg': 180.0,
                'trim_deg': 0.0, 'sync_offset_deg': 0.0, 'invert': False, 'pin': 23
            },
            'turntable_link_joint_dup_3': {
                'rad_min': -3.14159, 'rad_max': 3.14159,
                'servo_min_deg': 0.0, 'servo_max_deg': 180.0,
                'trim_deg': 0.0, 'sync_offset_deg': 0.0, 'invert': False, 'pin': 27
            },
            'wrist_twist_joint': {
                'rad_min': -3.14159, 'rad_max': 3.14159,
                'servo_min_deg': 0.0, 'servo_max_deg': 180.0,
                'trim_deg': 0.0, 'sync_offset_deg': 0.0, 'invert': False, 'pin': 27
            },
            'grip': {
                'rad_min': 0.0, 'rad_max': 3.14159,
                'servo_min_deg': 0.0, 'servo_max_deg': 180.0,
                'trim_deg': 0.0, 'sync_offset_deg': 0.0, 'invert': False, 'pin': 26
            },
        }

        # Load stored calibrations from disk
        self.load_calibration_file()

        mode_str = f"📶 Wi-Fi UDP ({self.esp32_ip}:{self.udp_port})" if self.use_wifi else f"🔌 USB Serial ({self.port})"
        self.get_logger().info(
            f"6-Physical-Servo Arm Commander Initializing...\n"
            f"  Communication Mode: {mode_str}\n"
            f"  Active Servo Control Joints:\n"
            + "\n".join([
                f"    • '{j}' [GPIO {c['pin']}]: range=[{c['servo_min_deg']}°, {c['servo_max_deg']}°], sync_offset={c.get('sync_offset_deg', 0.0)}°"
                for j, c in self.joint_calib.items()
            ])
        )

        # ── Network / Serial Connection Init ──────────────────────────────
        self.serial_conn = None
        self.udp_sock = None
        self.comm_lock = threading.Lock()

        if self.use_wifi:
            self.init_wifi_socket()
        else:
            self.connect_serial()

        # Send initial calibration limits on startup
        self.send_all_calibrations()

        # ── Per-joint state tracking ──────────────────────────────────────
        self.last_sent_deg = {}
        self.last_cmd_time = {}
        self.min_cmd_interval = 1.0 / max_cmd_rate

        # ── Subscriptions ────────────────────────────────────────────────
        self.joint_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_states_callback,
            10
        )

        self.calib_sub = self.create_subscription(
            StringMsg,
            '/servo_calibration',
            self.calibration_callback,
            10
        )

        # ── Timers ───────────────────────────────────────────────────────
        self.create_timer(0.05, self.read_incoming_callback)  # 20 Hz
        self.create_timer(1.0, self.check_connection_timeout)  # 1 Hz

        self.get_logger().info("✅ 5-Servo Robot Arm Commander Ready!")
        self.console('SYS', f"Commander started on {self.link_label()}")
        self.publish_link_state(force=True)

    # ── Link monitoring helpers ──────────────────────────────────────────
    def link_label(self):
        if self.use_wifi:
            return f"Wi-Fi UDP {self.esp32_ip}:{self.udp_port}"
        return f"USB serial {self.port} @ {self.baudrate}"

    def console(self, kind, text):
        """Publish one console line ('RX', 'TX', 'SYS' or 'ERR')."""
        try:
            self.console_pub.publish(StringMsg(data=f"{kind}:{text}"))
        except Exception:
            pass

    def note_error(self, text):
        """Record a link error once (callers already rate-limit repeats)."""
        self.err_count += 1
        if text != self.last_error:
            self.last_error = text
            self.console('ERR', text)
            self.publish_link_state()

    def link_state(self):
        now = time.time()
        if self.use_wifi:
            transport_ok = self.udp_sock is not None
        else:
            transport_ok = self.serial_conn is not None and self.serial_conn.is_open
        mismatch = self.mismatch_mode if (self.mismatch_mode and now - self.mismatch_at < LINK_TIMEOUT_S) else None
        if not transport_ok:
            state = 'port_error'
        elif mismatch:
            state = 'mismatch'
        elif self.is_connected:
            state = 'online'
        else:
            state = 'no_reply'
        return {
            'v': 1,
            'node': self.get_name(),
            'state': state,
            'connected': bool(self.is_connected),
            'transport': 'wifi' if self.use_wifi else 'serial',
            'target': self.link_label(),
            'port': self.port,
            'baudrate': self.baudrate,
            'ip': self.esp32_ip,
            'udp_port': self.udp_port,
            'transport_ok': transport_ok,
            'last_rx_age': round(now - self.last_rx_time, 2) if self.last_rx_time else None,
            'rtt_ms': self.rtt_ms,
            'rx': self.rx_count,
            'tx': self.tx_count,
            'errors': self.err_count,
            'last_error': self.last_error,
            'esp32_mode': self.esp32_mode,
            'esp32_ip': self.esp32_ip_reported,
            'mismatch_mode': mismatch,
            'verbose': self.verbose,
            'stamp': now,
        }

    def publish_link_state(self, force=False):
        state = self.link_state()
        # Re-publish when anything but the clock-driven fields changed, and
        # always once a second (check_connection_timeout) as a liveness signal.
        key = json.dumps({k: v for k, v in state.items() if k not in ('stamp', 'last_rx_age', 'rx', 'tx', 'rtt_ms')}, sort_keys=True)
        if force or key != self._last_state_json:
            self._last_state_json = key
            self.link_state_pub.publish(StringMsg(data=json.dumps(state)))

    def handle_link_command(self, cmd):
        """LINK:<command> from the dashboards. Returns nothing; reports on the console."""
        name, _, value = cmd.partition('=')
        name = name.strip().upper()
        value = value.strip()
        if name == 'RECONNECT':
            self.console('SYS', f"Reconnect requested ({self.link_label()})")
            self.reopen_transport()
        elif name == 'STATUS':
            pass
        elif name == 'VERBOSE':
            self.verbose = value in ('1', 'true', 'on', 'yes')
            self.console('SYS', f"Verbose console {'on: joint CMD/ACK traffic shown' if self.verbose else 'off'}")
        elif name == 'PORT':
            if not _PORT_RE.match(value):
                self.console('ERR', f"Rejected port {value!r}: expected /dev/tty... or COMn")
            else:
                self.port = value
                self.console('SYS', f"Serial port set to {value}")
                if not self.use_wifi:
                    self.reopen_transport()
        elif name == 'BAUD':
            try:
                baud = int(value)
            except ValueError:
                baud = None
            if baud not in ALLOWED_BAUDS:
                self.console('ERR', f"Rejected baud rate {value!r}")
            else:
                self.baudrate = baud
                self.console('SYS', f"Baud rate set to {baud}")
                if not self.use_wifi:
                    self.reopen_transport()
        elif name == 'IP':
            host = value.replace('http://', '').replace('https://', '').strip('/')
            try:
                if not _HOST_RE.match(host):
                    raise ValueError("invalid host name")
                self.esp32_ip = socket.gethostbyname(host)
                self.console('SYS', f"Wi-Fi target set to {host}" + (f" ({self.esp32_ip})" if self.esp32_ip != host else ""))
                if self.use_wifi:
                    self.reopen_transport()
            except (OSError, ValueError) as e:
                self.console('ERR', f"Could not resolve {value!r}: {e}")
        else:
            self.console('ERR', f"Unknown LINK command: {cmd}")
        self.publish_link_state(force=True)

    def reopen_transport(self):
        """Close and reopen the active link now (no 2 s reconnect back-off)."""
        self.is_connected = False
        self.mismatch_mode = None
        self.rtt_ms = None
        self.ping_sent_at = None
        if self.use_wifi:
            if self.udp_sock:
                try:
                    self.udp_sock.close()
                except Exception:
                    pass
            self.udp_sock = None
            self.init_wifi_socket()
        else:
            with self.comm_lock:
                if self.serial_conn:
                    try:
                        self.serial_conn.close()
                    except Exception:
                        pass
                self.serial_conn = None
            self._last_reconnect_t = 0.0
            self.connect_serial()
        self.send_all_calibrations()

    def handle_raw(self, line):
        """RAW:<line> from a dashboard console: send it to the ESP32 as typed.

        Joint commands are clamped to the commander's limits so the console
        cannot drive a servo past what the sliders allow.
        """
        line = line.strip()
        if not line or len(line) > 200 or '\n' in line or '\r' in line:
            self.console('ERR', "Rejected raw command (empty, multi-line or too long)")
            return
        m = _JOINT_CMD_RE.match(line)
        if m:
            j_name, deg = m.group(1), float(m.group(2))
            cfg = self.joint_calib.get('turntable_link_joint_dup_3' if j_name == 'wrist_twist_joint' else j_name)
            if cfg is None:
                self.console('ERR', f"Unknown joint {j_name!r}; not sent")
                return
            clamped = max(cfg['servo_min_deg'], min(cfg['servo_max_deg'], deg))
            if clamped != deg:
                self.console('SYS', f"{j_name}: {deg:.1f}° clamped to {clamped:.1f}° (limits {cfg['servo_min_deg']:.1f}–{cfg['servo_max_deg']:.1f}°)")
            # Firmware name for the wrist, as in joint_states_callback.
            target = 'wrist_twist_joint' if j_name == 'turntable_link_joint_dup_3' else j_name
            line = f"CMD:{target}={clamped:.1f}"
        self.console('TX', line)
        self.write_raw_data(line + "\n", logged=True)

    def init_wifi_socket(self):
        """Initialize non-blocking UDP socket for Wi-Fi communication."""
        try:
            self.udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.udp_sock.setblocking(False)
            self.udp_sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
            self.get_logger().info(f"✅ Wi-Fi UDP Socket Initialized for target {self.esp32_ip}:{self.udp_port}")
            self.console('SYS', f"UDP socket ready for {self.esp32_ip}:{self.udp_port}")
        except Exception as e:
            self.get_logger().error(f"Failed to create UDP socket: {e}")
            self.note_error(f"UDP socket failed: {e}")
            self.udp_sock = None

    def publish_status(self, is_conn, details=""):
        msg = StringMsg()
        mode_label = self.link_label()
        if is_conn:
            msg.data = f"STATUS:CONNECTED:🟢 Connected to ESP32 via {mode_label}"
        else:
            msg.data = f"STATUS:DISCONNECTED:🔴 ESP32 Disconnected / Offline via {mode_label} ({details})"
        self.status_pub.publish(msg)

    def calibration_callback(self, msg: StringMsg):
        """Handle live calibration limit updates & mode toggles from GUI."""
        text = msg.data.strip()
        if text.startswith('LINK:'):
            self.handle_link_command(text[5:])
            return
        if text.startswith('RAW:'):
            self.handle_raw(text[4:])
            return
        if text.startswith('MODE:'):
            mode = text[5:].strip().upper()
            self.get_logger().info(f"🔄 Commander Mode Switch Request: MODE:{mode}")
            self.console('TX', text)
            self.mismatch_mode = None

            # Send MODE command over BOTH channels so ESP32 gets it regardless of state
            cmd = text + "\n"
            if self.udp_sock:
                try:
                    self.udp_sock.sendto(cmd.encode('utf-8'), (self.esp32_ip, self.udp_port))
                except Exception:
                    pass

            if self.serial_conn is None or not self.serial_conn.is_open:
                self.connect_serial()
            if self.serial_conn and self.serial_conn.is_open:
                with self.comm_lock:
                    try:
                        self.serial_conn.write(cmd.encode('utf-8'))
                    except Exception:
                        pass

            # Switch commander active transport mode
            was_wifi = self.use_wifi
            if mode == 'SERIAL':
                self.use_wifi = False
                self.connect_serial()
                self.get_logger().info("🔌 Commander actively switched to USB Serial mode.")
            elif mode == 'WIFI':
                self.use_wifi = True
                self.init_wifi_socket()
                self.get_logger().info("📶 Commander actively switched to Wi-Fi UDP mode.")
            if self.use_wifi != was_wifi:
                # Replies on the old link don't prove the new one works.
                self.is_connected = False
                self.rtt_ms = None
                self.console('SYS', f"Commander now using {self.link_label()}")
            self.publish_link_state(force=True)
            return

        if text.startswith('SYNC:'):
            try:
                payload = text[5:].strip()
                eq_idx = payload.find('=')
                if eq_idx > 0:
                    j_name = payload[:eq_idx].strip()
                    val = float(payload[eq_idx+1:].strip())
                    if j_name in self.joint_calib:
                        self.joint_calib[j_name]['sync_offset_deg'] = val
                        self.get_logger().info(f"🎯 Live Sync Offset Updated for '{j_name}': {val:+.1f}°")
            except Exception as e:
                self.get_logger().error(f"Error parsing SYNC string: {e}")
            return

        if text.startswith('CMD:'):
            # A console CMD goes through the same limit clamp as RAW:CMD.
            self.handle_raw(text)
            return
        if text.startswith('CALIB:'):
            self.console('TX', text)
            self.write_raw_data(text + "\n", logged=True)
            if text.startswith('CALIB:'):
                try:
                    payload = text[6:]
                    eq_idx = payload.find('=')
                    if eq_idx > 0:
                        j_name = payload[:eq_idx]
                        limits = payload[eq_idx+1:].split(',')
                        if len(limits) == 2:
                            min_deg, max_deg = float(limits[0]), float(limits[1])
                            if j_name in self.joint_calib:
                                self.joint_calib[j_name]['servo_min_deg'] = min_deg
                                self.joint_calib[j_name]['servo_max_deg'] = max_deg
                                self.get_logger().info(f"⚙️ Live Limits Updated for '{j_name}': [{min_deg}°, {max_deg}°]")
                except Exception as e:
                    self.get_logger().error(f"Error parsing calibration string: {e}")
            return

        if text:
            self.console('ERR', f"Ignored {text[:60]!r}: unknown command (prefix it with RAW: to send it as is)")

    def load_calibration_file(self):
        """Load stored min/max limits, trim, and twin sync offsets from calibration profile."""
        if os.path.exists(CALIB_FILE_PATH):
            try:
                with open(CALIB_FILE_PATH, 'r') as f:
                    calib_data = json.load(f)
                    for j_name, c in calib_data.items():
                        if j_name in self.joint_calib and isinstance(c, dict):
                            if 'servo_min_deg' in c:
                                self.joint_calib[j_name]['servo_min_deg'] = float(c['servo_min_deg'])
                            if 'servo_max_deg' in c:
                                self.joint_calib[j_name]['servo_max_deg'] = float(c['servo_max_deg'])
                            if 'trim_deg' in c:
                                self.joint_calib[j_name]['trim_deg'] = float(c['trim_deg'])
                            if 'sync_offset_deg' in c:
                                self.joint_calib[j_name]['sync_offset_deg'] = float(c['sync_offset_deg'])
                self.get_logger().info(f"✅ Calibration profile loaded from {CALIB_FILE_PATH}")
            except Exception as e:
                self.get_logger().error(f"Error loading calibration file: {e}")

    def connect_serial(self):
        """Establish non-blocking serial connection to ESP32."""
        now = time.time()
        if hasattr(self, '_last_reconnect_t') and (now - self._last_reconnect_t < 2.0):
            return

        self._last_reconnect_t = now
        try:
            self.serial_conn = serial.Serial(
                self.port,
                self.baudrate,
                timeout=0.02,
                write_timeout=0.1
            )
            if self.serial_conn.in_waiting > 0:
                self.serial_conn.read(self.serial_conn.in_waiting)
            # Switch ESP32 firmware mode to SERIAL
            self.serial_conn.write(b"MODE:SERIAL\n")
            self.tx_count += 1
            self.get_logger().info(f"✅ Connected to ESP32 on {self.port} (Sent MODE:SERIAL)")
            self.console('SYS', f"Opened {self.port} @ {self.baudrate}; sent MODE:SERIAL")
            self.last_error = ""
        except (serial.SerialException, OSError) as e:
            now = time.time()
            if not hasattr(self, '_last_serial_warn_t') or (now - self._last_serial_warn_t > 5.0):
                self._last_serial_warn_t = now
                self.get_logger().warn(
                    f"⚠️ Could not open serial port {self.port}: {e}. Retrying dynamically..."
                )
            self.note_error(f"Cannot open {self.port}: {self.explain_serial_error(e)}")
            self.serial_conn = None

    @staticmethod
    def explain_serial_error(err):
        """Short reason plus the usual fix for a serial open failure."""
        text = str(err)
        errno_ = getattr(err, 'errno', None)
        if errno_ == 13 or 'Permission denied' in text:
            return "permission denied (add your user to the 'dialout' group, then log out and in)"
        if errno_ == 2 or 'No such file' in text or 'could not open port' in text and 'FileNotFound' in text:
            return "not found (is the ESP32 plugged in? check the port name)"
        if errno_ == 16 or 'busy' in text.lower():
            return "busy (another program has the port open)"
        return text

    def send_all_calibrations(self):
        """Transmit configured min/max safety limits to ESP32."""
        for j_name, c in self.joint_calib.items():
            calib_cmd = f"CALIB:{j_name}={c['servo_min_deg']:.1f},{c['servo_max_deg']:.1f}\n"
            self.write_raw_data(calib_cmd)

    def trace_tx(self, data_str):
        """Verbose mode: mirror stream traffic (joint CMDs, PINGs, CALIBs) to the console."""
        if self.verbose:
            self.console('TX', data_str.strip())

    def write_raw_data(self, data_str, logged=False):
        """Send command data over Wi-Fi UDP socket or USB Serial.

        logged=True means the caller already put the line on /esp32_console.
        """
        if self.use_wifi:
            if self.udp_sock is None:
                self.init_wifi_socket()
                if self.udp_sock is None:
                    return
            try:
                self.udp_sock.sendto(data_str.encode('utf-8'), (self.esp32_ip, self.udp_port))
                self.tx_count += 1
                if not logged:
                    self.trace_tx(data_str)
            except Exception as e:
                now = time.time()
                if not hasattr(self, '_last_udp_err_t') or (now - self._last_udp_err_t > 5.0):
                    self._last_udp_err_t = now
                    self.get_logger().warn(f"UDP send error (Wi-Fi unreachable): {e}")
                    self.note_error(f"UDP send to {self.esp32_ip}:{self.udp_port} failed: {e}")
        else:
            if self.serial_conn is None or not self.serial_conn.is_open:
                self.connect_serial()
                if self.serial_conn is None:
                    return
            with self.comm_lock:
                try:
                    self.serial_conn.write(data_str.encode('utf-8'))
                    self.tx_count += 1
                    if not logged:
                        self.trace_tx(data_str)
                except (serial.SerialException, OSError) as e:
                    now = time.time()
                    if not hasattr(self, '_last_serial_err_t') or (now - self._last_serial_err_t > 5.0):
                        self._last_serial_err_t = now
                        self.get_logger().warn(f"Serial write error: {e}")
                    self.note_error(f"Serial write to {self.port} failed: {e}")
                    self.is_connected = False
                    if self.serial_conn:
                        try:
                            self.serial_conn.close()
                        except Exception:
                            pass
                    self.serial_conn = None

    def map_rad_to_calibrated_deg(self, rad_val, cfg):
        """Map radians → calibrated degrees applying bounds, inversion, sync offset, trim, and safety limits."""
        r_min, r_max = cfg['rad_min'], cfg['rad_max']
        # No clamp here: the published angle includes the twin sync offset and
        # may legitimately lie outside [r_min, r_max]. Clamping before the offset
        # is removed would cut the servo's range short; the servo limits below
        # are the real safety bound.
        ratio = (rad_val - r_min) / (r_max - r_min)
        if cfg['invert']:
            ratio = 1.0 - ratio

        deg = 0.0 + ratio * 180.0
        deg -= cfg.get('sync_offset_deg', 0.0)
        deg += cfg['trim_deg']
        deg_clamped = max(cfg['servo_min_deg'], min(cfg['servo_max_deg'], deg))
        return round(deg_clamped, 1)

    def joint_states_callback(self, msg: JointState):
        """Handle incoming JointState messages and command active servos."""
        now = time.time()

        for j_name, cfg in self.joint_calib.items():
            if j_name in msg.name:
                idx = msg.name.index(j_name)
                if idx < len(msg.position):
                    rad_value = msg.position[idx]
                    if not math.isfinite(rad_value):
                        continue  # NaN/inf would defeat the min/max clamp
                    deg_value = self.map_rad_to_calibrated_deg(rad_value, cfg)

                    last_t = self.last_cmd_time.get(j_name, 0.0)
                    last_deg = self.last_sent_deg.get(j_name, None)

                    if now - last_t >= self.min_cmd_interval:
                        if last_deg is None or abs(deg_value - last_deg) >= 0.5:
                            target_name = "wrist_twist_joint" if j_name == "turntable_link_joint_dup_3" else j_name
                            cmd = f"CMD:{target_name}={deg_value:.1f}\n"
                            self.write_raw_data(cmd)
                            self.last_sent_deg[j_name] = deg_value
                            self.last_cmd_time[j_name] = now

    def on_valid_rx(self, line):
        """Called for every line received from the ESP32."""
        self.last_rx_time = time.time()
        self.rx_count += 1

        if line in _MISMATCH_ERRORS:
            # The ESP32 answered, but it only obeys the other link.
            self.mismatch_mode = _MISMATCH_ERRORS[line]
            self.mismatch_at = time.time()
            self.esp32_mode = self.mismatch_mode
        elif line == 'ACK:PONG':
            if self.ping_sent_at is not None:
                self.rtt_ms = round((time.monotonic() - self.ping_sent_at) * 1000.0, 1)
                self.ping_sent_at = None
            self.mismatch_mode = None
        elif line.startswith('ACK:MODE='):
            self.esp32_mode = line.split('=', 1)[1].strip().upper() or None
            self.mismatch_mode = None
        elif line.startswith('HEARTBEAT:'):
            for part in line[len('HEARTBEAT:'):].split('|'):
                key, _, val = part.strip().partition('=')
                if key == 'MODE':
                    self.esp32_mode = val.strip().upper() or None
                elif key == 'IP':
                    self.esp32_ip_reported = val.strip() or None

        if not self.is_connected:
            self.is_connected = True
            self.get_logger().info(f"🟢 Connection Established with ESP32 ({self.link_label()})")
            self.console('SYS', f"ESP32 replying on {self.link_label()}")
            self.publish_status(True)
        self.forward_rx(line)
        self.publish_link_state()

    def forward_rx(self, line):
        """Put an ESP32 line on /esp32_console, without flooding it.

        Joint ACKs and PONGs are stream traffic (verbose only). Any other line
        that repeats is forwarded at most every 5 s with a repeat count.
        """
        is_stream = line == 'ACK:PONG' or (line.startswith('ACK:') and '=' in line
                                          and not line.startswith(('ACK:MODE=', 'ACK:CALIB:')))
        if is_stream and not self.verbose:
            return
        now = time.time()
        entry = self._rx_throttle.get(line)
        if entry and now - entry[0] < 5.0:
            entry[1] += 1
            return
        suffix = f"  (repeated {entry[1]}x)" if entry and entry[1] else ""
        self._rx_throttle[line] = [now, 0]
        if len(self._rx_throttle) > 200:
            cutoff = now - 5.0
            self._rx_throttle = {k: v for k, v in self._rx_throttle.items() if v[0] >= cutoff}
        self.console('RX', line + suffix)

    def check_connection_timeout(self):
        """Periodic 1Hz timer: PING the ESP32, time out the link, publish state."""
        now = time.time()
        if self.ping_sent_at is not None and time.monotonic() - self.ping_sent_at > LINK_TIMEOUT_S:
            self.ping_sent_at = None   # lost; a fresh one goes out below
            self.rtt_ms = None
        if self.ping_sent_at is None:
            self.ping_sent_at = time.monotonic()
        self.write_raw_data("PING\n")

        if self.is_connected and (now - self.last_rx_time > LINK_TIMEOUT_S):
            self.is_connected = False
            self.rtt_ms = None
            self.get_logger().warn(f"🔴 Connection Lost to ESP32 ({self.link_label()})")
            self.console('ERR', f"No reply from ESP32 for {LINK_TIMEOUT_S:.0f} s on {self.link_label()}")
            self.publish_status(False, f"Timeout > {LINK_TIMEOUT_S:.0f}s")
        elif self.is_connected:
            self.publish_status(True)
        else:
            # Keep dashboards informed while it has never connected, too.
            transport_ok = self.udp_sock is not None if self.use_wifi else (
                self.serial_conn is not None and self.serial_conn.is_open)
            self.publish_status(False, "port not open" if not transport_ok else "no reply yet")
        self.publish_link_state(force=True)

    def read_incoming_callback(self):
        """Read responses (ACK / HEARTBEAT) over Wi-Fi UDP or Serial."""
        if self.use_wifi:
            if self.udp_sock is None:
                return
            try:
                while True:
                    data, addr = self.udp_sock.recvfrom(1024)
                    line = data.decode('utf-8', errors='ignore').strip()
                    if line:
                        self.on_valid_rx(line)
                        if line.startswith('ACK:'):
                            self.get_logger().debug(f"📶 [Wi-Fi ACK] {line}")
                        elif line.startswith('HEARTBEAT:'):
                            self.get_logger().debug(f"💓 {line}")
                        else:
                            self.get_logger().info(f"📶 [Wi-Fi ESP32] {line}")
            except BlockingIOError:
                pass
            except Exception as e:
                # e.g. ICMP port unreachable surfacing as ConnectionResetError
                now = time.time()
                if now - getattr(self, '_last_udp_rx_err_t', 0.0) > 5.0:
                    self._last_udp_rx_err_t = now
                    self.note_error(f"UDP receive error: {e}")
        else:
            if self.serial_conn is None or not self.serial_conn.is_open:
                self.connect_serial()
                if self.serial_conn is None:
                    return

            with self.comm_lock:
                try:
                    while self.serial_conn.in_waiting > 0:
                        line_bytes = self.serial_conn.readline()
                        line = line_bytes.decode('utf-8', errors='ignore').strip()
                        if line:
                            self.on_valid_rx(line)
                            if line.startswith('ACK:'):
                                self.get_logger().debug(f"✅ {line}")
                            elif line.startswith('HEARTBEAT:'):
                                self.get_logger().debug(f"💓 {line}")
                            elif line.startswith('ERR:'):
                                self.get_logger().warn(f"⚠️ ESP32 error: {line}")
                            else:
                                self.get_logger().debug(f"ESP32: {line}")
                except (serial.SerialException, OSError) as e:
                    self.get_logger().error(f"Serial read error: {e}")
                    self.note_error(f"Serial read from {self.port} failed: {e} (unplugged?)")
                    self.is_connected = False
                    if self.serial_conn:
                        try:
                            self.serial_conn.close()
                        except Exception:
                            pass
                    self.serial_conn = None
                    self.publish_link_state(force=True)


def main(args=None):
    rclpy.init(args=args)
    node = ServoSerialCommander()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down Commander...")
    finally:
        if node.serial_conn and node.serial_conn.is_open:
            node.serial_conn.close()
        if node.udp_sock:
            node.udp_sock.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
