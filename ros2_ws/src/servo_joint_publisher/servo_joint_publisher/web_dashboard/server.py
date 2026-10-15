#!/usr/bin/env python3
"""
Python Web Dashboard Server & ROS 2 Node for Robotic Arm
--------------------------------------------------------
Features:
  • Built-in ThreadingHTTPServer serving static assets, URDF models, and STL meshes.
  • REST API endpoints for joint state streaming & calibration limit persistence.
  • Publishes /joint_states at 50 Hz and forwards calibration / sync / mode
    changes to servo_serial_commander over /servo_calibration.
  • Listens on 127.0.0.1 only by default. Pass --host 0.0.0.0 to expose it on
    the LAN (anyone who can reach it can then move the robots).
"""

import sys
import os
import copy
import json
import math
import time
import threading
import urllib.error
import urllib.parse
import urllib.request
import re
import argparse
from http.server import HTTPServer, SimpleHTTPRequestHandler
from socketserver import ThreadingMixIn

import socket
import collections
import glob
try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
    from sensor_msgs.msg import JointState
    from std_msgs.msg import String as StringMsg
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


def find_project_dir():
    """Locate the repository root (holds servo_calibration.json and urdf/).

    Honours $ROBOT_PROJECT_DIR, otherwise walks up from this file, which works
    both from the source tree and from a colcon install inside ros2_ws/.
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
MESH_DIR = os.path.join(PROJECT_DIR, "urdf", "arm", "meshes", "stl")
QUARD_URDF_PATH = os.path.join(PROJECT_DIR, "urdf", "quard_bot", "urdf", "quard_bot.urdf")
QUARD_MESH_DIR = os.path.join(PROJECT_DIR, "urdf", "quard_bot", "meshes")
UNNAMED_MESH_DIR = os.path.join(PROJECT_DIR, "urdf", "unnamed", "meshes", "stl")
DOGU_DIR = os.path.join(PROJECT_DIR, "urdf", "dogu")          # built by scripts/dogu/
DOGU_MESH_DIR = os.path.join(DOGU_DIR, "meshes")
DOGU_DEFAULT_HOST = "http://192.168.4.1"    # src/dogu.cpp starts its own access point
DOGU_STALE_S = 2.0                          # a /start that waited longer is dropped
STATIC_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "static")

# Arm joints driven by the dashboard. Slider degrees (0..180) map linearly onto
# [rad_min, rad_max]; these ranges must match JOINTS in static/app.js and
# joint_calib in servo_serial_commander.py.
ARM_JOINTS = {
    'turntable_link_joint_dup':   {'rad_min': -3.0,     'rad_max': 3.0,     'def': 90.0},
    'turntable_link_joint':       {'rad_min': -2.0,     'rad_max': 2.0,     'def': 59.0},
    'turntable_link_joint_dup_1': {'rad_min': -2.0,     'rad_max': 2.0,     'def': 153.1},
    'turntable_link_joint_dup_2': {'rad_min': -2.0,     'rad_max': 2.0,     'def': 116.0},
    'turntable_link_joint_dup_3': {'rad_min': -3.14159, 'rad_max': 3.14159, 'def': 90.0},
}
# Older clients / firmware name for the wrist joint.
JOINT_ALIASES = {'wrist_twist_joint': 'turntable_link_joint_dup_3'}

LOOPBACK_HOSTS = {'localhost', '127.0.0.1', '::1'}


def finite_float(value):
    """float(value), rejecting NaN/inf (float() happily accepts 'nan')."""
    f = float(value)
    if not math.isfinite(f):
        raise ValueError(f"non-finite value: {value!r}")
    return f


def send_comm_mode_udp_broadcast(mode_str):
    """Fallback when ROS 2 is unavailable: broadcast MODE over UDP port 8888.

    Deliberately does not open a serial port: that would reset the ESP32 (DTR)
    and steal bytes from servo_serial_commander, which owns the port.
    """
    cmd_bytes = f"MODE:{mode_str}\n".encode('utf-8')
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        sock.settimeout(0.5)
        sock.sendto(cmd_bytes, ('255.255.255.255', 8888))
        sock.close()
        print(f"[Server] Sent MODE:{mode_str} via UDP broadcast")
    except Exception as e:
        print(f"[Server] UDP mode send error: {e}")


class ConsoleLog:
    """Per-robot console lines for the browser, polled via /api/console.

    Every entry has a stable id plus a seq that grows whenever it changes, so
    a client asking for seq > N also gets entries updated in place: repeats of
    the last line (count) or a new value for the same key (slider drags).
    """
    CHANNELS = ('arm', 'quard', 'sensors', 'dogu')

    def __init__(self, per_channel=500):
        self.lock = threading.Lock()
        self.seq = 0
        self.next_id = 0
        self.channels = {c: collections.deque(maxlen=per_channel) for c in self.CHANNELS}

    def add(self, channel, kind, text, key=None):
        text = str(text)[:500]
        with self.lock:
            q = self.channels[channel]
            last = q[-1] if q else None
            self.seq += 1
            if last is not None and ((key is not None and last['key'] == key) or
                                     (key is None and last['key'] is None and last['kind'] == kind and last['text'] == text)):
                if key is not None:
                    last['text'], last['kind'] = text, kind
                else:
                    last['count'] += 1
                last['t'], last['seq'] = time.time(), self.seq
                return
            self.next_id += 1
            q.append({'id': self.next_id, 'seq': self.seq, 't': time.time(), 'channel': channel,
                      'kind': kind, 'text': text, 'key': key, 'count': 1})

    def since(self, seq, limit=600):
        with self.lock:
            out = [dict(e) for q in self.channels.values() for e in q if e['seq'] > seq]
            latest = self.seq
        out.sort(key=lambda e: e['seq'])
        return latest, out[-limit:]


console_log = ConsoleLog()


def describe_net_error(err):
    """Plain-language reason for an HTTP request failure (same wording as the Qt GUI)."""
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


def explain_serial_error(err):
    text = str(err)
    errno_ = getattr(err, 'errno', None)
    if errno_ == 13 or 'Permission denied' in text:
        return "permission denied: add your user to the 'dialout' group, then log out and back in"
    if errno_ == 2 or 'No such file' in text:
        return "port not found: is the board plugged in?"
    if errno_ == 16 or 'busy' in text.lower():
        return "port busy: another program has it open"
    if 'readiness to read but returned no data' in text:
        return "device disconnected (unplugged or reset)"
    return text


def list_serial_ports():
    try:
        from serial.tools import list_ports
        return sorted((p.device, p.description if p.description and p.description != 'n/a' else '')
                      for p in list_ports.comports())
    except Exception:
        return [(d, '') for pattern in ('/dev/ttyUSB*', '/dev/ttyACM*') for d in sorted(glob.glob(pattern))]


_LINK_CMD_RE = re.compile(r'^(RECONNECT|STATUS|VERBOSE=[01]|PORT=(/dev/[A-Za-z0-9_./+-]+|COM\d+)|BAUD=\d{3,7}|IP=[A-Za-z0-9.:/-]{1,253})$')


# Rover readings before (or between) real data: None means "no reading", and
# the dashboards show "--" instead of a made-up value.
EMPTY_SENSOR_READINGS = {
    "temperature": None, "temp_dht": None, "humidity": None, "temp_bmp": None, "pressure": None,
    "dht_error": None, "accel": None, "gyro": None, "pitch": None, "roll": None,
    "raw_pitch": None, "raw_roll": None, "gas": None, "water": None, "flame": None,
    "air_quality": None, "lat": None, "lng": None, "gps_valid": None, "gps_sats": None,
    "alert": None, "sd_ok": None, "uptime_ms": None,
}


def _reading(d, keys, cast=float):
    """First present, finite value among keys in a rover JSON reply, else None."""
    for k in keys:
        if d.get(k) is None:
            continue
        try:
            v = float(d[k])
        except (TypeError, ValueError):
            return None
        if not math.isfinite(v):
            return None
        return int(v) if cast is int else v
    return None


def air_quality(gas):
    return "Clean" if gas < 600 else ("Moderate" if gas < 1500 else "Danger")


# Shared state between HTTP Server and ROS 2 Node
class RobotState:
    def __init__(self):
        self.lock = threading.Lock()
        self.joint_positions = {j: cfg['def'] for j, cfg in ARM_JOINTS.items()}
        self.calibration = self.load_calibration()
        self.is_simulating = False
        self.sim_speed = 1.0
        self.sim_time = 0.0
        self.sim_blend_start_time = 0.0
        self.sim_blend_duration = 2.0
        self.sim_blend_start_positions = {}
        self.sim_start_time = time.time()
        self.last_update = time.time()
        self.comm_mode = "SERIAL"
        # True while the ROS 2 bridge node is spinning (arm commands reach hardware).
        self.ros_bridge_active = False
        # Arm link as reported by servo_serial_commander (/esp32_link_state)
        self.arm_link = None
        self.arm_link_rx_t = 0.0
        self.arm_legacy = None          # (connected, text, time) from /esp32_connection_status
        self.arm_link_last_state = None
        # Quard Bot link, measured by /api/quardbot/ping and /cmd
        self.quard_link = {'online': None, 'rtt_ms': None, 'last_seen': 0.0, 'sent': 0, 'failed': 0,
                           'last_error': '', 'rssi': None}
        self.quard_resolved = {}        # hostname -> IP from the last good ping
        self.quard_cmd_failures = 0
        # Dogu robot dog (src/dogu.cpp): GET /start, /stop, /status over HTTP
        self.dogu_host = DOGU_DEFAULT_HOST
        self.dogu_link = {'online': None, 'rtt_ms': None, 'last_seen': 0.0, 'sent': 0, 'failed': 0,
                          'last_error': ''}
        self.dogu_status = None         # last /status reply: {walking, leg, phase}
        self.dogu_status_t = 0.0
        self.dogu_cmd_lock = threading.Lock()   # one /start or /stop at a time, in order
        # Rover data source: 'wifi' | 'serial' | 'off'
        self.sensor_link = {'source': 'off', 'phase': 'off', 'error': '', 'last_rx': 0.0,
                            'since': time.time(), 'gen': 0, 'target': ''}
        self.sensor_rx_times = collections.deque(maxlen=200)
        # Text commands for servo_serial_commander (CALIB:/SYNC:/MODE:),
        # drained by the ROS 2 bridge node onto /servo_calibration.
        self.commander_msgs = []
        # Quard Bot (Sesame Quadruped) State
        self.quardbot_host = "http://sesame-robot.local"
        self.quardbot_mode = "stand"
        self.quardbot_online = False
        self.quardbot_channels = [45, 45, 45, 45, 45, 45, 45, 45]
        self.quard_joints = {
            'joint_l1_hip': 0.0,
            'joint_l2_hip': 0.0,
            'joint_r1_hip': 0.0,
            'joint_r2_hip': 0.0,
            'joint_l3_foot': 0.0,
            'joint_l4_foot': 0.0,
            'joint_r3_foot': 0.0,
            'joint_r4_foot': 0.0,
        }

        # ESP32 Sensor Node (DHT11, BMP280, MPU6050, MQ Gas, Water, Flame, GPS) State
        self.sensor_state = dict(EMPTY_SENSOR_READINGS, **{
            "connected": False,
            "wifi_connected": False,
            "wifi_ip": "192.168.1.100",
            "port": "/dev/ttyUSB0",
            "baudrate": 115200,
            "last_seen": 0.0,
            "raw_log": []
        })

    def parse_sensor_json(self, raw_str):
        """Apply a rover /data reply. Only fields present in the reply change;
        nothing is filled in. Returns False if it isn't a JSON object."""
        try:
            d = json.loads(raw_str)
        except Exception:
            return False
        if not isinstance(d, dict):
            return False
        console_log.add('sensors', 'data', raw_str.strip()[:240])
        with self.lock:
            st = self.sensor_state
            st["raw_log"].append(f"[JSON] {raw_str[:90]}...")
            if len(st["raw_log"]) > 100:
                st["raw_log"].pop(0)
            st["last_seen"] = time.time()

            for key, keys, cast in (("temp_dht", ("temp_dht", "temperature"), float),
                                    ("humidity", ("humidity",), float),
                                    ("temp_bmp", ("temp_bmp",), float),
                                    ("pressure", ("pressure",), float),
                                    ("gas", ("air", "gas"), int),
                                    ("water", ("water",), int),
                                    ("flame", ("flame",), int),
                                    ("lat", ("lat",), float),
                                    ("lng", ("lng",), float),
                                    ("gps_sats", ("gps_sats",), int),
                                    ("uptime_ms", ("uptime_ms",), int)):
                v = _reading(d, keys, cast)
                if v is not None:
                    st[key] = v
            if st["temp_dht"] is not None:
                st["temperature"] = st["temp_dht"]
            if st["gas"] is not None:
                st["air_quality"] = air_quality(st["gas"])
            for key in ("gps_valid", "sd_ok", "dht_error"):
                if key in d and d[key] is not None:
                    st[key] = bool(d[key])
            if d.get("alert") is not None:
                st["alert"] = str(d["alert"]).upper()

            accel = [_reading(d, (k,)) for k in ("ax", "ay", "az")]
            if None not in accel:
                ax, ay, az = accel
                if abs(az) > 50 or abs(ax) > 50 or abs(ay) > 50:   # raw MPU6050 LSB (16384/g)
                    ax, ay, az = ((v / 16384.0) * 9.80665 for v in (ax, ay, az))
                st["accel"] = {"x": round(ax, 2), "y": round(ay, 2), "z": round(az, 2)}
                if ax or ay or az:
                    raw_p = round(math.degrees(math.atan2(ay, math.sqrt(ax ** 2 + az ** 2))), 1)
                    raw_r = round(math.degrees(math.atan2(-ax, az)), 1)
                    imu_t = self.calibration.get("imu_offsets", {})
                    st["raw_pitch"], st["raw_roll"] = raw_p, raw_r
                    st["pitch"] = round(raw_p - float(imu_t.get("pitch", 0.0)), 1)
                    st["roll"] = round(raw_r - float(imu_t.get("roll", 0.0)), 1)
            gyro = [_reading(d, (k,)) for k in ("gx", "gy", "gz")]
            if None not in gyro:
                if any(abs(v) > 50 for v in gyro):                   # raw LSB (131 per deg/s)
                    gyro = [math.radians(v / 131.0) for v in gyro]
                st["gyro"] = {"x": round(gyro[0], 3), "y": round(gyro[1], 3), "z": round(gyro[2], 3)}
        return True

    def parse_sensor_line(self, line):
        line = line.strip()
        if not line:
            return
        console_log.add('sensors', 'data', line)

        with self.lock:
            # Append line to raw_log (limit 100)
            self.sensor_state["raw_log"].append(line)
            if len(self.sensor_state["raw_log"]) > 100:
                self.sensor_state["raw_log"].pop(0)

            self.sensor_state["last_seen"] = time.time()

            # 1. DHT11 parsing
            if "DHT11: ERROR" in line:
                self.sensor_state["dht_error"] = True
            else:
                m_temp = re.search(r"(?:Temperature:|DHT11\s+temp:)\s*([-\d\.]+)", line, re.I)
                if m_temp:
                    self.sensor_state["temperature"] = float(m_temp.group(1))
                    self.sensor_state["temp_dht"] = float(m_temp.group(1))
                    self.sensor_state["dht_error"] = False

                m_hum = re.search(r"(?:Humidity:|hum:)\s*([-\d\.]+)", line, re.I)
                if m_hum:
                    self.sensor_state["humidity"] = float(m_hum.group(1))
                    self.sensor_state["dht_error"] = False

            # 2. BMP280
            m_bmp = re.search(r"BMP280\s+temp:\s*([-\d\.]+)\s*C\s+pres:\s*([-\d\.]+)", line, re.I)
            if m_bmp:
                self.sensor_state["temp_bmp"] = float(m_bmp.group(1))
                self.sensor_state["pressure"] = float(m_bmp.group(2))

            # 3. Gas & Water
            m_gas = re.search(r"(?:MQ-5 Analog:|Gas:)\s*(\d+)", line, re.I)
            if m_gas:
                gas_val = int(m_gas.group(1))
                self.sensor_state["gas"] = gas_val
                if gas_val < 600:
                    self.sensor_state["air_quality"] = "Clean"
                elif gas_val < 1500:
                    self.sensor_state["air_quality"] = "Moderate"
                else:
                    self.sensor_state["air_quality"] = "Danger"

            m_water = re.search(r"Water:\s*(\d+)", line, re.I)
            if m_water:
                self.sensor_state["water"] = int(m_water.group(1))

            # 4. Flame
            if "Flame:" in line:
                if "*** FIRE ***" in line or "FIRE" in line:
                    self.sensor_state["flame"] = 0
                    self.sensor_state["alert"] = "FIRE"
                else:
                    self.sensor_state["flame"] = 1

            # 5. MPU6050 parsing
            m_accel = re.search(r"(?:Accel X:|Accel:\s*X=)\s*([-\d\.]+)(?:\s*\|\s*Y:|\s+Y=)\s*([-\d\.]+)(?:\s*\|\s*Z:|\s+Z=)\s*([-\d\.]+)", line, re.I)
            if m_accel:
                ax = float(m_accel.group(1))
                ay = float(m_accel.group(2))
                az = float(m_accel.group(3))
                if abs(az) > 50 or abs(ax) > 50 or abs(ay) > 50:
                    ax = (ax / 16384.0) * 9.80665
                    ay = (ay / 16384.0) * 9.80665
                    az = (az / 16384.0) * 9.80665
                self.sensor_state["accel"] = {"x": round(ax, 2), "y": round(ay, 2), "z": round(az, 2)}

                # Compute estimated Pitch & Roll
                if az != 0 or ay != 0 or ax != 0:
                    pitch_rad = math.atan2(ay, math.sqrt(ax**2 + az**2))
                    roll_rad = math.atan2(-ax, az)
                    raw_p = round(pitch_rad * (180.0 / math.pi), 1)
                    raw_r = round(roll_rad * (180.0 / math.pi), 1)
                    self.sensor_state["raw_pitch"] = raw_p
                    self.sensor_state["raw_roll"] = raw_r
                    imu_t = self.calibration.get("imu_offsets", {})
                    p_off = float(imu_t.get("pitch", 0.0))
                    r_off = float(imu_t.get("roll", 0.0))
                    self.sensor_state["pitch"] = round(raw_p - p_off, 1)
                    self.sensor_state["roll"] = round(raw_r - r_off, 1)

            m_gyro = re.search(r"(?:Gyro X:|Gyro:\s*X=)\s*([-\d\.]+)(?:\s*\|\s*Y:|\s+Y=)\s*([-\d\.]+)(?:\s*\|\s*Z:|\s+Z=)\s*([-\d\.]+)", line, re.I)
            if m_gyro:
                gx = float(m_gyro.group(1))
                gy = float(m_gyro.group(2))
                gz = float(m_gyro.group(3))
                if abs(gx) > 20 or abs(gy) > 20 or abs(gz) > 20:
                    gx = (gx / 131.0) * (math.pi / 180.0)
                    gy = (gy / 131.0) * (math.pi / 180.0)
                    gz = (gz / 131.0) * (math.pi / 180.0)
                self.sensor_state["gyro"] = {"x": round(gx, 3), "y": round(gy, 3), "z": round(gz, 3)}

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

    def _deep_update(self, target, source):
        for k, v in source.items():
            if isinstance(v, dict) and k in target and isinstance(target[k], dict):
                self._deep_update(target[k], v)
            else:
                target[k] = v

    def load_calibration(self):
        if os.path.exists(CALIB_FILE_PATH):
            try:
                with open(CALIB_FILE_PATH, 'r') as f:
                    return json.load(f)
            except Exception as e:
                print(f"[Server] Error loading calibration: {e}")
        else:
            print(f"[Server] Calibration file not found at {CALIB_FILE_PATH}; using defaults")
        return {}

    @staticmethod
    def validate_calibration(new_calib):
        """Return a sanitised copy of a calibration update, or raise ValueError.

        Every leaf must be a finite number or a bool; a null/NaN written to the
        file would make servo_serial_commander abort its whole calibration load.
        """
        if not isinstance(new_calib, dict):
            raise ValueError("calibration update must be a JSON object")
        clean = {}
        for key, val in new_calib.items():
            key = str(key)
            if isinstance(val, dict):
                clean[key] = RobotState.validate_calibration(val)
            elif isinstance(val, bool):
                clean[key] = val
            else:
                clean[key] = finite_float(val)
        for j_name in ARM_JOINTS:
            cal = clean.get(j_name)
            if isinstance(cal, dict) and 'servo_min_deg' in cal and 'servo_max_deg' in cal:
                lo, hi = cal['servo_min_deg'], cal['servo_max_deg']
                if not (0.0 <= lo < hi <= 180.0):
                    raise ValueError(f"{j_name}: need 0 <= servo_min_deg < servo_max_deg <= 180")
        return clean

    def save_calibration(self, new_calib):
        """Merge, persist and forward a calibration update. Returns (ok, message)."""
        try:
            new_calib = self.validate_calibration(new_calib)
        except (TypeError, ValueError) as e:
            return False, str(e)
        with self.lock:
            self._deep_update(self.calibration, new_calib)
            self._queue_commander_updates(new_calib)
            try:
                with open(CALIB_FILE_PATH, 'w') as f:
                    json.dump(self.calibration, f, indent=2)
                return True, "saved"
            except Exception as e:
                print(f"[Server] Error saving calibration: {e}")
                return False, str(e)

    def _queue_commander_updates(self, new_calib):
        """Tell servo_serial_commander about changed limits / sync offsets (lock held)."""
        for j_name in ARM_JOINTS:
            upd = new_calib.get(j_name)
            if not isinstance(upd, dict):
                continue
            if 'servo_min_deg' in upd or 'servo_max_deg' in upd:
                cal = self.calibration.get(j_name, {})
                lo = cal.get('servo_min_deg', 0.0)
                hi = cal.get('servo_max_deg', 180.0)
                self.commander_msgs.append(f"CALIB:{j_name}={lo:.1f},{hi:.1f}")
            if 'sync_offset_deg' in upd:
                self.commander_msgs.append(f"SYNC:{j_name}={upd['sync_offset_deg']:.1f}")

    # ── link state (call with self.lock held unless noted) ────────────────
    def commander_alive(self):
        return self.arm_link is not None and time.time() - self.arm_link_rx_t < 3.5

    def ingest_arm_link(self, raw):
        """New /esp32_link_state JSON (lock not held)."""
        try:
            data = json.loads(raw)
        except ValueError:
            return
        if not isinstance(data, dict):
            return
        with self.lock:
            if self.arm_link_last_state == 'no_commander':
                console_log.add('arm', 'ok', f"servo_serial_commander is running ({data.get('target', '?')})")
            self.arm_link = data
            self.arm_link_rx_t = time.time()
            self.arm_link_last_state = data.get('state')

    def ingest_arm_legacy(self, text):
        for prefix, ok in (("STATUS:CONNECTED:", True), ("STATUS:DISCONNECTED:", False)):
            if text.startswith(prefix):
                with self.lock:
                    self.arm_legacy = (ok, text[len(prefix):].strip(), time.time())
                return

    @staticmethod
    def ingest_arm_console(text):
        kinds = {'RX': 'rx', 'TX': 'tx', 'SYS': 'info', 'ERR': 'error'}
        prefix, sep, rest = text.partition(':')
        if sep and prefix in kinds:
            console_log.add('arm', kinds[prefix], rest)
        else:
            console_log.add('arm', 'rx', text)

    def set_sensor_source(self, src, target=''):
        """Switch the rover data source; bumps gen so the old reader stops."""
        lk = self.sensor_link
        lk.update(source=src, phase='connecting' if src in ('wifi', 'serial') else src, error='',
                  last_rx=0.0, since=time.time(), gen=lk['gen'] + 1, target=target)
        self.sensor_rx_times.clear()
        st = self.sensor_state
        st.update(EMPTY_SENSOR_READINGS)    # old readings are not from the new source
        st['wifi_connected'] = src == 'wifi'
        st['connected'] = src == 'serial'
        label = {'off': "Rover data source off",
                 'wifi': f"Connecting to {target} …", 'serial': f"Opening {target} …"}[src]
        console_log.add('sensors', 'info', label)

    def sensor_rx(self, gen):
        lk = self.sensor_link
        if gen != lk['gen']:
            return
        now = time.time()
        if lk['phase'] != 'live' or now - lk['last_rx'] > 5.0:
            console_log.add('sensors', 'ok', f"Receiving data from {lk['target']}")
        lk['phase'], lk['error'], lk['last_rx'] = 'live', '', now
        self.sensor_rx_times.append(now)

    def sensor_event(self, gen, phase, detail=''):
        lk = self.sensor_link
        if gen != lk['gen']:
            return
        if phase == 'error' and detail != lk['error']:
            console_log.add('sensors', 'error', f"{lk['target']}: {detail}")
        elif phase == 'waiting':
            console_log.add('sensors', 'info', f"Opened {lk['target']}; waiting for data…")
        lk['phase'], lk['error'] = phase, (detail if phase == 'error' else '')
        if phase == 'waiting':
            lk['since'] = time.time()

    def quard_record(self, ok, rtt_ms=None, error='', is_ping=False):
        """Update the Quard Bot link after a ping or command."""
        lk = self.quard_link
        was = lk['online']
        if ok:
            lk.update(online=True, rtt_ms=rtt_ms, last_seen=time.time(), last_error='')
            self.quard_cmd_failures = 0
        else:
            lk['last_error'] = error or 'no reply'
            if is_ping:
                lk['online'] = False
            else:
                self.quard_cmd_failures += 1
                if self.quard_cmd_failures >= 2:   # one lost command isn't an outage
                    lk['online'] = False
        if lk['online'] is not None and lk['online'] != was:
            if lk['online']:
                rtt = f" (ping {rtt_ms:.0f} ms)" if rtt_ms is not None else ""
                console_log.add('quard', 'ok', f"Quard Bot online at {self.quardbot_host}{rtt}")
            else:
                console_log.add('quard', 'error', f"Quard Bot unreachable: {lk['last_error']}")

    def dogu_record(self, ok, rtt_ms=None, error=''):
        """Update the Dogu link after a ping or command (caller holds the lock)."""
        lk = self.dogu_link
        was = lk['online']
        if ok:
            lk.update(online=True, rtt_ms=rtt_ms, last_seen=time.time(), last_error='')
        else:
            lk.update(online=False, last_error=error or 'no reply')
        if lk['online'] != was:
            if lk['online']:
                rtt = f" (ping {rtt_ms:.0f} ms)" if rtt_ms is not None else ""
                console_log.add('dogu', 'ok', f"Dogu online at {self.dogu_host}{rtt}")
            elif was:
                walking = (" It was walking at last report."
                           if self.dogu_status and self.dogu_status['walking'] else "")
                console_log.add('dogu', 'error', f"Dogu unreachable: {lk['last_error']}.{walking}")

    def links_snapshot(self):
        now = time.time()
        if not ROS2_AVAILABLE:
            arm = {'state': 'no_ros'}
        elif self.commander_alive():
            arm = dict(self.arm_link)
            arm['age'] = round(now - self.arm_link_rx_t, 2)
        elif self.arm_legacy and now - self.arm_legacy[2] < 5.0:
            arm = {'state': 'legacy', 'connected': self.arm_legacy[0], 'detail': self.arm_legacy[1]}
        else:
            arm = {'state': 'no_commander', 'bridge': self.ros_bridge_active,
                   'last_heard': round(now - self.arm_link_rx_t, 1) if self.arm_link_rx_t else None}
            if self.arm_link_last_state not in (None, 'no_commander'):
                console_log.add('arm', 'error', "Lost contact with servo_serial_commander")
            self.arm_link_last_state = 'no_commander'
        q = dict(self.quard_link)
        q['host'] = self.quardbot_host
        q['last_seen_age'] = round(now - q['last_seen'], 1) if q['last_seen'] else None
        lk = self.sensor_link
        sensors = {k: lk[k] for k in ('source', 'phase', 'error', 'target')}
        sensors['last_rx_age'] = round(now - lk['last_rx'], 1) if lk['last_rx'] else None
        sensors['rate'] = round(sum(1 for t in self.sensor_rx_times if now - t < 5.0) / 5.0, 1)
        sensors['phase_age'] = round(now - lk['since'], 1)
        d = dict(self.dogu_link)
        d['host'] = self.dogu_host
        d['last_seen_age'] = round(now - d['last_seen'], 1) if d['last_seen'] else None
        d['status'] = dict(self.dogu_status) if self.dogu_status else None
        d['status_age'] = round(now - self.dogu_status_t, 1) if self.dogu_status else None
        return {'arm': arm, 'quard': q, 'sensors': sensors, 'dogu': d}

    def snapshot(self, obj):
        """Deep copy of shared state taken under the lock, safe to serialise later."""
        with self.lock:
            return copy.deepcopy(obj)

    def step_simulation(self):
        """Advance the demo animation. Called at 50 Hz from a background thread so
        the sim runs at real speed whether or not a browser is polling."""
        with self.lock:
            now = time.time()
            if self.is_simulating:
                dt = now - self.last_update
                dt = min(max(dt, 0.0), 0.1)
                self.sim_time += dt * self.sim_speed
                t = self.sim_time

                # Smooth cosine ease-in S-curve blend (C1-continuous)
                blend_elapsed = now - self.sim_blend_start_time
                progress = min(1.0, max(0.0, blend_elapsed / self.sim_blend_duration))
                w = 0.5 - 0.5 * math.cos(progress * math.pi)

                sim_targets = {
                    'turntable_link_joint_dup': 90.0 + 45.0 * math.sin(0.3 * t),
                    'turntable_link_joint': 59.0 + 20.0 * math.sin(0.4 * t + 1.0),
                    'turntable_link_joint_dup_1': 110.0 + 35.0 * math.sin(0.5 * t + 2.0),
                    'turntable_link_joint_dup_2': 116.0 + 25.0 * math.sin(0.6 * t + 1.5),
                    'turntable_link_joint_dup_3': 90.0 + 35.0 * math.sin(0.4 * t + 0.5),
                }

                for jk, target_val in sim_targets.items():
                    start_val = self.sim_blend_start_positions.get(
                        jk, self.joint_positions.get(jk, target_val)
                    )
                    blended = (1.0 - w) * start_val + w * target_val
                    cal = self.calibration.get(jk, {})
                    min_lim = cal.get("servo_min_deg", 0.0)
                    max_lim = cal.get("servo_max_deg", 180.0)
                    self.joint_positions[jk] = max(min_lim, min(max_lim, round(blended, 2)))

            self.last_update = now


robot_state = RobotState()

class ThreadedHTTPServer(ThreadingMixIn, HTTPServer):
    """Handle requests in separate threads for zero UI latency."""
    daemon_threads = True

class DashboardHTTPHandler(SimpleHTTPRequestHandler):
    # Set by main(): True when bound to a loopback address only.
    loopback_only = True

    def __init__(self, *args, **kwargs):
        super().__init__(*args, directory=STATIC_DIR, **kwargs)

    @staticmethod
    def _hostname(netloc):
        """'host:port' / '[::1]:port' -> 'host' / '::1' (lower-cased)."""
        netloc = (netloc or '').strip().lower()
        if netloc.startswith('['):
            return netloc[1:netloc.find(']')] if ']' in netloc else netloc
        return netloc.rsplit(':', 1)[0] if netloc.count(':') == 1 else netloc

    def request_allowed(self, is_post):
        """Reject cross-site and DNS-rebinding requests.

        * When bound to loopback, the Host header must be a loopback name, so a
          malicious domain re-pointed at 127.0.0.1 cannot drive the robots.
        * A browser-supplied Origin must match Host (blocks cross-site POSTs).
        * POST bodies must be JSON, which forces a CORS preflight that this
          server never approves.
        """
        host = self.headers.get('Host', '')
        if self.loopback_only and self._hostname(host) not in LOOPBACK_HOSTS:
            self.send_error(403, "Host not allowed")
            return False
        origin = self.headers.get('Origin')
        if origin and origin != 'null':
            if urllib.parse.urlparse(origin).netloc.lower() != host.lower():
                self.send_error(403, "Cross-origin request rejected")
                return False
        elif origin == 'null' and is_post:
            self.send_error(403, "Cross-origin request rejected")
            return False
        if is_post:
            ctype = self.headers.get('Content-Type', '').split(';', 1)[0].strip().lower()
            if ctype != 'application/json':
                self.send_error(415, "Content-Type must be application/json")
                return False
        return True

    def do_GET(self):
        if not self.request_allowed(is_post=False):
            return
        parsed = urllib.parse.urlparse(self.path)
        path = parsed.path

        if path == "/api/state":
            self.send_json(self.get_robot_state_json())
        elif path == "/api/calibration":
            self.send_json(robot_state.snapshot(robot_state.calibration))
        elif path == "/api/console":
            qs = urllib.parse.parse_qs(parsed.query)
            try:
                since = int(qs.get("since", ["0"])[0])
            except ValueError:
                since = 0
            latest, entries = console_log.since(since)
            # since > latest: the server restarted; the client should start over
            self.send_json({"seq": latest, "reset": since > latest, "entries": entries})
        elif path == "/api/serial_ports":
            with robot_state.lock:
                arm = robot_state.arm_link or {}
                arm_port = arm.get('port') if robot_state.commander_alive() and arm.get('transport') == 'serial' else None
            self.send_json({"ports": [{"device": d, "description": desc, "arm": d == arm_port}
                                      for d, desc in list_serial_ports()], "arm_port": arm_port})
        elif path == "/api/sensors/data":
            self.send_json(robot_state.snapshot(robot_state.sensor_state))
        elif path == "/api/quardbot/state":
            with robot_state.lock:
                quard = copy.deepcopy({
                    "host": robot_state.quardbot_host,
                    "online": robot_state.quardbot_online,
                    "mode": robot_state.quardbot_mode,
                    "channels": robot_state.quardbot_channels,
                    "joints": robot_state.quard_joints
                })
            self.send_json(quard)
        elif path.startswith("/meshes/"):
            mesh_filename = urllib.parse.unquote(os.path.basename(path))
            candidates = [
                mesh_filename,
                mesh_filename.replace('+', ' '),
                mesh_filename.replace('+', '_'),
                mesh_filename.replace('_', '+'),
                mesh_filename.replace(' ', '+')
            ]
            found_filepath = None
            search_dirs = [MESH_DIR, QUARD_MESH_DIR, UNNAMED_MESH_DIR, DOGU_MESH_DIR]
            for sdir in search_dirs:
                if not os.path.exists(sdir):
                    continue
                for c in candidates:
                    p = os.path.join(sdir, c)
                    if os.path.exists(p):
                        found_filepath = p
                        break
                if found_filepath:
                    break
            if found_filepath:
                self.send_file(found_filepath, "model/stl")
            else:
                self.send_error(404, f"Mesh {mesh_filename} not found")
        elif path in ("/models/dogu/model.json", "/models/dogu/animations.json"):
            fpath = os.path.join(DOGU_DIR, os.path.basename(path))
            if os.path.exists(fpath):
                self.send_file(fpath, "application/json")
            else:
                self.send_error(404, "Dogu model not built: run scripts/dogu/build_dogu.py")
        elif path in ["/urdf/quard_bot.urdf", "/urdf/quard.urdf"]:
            if os.path.exists(QUARD_URDF_PATH):
                self.send_file(QUARD_URDF_PATH, "application/xml")
            else:
                self.send_error(404, "Quard URDF not found")
        elif path in ["/urdf/arm.urdf", "/urdf/unnamed_gazebo.urdf"]:
            if os.path.exists(URDF_PATH):
                self.send_file(URDF_PATH, "application/xml")
            else:
                self.send_error(404, "URDF not found")
        else:
            super().do_GET()

    def do_POST(self):
        if not self.request_allowed(is_post=True):
            return
        try:
            parsed = urllib.parse.urlparse(self.path)
            path = parsed.path
            length = int(self.headers.get('Content-Length', 0))
            body = self.rfile.read(length).decode('utf-8') if length > 0 else ""

            try:
                data = json.loads(body) if body else {}
            except Exception:
                self.send_json({"status": "error", "message": "invalid JSON body"}, code=400)
                return
            if not isinstance(data, dict):
                self.send_json({"status": "error", "message": "JSON body must be an object"}, code=400)
                return

            if path == "/api/update_joints":
                joints = data.get("joints", {})
                if not isinstance(joints, dict):
                    raise ValueError("'joints' must be an object")
                updates = {}
                for k, v in joints.items():
                    k = JOINT_ALIASES.get(k, k)
                    if k not in ARM_JOINTS:
                        continue  # ignore unknown joints instead of growing state
                    updates[k] = max(0.0, min(180.0, finite_float(v)))
                with robot_state.lock:
                    robot_state.joint_positions.update(updates)
                self.send_json({"status": "ok"})
            elif path == "/api/quardbot/joints":
                joints = data.get("joints")
                channels = data.get("channels")
                quard_updates = {}
                if isinstance(joints, dict):
                    for k, v in joints.items():
                        if k in robot_state.quard_joints:
                            quard_updates[k] = finite_float(v)
                if isinstance(channels, list):
                    if len(channels) != 8:
                        raise ValueError("'channels' must have 8 entries")
                    channels = [max(0, min(180, int(finite_float(c)))) for c in channels]
                else:
                    channels = None
                with robot_state.lock:
                    robot_state.quard_joints.update(quard_updates)
                    if channels is not None:
                        robot_state.quardbot_channels = channels
                self.send_json({"status": "ok"})
            elif path == "/api/save_calibration":
                ok, msg = robot_state.save_calibration(data)
                self.send_json({"status": "ok" if ok else "error", "message": msg})
            elif path == "/api/sync_offset":
                calib_update = {}
                if "joint" in data and "offset" in data:
                    j_name = JOINT_ALIASES.get(data["joint"], data["joint"])
                    off_val = finite_float(data["offset"])
                    calib_update[j_name] = {"sync_offset_deg": off_val}
                elif "offsets" in data and isinstance(data["offsets"], dict):
                    for j_name, off_val in data["offsets"].items():
                        calib_update[JOINT_ALIASES.get(j_name, j_name)] = {"sync_offset_deg": finite_float(off_val)}
                elif "quard_channel" in data and "offset" in data:
                    ch_key = str(data["quard_channel"])
                    calib_update["quard_offsets"] = {ch_key: finite_float(data["offset"])}
                elif "quard_offsets" in data and isinstance(data["quard_offsets"], dict):
                    calib_update["quard_offsets"] = {str(k): finite_float(v) for k, v in data["quard_offsets"].items()}
                elif "imu_offsets" in data and isinstance(data["imu_offsets"], dict):
                    calib_update["imu_offsets"] = {str(k): finite_float(v) for k, v in data["imu_offsets"].items()}

                ok, msg = (robot_state.save_calibration(calib_update) if calib_update else (True, "nothing to save"))
                self.send_json({"status": "ok" if ok else "error", "message": msg,
                                "calibration": robot_state.snapshot(robot_state.calibration)})
            elif path == "/api/tare_pose":
                target = data.get("target", "all")
                ref_pose = data.get("reference_pose", {})
                if not isinstance(ref_pose, dict):
                    raise ValueError("'reference_pose' must be an object")
                calib_update = {}
                with robot_state.lock:
                    raw_p = robot_state.sensor_state.get("raw_pitch")
                    raw_r = robot_state.sensor_state.get("raw_roll")
                    have_imu = raw_p is not None and raw_r is not None
                    if target == "imu" and not have_imu:
                        self.send_json({"status": "error", "message":
                                        "no IMU reading yet: connect the rover first"}, code=409)
                        return
                    if target in ["imu", "all"] and have_imu:   # "all" skips a missing IMU
                        calib_update["imu_offsets"] = {
                            "pitch": raw_p,
                            "roll": raw_r,
                            "yaw": 0.0
                        }
                        robot_state.sensor_state["pitch"] = 0.0
                        robot_state.sensor_state["roll"] = 0.0
                    if target in ["arm", "all"]:
                        # The twin renders (slider + sync_offset). The physical arm is
                        # taken to be in its reference pose right now, so choose the
                        # offset that makes the twin show that pose: ref - current.
                        for j_name in ARM_JOINTS:
                            cur_val = robot_state.joint_positions[j_name]
                            ref_val = ref_pose.get(j_name, robot_state.calibration.get(j_name, {}).get("home_deg", 90.0))
                            calib_update[j_name] = {"sync_offset_deg": round(finite_float(ref_val) - float(cur_val), 1)}
                    if target in ["quard", "all"]:
                        calib_update["quard_offsets"] = {str(i): 0.0 for i in range(8)}

                ok, msg = (robot_state.save_calibration(calib_update) if calib_update else (True, "nothing to save"))
                self.send_json({"status": "ok" if ok else "error", "message": msg,
                                "calibration": robot_state.snapshot(robot_state.calibration)})
            elif path == "/api/toggle_sim":
                with robot_state.lock:
                    new_sim = data.get("simulating", not robot_state.is_simulating)
                    if new_sim and not robot_state.is_simulating:
                        # Capture exact starting positions for seamless ease-in
                        robot_state.sim_blend_start_positions = dict(robot_state.joint_positions)
                        client_joints = data.get("current_joints")
                        if isinstance(client_joints, dict):
                            for jk, jv in client_joints.items():
                                jk = JOINT_ALIASES.get(jk, jk)
                                if jk not in ARM_JOINTS:
                                    continue
                                try:
                                    val = max(0.0, min(180.0, finite_float(jv)))
                                    robot_state.sim_blend_start_positions[jk] = val
                                    robot_state.joint_positions[jk] = val
                                except (ValueError, TypeError):
                                    pass
                        robot_state.sim_blend_start_time = time.time()
                        robot_state.sim_time = 0.0
                    robot_state.is_simulating = bool(new_sim)
                    if "speed" in data:
                        robot_state.sim_speed = max(0.0, min(10.0, finite_float(data["speed"])))
                    simulating = robot_state.is_simulating
                self.send_json({"status": "ok", "simulating": simulating})
            elif path == "/api/comm_mode":
                mode = str(data.get("mode", "WIFI")).upper()
                if mode in ["WIFI", "SERIAL"]:
                    with robot_state.lock:
                        robot_state.comm_mode = mode
                        if ROS2_AVAILABLE:
                            # servo_serial_commander relays MODE to the ESP32 over
                            # both links and switches its own transport to match.
                            robot_state.commander_msgs.append(f"MODE:{mode}")
                    if not ROS2_AVAILABLE:
                        threading.Thread(target=send_comm_mode_udp_broadcast, args=(mode,), daemon=True).start()
                        console_log.add('arm', 'warn', f"MODE:{mode} broadcast over UDP (no ROS 2, so no commander to switch)")
                    elif not robot_state.commander_alive():
                        console_log.add('arm', 'warn', f"MODE:{mode} published, but servo_serial_commander isn't running")
                    self.send_json({"status": "ok", "mode": mode})
                else:
                    self.send_json({"status": "error", "message": "Invalid mode"})
            elif path == "/api/sensors/connect":
                port = str(data.get("port", "")).strip()
                baud = int(data.get("baudrate", 115200))
                if not re.match(r'^(/dev/[A-Za-z0-9_./+-]+|COM\d+)$', port):
                    raise ValueError(f"invalid serial port {port!r}")
                with robot_state.lock:
                    arm = robot_state.arm_link or {}
                    clash = (robot_state.commander_alive() and arm.get('transport') == 'serial'
                             and arm.get('port') == port)
                    if clash and not data.get("force"):
                        self.send_json({"status": "conflict", "message":
                                        f"{port} is the arm ESP32's port (servo_serial_commander has it open). "
                                        "Opening it here too resets that ESP32 and steals its replies."}, code=409)
                        return
                    robot_state.sensor_state["port"] = port
                    robot_state.sensor_state["baudrate"] = baud
                    robot_state.set_sensor_source('serial', f"{port} @ {baud}")
                self.send_json({"status": "ok", "port": port, "baudrate": baud})
            elif path == "/api/sensors/wifi_connect":
                ip = str(data.get("ip", "192.168.1.100")).strip()
                url = sensor_wifi_url(ip)
                with robot_state.lock:
                    robot_state.sensor_state["wifi_ip"] = ip
                    robot_state.set_sensor_source('wifi', url)
                self.send_json({"status": "ok", "ip": ip, "url": url})
            elif path in ("/api/sensors/wifi_disconnect", "/api/sensors/disconnect"):
                with robot_state.lock:
                    if path == "/api/sensors/disconnect" or robot_state.sensor_link['source'] == 'wifi':
                        robot_state.set_sensor_source('off')
                self.send_json({"status": "ok"})
            elif path == "/api/sensors/push_raw":
                raw_text = data.get("raw", "")
                for line in raw_text.splitlines():
                    robot_state.parse_sensor_line(line)
                self.send_json({"status": "ok"})
            elif path == "/api/quardbot/cmd":
                base = self.quard_host(data)
                params = data.get("params", {})
                if not isinstance(params, dict):
                    raise ValueError("'params' must be an object")
                keepalive = bool(data.get("keepalive"))
                query = urllib.parse.urlencode(params)
                key = f"ch{params['ch']}" if 'ch' in params else None
                t0 = time.monotonic()
                try:
                    req = urllib.request.Request(f"{self.quard_request_base(base)}/cmd?{query}")
                    with urllib.request.urlopen(req, timeout=2.0) as resp:
                        reply = resp.read().decode('utf-8', errors='ignore').strip()
                    rtt = (time.monotonic() - t0) * 1000.0
                    with robot_state.lock:
                        robot_state.quardbot_online = True
                        if "mode" in params:
                            robot_state.quardbot_mode = str(params["mode"])
                        robot_state.quard_link['sent'] += 1
                        robot_state.quard_record(True, rtt)
                    if not keepalive:
                        console_log.add('quard', 'tx', f"/cmd?{query}  ←  {reply or 'OK'}  ({rtt:.0f} ms)", key=key)
                    self.send_json({"status": "ok", "reply": reply, "rtt_ms": rtt})
                except urllib.error.HTTPError as e:
                    # The robot answered (e.g. 400 "Bad Args"): it is online but
                    # did not accept these parameters.
                    rtt = (time.monotonic() - t0) * 1000.0
                    try:
                        body = e.read().decode('utf-8', errors='ignore').strip()
                    except Exception:
                        body = ""
                    with robot_state.lock:
                        robot_state.quardbot_online = True
                        robot_state.quard_link['sent'] += 1
                        robot_state.quard_link['failed'] += 1
                        robot_state.quard_record(True, rtt)
                    console_log.add('quard', 'warn', f"/cmd?{query}  ←  refused: HTTP {e.code} {body or e.reason}", key=key)
                    self.send_json({"status": "rejected", "code": e.code, "message": body or str(e.reason)})
                except Exception as e:
                    reason = describe_net_error(e)
                    with robot_state.lock:
                        robot_state.quard_link['sent'] += 1
                        robot_state.quard_link['failed'] += 1
                        robot_state.quard_record(False, error=reason)
                    if keepalive:
                        console_log.add('quard', 'warn', f"Gait keep-alive failed: {reason}", key='keepalive')
                    else:
                        console_log.add('quard', 'error', f"/cmd?{query}  ✗  {reason}", key=key)
                    self.send_json({"status": "error", "message": reason})
            elif path == "/api/quardbot/ping":
                base = self.quard_host(data)
                manual = bool(data.get("manual"))
                parts = urllib.parse.urlsplit(base)
                if manual:
                    console_log.add('quard', 'tx', f"GET {base}/status")
                t0 = time.monotonic()
                try:
                    name = parts.hostname or ""
                    ip = socket.getaddrinfo(name, parts.port or 80, socket.AF_INET, socket.SOCK_STREAM)[0][4][0]
                    t0 = time.monotonic()   # RTT excludes the name lookup
                    netloc = ip + (f":{parts.port}" if parts.port else "")
                    url = urllib.parse.urlunsplit((parts.scheme, netloc, "/status", "", ""))
                    telemetry, note = {}, ""
                    try:
                        with urllib.request.urlopen(urllib.request.Request(url), timeout=2.0) as resp:
                            raw = resp.read().decode('utf-8', errors='ignore')
                        try:
                            telemetry = json.loads(raw)
                        except ValueError:
                            note = raw.strip()[:120]
                    except urllib.error.HTTPError as e:
                        note = f"online (HTTP {e.code} on /status)"   # it answered
                    rtt = (time.monotonic() - t0) * 1000.0
                    with robot_state.lock:
                        robot_state.quardbot_online = True
                        robot_state.quard_resolved = {name: ip} if ip != name else {}
                        robot_state.quard_link['rssi'] = telemetry.get('rssi') if isinstance(telemetry, dict) else None
                        robot_state.quard_record(True, rtt, is_ping=True)
                    if manual:
                        console_log.add('quard', 'rx', json.dumps(telemetry) if telemetry else (note or "online"))
                    self.send_json({"status": "ok", "online": True, "telemetry": telemetry, "rtt_ms": rtt})
                except Exception as e:
                    reason = describe_net_error(e)
                    with robot_state.lock:
                        robot_state.quardbot_online = False
                        robot_state.quard_resolved = {}
                        robot_state.quard_record(False, error=reason, is_ping=True)
                    if manual:
                        console_log.add('quard', 'error', f"No answer from {base}: {reason}")
                    self.send_json({"status": "ok", "online": False, "error": reason})
            elif path == "/api/dogu/ping":
                base = self.dogu_host(data)
                manual = bool(data.get("manual"))
                if manual:
                    console_log.add('dogu', 'tx', f"GET {base}/status")
                t0 = time.monotonic()
                status, note = None, ""
                try:
                    try:
                        with urllib.request.urlopen(f"{base}/status", timeout=2.0) as resp:
                            raw = resp.read().decode('utf-8', errors='ignore')
                        try:
                            reply = json.loads(raw)
                        except ValueError:
                            reply = None
                        if isinstance(reply, dict) and isinstance(reply.get('walking'), bool):
                            status = {'walking': reply['walking'], 'leg': str(reply.get('leg', ''))[:40],
                                      'phase': str(reply.get('phase', ''))[:40]}
                        else:
                            note = f"answered, but not with Dogu's /status JSON: {raw.strip()[:80]!r}"
                    except urllib.error.HTTPError as e:
                        note = f"answered HTTP {e.code} on /status: is this the Dogu firmware?"
                    rtt = (time.monotonic() - t0) * 1000.0
                    with robot_state.lock:
                        if base == robot_state.dogu_host:
                            was = robot_state.dogu_link['online']
                            prev = robot_state.dogu_status
                            if status and prev and status['walking'] != prev['walking']:
                                console_log.add('dogu', 'info' if status['walking'] else 'ok',
                                                "Robot reports: walking" if status['walking'] else "Robot reports: stopped")
                            robot_state.dogu_status = status
                            if status:
                                robot_state.dogu_status_t = time.time()
                            robot_state.dogu_record(True, rtt)
                            if note and not manual and was is not True:
                                console_log.add('dogu', 'warn', note)
                    if manual:
                        console_log.add('dogu', 'rx', json.dumps(status) if status else (note or "online"))
                    self.send_json({"status": "ok", "online": True, "robot": status, "rtt_ms": rtt})
                except Exception as e:
                    reason = describe_net_error(e)
                    with robot_state.lock:
                        if base == robot_state.dogu_host:
                            robot_state.dogu_record(False, error=reason)
                    if manual:
                        console_log.add('dogu', 'error', f"No answer from {base}: {reason}")
                    self.send_json({"status": "ok", "online": False, "error": reason})
            elif path == "/api/dogu/cmd":
                base = self.dogu_host(data)
                cmd = str(data.get("cmd", ""))
                if cmd not in ("start", "stop"):
                    raise ValueError("cmd must be 'start' or 'stop'")
                queued = time.monotonic()
                with robot_state.dogu_cmd_lock:
                    waited = time.monotonic() - queued
                    if cmd == "start" and waited > DOGU_STALE_S:
                        # Walking now would surprise whoever is watching.
                        console_log.add('dogu', 'warn', f"/start not sent: waited {waited:.1f} s behind a slow "
                                                        "request (motion is never sent late)")
                        self.send_json({"status": "dropped", "message": "waited too long"})
                        return
                    t0 = time.monotonic()
                    try:
                        with urllib.request.urlopen(f"{base}/{cmd}", timeout=2.0) as resp:
                            reply = resp.read().decode('utf-8', errors='ignore').strip()[:200]
                        rtt = (time.monotonic() - t0) * 1000.0
                        with robot_state.lock:
                            robot_state.dogu_link['sent'] += 1
                            robot_state.dogu_record(True, rtt)
                        console_log.add('dogu', 'rx', f"{reply or 'OK'}  (/{cmd}, {rtt:.0f} ms)")
                        self.send_json({"status": "ok", "reply": reply, "rtt_ms": rtt})
                    except Exception as e:
                        reason = describe_net_error(e)
                        answered = isinstance(e, urllib.error.HTTPError)
                        with robot_state.lock:
                            robot_state.dogu_link['sent'] += 1
                            robot_state.dogu_link['failed'] += 1
                            if answered:
                                robot_state.dogu_record(True, (time.monotonic() - t0) * 1000.0)
                            else:
                                robot_state.dogu_record(False, error=reason)
                        if cmd == "stop":
                            console_log.add('dogu', 'error', f"/stop failed: {reason}. The dog may still be "
                                                             "walking: stop it from its own page or cut its power.")
                        else:
                            console_log.add('dogu', 'error', f"/{cmd}  ✗  {reason}")
                        self.send_json({"status": "error", "message": reason})
            elif path == "/api/arm/console":
                line = str(data.get("line", "")).strip()
                if not line or len(line) > 200 or "\n" in line:
                    raise ValueError("line must be 1-200 characters, one line")
                if not ROS2_AVAILABLE:
                    console_log.add('arm', 'error', f"Not sent (ROS 2 not available): {line}")
                    self.send_json({"status": "error", "message": "ROS 2 is not available"}, code=503)
                    return
                upper = line.upper()
                prefix = next((k for k in ('MODE:', 'CMD:', 'CALIB:', 'SYNC:', 'LINK:', 'RAW:') if upper.startswith(k)), None)
                payload = (prefix + line[len(prefix):]) if prefix else "RAW:" + line
                with robot_state.lock:
                    robot_state.commander_msgs.append(payload)
                    alive = robot_state.commander_alive()
                if not alive:
                    console_log.add('arm', 'warn', f"{line}: published, but servo_serial_commander isn't running")
                self.send_json({"status": "ok", "sent": payload})
            elif path == "/api/arm/link":
                cmd = str(data.get("cmd", "")).strip()
                if not _LINK_CMD_RE.match(cmd):
                    raise ValueError(f"unsupported link command {cmd!r}")
                if not ROS2_AVAILABLE:
                    self.send_json({"status": "error", "message": "ROS 2 is not available"}, code=503)
                    return
                with robot_state.lock:
                    robot_state.commander_msgs.append(f"LINK:{cmd}")
                    alive = robot_state.commander_alive()
                if not alive:
                    console_log.add('arm', 'warn', f"LINK:{cmd} sent, but servo_serial_commander isn't running")
                self.send_json({"status": "ok"})
            else:
                self.send_error(404)
        except (TypeError, ValueError) as err:
            self.send_json({"status": "error", "message": str(err)}, code=400)
        except Exception as err:
            print(f"[Server] Error handling POST request to {self.path}: {err}")
            self.send_json({"status": "error", "message": str(err)}, code=500)

    @staticmethod
    def quard_request_base(base):
        """base with the host name swapped for the IP the last good ping
        resolved, so commands skip slow .local lookups."""
        parts = urllib.parse.urlsplit(base)
        with robot_state.lock:
            ip = robot_state.quard_resolved.get(parts.hostname or "")
        if not ip:
            return base.rstrip('/')
        netloc = ip + (f":{parts.port}" if parts.port else "")
        return urllib.parse.urlunsplit((parts.scheme, netloc, parts.path, "", "")).rstrip('/')

    @staticmethod
    def dogu_host(data):
        """Dogu base URL from the request (remembered), restricted to http(s)."""
        host = str(data.get("host") or robot_state.dogu_host).strip().rstrip('/')
        if not host.startswith(("http://", "https://")):
            host = "http://" + host
        parsed = urllib.parse.urlsplit(host)
        if parsed.scheme not in ("http", "https") or not parsed.hostname:
            raise ValueError(f"invalid Dogu host: {host!r}")
        with robot_state.lock:
            if robot_state.dogu_host != host:
                robot_state.dogu_link.update(online=None, rtt_ms=None, last_error='')
                robot_state.dogu_status = None
            robot_state.dogu_host = host
        return host

    @staticmethod
    def quard_host(data):
        """Quard Bot base URL from the request (remembered), restricted to http(s)."""
        host = str(data.get("host") or robot_state.quardbot_host).strip()
        if not host.startswith(("http://", "https://")):
            host = "http://" + host
        parsed = urllib.parse.urlparse(host)
        if parsed.scheme not in ("http", "https") or not parsed.netloc:
            raise ValueError(f"invalid Quard Bot host: {host!r}")
        with robot_state.lock:
            if robot_state.quardbot_host != host:
                robot_state.quard_resolved = {}
                robot_state.quard_link.update(online=None, rtt_ms=None, last_error='')
            robot_state.quardbot_host = host
        return host.rstrip('/')

    def send_json(self, obj, code=200):
        data = json.dumps(obj).encode('utf-8')
        self.send_response(code)
        self.send_header('Content-Type', 'application/json')
        self.send_header('Cache-Control', 'no-store')
        self.send_header('Content-Length', str(len(data)))
        self.end_headers()
        self.wfile.write(data)

    def send_file(self, filepath, content_type):
        try:
            with open(filepath, 'rb') as f:
                content = f.read()
            self.send_response(200)
            self.send_header('Content-Type', content_type)
            self.send_header('Content-Length', str(len(content)))
            self.end_headers()
            self.wfile.write(content)
        except Exception as e:
            self.send_error(500, str(e))

    def get_robot_state_json(self):
        # Copy under the lock; serialising live dicts after releasing it could
        # race a concurrent calibration update ("dict changed size").
        with robot_state.lock:
            return copy.deepcopy({
                "joints": robot_state.joint_positions,
                "calibration": robot_state.calibration,
                "simulating": robot_state.is_simulating,
                "sim_speed": robot_state.sim_speed,
                "comm_mode": robot_state.comm_mode,
                "ros_bridge": robot_state.ros_bridge_active,
                "links": robot_state.links_snapshot()
            })

    def log_message(self, format, *args):
        # Suppress verbose HTTP GET logs
        return

if ROS2_AVAILABLE:
    class ROS2BridgeNode(Node):
        def __init__(self):
            super().__init__('web_dashboard_bridge')
            self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)
            self.calib_pub = self.create_publisher(StringMsg, '/servo_calibration', 10)
            self.create_timer(0.02, self.publish_joint_states) # 50 Hz loop
            self.create_timer(0.1, self.flush_commander_msgs)
            self.create_subscription(StringMsg, '/esp32_link_state',
                                     lambda m: robot_state.ingest_arm_link(m.data), LINK_STATE_QOS)
            self.create_subscription(StringMsg, '/esp32_console',
                                     lambda m: robot_state.ingest_arm_console(m.data), CONSOLE_QOS)
            self.create_subscription(StringMsg, '/esp32_connection_status',
                                     lambda m: robot_state.ingest_arm_legacy(m.data), 10)
            # Bring the commander's limits/offsets in line with the file the
            # dashboard is using, in case it loaded a different copy.
            with robot_state.lock:
                robot_state._queue_commander_updates(robot_state.calibration)

        @staticmethod
        def slider_deg_to_rad(jname, deg, sync_deg):
            """Same linear map as calibrated_joint_gui / app.js degToRad.

            The published angle is the twin pose (slider + sync offset);
            servo_serial_commander subtracts the sync offset again, so the
            physical servo receives exactly the slider value.
            """
            cfg = ARM_JOINTS[jname]
            r_min, r_max = cfg['rad_min'], cfg['rad_max']
            return r_min + ((deg + sync_deg) / 180.0) * (r_max - r_min)

        def publish_joint_states(self):
            msg = JointState()
            msg.header.stamp = self.get_clock().now().to_msg()
            names = []
            positions = []

            with robot_state.lock:
                for jname, deg in robot_state.joint_positions.items():
                    sync = float(robot_state.calibration.get(jname, {}).get('sync_offset_deg', 0.0))
                    names.append(jname)
                    positions.append(self.slider_deg_to_rad(jname, deg, sync))
                for jname, rad in robot_state.quard_joints.items():
                    names.append(jname)
                    positions.append(float(rad))

            msg.name = names
            msg.position = positions
            self.joint_pub.publish(msg)

        def flush_commander_msgs(self):
            with robot_state.lock:
                pending, robot_state.commander_msgs = robot_state.commander_msgs, []
            for text in pending:
                self.calib_pub.publish(StringMsg(data=text))

def start_ros2_node():
    if not ROS2_AVAILABLE:
        print("[Server] ⚠️ rclpy not importable (did you source /opt/ros/<distro>/setup.bash?).")
        print("[Server] ⚠️ Running UI-only: nothing will be published to the robots.")
        return
    try:
        rclpy.init()
        node = ROS2BridgeNode()
        robot_state.ros_bridge_active = True
        try:
            rclpy.spin(node)
        finally:
            robot_state.ros_bridge_active = False
        node.destroy_node()
        rclpy.shutdown()
    except Exception as e:
        print(f"[Server] ROS 2 bridge stopped: {e}")


def simulation_loop():
    while True:
        try:
            robot_state.step_simulation()
        except Exception as e:
            print(f"[Server] Simulation step error: {e}")
        time.sleep(0.02)

def sensor_wifi_url(ip):
    cur = (ip or "").strip() or "192.168.1.100"
    if not cur.startswith(("http://", "https://")):
        cur = "http://" + cur
    cur = cur.rstrip('/')
    return cur if cur.endswith('/data') else cur + "/data"


def read_sensor_serial(gen, port, baud):
    """Read the rover over serial until the source changes. Raises on errors."""
    import serial
    with serial.Serial(port, baud, timeout=0.5) as ser:
        with robot_state.lock:
            robot_state.sensor_event(gen, 'waiting')
        while True:
            with robot_state.lock:
                if robot_state.sensor_link['gen'] != gen:
                    return
            raw_bytes = ser.readline()
            if raw_bytes:
                line = raw_bytes.decode('utf-8', errors='ignore').strip()
                if line:
                    robot_state.parse_sensor_line(line)
                    with robot_state.lock:
                        robot_state.sensor_rx(gen)


def sensor_background_loop():
    """Feed rover telemetry from the selected source: Wi-Fi (/data JSON) or serial."""
    wifi_failures = 0
    while True:
        with robot_state.lock:
            lk = robot_state.sensor_link
            src, gen, target = lk['source'], lk['gen'], lk['target']
            port = robot_state.sensor_state.get("port", "/dev/ttyUSB0")
            baud = robot_state.sensor_state.get("baudrate", 115200)
        try:
            if src == 'wifi':
                try:
                    req = urllib.request.Request(target, headers={"User-Agent": "RobotArm-Server/1.0"})
                    with urllib.request.urlopen(req, timeout=2.0) as resp:
                        raw = resp.read().decode('utf-8', errors='ignore')
                    if not robot_state.parse_sensor_json(raw):
                        raise ValueError(f"reply from {target} is not JSON: {raw.strip()[:60]!r}")
                    wifi_failures = 0
                    with robot_state.lock:
                        robot_state.sensor_rx(gen)
                    time.sleep(0.5)
                except Exception as err:
                    wifi_failures += 1
                    if isinstance(err, urllib.error.HTTPError) and err.code == 404:
                        detail = f"HTTP 404: no /data page at {target}"
                    elif isinstance(err, ValueError):
                        detail = str(err)
                    else:
                        detail = describe_net_error(err)
                    with robot_state.lock:
                        robot_state.sensor_event(gen, 'error', detail)
                    time.sleep(min(5.0, 0.5 * (2 ** min(wifi_failures, 4))))
            elif src == 'serial':
                try:
                    read_sensor_serial(gen, port, baud)
                except ImportError:
                    with robot_state.lock:
                        robot_state.sensor_event(gen, 'error', "pyserial is not installed (sudo apt install python3-serial)")
                        robot_state.set_sensor_source('off')
                except Exception as err:
                    with robot_state.lock:
                        robot_state.sensor_event(gen, 'error', explain_serial_error(err))
                    time.sleep(2.0)
            else:
                time.sleep(0.3)
        except Exception as e:
            print(f"[Server] Sensor loop error: {e}")
            time.sleep(1.0)


def parse_args(argv):
    parser = argparse.ArgumentParser(description="Robot WebGL dashboard server")
    parser.add_argument("port_pos", nargs="?", type=int, help=argparse.SUPPRESS)
    parser.add_argument("-p", "--port", type=int, default=int(os.environ.get("PORT", 8000)))
    parser.add_argument("--host", default=os.environ.get("DASHBOARD_HOST", "127.0.0.1"),
                        help="Interface to bind (default 127.0.0.1). Use 0.0.0.0 to allow other "
                             "machines on the network; they get full control of the robots.")
    args, _unknown = parser.parse_known_args(argv)  # tolerate --ros-args etc.
    if args.port_pos is not None:
        args.port = args.port_pos
    return args


def main():
    # Emoji in log lines would crash on consoles without UTF-8 (e.g. Windows cp1252).
    for stream in (sys.stdout, sys.stderr):
        try:
            stream.reconfigure(errors='replace')
        except Exception:
            pass
    args = parse_args(sys.argv[1:])
    DashboardHTTPHandler.loopback_only = args.host in ("127.0.0.1", "localhost", "::1")
    httpd = ThreadedHTTPServer((args.host, args.port), DashboardHTTPHandler)
    print(f"=========================================================")
    print(f" 🚀 Light-Themed Robot Arm WebGL Dashboard Server Started")
    print(f" 🌐 Access Dashboard at: http://localhost:{args.port}")
    print(f" 📁 Project dir: {PROJECT_DIR}")
    if not DashboardHTTPHandler.loopback_only:
        print(f" ⚠️  Bound to {args.host}: anyone on this network can move the robots.")
    print(f"=========================================================")

    for target in (start_ros2_node, sensor_background_loop, simulation_loop):
        threading.Thread(target=target, daemon=True).start()

    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        print("\n[Server] Shutting down...")
    finally:
        httpd.server_close()

if __name__ == '__main__':
    main()

