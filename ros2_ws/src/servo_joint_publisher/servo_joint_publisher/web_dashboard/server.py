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
import urllib.parse
import urllib.request
import re
import random
import argparse
from http.server import HTTPServer, SimpleHTTPRequestHandler
from socketserver import ThreadingMixIn

import socket
try:
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import JointState
    from std_msgs.msg import String as StringMsg
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
        self.sensor_state = {
            "temperature": 24.5,
            "temp_dht": 24.5,
            "humidity": 48.0,
            "temp_bmp": 24.8,
            "pressure": 1013.25,
            "dht_error": False,
            "accel": {"x": 0.12, "y": -0.05, "z": 9.81},
            "gyro": {"x": 0.01, "y": 0.00, "z": -0.02},
            "ax": 120, "ay": -50, "az": 16384,
            "gx": 10, "gy": 0, "gz": -20,
            "pitch": -0.3,
            "roll": 0.7,
            "raw_pitch": -0.3,
            "raw_roll": 0.7,
            "gas": 415,
            "water": 120,
            "flame": 1,
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
            "last_seen": time.time(),
            "raw_log": []
        }

    def parse_sensor_json(self, raw_str):
        try:
            d = json.loads(raw_str)
        except Exception:
            return

        with self.lock:
            self.sensor_state["raw_log"].append(f"[JSON] {raw_str[:90]}...")
            if len(self.sensor_state["raw_log"]) > 100:
                self.sensor_state["raw_log"].pop(0)

            self.sensor_state["last_seen"] = time.time()

            t_dht = float(d.get("temp_dht", self.sensor_state["temp_dht"]))
            hum = float(d.get("humidity", self.sensor_state["humidity"]))
            t_bmp = float(d.get("temp_bmp", self.sensor_state["temp_bmp"]))
            pres = float(d.get("pressure", self.sensor_state["pressure"]))
            gas_val = int(d.get("air", d.get("gas", self.sensor_state["gas"])))
            water_val = int(d.get("water", self.sensor_state["water"]))
            flame_val = int(d.get("flame", self.sensor_state["flame"]))

            self.sensor_state["temp_dht"] = t_dht
            self.sensor_state["temperature"] = t_dht
            self.sensor_state["humidity"] = hum
            self.sensor_state["temp_bmp"] = t_bmp
            self.sensor_state["pressure"] = pres
            self.sensor_state["gas"] = gas_val
            self.sensor_state["water"] = water_val
            self.sensor_state["flame"] = flame_val

            if gas_val < 600:
                self.sensor_state["air_quality"] = "Clean"
            elif gas_val < 1500:
                self.sensor_state["air_quality"] = "Moderate"
            else:
                self.sensor_state["air_quality"] = "Danger"

            ax = float(d.get("ax", 0))
            ay = float(d.get("ay", 0))
            az = float(d.get("az", 16384))
            gx = float(d.get("gx", 0))
            gy = float(d.get("gy", 0))
            gz = float(d.get("gz", 0))

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

            if az_mps2 != 0 or ay_mps2 != 0 or ax_mps2 != 0:
                pitch_rad = math.atan2(ay_mps2, math.sqrt(ax_mps2**2 + az_mps2**2))
                roll_rad = math.atan2(-ax_mps2, az_mps2)
                raw_p = round(pitch_rad * (180.0 / math.pi), 1)
                raw_r = round(roll_rad * (180.0 / math.pi), 1)
                self.sensor_state["raw_pitch"] = raw_p
                self.sensor_state["raw_roll"] = raw_r
                imu_t = self.calibration.get("imu_offsets", {})
                p_off = float(imu_t.get("pitch", 0.0))
                r_off = float(imu_t.get("roll", 0.0))
                self.sensor_state["pitch"] = round(raw_p - p_off, 1)
                self.sensor_state["roll"] = round(raw_r - r_off, 1)

            self.sensor_state["lat"] = float(d.get("lat", self.sensor_state["lat"]))
            self.sensor_state["lng"] = float(d.get("lng", self.sensor_state["lng"]))
            self.sensor_state["gps_valid"] = bool(d.get("gps_valid", False))
            self.sensor_state["gps_sats"] = int(d.get("gps_sats", 0))
            self.sensor_state["alert"] = str(d.get("alert", "OK"))
            self.sensor_state["sd_ok"] = bool(d.get("sd_ok", True))
            self.sensor_state["uptime_ms"] = int(d.get("uptime_ms", 0))

    def parse_sensor_line(self, line):
        line = line.strip()
        if not line:
            return

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
            search_dirs = [MESH_DIR, QUARD_MESH_DIR, UNNAMED_MESH_DIR]
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
                    if target in ["imu", "all"]:
                        raw_p = robot_state.sensor_state.get("raw_pitch", robot_state.sensor_state.get("pitch", 0.0))
                        raw_r = robot_state.sensor_state.get("raw_roll", robot_state.sensor_state.get("roll", 0.0))
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
                    self.send_json({"status": "ok", "mode": mode})
                else:
                    self.send_json({"status": "error", "message": "Invalid mode"})
            elif path == "/api/sensors/sim":
                with robot_state.lock:
                    robot_state.sensor_state["simulating"] = bool(data.get("simulating", True))
                self.send_json({"status": "ok", "simulating": robot_state.sensor_state["simulating"]})
            elif path == "/api/sensors/connect":
                port = data.get("port", "/dev/ttyUSB0")
                baud = int(data.get("baudrate", 115200))
                with robot_state.lock:
                    robot_state.sensor_state["port"] = port
                    robot_state.sensor_state["baudrate"] = baud
                    robot_state.sensor_state["connected"] = True
                    robot_state.sensor_state["wifi_connected"] = False
                    robot_state.sensor_state["simulating"] = False
                self.send_json({"status": "ok", "port": port, "baudrate": baud})
            elif path == "/api/sensors/wifi_connect":
                ip = data.get("ip", "192.168.1.100").strip()
                with robot_state.lock:
                    robot_state.sensor_state["wifi_ip"] = ip
                    robot_state.sensor_state["wifi_connected"] = True
                    robot_state.sensor_state["connected"] = False
                    robot_state.sensor_state["simulating"] = False
                self.send_json({"status": "ok", "ip": ip})
            elif path == "/api/sensors/wifi_disconnect":
                with robot_state.lock:
                    robot_state.sensor_state["wifi_connected"] = False
                self.send_json({"status": "ok"})
            elif path == "/api/sensors/push_raw":
                raw_text = data.get("raw", "")
                for line in raw_text.splitlines():
                    robot_state.parse_sensor_line(line)
                self.send_json({"status": "ok"})
            elif path == "/api/quardbot/cmd":
                target_host = self.quard_host(data)
                cmd_url = f"{target_host.rstrip('/')}/cmd"
                params = data.get("params", {})
                if not isinstance(params, dict):
                    raise ValueError("'params' must be an object")
                try:
                    query_str = urllib.parse.urlencode(params)
                    full_url = f"{cmd_url}?{query_str}"
                    req = urllib.request.Request(full_url)
                    with urllib.request.urlopen(req, timeout=2.0) as resp:
                        reply = resp.read().decode('utf-8', errors='ignore')
                        with robot_state.lock:
                            robot_state.quardbot_online = True
                            if "mode" in params:
                                robot_state.quardbot_mode = str(params["mode"])
                        self.send_json({"status": "ok", "reply": reply})
                except Exception as e:
                    self.send_json({"status": "error", "message": str(e)})
            elif path == "/api/quardbot/ping":
                target_host = self.quard_host(data)
                try:
                    req = urllib.request.Request(target_host.rstrip('/') + "/status")
                    with urllib.request.urlopen(req, timeout=1.5) as resp:
                        raw = resp.read().decode('utf-8', errors='ignore')
                        try:
                            telemetry = json.loads(raw)
                        except Exception:
                            telemetry = {"status": "ok"}
                        with robot_state.lock:
                            robot_state.quardbot_online = True
                        self.send_json({"status": "ok", "online": True, "telemetry": telemetry})
                except Exception as e:
                    try:
                        req2 = urllib.request.Request(target_host.rstrip('/') + "/")
                        with urllib.request.urlopen(req2, timeout=1.2) as resp:
                            with robot_state.lock:
                                robot_state.quardbot_online = True
                            self.send_json({"status": "ok", "online": True, "telemetry": {"status": "ok"}})
                    except Exception as e2:
                        with robot_state.lock:
                            robot_state.quardbot_online = False
                        self.send_json({"status": "ok", "online": False, "error": str(e2)})
            else:
                self.send_error(404)
        except (TypeError, ValueError) as err:
            self.send_json({"status": "error", "message": str(err)}, code=400)
        except Exception as err:
            print(f"[Server] Error handling POST request to {self.path}: {err}")
            self.send_json({"status": "error", "message": str(err)}, code=500)

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
            robot_state.quardbot_host = host
        return host

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
                "sim_speed": robot_state.sim_speed
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
        rclpy.spin(node)
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

def sensor_background_loop():
    """Background thread polling Wi-Fi JSON endpoint, generating simulation telemetry, or reading hardware serial."""
    while True:
        try:
            with robot_state.lock:
                simulating = robot_state.sensor_state.get("simulating", True)
                connected = robot_state.sensor_state.get("connected", False)
                wifi_connected = robot_state.sensor_state.get("wifi_connected", False)
                wifi_ip = robot_state.sensor_state.get("wifi_ip", "192.168.1.100")
                port = robot_state.sensor_state.get("port", "/dev/ttyUSB0")
                baud = robot_state.sensor_state.get("baudrate", 115200)

            if wifi_connected:
                cur_ip = wifi_ip.strip()
                if not cur_ip.startswith("http://") and not cur_ip.startswith("https://"):
                    url = f"http://{cur_ip}/data"
                elif cur_ip.endswith("/data"):
                    url = cur_ip
                else:
                    url = f"{cur_ip.rstrip('/')}/data"

                try:
                    req = urllib.request.Request(url, headers={"User-Agent": "RobotArm-Server/1.0"})
                    with urllib.request.urlopen(req, timeout=1.5) as resp:
                        if resp.status == 200:
                            raw = resp.read().decode('utf-8', errors='ignore')
                            robot_state.parse_sensor_json(raw)
                except Exception as err:
                    with robot_state.lock:
                        robot_state.sensor_state["raw_log"].append(f"[Wi-Fi Error] {err}")
                        if len(robot_state.sensor_state["raw_log"]) > 100:
                            robot_state.sensor_state["raw_log"].pop(0)

            elif simulating:
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

                with robot_state.lock:
                    robot_state.sensor_state["temp_dht"] = temp_d
                    robot_state.sensor_state["temperature"] = temp_d
                    robot_state.sensor_state["humidity"] = hum
                    robot_state.sensor_state["temp_bmp"] = temp_b
                    robot_state.sensor_state["pressure"] = pres
                    robot_state.sensor_state["gas"] = gas
                    robot_state.sensor_state["water"] = water
                    robot_state.sensor_state["flame"] = 1
                    robot_state.sensor_state["alert"] = "OK"
                    robot_state.sensor_state["accel"] = {"x": ax, "y": ay, "z": az}
                    robot_state.sensor_state["gyro"] = {"x": gx, "y": gy, "z": gz}
                    robot_state.sensor_state["lat"] = 17.385044
                    robot_state.sensor_state["lng"] = 78.486671
                    robot_state.sensor_state["gps_valid"] = True
                    robot_state.sensor_state["gps_sats"] = 7
                    robot_state.sensor_state["sd_ok"] = True
                    robot_state.sensor_state["uptime_ms"] = uptime

                    pitch_rad = math.atan2(ay, math.sqrt(ax**2 + az**2))
                    roll_rad = math.atan2(-ax, az)
                    raw_p = round(pitch_rad * (180.0 / math.pi), 1)
                    raw_r = round(roll_rad * (180.0 / math.pi), 1)
                    robot_state.sensor_state["raw_pitch"] = raw_p
                    robot_state.sensor_state["raw_roll"] = raw_r
                    imu_t = robot_state.calibration.get("imu_offsets", {})
                    p_off = float(imu_t.get("pitch", 0.0))
                    r_off = float(imu_t.get("roll", 0.0))
                    robot_state.sensor_state["pitch"] = round(raw_p - p_off, 1)
                    robot_state.sensor_state["roll"] = round(raw_r - r_off, 1)

                    for line in block:
                        robot_state.sensor_state["raw_log"].append(line)
                    if len(robot_state.sensor_state["raw_log"]) > 100:
                        robot_state.sensor_state["raw_log"] = robot_state.sensor_state["raw_log"][-100:]

            elif connected and os.path.exists(port):
                try:
                    import serial
                    with serial.Serial(port, baud, timeout=1.0) as s:
                        while not robot_state.sensor_state["simulating"] and not robot_state.sensor_state.get("wifi_connected", False):
                            raw_bytes = s.readline()
                            if raw_bytes:
                                line = raw_bytes.decode('utf-8', errors='ignore')
                                robot_state.parse_sensor_line(line)
                except Exception:
                    with robot_state.lock:
                        robot_state.sensor_state["connected"] = False
        except Exception:
            pass

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

