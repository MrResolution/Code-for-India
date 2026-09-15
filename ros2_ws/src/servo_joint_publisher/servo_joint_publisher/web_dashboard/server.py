#!/usr/bin/env python3
"""
Python Web Dashboard Server & ROS 2 Node for Robotic Arm
--------------------------------------------------------
Features:
  • Built-in ThreadingHTTPServer serving static assets, URDF models, and STL meshes.
  • REST API endpoints for joint state streaming & calibration limit persistence.
  • Subscribes to /publishes /joint_states and /servo_calibration ROS 2 topics at 50 Hz.
"""

import sys
import os
import json
import math
import time
import threading
import urllib.parse
import urllib.request
import re
import random
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

def send_comm_mode_to_esp32(mode_str):
    """Broadcast and unicast MODE command via UDP port 8888 & Serial backup."""
    cmd_bytes = f"MODE:{mode_str}\n".encode('utf-8')
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        sock.settimeout(0.5)
        sock.sendto(cmd_bytes, ('255.255.255.255', 8888))
        sock.sendto(cmd_bytes, ('10.216.192.100', 8888))
        sock.close()
        print(f"[Server] Sent MODE:{mode_str} via UDP")
    except Exception as e:
        print(f"[Server] UDP mode send error: {e}")

    for port in ['/dev/ttyUSB0', '/dev/ttyACM0']:
        if os.path.exists(port):
            try:
                import serial
                s = serial.Serial(port, 115200, timeout=0.1)
                s.write(cmd_bytes)
                s.close()
                print(f"[Server] Sent MODE:{mode_str} via Serial ({port})")
            except Exception as e:
                pass

CALIB_FILE_PATH = "/home/sabo/Documents/learn_/Hardware/servo_calibration.json"
URDF_PATH = "/home/sabo/Documents/learn_/Hardware/urdf/arm/urdf/arm.urdf"
MESH_DIR = "/home/sabo/Documents/learn_/Hardware/urdf/arm/meshes/stl"
QUARD_URDF_PATH = "/home/sabo/Documents/learn_/Hardware/urdf/quard_bot/urdf/quard_bot.urdf"
QUARD_MESH_DIR = "/home/sabo/Documents/learn_/Hardware/urdf/quard_bot/meshes"
STATIC_DIR = os.path.join(os.path.dirname(__file__), "static")

# Shared state between HTTP Server and ROS 2 Node
class RobotState:
    def __init__(self):
        self.lock = threading.Lock()
        self.joint_positions = {
            'turntable_link_joint_dup': 90.0,
            'turntable_link_joint': 59.0,
            'turntable_link_joint_dup_1': 153.1,
            'turntable_link_joint_dup_2': 116.0,
            'turntable_link_joint_dup_3': 90.0,
            'wrist_twist_joint': 90.0
        }
        self.calibration = self.load_calibration()
        self.is_simulating = False
        self.sim_speed = 1.0
        self.sim_time = 0.0
        self.sim_blend_start_time = 0.0
        self.sim_blend_duration = 2.0
        self.sim_blend_start_positions = {}
        self.sim_start_time = time.time()
        self.last_update = time.time()
        # Quard Bot (Sesame Quadruped) State
        self.quardbot_host = "http://sesame-robot.local"
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
        return {}

    def save_calibration(self, new_calib):
        with self.lock:
            self._deep_update(self.calibration, new_calib)
            try:
                with open(CALIB_FILE_PATH, 'w') as f:
                    json.dump(self.calibration, f, indent=2)
                return True
            except Exception as e:
                print(f"[Server] Error saving calibration: {e}")
                return False

robot_state = RobotState()

# Initialize Visual SLAM & ToF Manager
try:
    from slam_service import SLAMManager
    slam_manager = SLAMManager(udp_port=5000)
    slam_manager.start()
    SLAM_AVAILABLE = True
    print("[Server] Initialized and started Visual SLAM service on UDP 5000")
except Exception as e:
    print(f"[Server] Could not initialize SLAMManager: {e}")
    slam_manager = None
    SLAM_AVAILABLE = False

class ThreadedHTTPServer(ThreadingMixIn, HTTPServer):
    """Handle requests in separate threads for zero UI latency."""
    daemon_threads = True

class DashboardHTTPHandler(SimpleHTTPRequestHandler):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, directory=STATIC_DIR, **kwargs)

    def do_GET(self):
        parsed = urllib.parse.urlparse(self.path)
        path = parsed.path

        if path == "/api/state":
            self.send_json(self.get_robot_state_json())
        elif path == "/api/calibration":
            with robot_state.lock:
                self.send_json(robot_state.calibration)
        elif path == "/api/sensors/data":
            with robot_state.lock:
                self.send_json(robot_state.sensor_state)
        elif path == "/api/quardbot/state":
            with robot_state.lock:
                self.send_json({
                    "host": robot_state.quardbot_host,
                    "online": robot_state.quardbot_online,
                    "mode": robot_state.quardbot_mode,
                    "channels": robot_state.quardbot_channels,
                    "joints": robot_state.quard_joints
                })
        elif path == "/api/slam/state":
            if slam_manager:
                self.send_json(slam_manager.get_state())
            else:
                self.send_json({"active": False, "status": "Unavailable"})
        elif path == "/api/slam/map":
            if slam_manager:
                self.send_json(slam_manager.get_map_data())
            else:
                self.send_json({"points": [], "colors": [], "trajectory": [], "pose": []})
        elif path == "/api/slam/frame":
            if slam_manager:
                jpeg = slam_manager.get_latest_hud_jpeg()
                self.send_response(200)
                self.send_header('Content-Type', 'image/jpeg')
                self.send_header('Cache-Control', 'no-cache, no-store, must-revalidate')
                self.send_header('Access-Control-Allow-Origin', '*')
                self.send_header('Content-Length', str(len(jpeg)))
                self.end_headers()
                self.wfile.write(jpeg)
            else:
                self.send_error(404, "SLAM Manager unavailable")
        elif path == "/api/slam/map_2d":
            if slam_manager:
                jpeg = slam_manager.get_latest_map2d_jpeg()
                self.send_response(200)
                self.send_header('Content-Type', 'image/jpeg')
                self.send_header('Cache-Control', 'no-cache, no-store, must-revalidate')
                self.send_header('Access-Control-Allow-Origin', '*')
                self.send_header('Content-Length', str(len(jpeg)))
                self.end_headers()
                self.wfile.write(jpeg)
            else:
                self.send_error(404, "SLAM Manager unavailable")
        elif path == "/api/slam/stream":
            if slam_manager:
                self.send_response(200)
                self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=frame')
                self.send_header('Cache-Control', 'no-cache, private')
                self.send_header('Pragma', 'no-cache')
                self.send_header('Access-Control-Allow-Origin', '*')
                self.end_headers()
                try:
                    while slam_manager.running:
                        jpeg = slam_manager.get_latest_hud_jpeg()
                        self.wfile.write(b"--frame\r\n")
                        self.wfile.write(b"Content-Type: image/jpeg\r\n")
                        self.wfile.write(f"Content-Length: {len(jpeg)}\r\n\r\n".encode('utf-8'))
                        self.wfile.write(jpeg)
                        self.wfile.write(b"\r\n")
                        time.sleep(0.05)
                except (BrokenPipeError, ConnectionResetError):
                    pass
            else:
                self.send_error(404, "SLAM Manager unavailable")
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
            search_dirs = [
                MESH_DIR,
                QUARD_MESH_DIR,
                "/home/sabo/Documents/learn_/Hardware/urdf/unnamed/meshes/stl"
            ]
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
        try:
            parsed = urllib.parse.urlparse(self.path)
            path = parsed.path
            length = int(self.headers.get('Content-Length', 0))
            body = self.rfile.read(length).decode('utf-8') if length > 0 else ""

            try:
                data = json.loads(body) if body else {}
            except Exception:
                data = {}

            if path == "/api/update_joints":
                with robot_state.lock:
                    for k, v in data.get("joints", {}).items():
                        robot_state.joint_positions[k] = float(v)
                    if 'wrist_twist_joint' in data.get("joints", {}):
                        robot_state.joint_positions['turntable_link_joint_dup_3'] = float(data["joints"]['wrist_twist_joint'])
                    elif 'turntable_link_joint_dup_3' in data.get("joints", {}):
                        robot_state.joint_positions['wrist_twist_joint'] = float(data["joints"]['turntable_link_joint_dup_3'])
                self.send_json({"status": "ok"})
            elif path == "/api/quardbot/joints":
                with robot_state.lock:
                    if "joints" in data and isinstance(data["joints"], dict):
                        for k, v in data["joints"].items():
                            robot_state.quard_joints[k] = float(v)
                    if "channels" in data and isinstance(data["channels"], list):
                        robot_state.quardbot_channels = data["channels"]
                self.send_json({"status": "ok"})
            elif path == "/api/save_calibration":
                success = robot_state.save_calibration(data)
                self.send_json({"status": "ok" if success else "error"})
            elif path == "/api/sync_offset":
                calib_update = {}
                if "joint" in data and "offset" in data:
                    j_name = data["joint"]
                    off_val = float(data["offset"])
                    calib_update[j_name] = {"sync_offset_deg": off_val}
                elif "offsets" in data and isinstance(data["offsets"], dict):
                    for j_name, off_val in data["offsets"].items():
                        calib_update[j_name] = {"sync_offset_deg": float(off_val)}
                elif "quard_channel" in data and "offset" in data:
                    ch_key = str(data["quard_channel"])
                    calib_update["quard_offsets"] = {ch_key: float(data["offset"])}
                elif "quard_offsets" in data and isinstance(data["quard_offsets"], dict):
                    calib_update["quard_offsets"] = {str(k): float(v) for k, v in data["quard_offsets"].items()}
                elif "imu_offsets" in data and isinstance(data["imu_offsets"], dict):
                    calib_update["imu_offsets"] = {str(k): float(v) for k, v in data["imu_offsets"].items()}

                if calib_update:
                    robot_state.save_calibration(calib_update)
                self.send_json({"status": "ok", "calibration": robot_state.calibration})
            elif path == "/api/tare_pose":
                target = data.get("target", "all")
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
                        ref_pose = data.get("reference_pose", {})
                        for j_name, cur_val in robot_state.joint_positions.items():
                            ref_val = ref_pose.get(j_name, robot_state.calibration.get(j_name, {}).get("home_deg", 90.0))
                            calib_update[j_name] = {"sync_offset_deg": round(float(cur_val) - float(ref_val), 1)}
                    if target in ["quard", "all"]:
                        calib_update["quard_offsets"] = {str(i): 0.0 for i in range(8)}

                if calib_update:
                    robot_state.save_calibration(calib_update)
                self.send_json({"status": "ok", "calibration": robot_state.calibration})
            elif path == "/api/toggle_sim":
                with robot_state.lock:
                    new_sim = data.get("simulating", not robot_state.is_simulating)
                    if new_sim and not robot_state.is_simulating:
                        # Capture exact starting positions for seamless ease-in
                        robot_state.sim_blend_start_positions = dict(robot_state.joint_positions)
                        client_joints = data.get("current_joints")
                        if isinstance(client_joints, dict):
                            for jk, jv in client_joints.items():
                                try:
                                    val = float(jv)
                                    robot_state.sim_blend_start_positions[jk] = val
                                    robot_state.joint_positions[jk] = val
                                except (ValueError, TypeError):
                                    pass
                        robot_state.sim_blend_start_time = time.time()
                        robot_state.sim_time = 0.0
                    robot_state.is_simulating = new_sim
                    if "speed" in data:
                        robot_state.sim_speed = float(data["speed"])
                self.send_json({"status": "ok", "simulating": robot_state.is_simulating})
            elif path == "/api/comm_mode":
                mode = str(data.get("mode", "WIFI")).upper()
                if mode in ["WIFI", "SERIAL"]:
                    with robot_state.lock:
                        robot_state.comm_mode = mode
                    threading.Thread(target=send_comm_mode_to_esp32, args=(mode,), daemon=True).start()
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
                target_host = data.get("host", robot_state.quardbot_host)
                cmd_url = f"{target_host.rstrip('/')}/cmd"
                params = data.get("params", {})
                try:
                    query_str = urllib.parse.urlencode(params)
                    full_url = f"{cmd_url}?{query_str}"
                    req = urllib.request.Request(full_url)
                    with urllib.request.urlopen(req, timeout=2.0) as resp:
                        reply = resp.read().decode('utf-8', errors='ignore')
                        with robot_state.lock:
                            robot_state.quardbot_online = True
                        self.send_json({"status": "ok", "reply": reply})
                except Exception as e:
                    self.send_json({"status": "error", "message": str(e)})
            elif path == "/api/quardbot/ping":
                target_host = data.get("host", robot_state.quardbot_host)
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
            elif path == "/api/slam/toggle":
                if slam_manager:
                    if slam_manager.running:
                        slam_manager.stop()
                    else:
                        slam_manager.start()
                    self.send_json({"status": "ok", "active": slam_manager.running, "state": slam_manager.get_state()})
                else:
                    self.send_json({"status": "error", "message": "SLAM service not available"})
            elif path == "/api/slam/reset":
                if slam_manager:
                    slam_manager.reset_map()
                    self.send_json({"status": "ok", "state": slam_manager.get_state()})
                else:
                    self.send_json({"status": "error", "message": "SLAM service not available"})
            elif path == "/api/slam/connect":
                target = data.get("target") or data.get("ip") or "10.88.106.30"
                if slam_manager:
                    res = slam_manager.connect_to(target)
                    self.send_json({"status": "ok", "target": target, "state": slam_manager.get_state(), "result": res})
                else:
                    self.send_json({"status": "error", "message": "SLAM service not available"})
            elif path == "/api/slam/disconnect":
                if slam_manager:
                    res = slam_manager.disconnect()
                    self.send_json({"status": "ok", "state": slam_manager.get_state(), "result": res})
                else:
                    self.send_json({"status": "error", "message": "SLAM service not available"})
            else:
                self.send_error(404)
        except Exception as err:
            print(f"[Server] Error handling POST request to {self.path}: {err}")
            self.send_json({"status": "error", "message": str(err)})

    def send_json(self, obj):
        data = json.dumps(obj).encode('utf-8')
        self.send_response(200)
        self.send_header('Content-Type', 'application/json')
        self.send_header('Access-Control-Allow-Origin', '*')
        self.send_header('Content-Length', str(len(data)))
        self.end_headers()
        self.wfile.write(data)

    def send_file(self, filepath, content_type):
        try:
            with open(filepath, 'rb') as f:
                content = f.read()
            self.send_response(200)
            self.send_header('Content-Type', content_type)
            self.send_header('Access-Control-Allow-Origin', '*')
            self.send_header('Content-Length', str(len(content)))
            self.end_headers()
            self.wfile.write(content)
        except Exception as e:
            self.send_error(500, str(e))

    def get_robot_state_json(self):
        with robot_state.lock:
            now = time.time()
            if robot_state.is_simulating:
                dt = now - robot_state.last_update
                dt = min(max(dt, 0.0), 0.1)
                robot_state.sim_time += dt * robot_state.sim_speed
                t = robot_state.sim_time

                # Smooth cosine ease-in S-curve blend (C1-continuous)
                blend_elapsed = now - robot_state.sim_blend_start_time
                progress = min(1.0, max(0.0, blend_elapsed / robot_state.sim_blend_duration))
                w = 0.5 - 0.5 * math.cos(progress * math.pi)

                sim_targets = {
                    'turntable_link_joint_dup': 90.0 + 45.0 * math.sin(0.3 * t),
                    'turntable_link_joint': 59.0 + 20.0 * math.sin(0.4 * t + 1.0),
                    'turntable_link_joint_dup_1': 110.0 + 35.0 * math.sin(0.5 * t + 2.0),
                    'turntable_link_joint_dup_2': 116.0 + 25.0 * math.sin(0.6 * t + 1.5),
                    'turntable_link_joint_dup_3': 90.0 + 35.0 * math.sin(0.4 * t + 0.5),
                    'wrist_twist_joint': 90.0 + 35.0 * math.sin(0.4 * t + 0.5),
                }

                for jk, target_val in sim_targets.items():
                    start_val = robot_state.sim_blend_start_positions.get(
                        jk, robot_state.joint_positions.get(jk, target_val)
                    )
                    blended = (1.0 - w) * start_val + w * target_val
                    cal = robot_state.calibration.get(jk, {})
                    min_lim = cal.get("min_deg", 0.0)
                    max_lim = cal.get("max_deg", 180.0)
                    robot_state.joint_positions[jk] = max(min_lim, min(max_lim, round(blended, 2)))

            robot_state.last_update = now

            return {
                "joints": robot_state.joint_positions,
                "calibration": robot_state.calibration,
                "simulating": robot_state.is_simulating,
                "sim_speed": robot_state.sim_speed
            }

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

        def publish_joint_states(self):
            msg = JointState()
            msg.header.stamp = self.get_clock().now().to_msg()
            names = []
            positions = []

            with robot_state.lock:
                for jname, deg in robot_state.joint_positions.items():
                    names.append(jname)
                    # Map 0..180 deg to URDF rad bounds
                    rad = (deg - 90.0) * (math.pi / 180.0)
                    positions.append(rad)
                for jname, rad in robot_state.quard_joints.items():
                    names.append(jname)
                    positions.append(float(rad))

            msg.name = names
            msg.position = positions
            self.joint_pub.publish(msg)

def start_ros2_node():
    if not ROS2_AVAILABLE:
        print("[Server] ROS 2 dependencies not found. Skipping ROS 2 bridge node.")
        return
    rclpy.init()
    node = ROS2BridgeNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

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

def main():
    port = int(os.environ.get("PORT", 8000))
    if len(sys.argv) > 2 and sys.argv[1] in ["--port", "-p"]:
        port = int(sys.argv[2])
    elif len(sys.argv) > 1 and sys.argv[1].isdigit():
        port = int(sys.argv[1])
    server_address = ('', port)
    httpd = ThreadedHTTPServer(server_address, DashboardHTTPHandler)
    print(f"=========================================================")
    print(f" 🚀 Light-Themed Robot Arm WebGL Dashboard Server Started")
    print(f" 🌐 Access Dashboard at: http://localhost:{port}")
    print(f"=========================================================")

    ros_thread = threading.Thread(target=start_ros2_node, daemon=True)
    ros_thread.start()

    sensor_thread = threading.Thread(target=sensor_background_loop, daemon=True)
    sensor_thread.start()

    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        print("\n[Server] Shutting down...")
        if slam_manager:
            slam_manager.stop()
        httpd.shutdown()

if __name__ == '__main__':
    main()

