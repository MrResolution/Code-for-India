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

CALIB_FILE_PATH = "/home/chakradhar/Documents/Hardware/servo_calibration.json"
URDF_PATH = "/home/chakradhar/Documents/Hardware/urdf/unnamed/urdf/unnamed_gazebo.urdf"
MESH_DIR = "/home/chakradhar/Documents/Hardware/urdf/unnamed/meshes/stl"
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
            'wrist_twist_joint': 90.0,
            'turntable_link_joint_dup_4': 90.0
        }
        self.calibration = self.load_calibration()
        self.is_simulating = False
        self.sim_speed = 1.0
        self.sim_start_time = time.time()
        self.last_update = time.time()
        # Quard Bot (Sesame Quadruped) State
        self.quardbot_host = "http://sesame-robot.local"
        self.quardbot_online = False
        self.quardbot_mode = "stand"
        self.quardbot_channels = [90, 0, 0, 90, 90, 0, 90, 0]

        # ESP32 Sensor Node (DHT11, MPU6050, MQ-5) State
        self.sensor_state = {
            "temperature": 24.5,
            "humidity": 48.0,
            "dht_error": False,
            "accel": {"x": 0.12, "y": -0.05, "z": 9.81},
            "gyro": {"x": 0.01, "y": 0.00, "z": -0.02},
            "pitch": -0.3,
            "roll": 0.7,
            "gas": 415,
            "air_quality": "Clean",
            "connected": False,
            "simulating": True,
            "port": "/dev/ttyUSB0",
            "baudrate": 115200,
            "last_seen": time.time(),
            "raw_log": []
        }

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
                m_temp = re.search(r"Temperature:\s*([-\d\.]+)", line)
                if m_temp:
                    self.sensor_state["temperature"] = float(m_temp.group(1))
                    self.sensor_state["dht_error"] = False

                m_hum = re.search(r"Humidity:\s*([-\d\.]+)", line)
                if m_hum:
                    self.sensor_state["humidity"] = float(m_hum.group(1))
                    self.sensor_state["dht_error"] = False

            # 2. MPU6050 parsing
            m_accel = re.search(r"Accel X:\s*([-\d\.]+)\s*\|\s*Y:\s*([-\d\.]+)\s*\|\s*Z:\s*([-\d\.]+)", line)
            if m_accel:
                ax = float(m_accel.group(1))
                ay = float(m_accel.group(2))
                az = float(m_accel.group(3))
                self.sensor_state["accel"] = {"x": ax, "y": ay, "z": az}

                # Compute estimated Pitch & Roll
                if az != 0 or ay != 0 or ax != 0:
                    pitch_rad = math.atan2(ay, math.sqrt(ax**2 + az**2))
                    roll_rad = math.atan2(-ax, az)
                    self.sensor_state["pitch"] = round(pitch_rad * (180.0 / math.pi), 1)
                    self.sensor_state["roll"] = round(roll_rad * (180.0 / math.pi), 1)

            m_gyro = re.search(r"Gyro X:\s*([-\d\.]+)\s*\|\s*Y:\s*([-\d\.]+)\s*\|\s*Z:\s*([-\d\.]+)", line)
            if m_gyro:
                gx = float(m_gyro.group(1))
                gy = float(m_gyro.group(2))
                gz = float(m_gyro.group(3))
                self.sensor_state["gyro"] = {"x": gx, "y": gy, "z": gz}

            # 3. MQ-5 parsing
            m_gas = re.search(r"MQ-5 Analog:\s*(\d+)", line)
            if m_gas:
                gas_val = int(m_gas.group(1))
                self.sensor_state["gas"] = gas_val
                if gas_val < 600:
                    self.sensor_state["air_quality"] = "Clean"
                elif gas_val < 1500:
                    self.sensor_state["air_quality"] = "Moderate"
                else:
                    self.sensor_state["air_quality"] = "Danger"

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
            self.calibration.update(new_calib)
            try:
                with open(CALIB_FILE_PATH, 'w') as f:
                    json.dump(self.calibration, f, indent=2)
                return True
            except Exception as e:
                print(f"[Server] Error saving calibration: {e}")
                return False

robot_state = RobotState()

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
        elif path.startswith("/meshes/"):
            mesh_filename = os.path.basename(path)
            mesh_filepath = os.path.join(MESH_DIR, mesh_filename)
            if os.path.exists(mesh_filepath):
                self.send_file(mesh_filepath, "model/stl")
            else:
                self.send_error(404, f"Mesh {mesh_filename} not found")
        elif path == "/urdf/unnamed_gazebo.urdf":
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
                self.send_json({"status": "ok"})
            elif path == "/api/save_calibration":
                success = robot_state.save_calibration(data)
                self.send_json({"status": "ok" if success else "error"})
            elif path == "/api/toggle_sim":
                with robot_state.lock:
                    robot_state.is_simulating = data.get("simulating", not robot_state.is_simulating)
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
                    robot_state.sensor_state["simulating"] = False
                self.send_json({"status": "ok", "port": port, "baudrate": baud})
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
                    req = urllib.request.Request(target_host.rstrip('/') + "/")
                    with urllib.request.urlopen(req, timeout=1.5) as resp:
                        with robot_state.lock:
                            robot_state.quardbot_online = True
                        self.send_json({"status": "ok", "online": True})
                except Exception as e:
                    with robot_state.lock:
                        robot_state.quardbot_online = False
                    self.send_json({"status": "ok", "online": False, "error": str(e)})
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
                robot_state.sim_start_time += dt * robot_state.sim_speed
                t = robot_state.sim_start_time
                robot_state.joint_positions['turntable_link_joint_dup'] = 90.0 + 45.0 * math.sin(0.3 * t)
                robot_state.joint_positions['turntable_link_joint'] = 59.0 + 20.0 * math.sin(0.4 * t + 1.0)
                robot_state.joint_positions['turntable_link_joint_dup_1'] = 153.1 + 30.0 * math.sin(0.5 * t + 2.0)

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
    """Background thread generating simulation telemetry or reading hardware serial data."""
    while True:
        try:
            with robot_state.lock:
                simulating = robot_state.sensor_state["simulating"]
                connected = robot_state.sensor_state["connected"]
                port = robot_state.sensor_state["port"]
                baud = robot_state.sensor_state["baudrate"]

            if simulating:
                t = time.time()
                temp = round(24.5 + 1.2 * math.sin(0.2 * t), 2)
                hum = round(48.0 + 3.5 * math.cos(0.15 * t), 2)
                ax = round(0.4 * math.sin(0.5 * t), 2)
                ay = round(0.3 * math.cos(0.4 * t), 2)
                az = round(9.81 + 0.1 * math.sin(0.8 * t), 2)
                gx = round(0.02 * math.sin(0.6 * t), 3)
                gy = round(0.01 * math.cos(0.5 * t), 3)
                gz = round(-0.01 * math.sin(0.3 * t), 3)
                gas = int(410 + 25 * math.sin(0.1 * t) + random.randint(-4, 4))

                block = [
                    "========== SENSOR DATA ==========",
                    f"Temperature: {temp:.2f} °C",
                    f"Humidity: {hum:.2f} %",
                    "MPU6050:",
                    f"Accel X: {ax:.2f} | Y: {ay:.2f} | Z: {az:.2f} m/s^2",
                    f"Gyro X: {gx:.3f} | Y: {gy:.3f} | Z: {gz:.3f} rad/s",
                    f"MQ-5 Analog: {gas}",
                    "================================="
                ]

                for line in block:
                    robot_state.parse_sensor_line(line)

            elif connected and os.path.exists(port):
                try:
                    import serial
                    with serial.Serial(port, baud, timeout=1.0) as s:
                        while not robot_state.sensor_state["simulating"]:
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
    port = 8000
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
        httpd.shutdown()

if __name__ == '__main__':
    main()

