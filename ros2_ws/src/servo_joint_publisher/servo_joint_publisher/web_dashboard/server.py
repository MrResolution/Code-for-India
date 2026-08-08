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
from http.server import HTTPServer, SimpleHTTPRequestHandler
from socketserver import ThreadingMixIn

import socket
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String as StringMsg

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
URDF_PATH = "/home/sabo/Documents/learn_/Hardware/urdf/unnamed/urdf/unnamed_gazebo.urdf"
MESH_DIR = "/home/sabo/Documents/learn_/Hardware/urdf/unnamed/meshes/stl"
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
        elif path == "/api/quardbot/cmd":
            target_host = data.get("host", robot_state.quardbot_host)
            cmd_url = f"{target_host.rstrip('/')}/cmd"
            params = data.get("params", {})
            try:
                import urllib.request
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
                import urllib.request
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
    rclpy.init()
    node = ROS2BridgeNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

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

    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        print("\n[Server] Shutting down...")
        httpd.shutdown()

if __name__ == '__main__':
    main()
