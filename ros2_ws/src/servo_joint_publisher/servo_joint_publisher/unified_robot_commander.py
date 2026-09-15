#!/usr/bin/env python3
"""
ROS 2 Node: unified_robot_commander
-----------------------------------
Unified fleet communication coordinator for:
  - Robotic Arm (6-DOF)
  - Sesame Quadruped (8-DOF)
  - BTS7960 Motor Rover
  - Sensor Rover (DHT11 + MPU6050 + MQ-5)

Features:
  - UDP Broadcast Auto-Discovery via PING on port 8888
  - Live fleet status tracking & publishing on /fleet/status
  - Telemetry streaming & publishing on /fleet/telemetry
  - Centralized command routing via /fleet/cmd (e.g. 'sesame:CMD:move=forward')
  - Backward-compatible /joint_states forwarding for the Robotic Arm
"""

import json
import math
import os
import socket
import threading
import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String as StringMsg

CALIB_FILE_PATH = "/home/sabo/Documents/learn_/Hardware/servo_calibration.json"


class FleetRobot:
    def __init__(self, key, display_name, default_ip=None, udp_port=8888):
        self.key = key
        self.display_name = display_name
        self.ip = default_ip
        self.port = udp_port
        self.is_online = False
        self.last_seen = 0.0
        self.status_data = {}
        self.mode = "UNKNOWN"

    def mark_seen(self, ip, port=None):
        self.ip = ip
        if port:
            self.port = port
        self.last_seen = time.time()
        self.is_online = True

    def check_timeout(self, timeout_s=4.0):
        if self.is_online and (time.time() - self.last_seen > timeout_s):
            self.is_online = False
            return True
        return False

    def to_dict(self):
        return {
            "name": self.display_name,
            "key": self.key,
            "ip": self.ip,
            "port": self.port,
            "online": self.is_online,
            "last_seen_sec": round(time.time() - self.last_seen, 1) if self.last_seen > 0 else None,
            "status": self.status_data
        }


class UnifiedRobotCommander(Node):

    def __init__(self):
        super().__init__('unified_robot_commander')

        # ── Parameters ───────────────────────────────────────────────────
        self.declare_parameter('udp_port', 8888)
        self.declare_parameter('broadcast_ip', '255.255.255.255')
        self.declare_parameter('arm_ip', '10.216.192.100')
        self.declare_parameter('discovery_interval', 2.5)  # seconds
        self.declare_parameter('max_cmd_rate', 50.0)      # Hz

        self.udp_port = self.get_parameter('udp_port').get_parameter_value().integer_value
        self.broadcast_ip = self.get_parameter('broadcast_ip').get_parameter_value().string_value
        arm_ip = self.get_parameter('arm_ip').get_parameter_value().string_value
        discovery_interval = self.get_parameter('discovery_interval').get_parameter_value().double_value
        max_cmd_rate = self.get_parameter('max_cmd_rate').get_parameter_value().double_value

        # ── Fleet Registry ───────────────────────────────────────────────
        self.fleet = {
            'arm': FleetRobot('arm', 'Robotic Arm', default_ip=arm_ip, udp_port=self.udp_port),
            'sesame': FleetRobot('sesame', 'Sesame Quadruped', udp_port=self.udp_port),
            'bts_rover': FleetRobot('bts_rover', 'BTS Motor Rover', udp_port=self.udp_port),
            'rover': FleetRobot('rover', 'Sensor Rover', udp_port=self.udp_port),
        }

        # IP-to-Robot reverse mapping cache
        self.ip_to_bot = {}
        if arm_ip:
            self.ip_to_bot[arm_ip] = 'arm'

        # Arm calibration & joint tracking
        self.joint_calib = {
            'turntable_link_joint_dup': {'rad_min': -3.0, 'rad_max': 3.0, 'servo_min_deg': 0.0, 'servo_max_deg': 180.0, 'trim_deg': 0.0, 'sync_offset_deg': 0.0, 'invert': False},
            'turntable_link_joint': {'rad_min': -2.0, 'rad_max': 2.0, 'servo_min_deg': 0.0, 'servo_max_deg': 180.0, 'trim_deg': 0.0, 'sync_offset_deg': 0.0, 'invert': False},
            'turntable_link_joint_dup_1': {'rad_min': -2.0, 'rad_max': 2.0, 'servo_min_deg': 0.0, 'servo_max_deg': 180.0, 'trim_deg': 0.0, 'sync_offset_deg': 0.0, 'invert': False},
            'turntable_link_joint_dup_2': {'rad_min': -2.0, 'rad_max': 2.0, 'servo_min_deg': 0.0, 'servo_max_deg': 180.0, 'trim_deg': 0.0, 'sync_offset_deg': 0.0, 'invert': False},
            'wrist_twist_joint': {'rad_min': -3.14159, 'rad_max': 3.14159, 'servo_min_deg': 0.0, 'servo_max_deg': 180.0, 'trim_deg': 0.0, 'sync_offset_deg': 0.0, 'invert': False},
            'grip': {'rad_min': 0.0, 'rad_max': 3.14159, 'servo_min_deg': 0.0, 'servo_max_deg': 180.0, 'trim_deg': 0.0, 'sync_offset_deg': 0.0, 'invert': False},
        }
        self.load_calibration_file()

        self.last_sent_deg = {}
        self.last_cmd_time = {}
        self.min_cmd_interval = 1.0 / max_cmd_rate

        # ── Network Setup ────────────────────────────────────────────────
        self.udp_sock = None
        self.init_udp_socket()

        # ── Publishers ───────────────────────────────────────────────────
        self.fleet_pub = self.create_publisher(StringMsg, '/fleet/status', 10)
        self.telemetry_pub = self.create_publisher(StringMsg, '/fleet/telemetry', 10)
        self.arm_status_pub = self.create_publisher(StringMsg, '/esp32_connection_status', 10)

        # ── Subscriptions ────────────────────────────────────────────────
        self.cmd_sub = self.create_subscription(StringMsg, '/fleet/cmd', self.fleet_cmd_callback, 10)
        self.joint_sub = self.create_subscription(JointState, '/joint_states', self.joint_states_callback, 10)
        self.calib_sub = self.create_subscription(StringMsg, '/servo_calibration', self.calibration_callback, 10)

        # ── Timers ───────────────────────────────────────────────────────
        self.create_timer(0.02, self.receive_packets_callback)        # 50 Hz UDP reader
        self.create_timer(discovery_interval, self.discovery_callback) # Fleet PING
        self.create_timer(1.0, self.health_check_callback)             # 1 Hz fleet status broadcast

        self.get_logger().info(
            "🚀 Unified Robot Commander Initialized!\n"
            f"   Listening on UDP Port: {self.udp_port}\n"
            f"   Fleet targets registered: {list(self.fleet.keys())}\n"
            "   Topics: /fleet/cmd (sub), /fleet/status (pub), /fleet/telemetry (pub)"
        )

    def init_udp_socket(self):
        try:
            self.udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.udp_sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
            self.udp_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.udp_sock.setblocking(False)
            # Bind to allow receiving unicast + broadcast replies
            try:
                self.udp_sock.bind(('', self.udp_port))
            except Exception as bind_err:
                self.get_logger().warn(f"Could not bind to 0.0.0.0:{self.udp_port} ({bind_err}), sending only mode active.")
        except Exception as e:
            self.get_logger().error(f"Failed to create UDP socket: {e}")
            self.udp_sock = None

    def send_udp(self, data_str, ip, port=None):
        if self.udp_sock is None or not ip:
            return
        if port is None:
            port = self.udp_port
        try:
            self.udp_sock.sendto(data_str.encode('utf-8'), (ip, port))
        except Exception as e:
            self.get_logger().debug(f"UDP send to {ip}:{port} error: {e}")

    def broadcast_udp(self, data_str):
        if self.udp_sock is None:
            return
        try:
            self.udp_sock.sendto(data_str.encode('utf-8'), (self.broadcast_ip, self.udp_port))
        except Exception as e:
            self.get_logger().debug(f"Broadcast error: {e}")

    def discovery_callback(self):
        """Periodically broadcast PING to discover any newly powered on robots."""
        self.broadcast_udp("PING\n")

        # Also send directed PING to robots with known static IPs in case broadcast is filtered
        for bot in self.fleet.values():
            if bot.ip:
                self.send_udp("PING\n", bot.ip, bot.port)

    def health_check_callback(self):
        """Check connection timeouts and publish full fleet state."""
        fleet_dict = {}
        for key, bot in self.fleet.items():
            timed_out = bot.check_timeout(timeout_s=5.0)
            if timed_out:
                self.get_logger().warn(f"🔴 Robot '{bot.display_name}' timed out / offline.")
            fleet_dict[key] = bot.to_dict()

        # Publish fleet status JSON
        msg = StringMsg()
        msg.data = json.dumps({"timestamp": time.time(), "fleet": fleet_dict})
        self.fleet_pub.publish(msg)

        # Update Arm legacy connection status
        arm_bot = self.fleet['arm']
        arm_msg = StringMsg()
        if arm_bot.is_online:
            arm_msg.data = f"STATUS:CONNECTED:🟢 Connected to Robot Arm via UDP ({arm_bot.ip}:{arm_bot.port})"
        else:
            arm_msg.data = f"STATUS:DISCONNECTED:🔴 Robot Arm Disconnected ({arm_bot.ip})"
        self.arm_status_pub.publish(arm_msg)

    def identify_bot_from_text(self, text, sender_ip):
        """Identify which bot sent this response based on message content or IP."""
        text_lower = text.lower()
        if "sesame" in text_lower or "quard" in text_lower:
            return "sesame"
        if "bts" in text_lower or "bt_name" in text_lower or "esp32_robot" in text_lower:
            return "bts_rover"
        if "sensor rover" in text_lower or "rover-sensors" in text_lower or "temp" in text_lower and "gas" in text_lower:
            return "rover"
        if "robot arm" in text_lower or "turntable" in text_lower:
            return "arm"

        # Fallback to IP cache
        return self.ip_to_bot.get(sender_ip, None)

    def receive_packets_callback(self):
        """Read all incoming UDP packets non-blockingly."""
        if self.udp_sock is None:
            return

        while True:
            try:
                data, (sender_ip, sender_port) = self.udp_sock.recvfrom(2048)
            except (BlockingIOError, InterruptedError):
                break
            except Exception as e:
                break

            line = data.decode('utf-8', errors='ignore').strip()
            if not line:
                continue

            bot_key = self.identify_bot_from_text(line, sender_ip)
            if bot_key and bot_key in self.fleet:
                bot = self.fleet[bot_key]
                if not bot.is_online:
                    self.get_logger().info(f"🟢 Discovered / Connected: {bot.display_name} @ {sender_ip}:{sender_port}")
                bot.mark_seen(sender_ip, sender_port)
                self.ip_to_bot[sender_ip] = bot_key

            # Process Message Types
            if line.startswith("ACK:PONG"):
                # Robot answered discovery ping — immediately request full status
                self.send_udp("STATUS\n", sender_ip, sender_port)

            elif line.startswith("STATUS:"):
                payload = line[7:].strip()
                try:
                    data_obj = json.loads(payload)
                    if bot_key and bot_key in self.fleet:
                        self.fleet[bot_key].status_data = data_obj
                except Exception:
                    pass

            elif line.startswith("TELEMETRY:"):
                telemetry_payload = line[10:].strip()
                t_msg = StringMsg()
                t_msg.data = telemetry_payload
                self.telemetry_pub.publish(t_msg)

                try:
                    data_obj = json.loads(telemetry_payload)
                    if bot_key and bot_key in self.fleet:
                        self.fleet[bot_key].status_data = data_obj
                except Exception:
                    pass

            elif line.startswith("HEARTBEAT:"):
                # Handled via mark_seen
                pass

    def fleet_cmd_callback(self, msg: StringMsg):
        """
        Handle commands published to /fleet/cmd.
        Supported formats:
          1. 'target:CMD_STRING' -> e.g. 'sesame:CMD:move=forward'
          2. 'all:CMD_STRING'    -> e.g. 'all:PING' or 'all:STATUS'
          3. JSON format        -> '{"target":"sesame","cmd":"CMD:move=forward"}'
        """
        raw = msg.data.strip()
        if not raw:
            return

        target_bot = None
        cmd_text = raw

        # Check JSON format
        if raw.startswith('{'):
            try:
                obj = json.loads(raw)
                target_bot = obj.get('target', '').lower()
                cmd_text = obj.get('cmd', '')
            except Exception as e:
                self.get_logger().error(f"Error parsing JSON command: {e}")
                return

        # Check 'target:command' prefix format
        elif ':' in raw:
            parts = raw.split(':', 1)
            possible_target = parts[0].strip().lower()
            if possible_target in self.fleet or possible_target == 'all':
                target_bot = possible_target
                cmd_text = parts[1].strip()

        if not cmd_text.endswith('\n'):
            cmd_text += '\n'

        # Dispatch
        if target_bot == 'all' or target_bot is None:
            self.get_logger().info(f"Broadcasting command to entire fleet: {cmd_text.strip()}")
            self.broadcast_udp(cmd_text)
        elif target_bot in self.fleet:
            bot = self.fleet[target_bot]
            if bot.ip:
                self.send_udp(cmd_text, bot.ip, bot.port)
                self.get_logger().info(f"Sent to {bot.display_name} ({bot.ip}): {cmd_text.strip()}")
            else:
                self.get_logger().warn(f"Cannot send to {bot.display_name}: IP unknown (broadcasting)")
                self.broadcast_udp(cmd_text)
        else:
            self.get_logger().warn(f"Unknown target bot '{target_bot}' in command: {raw}")

    # ── Arm Joint State Forwarding ───────────────────────────────────────
    def map_rad_to_calibrated_deg(self, rad_val, cfg):
        r_min, r_max = cfg['rad_min'], cfg['rad_max']
        rad_clamped = max(r_min, min(r_max, rad_val))
        ratio = (rad_clamped - r_min) / (r_max - r_min)
        if cfg['invert']:
            ratio = 1.0 - ratio
        deg = 0.0 + ratio * 180.0
        deg -= cfg.get('sync_offset_deg', 0.0)
        deg += cfg['trim_deg']
        return round(max(cfg['servo_min_deg'], min(cfg['servo_max_deg'], deg)), 1)

    def joint_states_callback(self, msg: JointState):
        arm_bot = self.fleet['arm']
        if not arm_bot.ip:
            return

        now = time.time()
        for j_name, cfg in self.joint_calib.items():
            if j_name in msg.name:
                idx = msg.name.index(j_name)
                if idx < len(msg.position):
                    rad_value = msg.position[idx]
                    deg_value = self.map_rad_to_calibrated_deg(rad_value, cfg)

                    last_t = self.last_cmd_time.get(j_name, 0.0)
                    last_deg = self.last_sent_deg.get(j_name, None)

                    if now - last_t >= self.min_cmd_interval:
                        if last_deg is None or abs(deg_value - last_deg) >= 0.5:
                            cmd = f"CMD:{j_name}={deg_value:.1f}\n"
                            self.send_udp(cmd, arm_bot.ip, arm_bot.port)
                            self.last_sent_deg[j_name] = deg_value
                            self.last_cmd_time[j_name] = now

    def calibration_callback(self, msg: StringMsg):
        text = msg.data.strip()
        arm_bot = self.fleet['arm']
        if arm_bot.ip:
            self.send_udp(text + "\n", arm_bot.ip, arm_bot.port)

    def load_calibration_file(self):
        if os.path.exists(CALIB_FILE_PATH):
            try:
                with open(CALIB_FILE_PATH, 'r') as f:
                    calib_data = json.load(f)
                    for j_name, c in calib_data.items():
                        if j_name in self.joint_calib and isinstance(c, dict):
                            for k in ['servo_min_deg', 'servo_max_deg', 'trim_deg', 'sync_offset_deg']:
                                if k in c:
                                    self.joint_calib[j_name][k] = float(c[k])
            except Exception as e:
                self.get_logger().error(f"Error loading calibration file: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = UnifiedRobotCommander()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        if node.udp_sock:
            node.udp_sock.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
