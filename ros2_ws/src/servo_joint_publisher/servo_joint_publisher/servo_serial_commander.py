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
"""

import math
import socket
import threading
import time
import serial
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String as StringMsg


class ServoSerialCommander(Node):

    def __init__(self):
        super().__init__('servo_serial_commander')

        # ── ROS 2 Node Parameters ─────────────────────────────────────────
        self.declare_parameter('use_wifi', False)
        self.declare_parameter('esp32_ip', '10.216.192.100') # Assigned Wi-Fi IP on network 'Sabo'
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

        # Per-joint calibration dictionary for the 4 URDF arm joints:
        self.joint_calib = {
            'turntable_link_joint_dup': {
                'rad_min': -3.0, 'rad_max': 3.0,
                'servo_min_deg': 0.0, 'servo_max_deg': 180.0,
                'trim_deg': 0.0, 'invert': False, 'pin': 18
            },
            'turntable_link_joint': {
                'rad_min': -2.0, 'rad_max': 2.0,
                'servo_min_deg': 0.0, 'servo_max_deg': 180.0,
                'trim_deg': 0.0, 'invert': False, 'pin': 19 # Drives GPIO 19 Primary & GPIO 21 Slave
            },
            'turntable_link_joint_dup_1': {
                'rad_min': -2.0, 'rad_max': 2.0,
                'servo_min_deg': 0.0, 'servo_max_deg': 180.0,
                'trim_deg': 0.0, 'invert': False, 'pin': 22
            },
            'turntable_link_joint_dup_2': {
                'rad_min': -2.0, 'rad_max': 2.0,
                'servo_min_deg': 0.0, 'servo_max_deg': 180.0,
                'trim_deg': 0.0, 'invert': False, 'pin': 23
            },
            'wrist_twist_joint': {
                'rad_min': -3.14159, 'rad_max': 3.14159,
                'servo_min_deg': 0.0, 'servo_max_deg': 180.0,
                'trim_deg': 0.0, 'invert': False, 'pin': 27
            },
        }

        mode_str = f"📶 Wi-Fi UDP ({self.esp32_ip}:{self.udp_port})" if self.use_wifi else f"🔌 USB Serial ({self.port})"
        self.get_logger().info(
            f"6-Physical-Servo Arm Commander Initializing...\n"
            f"  Communication Mode: {mode_str}\n"
            f"  Active Servo Control Joints:\n"
            + "\n".join([
                f"    • '{j}' [GPIO {c['pin']}]: range=[{c['servo_min_deg']}°, {c['servo_max_deg']}°]"
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

    def init_wifi_socket(self):
        """Initialize non-blocking UDP socket for Wi-Fi communication."""
        try:
            self.udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.udp_sock.setblocking(False)
            self.udp_sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
            self.get_logger().info(f"✅ Wi-Fi UDP Socket Initialized for target {self.esp32_ip}:{self.udp_port}")
        except Exception as e:
            self.get_logger().error(f"Failed to create UDP socket: {e}")
            self.udp_sock = None

    def publish_status(self, is_conn, details=""):
        msg = StringMsg()
        mode_label = f"Wi-Fi UDP ({self.esp32_ip}:{self.udp_port})" if self.use_wifi else f"USB Serial ({self.port})"
        if is_conn:
            msg.data = f"STATUS:CONNECTED:🟢 Connected to ESP32 via {mode_label}"
        else:
            msg.data = f"STATUS:DISCONNECTED:🔴 ESP32 Disconnected / Offline via {mode_label} ({details})"
        self.status_pub.publish(msg)

    def calibration_callback(self, msg: StringMsg):
        """Handle live calibration limit updates & mode toggles from GUI."""
        text = msg.data.strip()
        if text.startswith('MODE:'):
            mode = text[5:].strip().upper()
            self.get_logger().info(f"🔄 Commander Mode Switch Request: MODE:{mode}")

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
            if mode == 'SERIAL':
                self.use_wifi = False
                self.connect_serial()
                self.get_logger().info("🔌 Commander actively switched to USB Serial mode.")
            elif mode == 'WIFI':
                self.use_wifi = True
                self.init_wifi_socket()
                self.get_logger().info("📶 Commander actively switched to Wi-Fi UDP mode.")
            return

        if text.startswith('CMD:') or text.startswith('CALIB:'):
            self.write_raw_data(text + "\n")
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
            self.get_logger().info(f"✅ Connected to ESP32 on {self.port} (Sent MODE:SERIAL)")
        except (serial.SerialException, OSError) as e:
            now = time.time()
            if not hasattr(self, '_last_serial_warn_t') or (now - self._last_serial_warn_t > 5.0):
                self._last_serial_warn_t = now
                self.get_logger().warn(
                    f"⚠️ Could not open serial port {self.port}: {e}. Retrying dynamically..."
                )
            self.serial_conn = None

    def send_all_calibrations(self):
        """Transmit configured min/max safety limits to ESP32."""
        for j_name, c in self.joint_calib.items():
            calib_cmd = f"CALIB:{j_name}={c['servo_min_deg']:.1f},{c['servo_max_deg']:.1f}\n"
            self.write_raw_data(calib_cmd)

    def write_raw_data(self, data_str):
        """Send command data over Wi-Fi UDP socket or USB Serial."""
        if self.use_wifi:
            if self.udp_sock is None:
                self.init_wifi_socket()
                if self.udp_sock is None:
                    return
            try:
                self.udp_sock.sendto(data_str.encode('utf-8'), (self.esp32_ip, self.udp_port))
            except Exception as e:
                now = time.time()
                if not hasattr(self, '_last_udp_err_t') or (now - self._last_udp_err_t > 5.0):
                    self._last_udp_err_t = now
                    self.get_logger().warn(f"UDP send error (Wi-Fi unreachable): {e}")
        else:
            if self.serial_conn is None or not self.serial_conn.is_open:
                self.connect_serial()
                if self.serial_conn is None:
                    return
            with self.comm_lock:
                try:
                    self.serial_conn.write(data_str.encode('utf-8'))
                except (serial.SerialException, OSError) as e:
                    now = time.time()
                    if not hasattr(self, '_last_serial_err_t') or (now - self._last_serial_err_t > 5.0):
                        self._last_serial_err_t = now
                        self.get_logger().warn(f"Serial write error: {e}")
                    if self.serial_conn:
                        try:
                            self.serial_conn.close()
                        except Exception:
                            pass
                    self.serial_conn = None

    def map_rad_to_calibrated_deg(self, rad_val, cfg):
        """Map radians → calibrated degrees applying bounds, inversion, trim, and safety limits."""
        r_min, r_max = cfg['rad_min'], cfg['rad_max']
        rad_clamped = max(r_min, min(r_max, rad_val))

        ratio = (rad_clamped - r_min) / (r_max - r_min)
        if cfg['invert']:
            ratio = 1.0 - ratio

        deg = 0.0 + ratio * 180.0
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
                    deg_value = self.map_rad_to_calibrated_deg(rad_value, cfg)

                    last_t = self.last_cmd_time.get(j_name, 0.0)
                    last_deg = self.last_sent_deg.get(j_name, None)

                    if now - last_t >= self.min_cmd_interval:
                        if last_deg is None or abs(deg_value - last_deg) >= 0.5:
                            cmd = f"CMD:{j_name}={deg_value:.1f}\n"
                            self.write_raw_data(cmd)
                            self.last_sent_deg[j_name] = deg_value
                            self.last_cmd_time[j_name] = now

    def on_valid_rx(self, line):
        """Called when valid response packet is received from ESP32."""
        self.last_rx_time = time.time()
        if not self.is_connected:
            self.is_connected = True
            self.get_logger().info(f"🟢 Connection Established with ESP32 ({self.esp32_ip}:{self.udp_port})")
            self.publish_status(True)

    def check_connection_timeout(self):
        """Periodic 1Hz timer to verify live connection state."""
        now = time.time()
        self.write_raw_data("PING\n")

        if self.is_connected and (now - self.last_rx_time > 4.0):
            self.is_connected = False
            self.get_logger().warn(f"🔴 Connection Lost to ESP32 ({self.esp32_ip}:{self.udp_port})")
            self.publish_status(False, "Timeout > 4s")
        elif self.is_connected:
            self.publish_status(True)

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
                            self.get_logger().info(f"📶 [Wi-Fi ACK] {line}")
                        elif line.startswith('HEARTBEAT:'):
                            self.get_logger().debug(f"💓 {line}")
                        else:
                            self.get_logger().info(f"📶 [Wi-Fi ESP32] {line}")
            except BlockingIOError:
                pass
            except Exception as e:
                pass
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
                                self.get_logger().info(f"✅ {line}")
                            elif line.startswith('HEARTBEAT:'):
                                self.get_logger().debug(f"💓 {line}")
                            elif line.startswith('ERR:'):
                                self.get_logger().warn(f"⚠️ ESP32 error: {line}")
                            else:
                                self.get_logger().debug(f"ESP32: {line}")
                except (serial.SerialException, OSError) as e:
                    self.get_logger().error(f"Serial read error: {e}")
                    if self.serial_conn:
                        try:
                            self.serial_conn.close()
                        except Exception:
                            pass
                    self.serial_conn = None


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
