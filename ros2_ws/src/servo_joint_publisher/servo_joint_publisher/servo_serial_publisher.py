#!/usr/bin/env python3
"""
ROS 2 Node: servo_serial_publisher
-----------------------------------
Reads real-time servo angle values from a microcontroller via USB Serial (pyserial),
converts the values from degrees to radians, and streams sensor_msgs/msg/JointState
messages to the /joint_states topic for RViz2 3D visual updating.
"""

import math
import re
import time
import serial
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class ServoSerialPublisher(Node):

    def __init__(self):
        super().__init__('servo_serial_publisher')

        # Declare ROS 2 node parameters
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('joint_name', 'servo_joint')
        self.declare_parameter('publish_rate', 50.0) # Hz

        self.port = self.get_parameter('port').get_parameter_value().string_value
        self.baudrate = self.get_parameter('baudrate').get_parameter_value().integer_value
        self.joint_name = self.get_parameter('joint_name').get_parameter_value().string_value
        publish_rate = self.get_parameter('publish_rate').get_parameter_value().double_value

        self.get_logger().info(
            f"Initializing Servo Serial Publisher on port '{self.port}' @ {self.baudrate} baud..."
        )

        # Publisher on /joint_states topic
        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)

        # Serial connection reference
        self.serial_conn = None
        self.current_angle_deg = 90.0 # Default starting position (degrees)

        # Try connecting to USB serial port
        self.connect_serial()

        # Timer callback for reading serial and publishing JointState
        timer_period = 1.0 / publish_rate
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def connect_serial(self):
        """Establish connection with microcontroller over serial."""
        try:
            self.serial_conn = serial.Serial(self.port, self.baudrate, timeout=0.02)
            self.get_logger().info(f"✅ Successfully connected to Serial Port: {self.port}")
        except serial.SerialException as e:
            self.get_logger().warn(
                f"⚠️ Could not open serial port {self.port}: {e}. Retrying dynamically in loop..."
            )
            self.serial_conn = None

    def timer_callback(self):
        """Read serial data, parse angle, convert to radians, and publish to /joint_states."""
        # Attempt reconnect if disconnected
        if self.serial_conn is None or not self.serial_conn.is_open:
            self.connect_serial()
            if self.serial_conn is None:
                return

        try:
            while self.serial_conn.in_waiting > 0:
                line_bytes = self.serial_conn.readline()
                line = line_bytes.decode('utf-8', errors='ignore').strip()

                if line:
                    # Parse angle string e.g. "ANGLE: 90.0" or "90.0" or "Angle: 45"
                    match = re.search(r'([-+]?\d*\.?\d+)', line)
                    if match:
                        parsed_deg = float(match.group(1))
                        # Clamp degrees to valid servo bounds (0 to 180 deg)
                        self.current_angle_deg = max(0.0, min(180.0, parsed_deg))

        except serial.SerialException as e:
            self.get_logger().error(f"Serial communication error: {e}")
            if self.serial_conn:
                self.serial_conn.close()
            self.serial_conn = None
            return

        # Convert angle from degrees to radians
        # Formula: radians = degrees * (pi / 180)
        angle_radians = math.radians(self.current_angle_deg)

        # Construct sensor_msgs/msg/JointState message
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = [self.joint_name]
        msg.position = [angle_radians]
        msg.velocity = []
        msg.effort = []

        # Publish JointState to /joint_states
        self.joint_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ServoSerialPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down Servo Serial Publisher Node...")
    finally:
        if node.serial_conn and node.serial_conn.is_open:
            node.serial_conn.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
