#!/usr/bin/env python3
"""
ROS 2 Node: servo_simulator
----------------------------
Simulates dynamic, multi-joint servo motion by generating continuous sinusoidal
trajectory curves and streaming sensor_msgs/msg/JointState to /joint_states at 50 Hz.
Allows full 3D robotic arm kinematic simulation in RViz2 without hardware.
"""

import math
import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class ServoSimulatorNode(Node):

    def __init__(self):
        super().__init__('servo_simulator')

        self.declare_parameter('publish_rate', 50.0) # Hz
        publish_rate = self.get_parameter('publish_rate').get_parameter_value().double_value

        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)

        # Define joints to animate with their sinusoidal motion parameters:
        # (joint_name, center_angle_rad, amplitude_rad, frequency_hz, phase_shift_rad)
        self.joint_configs = [
            ('joint_base_spin',      0.0,   1.2,  0.2, 0.0),       # Waist swing: -1.2 to +1.2 rad
            ('joint_shoulder',       0.0,   0.6,  0.3, math.pi/4), # Shoulder lift: -0.6 to +0.6 rad
            ('joint_elbow',          0.5,   0.8,  0.25, math.pi/2),# Elbow flex: -0.3 to +1.3 rad
            ('joint_wrist',          0.0,   0.7,  0.4, math.pi/3), # Wrist pitch: -0.7 to +0.7 rad
            ('joint_gripper_finger', 0.015, 0.012,0.5, 0.0),       # Gripper slide: 0.003m to 0.027m
            ('servo_joint',          1.57,  1.4,  0.3, 0.0)        # Single servo: 0.17 to 2.97 rad
        ]

        self.start_time = time.time()
        timer_period = 1.0 / publish_rate
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.get_logger().info("🤖 Servo Motion Simulator Node started! Streaming sinusoidal joint states to /joint_states...")

    def timer_callback(self):
        elapsed = time.time() - self.start_time

        names = []
        positions = []

        for j_name, center, amp, freq, phase in self.joint_configs:
            # Formula: angle(t) = center + amplitude * sin(2 * pi * freq * t + phase)
            angle = center + amp * math.sin(2.0 * math.pi * freq * elapsed + phase)
            names.append(j_name)
            positions.append(angle)

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = names
        msg.position = positions

        self.joint_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ServoSimulatorNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Stopping Servo Simulator Node...")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
