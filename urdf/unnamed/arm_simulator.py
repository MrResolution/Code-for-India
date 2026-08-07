#!/usr/bin/env python3
"""
ROS 2 Node: arm_simulator
--------------------------
Simulates dynamic, multi-joint movement for the robot arm by publishing continuous
sinusoidal joint trajectories to /joint_states at 50 Hz.
"""

import math
import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class ArmSimulatorNode(Node):

    def __init__(self):
        super().__init__('arm_simulator')

        self.declare_parameter('publish_rate', 50.0)  # Hz
        publish_rate = self.get_parameter('publish_rate').get_parameter_value().double_value

        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)

        # Joint configurations for smooth sinusoidal animation:
        # (joint_name, center_angle_rad, amplitude_rad, frequency_hz, phase_shift_rad)
        self.joint_configs = [
            ('turntable_link_joint_dup',   0.0,   1.0,  0.2,  0.0),            # Base Waist Swing: -1.0 to +1.0 rad
            ('turntable_link_joint',       0.0,   0.5,  0.25, math.pi / 4),    # Shoulder Pitch: -0.5 to +0.5 rad
            ('turntable_link_joint_dup_1', 0.2,   0.6,  0.3,  math.pi / 2),    # Elbow Pitch: -0.4 to +0.8 rad
            ('turntable_link_joint_dup_2', 0.0,   0.4,  0.35, math.pi / 3),    # Wrist Pitch: -0.4 to +0.4 rad
        ]

        self.start_time = time.time()
        timer_period = 1.0 / publish_rate
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.get_logger().info("🤖 Robot Arm Motion Simulator Node started! Streaming sinusoidal joint states to /joint_states...")

    def timer_callback(self):
        elapsed = time.time() - self.start_time

        names = []
        positions = []
        velocities = []

        for j_name, center, amp, freq, phase in self.joint_configs:
            # angle(t) = center + amplitude * sin(2 * pi * freq * t + phase)
            angle = center + amp * math.sin(2.0 * math.pi * freq * elapsed + phase)
            vel = amp * 2.0 * math.pi * freq * math.cos(2.0 * math.pi * freq * elapsed + phase)
            names.append(j_name)
            positions.append(angle)
            velocities.append(vel)

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = names
        msg.position = positions
        msg.velocity = velocities

        self.joint_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ArmSimulatorNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Stopping Arm Simulator Node...")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
