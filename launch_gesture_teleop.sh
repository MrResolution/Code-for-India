#!/usr/bin/env bash
source /opt/ros/jazzy/setup.bash
source /home/chakradhar/Documents/Hardware/ros2_ws/install/setup.bash

pkill -9 -f gesture_teleop_node 2>/dev/null || true
pkill -9 -f servo_serial_commander 2>/dev/null || true
pkill -9 -f robot_state_publisher 2>/dev/null || true
pkill -9 -f rviz2 2>/dev/null || true

echo "================================================================="
echo " 🤖 Launching Body Gesture Robot Teleoperation System & RViz 3D Twin"
echo " 🔌 Hardware Transport: USB Serial (/dev/ttyUSB0 @ 115200 baud)"
echo " 🎥 Computer Vision: OpenCV & MediaPipe Upper-Body Tracking (/dev/video0)"
echo " 🖥️ 3D Viewport: RViz2 (robot.rviz)"
echo "================================================================="

# 1. Start Physical 5-Servo Serial Commander
(ros2 run servo_joint_publisher servo_serial_commander --ros-args -p use_wifi:=false -p port:=/dev/ttyUSB0 &)

# 2. Start Robot State Publisher for URDF model
(ros2 run robot_state_publisher robot_state_publisher --ros-args -p robot_description:="$(cat /home/chakradhar/Documents/Hardware/urdf/unnamed/urdf/unnamed_gazebo.urdf)" &)

# 3. Start RViz2 3D Digital Twin Visualization
(rviz2 -d /home/chakradhar/Documents/Hardware/urdf/unnamed/robot.rviz &)

# 4. Start Body Gesture Teleoperation Node
ros2 run servo_joint_publisher gesture_teleop_node
