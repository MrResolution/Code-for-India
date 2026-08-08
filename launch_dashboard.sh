#!/usr/bin/env bash
source /opt/ros/jazzy/setup.bash
source /home/sabo/Documents/learn_/Hardware/ros2_ws/install/setup.bash
(ros2 run servo_joint_publisher servo_serial_commander &)
ros2 launch /home/sabo/Documents/learn_/Hardware/urdf/unnamed/display_ros2.launch.py simulate:=false
