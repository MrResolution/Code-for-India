#!/bin/bash
source /opt/ros/jazzy/setup.bash 2>/dev/null || source /home/sabo/Documents/learn_/Hardware/ros2_ws/install/setup.bash 2>/dev/null

pkill -9 -f rviz2 2>/dev/null
pkill -9 -f joint_state_publisher 2>/dev/null
pkill -9 -f robot_state_publisher 2>/dev/null
pkill -9 -f arm_simulator 2>/dev/null
pkill -9 -f calibrated_joint_gui 2>/dev/null

if [ "$1" == "--sim" ] || [ "$1" == "1" ]; then
    echo "🤖 Starting Automated Motion Simulator..."
    ros2 launch /home/sabo/Documents/learn_/Hardware/urdf/unnamed/display_ros2.launch.py simulate:=true
elif [ "$1" == "--gazebo" ] || [ "$1" == "3" ]; then
    echo "🌐 Starting Gazebo Physics Simulation..."
    ros2 launch /home/sabo/Documents/learn_/Hardware/urdf/unnamed/gazebo_ros2.launch.py
else
    echo "================================================================="
    echo " 🚀 Launching Integrated Robot Arm Control & RViz 3D Dashboard"
    echo "================================================================="
    ros2 launch /home/sabo/Documents/learn_/Hardware/urdf/unnamed/display_ros2.launch.py simulate:=false
fi
