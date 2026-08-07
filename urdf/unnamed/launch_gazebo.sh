#!/bin/bash
source /opt/ros/jazzy/setup.bash

URDF_PATH="/home/sabo/Documents/learn_/Hardware/urdf/unnamed/urdf/unnamed_gazebo.urdf"

if command -v gz &> /dev/null; then
    echo "Launching model in Gazebo..."
    gz sim -r "$URDF_PATH"
elif ros2 pkg list 2>/dev/null | grep -q ros_gz_sim; then
    echo "Launching model with ros_gz_sim..."
    ros2 launch ros_gz_sim gz_sim.launch.py gz_args:="-r $URDF_PATH"
elif command -v gazebo &> /dev/null; then
    echo "Launching model in Gazebo Classic..."
    gazebo "$URDF_PATH"
else
    echo "Gazebo is not installed on your system yet."
    echo "To install Gazebo for ROS 2 Jazzy, run:"
    echo "  sudo apt update && sudo apt install -y ros-jazzy-ros-gz"
    echo ""
    echo "In the meantime, you can preview and test joint controls in RViz using:"
    echo "  ./launch_rviz.sh"
fi
