#!/bin/bash
source /opt/ros/jazzy/setup.bash

echo "Select ROS 2 Launch Mode:"
echo "1) Pure ROS 2 (RViz2 + Joint Sliders GUI)"
echo "2) Gazebo Sim inside ROS 2 (Physics + Topic Bridge)"
read -p "Enter choice [1 or 2]: " choice

if [ "$choice" == "2" ]; then
    echo "Launching Gazebo Sim in ROS 2..."
    ros2 launch /home/sabo/Documents/learn_/Hardware/urdf/unnamed/gazebo_ros2.launch.py
else
    echo "Launching RViz2 with Joint Sliders in ROS 2..."
    ros2 launch /home/sabo/Documents/learn_/Hardware/urdf/unnamed/display_ros2.launch.py
fi
