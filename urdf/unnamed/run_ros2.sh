#!/bin/bash
source /opt/ros/jazzy/setup.bash

pkill -9 -f rviz2; pkill -9 -f joint_state_publisher; pkill -9 -f robot_state_publisher; pkill -9 -f arm_simulator

echo "================================================="
echo "       ROS 2 Robot Arm Control Center            "
echo "================================================="
echo "1) 🤖 Automated Motion Simulator (Sinusoidal Input)"
echo "2) 🎛️ Manual Joint Control (RViz2 Sliders GUI)"
echo "3) 🌐 Gazebo Physics Simulation (Gazebo Sim + Bridge)"
read -p "Enter choice [1, 2, or 3]: " choice

if [ "$choice" == "1" ]; then
    echo "🤖 Starting Automated Motion Simulator..."
    ros2 launch /home/sabo/Documents/learn_/Hardware/urdf/unnamed/display_ros2.launch.py simulate:=true
elif [ "$choice" == "2" ]; then
    echo "🎛️ Starting Manual Joint Control GUI..."
    ros2 launch /home/sabo/Documents/learn_/Hardware/urdf/unnamed/display_ros2.launch.py simulate:=false
elif [ "$choice" == "3" ]; then
    echo "🌐 Starting Gazebo Physics Simulation..."
    ros2 launch /home/sabo/Documents/learn_/Hardware/urdf/unnamed/gazebo_ros2.launch.py
else
    echo "Invalid choice. Starting default Automated Simulation..."
    ros2 launch /home/sabo/Documents/learn_/Hardware/urdf/unnamed/display_ros2.launch.py simulate:=true
fi
