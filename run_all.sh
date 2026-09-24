#!/usr/bin/env bash
set -e

# Export paths & source ROS 2 environments
export PATH=$HOME/.local/bin:$PATH
source /opt/ros/jazzy/setup.bash
source /home/chakradhar/Nurobots_kerela/Code-for-india/ros2_ws/install/setup.bash

MODE="gui"
TARGET_IP=""

while [[ $# -gt 0 ]]; do
  case $1 in
    --ip)
      TARGET_IP="$2"
      shift 2
      ;;
    sim|gui|gesture)
      MODE="$1"
      shift
      ;;
    *)
      if [[ "$1" =~ ^[0-9]+\.[0-9]+\.[0-9]+\.[0-9]+$ ]]; then
        TARGET_IP="$1"
      fi
      shift
      ;;
  esac
done

# Ensure background web server is cleaned up on exit
cleanup() {
    echo ""
    echo "🛑 Shutting down Web Dashboard & ROS 2 nodes..."
    kill $(jobs -p) 2>/dev/null || true
}
trap cleanup EXIT INT TERM

echo "========================================================="
echo " 🚀 Launching Code-For-India Multi-DOF Robotic Arm System"
echo " Mode: $MODE | Target IP: ${TARGET_IP:-Saved Config}"
echo " Usage: ./run_all.sh [gui|sim|gesture] [--ip 192.168.1.100]"
echo "========================================================="

# If IP provided on command line, update servo_calibration.json
if [ -n "$TARGET_IP" ]; then
    echo "📡 Setting target ESP32 IP to: $TARGET_IP"
    python3 -c "
import json, os
p = '/home/chakradhar/Nurobots_kerela/Code-for-india/servo_calibration.json'
data = {}
if os.path.exists(p):
    try:
        with open(p, 'r') as f: data = json.load(f)
    except: pass
data['esp32_ip'] = '$TARGET_IP'
with open(p, 'w') as f: json.dump(data, f, indent=2)
"
fi

# 1. Start Web Dashboard Server in background
echo "🌐 Starting Web Dashboard Server on http://localhost:8000 ..."
python3 /home/chakradhar/Nurobots_kerela/Code-for-india/ros2_ws/src/servo_joint_publisher/servo_joint_publisher/web_dashboard/server.py --port 8000 &
sleep 2

# 2. Launch ROS 2 System based on mode
if [ "$MODE" = "sim" ]; then
    echo "🤖 Launching Automated Motion Simulator & RViz2..."
    ros2 launch /home/chakradhar/Nurobots_kerela/Code-for-india/urdf/unnamed/display_ros2.launch.py simulate:=true
elif [ "$MODE" = "gesture" ]; then
    echo "🖐️ Launching Computer Vision Gesture Teleoperation Node & RViz2..."
    /home/chakradhar/Nurobots_kerela/Code-for-india/launch_gesture_teleop.sh
else
    echo "🦾 Launching Interactive Slider Controller & RViz2..."
    ros2 launch /home/chakradhar/Nurobots_kerela/Code-for-india/urdf/unnamed/display_ros2.launch.py simulate:=false
fi
