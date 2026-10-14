#!/usr/bin/env bash
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Export paths & source ROS 2 environments if available
export PATH=$HOME/.local/bin:$PATH
if [ -f /opt/ros/jazzy/setup.bash ]; then
    source /opt/ros/jazzy/setup.bash
fi
if [ -f "$SCRIPT_DIR/ros2_ws/install/setup.bash" ]; then
    source "$SCRIPT_DIR/ros2_ws/install/setup.bash"
fi

MODE="gui"
TARGET_IP=""

while [[ $# -gt 0 ]]; do
  case $1 in
    --ip)
      TARGET_IP="$2"
      shift 2
      ;;
    sim|gui|gesture|web)
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
echo " Usage: ./run_all.sh [gui|sim|gesture|web] [--ip 192.168.1.100]"
echo "========================================================="

# If IP provided on command line, update servo_calibration.json
if [ -n "$TARGET_IP" ]; then
    echo "📡 Setting target ESP32 IP to: $TARGET_IP"
    python3 -c "
import json, os
p = '$SCRIPT_DIR/servo_calibration.json'
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
pkill -9 -f "web_dashboard/server.py" 2>/dev/null || true
python3 "$SCRIPT_DIR/ros2_ws/src/servo_joint_publisher/servo_joint_publisher/web_dashboard/server.py" --port 8000 &
sleep 2

if [ "$MODE" = "web" ]; then
    echo "🌐 Running in Web mode. Dashboard available at http://localhost:8000"
    wait
    exit 0
fi

# Check if ROS 2 is installed for desktop/sim/gesture modes
if ! command -v ros2 &> /dev/null; then
    echo "⚠️ ROS 2 is not installed or sourced on this system."
    echo "🌐 The WebGL 3D Dashboard is running at: http://localhost:8000"
    echo "   (Open http://localhost:8000 in your browser to interact with the 3D robot model and calibration)"
    wait
    exit 0
fi

# 2. Launch ROS 2 System based on mode
if [ "$MODE" = "sim" ]; then
    echo "🤖 Launching Automated Motion Simulator & RViz2..."
    ros2 launch "$SCRIPT_DIR/urdf/unnamed/display_ros2.launch.py" simulate:=true
elif [ "$MODE" = "gesture" ]; then
    echo "🖐️ Launching Computer Vision Gesture Teleoperation Node & RViz2..."
    "$SCRIPT_DIR/launch_gesture_teleop.sh"
else
    echo "🦾 Launching Interactive Slider Controller & RViz2..."
    ros2 launch "$SCRIPT_DIR/urdf/unnamed/display_ros2.launch.py" simulate:=false
fi
