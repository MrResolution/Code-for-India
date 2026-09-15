#!/usr/bin/env bash
set -e

ROBOT_MODE="arm"
USE_WEB=false

for arg in "$@"; do
  case $arg in
    --quard|quard|-q)
      ROBOT_MODE="quard"
      ;;
    --dual|dual|-d)
      ROBOT_MODE="dual"
      ;;
    --web|web|-w)
      USE_WEB=true
      ;;
  esac
done

if [ "$USE_WEB" = true ]; then
  echo "🌐 Starting WebGL 3D Dashboard on http://localhost:8000 ..."
  pkill -9 -f "web_dashboard/server.py" 2>/dev/null || true
  python3 /home/sabo/Documents/learn_/Hardware/ros2_ws/src/servo_joint_publisher/servo_joint_publisher/web_dashboard/server.py
  exit 0
fi

source /opt/ros/jazzy/setup.bash
source /home/sabo/Documents/learn_/Hardware/ros2_ws/install/setup.bash
export QT_QPA_PLATFORM=xcb

pkill -9 -f calibrated_joint_gui 2>/dev/null || true
pkill -9 -f robot_state_publisher 2>/dev/null || true
pkill -9 -f rviz2 2>/dev/null || true

if [ "$ROBOT_MODE" = "arm" ] || [ "$ROBOT_MODE" = "dual" ]; then
  (ros2 run servo_joint_publisher servo_serial_commander &)
fi

echo "🤖 Launching Desktop Dashboard for $ROBOT_MODE ..."
ros2 launch /home/sabo/Documents/learn_/Hardware/urdf/unnamed/display_ros2.launch.py simulate:=false robot:="$ROBOT_MODE"
