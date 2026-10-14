#!/usr/bin/env bash
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
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
  (
    sleep 1.2
    if command -v xdg-open &>/dev/null; then
      xdg-open "http://localhost:8000" >/dev/null 2>&1 || true
    elif command -v brave-browser &>/dev/null; then
      brave-browser "http://localhost:8000" >/dev/null 2>&1 || true
    fi
  ) &
  exec python3 "$SCRIPT_DIR/ros2_ws/src/servo_joint_publisher/servo_joint_publisher/web_dashboard/server.py"
fi

# Environment for Native Desktop App
export LD_LIBRARY_PATH="/home/sabo/miniconda3/lib:${LD_LIBRARY_PATH:-}"
export QT_QPA_PLATFORM=xcb
export ROBOT_TARGET="$ROBOT_MODE"

PYTHON_BIN="$SCRIPT_DIR/.venv/bin/python"
if [ ! -f "$PYTHON_BIN" ]; then
  PYTHON_BIN="python3"
fi

pkill -9 -f calibrated_joint_gui 2>/dev/null || true

# If ROS 2 Jazzy is present, launch full ROS 2 stack + RViz2
if [ -f /opt/ros/jazzy/setup.bash ]; then
  source /opt/ros/jazzy/setup.bash
  if [ -f "$SCRIPT_DIR/ros2_ws/install/setup.bash" ]; then
    source "$SCRIPT_DIR/ros2_ws/install/setup.bash"
  fi
  pkill -9 -f robot_state_publisher 2>/dev/null || true
  pkill -9 -f rviz2 2>/dev/null || true

  if [ "$ROBOT_MODE" = "arm" ] || [ "$ROBOT_MODE" = "dual" ]; then
    (ros2 run servo_joint_publisher servo_serial_commander &)
  fi

  echo "🤖 Launching ROS 2 Desktop Dashboard for $ROBOT_MODE ..."
  ros2 launch "$SCRIPT_DIR/urdf/unnamed/display_ros2.launch.py" simulate:=false robot:="$ROBOT_MODE"
else
  echo "🦾 Launching Native PyQt5 Desktop Dashboard for $ROBOT_MODE ..."
  exec "$PYTHON_BIN" "$SCRIPT_DIR/ros2_ws/src/servo_joint_publisher/servo_joint_publisher/calibrated_joint_gui.py" "$@"
fi
