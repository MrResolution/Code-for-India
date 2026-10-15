#!/usr/bin/env bash
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export ROBOT_PROJECT_DIR="$SCRIPT_DIR"
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
    --dogu|dogu|-g)
      ROBOT_MODE="dogu"
      ;;
    --web|web|-w)
      USE_WEB=true
      ;;
  esac
done

if [ "$USE_WEB" = true ]; then
  # Sources ROS, starts servo_serial_commander and opens the browser.
  exec "$SCRIPT_DIR/launch_web_dashboard.sh"
fi

export LD_LIBRARY_PATH="${LD_LIBRARY_PATH:-}:/home/sabo/miniconda3/lib"
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
  pkill -9 -f servo_serial_commander 2>/dev/null || true

  # Stop background helpers when the dashboard exits or Ctrl+C is pressed.
  CHILD_PIDS=()
  cleanup() {
    for pid in "${CHILD_PIDS[@]}"; do
      kill "$pid" 2>/dev/null || true
    done
  }
  trap cleanup EXIT INT TERM

  if [ "$ROBOT_MODE" = "arm" ] || [ "$ROBOT_MODE" = "dual" ]; then
    ros2 run servo_joint_publisher servo_serial_commander &
    CHILD_PIDS+=($!)
  fi

  echo "🤖 Launching ROS 2 Desktop Dashboard for $ROBOT_MODE ..."
  ros2 launch "$SCRIPT_DIR/urdf/unnamed/display_ros2.launch.py" simulate:=false robot:="$ROBOT_MODE"
else
  echo "🦾 Launching Native PyQt5 Desktop Dashboard for $ROBOT_MODE ..."
  exec "$PYTHON_BIN" "$SCRIPT_DIR/ros2_ws/src/servo_joint_publisher/servo_joint_publisher/calibrated_joint_gui.py" "$@"
fi
