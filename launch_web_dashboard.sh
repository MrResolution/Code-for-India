#!/usr/bin/env bash
# Local WebGL dashboard on http://localhost:8000
#   ./launch_web_dashboard.sh                 # dashboard + servo_serial_commander
#   ./launch_web_dashboard.sh --no-commander  # dashboard only
#   ./launch_web_dashboard.sh --host 0.0.0.0  # also reachable from the LAN (no auth!)
set -e

PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export ROBOT_PROJECT_DIR="$PROJECT_DIR"

START_COMMANDER=true
SERVER_ARGS=()
for arg in "$@"; do
  case $arg in
    --no-commander) START_COMMANDER=false ;;
    *) SERVER_ARGS+=("$arg") ;;
  esac
done

# Without ROS sourced the server cannot import rclpy and silently runs UI-only.
ROS_SETUP="${ROS_SETUP:-/opt/ros/${ROS_DISTRO:-jazzy}/setup.bash}"
if [ -f "$ROS_SETUP" ]; then
  # shellcheck disable=SC1090
  source "$ROS_SETUP"
else
  echo "⚠️  $ROS_SETUP not found: dashboard will run UI-only (no ROS publishing)."
  START_COMMANDER=false
fi
if [ -f "$PROJECT_DIR/ros2_ws/install/setup.bash" ]; then
  # shellcheck disable=SC1091
  source "$PROJECT_DIR/ros2_ws/install/setup.bash"
fi

pkill -9 -f "web_dashboard/server.py" 2>/dev/null || true

CHILD_PIDS=()
cleanup() {
  for pid in "${CHILD_PIDS[@]}"; do
    kill "$pid" 2>/dev/null || true
  done
}
trap cleanup EXIT INT TERM

if [ "$START_COMMANDER" = true ] && ros2 pkg prefix servo_joint_publisher >/dev/null 2>&1; then
  pkill -f servo_serial_commander 2>/dev/null || true
  ros2 run servo_joint_publisher servo_serial_commander &
  CHILD_PIDS+=($!)
fi

echo "🌐 Starting WebGL 3D Dashboard Server on http://localhost:8000 ..."
cd "$PROJECT_DIR"
(
  sleep 1.2
  if command -v xdg-open &>/dev/null; then
    xdg-open "http://localhost:8000" >/dev/null 2>&1 || true
  elif command -v brave-browser &>/dev/null; then
    brave-browser "http://localhost:8000" >/dev/null 2>&1 || true
  fi
) &
# Not exec'd, so the trap above can stop the commander when the server exits.
python3 "$PROJECT_DIR/ros2_ws/src/servo_joint_publisher/servo_joint_publisher/web_dashboard/server.py" "${SERVER_ARGS[@]}"
