#!/usr/bin/env bash
# Body-gesture teleoperation without the dashboard:
#   webcam → gesture_teleop_node → /joint_states → servo_serial_commander → arm, plus RViz.
# servo_serial_commander clamps to servo_calibration.json, but there is no E-STOP
# here: close the gesture window (q / Esc) or press Ctrl+C to stop. The desktop
# app's "✋ Gesture Mode" runs the same node behind its E-STOP.
#
#   ./launch_gesture_teleop.sh [--port /dev/ttyUSB0] [--camera 0]
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export ROBOT_PROJECT_DIR="$SCRIPT_DIR"
SERIAL_PORT=/dev/ttyUSB0
CAMERA=0

while [ $# -gt 0 ]; do
  case "$1" in
    --port) SERIAL_PORT="$2"; shift 2 ;;
    --camera) CAMERA="$2"; shift 2 ;;
    *) echo "Usage: $0 [--port /dev/ttyUSB0] [--camera 0]" >&2; exit 1 ;;
  esac
done

if [ ! -f /opt/ros/jazzy/setup.bash ]; then
  echo "❌ ROS 2 Jazzy not found (/opt/ros/jazzy). See docs/QUICK_INSTALL_GUIDE.md." >&2
  exit 1
fi
source /opt/ros/jazzy/setup.bash
# Sources ros2_ws/install, rebuilding it first if it is older than the checkout.
source "$SCRIPT_DIR/scripts/ensure_ros_build.sh"
ensure_ros_build "$SCRIPT_DIR"

pkill -9 -f gesture_teleop_node 2>/dev/null || true
pkill -9 -f servo_serial_commander 2>/dev/null || true
pkill -9 -f robot_state_publisher 2>/dev/null || true
pkill -9 -f rviz2 2>/dev/null || true

# Stop the helpers when this script ends. `ros2 run` passes no signal on to the
# node it started, so signal that child first, then the wrapper.
CHILD_PIDS=()
cleanup() {
  for pid in "${CHILD_PIDS[@]}"; do
    pkill -TERM -P "$pid" 2>/dev/null || true
    kill -TERM "$pid" 2>/dev/null || true
  done
}
trap cleanup EXIT INT TERM

echo "================================================================="
echo " 🤖 Launching Body Gesture Robot Teleoperation System & RViz 3D Twin"
echo " 🔌 Hardware Transport: USB Serial ($SERIAL_PORT @ 115200 baud)"
echo " 🎥 Computer Vision: OpenCV & MediaPipe Upper-Body Tracking (/dev/video$CAMERA)"
echo " 🖥️ 3D Viewport: RViz2 (robot.rviz)"
echo "================================================================="

# 1. Start Physical 5-Servo Serial Commander
ros2 run servo_joint_publisher servo_serial_commander --ros-args -p use_wifi:=false -p port:="$SERIAL_PORT" &
CHILD_PIDS+=($!)

# 2. Start Robot State Publisher for URDF model (mesh paths moved to this checkout)
URDF="$(sed "s#file:///home/sabo/Documents/learn_/Hardware#file://$SCRIPT_DIR#g" "$SCRIPT_DIR/urdf/unnamed/urdf/unnamed_gazebo.urdf")"
ros2 run robot_state_publisher robot_state_publisher --ros-args -p robot_description:="$URDF" &
CHILD_PIDS+=($!)

# 3. Start RViz2 3D Digital Twin Visualization
rviz2 -d "$SCRIPT_DIR/urdf/unnamed/robot.rviz" &
CHILD_PIDS+=($!)

# 4. Start Body Gesture Teleoperation Node (it exits with an error if it can't track)
ros2 run servo_joint_publisher gesture_teleop_node --ros-args -p video_device:="$CAMERA"
