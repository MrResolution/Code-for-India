#!/usr/bin/env bash
set -e

# Source ROS 2 Jazzy
source /opt/ros/jazzy/setup.bash

PROJECT_DIR="/home/sabo/Documents/learn_/Hardware"
mkdir -p "$PROJECT_DIR/.ros_logs"
export ROS_LOG_DIR="$PROJECT_DIR/.ros_logs"

URDF_PATH="${1:-$PROJECT_DIR/urdf/quard_bot/urdf/quard_bot.urdf}"

RVIZ_CONFIG="${2:-$PROJECT_DIR/urdf/quard_bot/rviz/quard_bot.rviz}"

echo "🤖 Launching Quard Bot (Sesame Quadruped) in RViz2..."
echo "📄 Model: $URDF_PATH"
echo "🎨 RViz Config: $RVIZ_CONFIG"

cd "$PROJECT_DIR"

ros2 launch urdf_tutorial display.launch.py \
  model:="$URDF_PATH" \
  rvizconfig:="$RVIZ_CONFIG"


