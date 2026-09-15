#!/usr/bin/env bash
set -e

PROJECT_DIR="/home/sabo/Documents/learn_/Hardware"
echo "🌐 Starting WebGL 3D Dashboard Server on http://localhost:8000 ..."
pkill -9 -f "web_dashboard/server.py" 2>/dev/null || true
cd "$PROJECT_DIR"
python3 "$PROJECT_DIR/ros2_ws/src/servo_joint_publisher/servo_joint_publisher/web_dashboard/server.py"
