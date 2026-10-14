#!/usr/bin/env bash
set -e

PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
echo "🌐 Starting WebGL 3D Dashboard Server on http://localhost:8000 ..."
pkill -9 -f "web_dashboard/server.py" 2>/dev/null || true
cd "$PROJECT_DIR"
(
  sleep 1.2
  if command -v xdg-open &>/dev/null; then
    xdg-open "http://localhost:8000" >/dev/null 2>&1 || true
  elif command -v brave-browser &>/dev/null; then
    brave-browser "http://localhost:8000" >/dev/null 2>&1 || true
  fi
) &
exec python3 "$PROJECT_DIR/ros2_ws/src/servo_joint_publisher/servo_joint_publisher/web_dashboard/server.py"
