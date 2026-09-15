#!/usr/bin/env bash
# ESP32-CAM Visual SLAM Launcher Script
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR/esp32_visual_slam"

VENV_PATH="$SCRIPT_DIR/venv"
if [ -d "$VENV_PATH" ]; then
    source "$VENV_PATH/bin/activate"
fi

# Use the system-installed python3-opencv (Ubuntu Qt5-linked) which works
# with the system display, instead of the manylinux pip wheel that can't
# find the xcb platform plugin.
export PYTHONPATH="/usr/lib/python3/dist-packages:${PYTHONPATH:-}"

python3 main.py "$@"
