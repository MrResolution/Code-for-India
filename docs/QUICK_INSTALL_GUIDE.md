# 🛠️ Quick Installation & Setup Guide

This guide walks you through setting up **ROS 2 Jazzy**, **RViz2**, and the required system utilities on **Ubuntu 24.04 (Noble)** and **Zorin OS 18**.

---

## 📋 System Requirements

- **Operating System**: Ubuntu 24.04 LTS or Zorin OS 18.1 (x86_64)
- **Windowing System**: X11 (or XWayland)
- **Python**: 3.12+
- **Hardware Targets**:
  - 🦾 6-DOF Robotic Arm with Geared Parallel Gripper
  - 🤖 Sesame 8-Servo Quadruped ("Quard Bot")
  - 🚀 ESP32 Rover Telemetry Node

---

## ⚡ 1. Fast Automated Installation

Run the following commands in your terminal to configure the ROS 2 repository, install RViz2, and install `xdotool` (needed for embedded 3D viewport docking):

```bash
# Step 1: Enable Universe repository & install curl and xdotool
sudo apt update
sudo apt install -y software-properties-common curl xdotool xprop
sudo add-apt-repository -y universe

# Step 2: Add official ROS 2 GPG key
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

# Step 3: Add ROS 2 Jazzy repository
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu noble main" | sudo tee /etc/apt/sources.list.d/ros2.list

# Step 4: Update package index
sudo apt update

# Step 5: Install ROS 2 base, RViz2, and robot state publishers
sudo apt install -y \
  ros-jazzy-ros-base \
  ros-jazzy-rviz2 \
  ros-jazzy-robot-state-publisher \
  ros-jazzy-joint-state-publisher-gui \
  ros-jazzy-xacro \
  python3-colcon-common-extensions

# Step 6: Desktop dashboard (PyQt5), embedded WebGL twin, serial access
sudo apt install -y python3-pyqt5 python3-pyqt5.qtwebengine python3-serial

# Step 7: Enable ROS 2 environment in shell
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
source /opt/ros/jazzy/setup.bash
```

> If a project virtualenv exists at `.venv/`, `launch_dashboard.sh` uses its Python instead of the system one; install `PyQt5` and `pyserial` there too.

---

## 📦 2. Build the ROS 2 Workspace

Once ROS 2 is installed, build the project workspace:

```bash
cd ros2_ws        # from the repository root
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

---

## 🚀 3. Launching the Systems

From the repository root directory, run any of the dedicated launch scripts:

| Robot Target | Command | Mode |
|---|---|---|
| **🤖 Quard Bot (Quadruped)** | `./launch_quard_dashboard.sh` | Desktop GUI + Embedded RViz2 |
| **🦾 6-DOF Robotic Arm** | `./launch_dashboard.sh` | Desktop GUI + Embedded RViz2 |
| **⚡ Dual Robots (Both)** | `./launch_dual_dashboard.sh` | Desktop GUI + Embedded RViz2 |
| **🐕 Dogu (model preview)** | `./launch_dogu_dashboard.sh` | Desktop GUI + Embedded RViz2 |
| **🌐 WebGL Browser Twin** | `./launch_web_dashboard.sh` | Three.js Web (http://localhost:8000) |
| **🎮 Gesture Teleoperation** | `./launch_gesture_teleop.sh` | MediaPipe Hand Tracking |

The dashboard launchers (`launch_dashboard.sh`, `launch_quard_dashboard.sh`, `launch_dual_dashboard.sh`, `launch_dogu_dashboard.sh`, `launch_web_dashboard.sh`) find the project from their own location (`ROBOT_PROJECT_DIR`), so the repository can live anywhere. `launch_gesture_teleop.sh` and `launch_quard_rviz.sh` still expect it at `/home/sabo/Documents/learn_/Hardware`.

Web dashboard options: `--no-commander` (don't start `servo_serial_commander`), `--host 0.0.0.0` (reachable from phones/tablets on your network; there is no authentication), `--port N`.

> ⛔ Both dashboards have an **E-STOP** in the header (or press **Esc**). It is a software stop, so keep a hardware power switch for the servos. Details: [Dashboard guide → Safety Controls](DASHBOARD_AND_RVIZ_GUIDE.md#-5-safety-controls).

---

## 🔍 4. Verification Checklist

Run these quick checks to ensure your environment is healthy:

```bash
which ros2      # Should output: /opt/ros/jazzy/bin/ros2
which rviz2     # Should output: /opt/ros/jazzy/bin/rviz2
which xdotool   # Should output: /usr/bin/xdotool
which xprop     # Should output: /usr/bin/xprop
python3 -c "import PyQt5, serial; print('PyQt5 + pyserial OK')"
```

---

## 💡 Standalone WebGL Fallback (No ROS Required)

If you need to run on a machine without ROS 2 installed, the system automatically falls back to the **WebGL 3D Digital Twin** (rendered via Three.js in your browser or embedded webview):

```bash
./launch_dashboard.sh --web
# Opens http://localhost:8000 automatically
```
