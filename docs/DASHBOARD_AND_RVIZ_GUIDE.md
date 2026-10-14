# 🖥️ Robotics Integrated Dashboard & Embedded RViz2 Guide

This guide explains the architecture, window docking mechanics, control tabs, and fail-safe designs of the **Robotics Integrated Control Center**.

---

## 🏗️ 1. Architecture Overview

The system features a **split-view desktop application** built with **PyQt5** and **ROS 2 Jazzy**:

```text
┌────────────────────────────────────────────────────────────────────────────────────────┐
│ 🤖🦾 Robotics Integrated Control Center                   ROS 2 Jazzy  [Serial] [WiFi] │
├─────────────────────────────────────────┬──────────────────────────────────────────────┤
│               LEFT TILE                 │                  RIGHT TILE                  │
│       Multi-Robot Control Panels        │             3D Viewport Container            │
│ ┌─────┬───────────┬─────────┬─────────┐ │ ┌──────────────────────────────────────────┐ │
│ │ Arm │ Quard Bot │ Sensors │  SLAM   │ │ │ 🧊 Embedded Native RViz2 (ROS 2)        │ │
│ ├─────┴───────────┴─────────┴─────────┤ │ │                                          │ │
│ │ • 6-DOF Slider Calibration          │ │ │   [ Interactive 3D Robot Models:         │ │
│ │ • PCA9685 8-Channel Servo Controls  │ │ │     Arm / Quard / Dual Bots / SLAM Map ] │ │
│ │ • WASD / D-Pad Gait Locomotion      │ │ │                                          │ │
│ │ • Climate, Gas, Flame & IMU Sensor  │ │ │ (Auto-fallback to Three.js WebGL Twin   │ │
│ │ • ESP32-CAM Visual SLAM & 2D Map    │ │ │  if running without ROS 2)               │ │
│ └─────────────────────────────────────┘ │ └──────────────────────────────────────────┘ │
└─────────────────────────────────────────┴──────────────────────────────────────────────┘
```

---

## 🧊 2. Native RViz2 Window Docking Mechanics

The dashboard embeds the native **RViz2** 3D viewport side-by-side using an X11 reparenting pipeline:

### Process Lifecycle & Safety Isolation:
1. **Direct Child Process**: `calibrated_joint_gui` launches RViz2 directly using `QProcess` with the target robot's RViz config (`arm.rviz`, `quard_bot.rviz`, or `dual_robots.rviz`).
2. **PID-Locked Window Discovery**: The dashboard identifies the window strictly using the child's exact **Process ID (`_NET_WM_PID`)** and verifies `WM_CLASS == "rviz2"`.
3. **Host Window Isolation**: The search actively verifies window properties and **strictly excludes** host applications (`antigravity`, VS Code, browsers, or terminal emulators) from being touched or unmapped.
4. **Reparenting into Qt**: The X11 window is reparented into the PyQt frame (`self.rviz_frame`) using `xdotool windowreparent`.
5. **Qt Container Integration**: The native X11 window is wrapped in a `QWindow.fromWinId()` and embedded inside a `QWidget.createWindowContainer()`, allowing dynamic resizing alongside the control panel splitter.

---

## 🎛️ 3. Dashboard Control Panels

### 🦾 Panel 1: Robotic Arm (6-DOF)
- **Joint Sliders**: Real-time control in degrees and radians.
- **Safety Limits**: "📍 Set Low" and "📍 Set High" calibration buttons next to each joint.
- **Twin Sync**: Fine-tune 3D orientation offsets relative to physical servos.
- **Global Actions**:
  - `🏠 Go Home`: Smooth 2.0s return to safe resting posture.
  - `📌 Set Home`: Save current pose as the new home baseline.
  - `▶️ Start Sim`: Continuous sinusoidal motion simulation.
  - `💾 Save Config`: Persists calibration to `servo_calibration.json`.

### 🤖 Panel 2: Quard Bot (Sesame Quadruped)
- **Locomotion Controller (D-Pad / WASD)**:
  - `▲ Forward (W)` / `▼ Backward (S)`: 4-phase diagonal gait with leg swing.
  - `◀ Left (A)` / `▶ Right (D)`: Coordinated turning gait.
  - `⏹ Stand (Space)`: Ground all 4 feet to stable neutral stance.
  - **Adjustable Step Delay**: 100 ms to 400 ms slider.
- **Quick Stances**:
  - `Stand (45°)` · `Base Zero (0°)` · `Crouch (30°)` · `High (60°)` · `Wave Animation`
- **PCA9685 8-Channel Sliders**:
  - Direct individual servo control for Front/Rear Hips and Knee joints.
  - "Set All Channels" master slider.

### 🌡️ Panel 3: ESP32 Rover Sensor Node
- **Climate**: DHT11 & BMP280 temperature, humidity, and barometric pressure.
- **Safety Hazards**: Real-time meters for MQ Combustible Gas, Water Level, and Optical Flame detector with audio/visual fire alert banner.
- **6-DOF IMU (MPU6050)**: Live Accelerometer, Gyroscope, and estimated Pitch/Roll tilt.
- **GPS NEO-6M**: Real-time latitude, longitude, and satellite lock count.

### 👁️ Panel 4: ESP32-CAM Visual SLAM
- **Live Video Feeds**: Camera HUD, 2D Top-Down Occupancy Map, or Dual View.
- **ToF Range**: VL53L1X distance sensor measurement (0.00 to 4.00 m).
- **Visual Odometry**: Real-time ORB keypoint tracking, inlier count, and 6-DOF pose estimation.

---

## 🔄 4. WebGL Fallback Mode

If ROS 2 is not present, the dashboard automatically falls back to an integrated **Three.js WebGL Digital Twin**:
- Loads STL CAD models directly in browser or embedded webview.
- Allows full orbit controls (Rotate, Pan, Zoom).
- Accessible independently at `http://localhost:8000`.

---

## 🛠️ 5. Troubleshooting & FAQ

### Q: Why does the 3D panel show WebGL instead of RViz?
- **Cause**: ROS 2 Jazzy or RViz2 is not installed or not in PATH.
- **Fix**: Run the one-line install commands in [`docs/QUICK_INSTALL_GUIDE.md`](QUICK_INSTALL_GUIDE.md).

### Q: Can I run both the Robotic Arm and Quard Bot together?
- **Yes**: Run `./launch_dual_dashboard.sh`. The dashboard loads both robots simultaneously into RViz2.

### Q: How do I connect to physical hardware?
- **Serial**: Select `/dev/ttyUSB0` at 115200 baud and click **Connect Serial**.
- **Wi-Fi**: Enter the ESP32 IP or mDNS hostname (`http://sesame-robot.local`) and click **Test Ping**.
