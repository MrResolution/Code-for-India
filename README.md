# 🤖🦾 Robotics Integrated Control Center & Multi-Robot Simulation

[![ROS 2 Jazzy](https://img.shields.io/badge/ROS_2-Jazzy_Jalisco-34495e?logo=ros&logoColor=white)](https://docs.ros.org/en/jazzy/)
[![GUI](https://img.shields.io/badge/GUI-PyQt5-41cd52?logo=qt&logoColor=white)](https://riverbankcomputing.com/software/pyqt/)
[![3D Viewport](https://img.shields.io/badge/3D_Viewport-Embedded_RViz2_%2B_Three.js-blue)](docs/DASHBOARD_AND_RVIZ_GUIDE.md)
[![Microcontroller](https://img.shields.io/badge/Firmware-ESP32_%2F_Arduino-e7352f?logo=espressif&logoColor=white)](src/)

A unified robotics engineering platform featuring a **Multi-DOF Robotic Arm with a parallel geared gripper**, an **8-servo Sesame Quadruped Robot ("Quard Bot")**, an **ESP32 Rover Telemetry Node**, and an **interactive PyQt5 Dashboard** with side-by-side **embedded native RViz2 3D visualization** and **Three.js WebGL digital twins**.

---

## ⚡ Quick Launch

From the project root directory:

```bash
# 🤖 Launch Quard Bot (Quadruped) in Dashboard + Embedded RViz2
./launch_quard_dashboard.sh

# 🦾 Launch 6-DOF Robotic Arm in Dashboard + Embedded RViz2
./launch_dashboard.sh

# ⚡ Launch Dual Robots (Arm + Quadruped simultaneously in RViz2)
./launch_dual_dashboard.sh

# 🌐 Launch Standalone WebGL 3D Dashboard (Browser: http://localhost:8000)
./launch_web_dashboard.sh                  # add --host 0.0.0.0 for phones/tablets (no auth!)
```

> ⛔ **E-STOP:** both dashboards have an E-STOP button in the header (or press **Esc**). It stops all dashboard-driven motion and locks the controls. It is a software stop and does not cut servo power. See [Safety Controls](docs/DASHBOARD_AND_RVIZ_GUIDE.md#-4-safety-controls).

> **First time running?** See the [🛠️ Quick Installation Guide](docs/QUICK_INSTALL_GUIDE.md) to set up ROS 2 Jazzy, RViz2, and `xdotool` in under 3 minutes.

---

## 🏗️ System Highlights

- **Embedded Native RViz2**: Seamlessly docked into the desktop dashboard frame via X11 reparenting with strict PID and window-class isolation.
- **Web Dashboard**: Three.js twin at `http://localhost:8000`.
  - Light and dark themes; works at phone width.
  - Live status badges for the ROS bridge, Quard Bot and sensor node.
  - Runs UI-only without ROS 2.
- **Safety**:
  - E-STOP (button or **Esc**) in both UIs.
  - The Quard firmware's 2 s locomotion watchdog is honoured: the active gait is resent every second.
- **Quard Bot Quadruped Control**:
  - D-Pad & keyboard locomotion (WASD / Space) with 4-phase gait trajectory generation.
  - Quick stance presets (`Stand 45°`, `Base Zero 0°`, `Crouch 30°`, `High 60°`, `Wave`).
  - PCA9685 8-channel live servo slider management.
- **Robotic Arm Control**:
  - Joint angle sliders with real-time degree and radian readouts.
  - "📍 Set Low" and "📍 Set High" safety boundary calibration (the web UI shows the saved range on each slider).
  - Smooth homing (`🏠 Go Home`), `📌 Set Home`, and automated sinusoidal motion simulation.
  - Save calibration with `💾 Save Config` or Ctrl+S (desktop).
- **Rover Telemetry & Sensors**:
  - Climate: DHT11 & BMP280 temperature, humidity, and barometric pressure.
  - Hazards: MQ Gas, water level, and optical flame detector with emergency fire alerts.
  - Motion: 6-DOF MPU6050 IMU accelerometer, gyroscope, and pitch/roll tilt.
  - Navigation: NEO-6M GPS coordinates and satellite lock telemetry.

---

## 📂 Project Structure

```text
.
├── launch_dashboard.sh             # Main dashboard launcher (Arm by default, flags: --quard, --dual, --web)
├── launch_quard_dashboard.sh       # One-click launcher for Quard Bot (Quadruped)
├── launch_dual_dashboard.sh        # One-click launcher for Dual Robots simultaneous view
├── launch_web_dashboard.sh         # One-click launcher for WebGL Three.js twin
├── launch_gesture_teleop.sh        # MediaPipe hand gesture teleoperation launcher
│
├── docs/                           # Clean documentation & guides
│   ├── QUICK_INSTALL_GUIDE.md      # Fast step-by-step ROS 2 & RViz2 setup
│   └── DASHBOARD_AND_RVIZ_GUIDE.md # Deep-dive into dashboard layout & window docking
│
├── ros2_ws/                        # ROS 2 Colcon Workspace
│   └── src/servo_joint_publisher/  # Main ROS 2 package
│       ├── servo_joint_publisher/
│       │   ├── calibrated_joint_gui.py   # PyQt5 Dashboard with embedded RViz2 container
│       │   ├── servo_serial_commander.py # Sends /joint_states to the arm ESP32 (serial or Wi-Fi)
│       │   ├── servo_serial_publisher.py # ESP32 USB serial joint state reader
│       │   ├── servo_simulator.py        # Automated trajectory generator
│       │   └── web_dashboard/            # Three.js WebGL Digital Twin server & frontend
│
├── urdf/                           # Robot URDF models & RViz configurations
│   ├── arm/                        # 6-DOF Arm URDF, meshes, and tuned rviz config
│   ├── quard_bot/                  # Quard Bot URDF, meshes, and tuned rviz config
│   └── dual_robots.urdf            # Combined multi-robot URDF & rviz config
│
├── Quard bot/                      # Sesame Quadruped Robot STLs & firmware
│   ├── stl/                        # 3D printable STL parts (L1..L4, R1..R4, Frame)
│   └── firmware/                   # ESP32 firmware (PCA9685, SSD1306 OLED, WebServer)
│
├── arm/                            # Blender 3D models & STL part files
├── extracted_stls/                 # STL parts extracted from 3MF archives
├── bts_motor_control/              # Dual BTS7960 43A H-Bridge Motor Controller
├── robotic_arm_motherboard/        # Modular Carrier PCB, KiCad files & Gerbers
├── src/                            # Embedded C++ firmware (PlatformIO)
└── platformio.ini                  # PlatformIO environment configuration
```

---

## 🦾 Kinematic Architecture

### 1. 6-DOF Robotic Arm & Parallel Gripper

| Joint Name | Type | Axis | Physical Function |
|---|---|---|---|
| `turntable_link_joint_dup` | Revolute | Z | Base Turntable Yaw (±180°) |
| `turntable_link_joint` | Revolute | Y | Shoulder Pitch (Dual Servos: GPIO 19 & 21) |
| `turntable_link_joint_dup_1` | Revolute | Y | Elbow 1 Pitch |
| `turntable_link_joint_dup_2` | Revolute | Y | Elbow 2 Pitch |
| `turntable_link_joint_dup_3` | Revolute | Z | Wrist Twist / Roll (GPIO 27) |
| `grip` | Revolute | Z | Geared Parallel Gripper (GPIO 26) |
| `mimic ×5` | Mimic | — | 4-bar parallel follower links & opposing gear |

### 2. Sesame Quadruped ("Quard Bot")

| PCA9685 Channel | Leg Location | Joint Role | Neutral |
|---|---|---|---|
| **Ch 0** | Right Front (R1) | Hip Joint | 45° |
| **Ch 1** | Right Rear (R2)  | Hip Joint | 45° |
| **Ch 2** | Left Rear (L2)   | Hip Joint | 45° |
| **Ch 3** | Left Front (L1)  | Hip Joint | 45° |
| **Ch 4** | Right Front (R3) | Knee Joint | 45° |
| **Ch 5** | Right Rear (R4)  | Knee Joint | 45° |
| **Ch 6** | Left Rear (L4)   | Knee Joint | 45° |
| **Ch 7** | Left Front (L3)  | Knee Joint | 45° |

---

## 📚 Documentation Reference

- **[Quick Installation Guide](docs/QUICK_INSTALL_GUIDE.md)**: ROS 2 Jazzy, RViz2, and `xdotool` setup.
- **[Dashboard & RViz Docking Guide](docs/DASHBOARD_AND_RVIZ_GUIDE.md)**: Both UIs, control panels, E-STOP and safety, keyboard shortcuts, web dashboard options, troubleshooting.
- **[Project Context & Handoff](PROJECT_CONTEXT_README.md)**: Phobos patch details, gripper mimic joint math, and developer notes.
- **[Quard Bot Firmware & Hardware](Quard%20bot/README.md)**: Pinouts, OLED animations, and Web AP instructions.
- **[Robotic Arm Carrier PCB](robotic_arm_motherboard/README.md)**: Custom KiCad carrier board and Gerbers.

---

## 📄 License

This project is open-source under standard robotics educational licenses. See individual component directories for specific asset licensing.
