# 🤖🦾 Robotics Integrated Control Center & Multi-Robot Simulation

[![ROS 2 Jazzy](https://img.shields.io/badge/ROS_2-Jazzy_Jalisco-34495e?logo=ros&logoColor=white)](https://docs.ros.org/en/jazzy/)
[![GUI](https://img.shields.io/badge/GUI-PyQt5-41cd52?logo=qt&logoColor=white)](https://riverbankcomputing.com/software/pyqt/)
[![3D Viewport](https://img.shields.io/badge/3D_Viewport-Embedded_RViz2_%2B_Three.js-blue)](docs/DASHBOARD_AND_RVIZ_GUIDE.md)
[![Microcontroller](https://img.shields.io/badge/Firmware-ESP32_%2F_Arduino-e7352f?logo=espressif&logoColor=white)](src/)

A unified robotics engineering platform featuring a **Multi-DOF Robotic Arm with a parallel geared gripper**, an **8-servo Sesame Quadruped Robot ("Quard Bot")**, an **ESP32 Rover Telemetry Node**, **Dogu** (a 12-servo robot dog you can start and stop over Wi-Fi, with a 3D model preview), and an **interactive PyQt5 Dashboard** with side-by-side **embedded native RViz2 3D visualization** and **Three.js WebGL digital twins**.

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

# 🐕 Launch Dogu (12-joint quadruped model preview + animations) in RViz2
./launch_dogu_dashboard.sh

# 🌐 Launch Standalone WebGL 3D Dashboard (Browser: http://localhost:8000)
./launch_web_dashboard.sh                  # add --host 0.0.0.0 for phones/tablets (no auth!)
```

> ⛔ **E-STOP:** both dashboards have an E-STOP button in the header (or press **Esc**). It stops all dashboard-driven motion and locks the controls. It is a software stop and does not cut servo power. See [Safety Controls](docs/DASHBOARD_AND_RVIZ_GUIDE.md#-5-safety-controls).

> 🗺️ **New to the repo (human or AI agent)?** Start with [PROJECT_MAP.md](PROJECT_MAP.md): where everything lives, how the pieces talk, and the pitfalls.

> **First time running?** See the [🛠️ Quick Installation Guide](docs/QUICK_INSTALL_GUIDE.md) to set up ROS 2 Jazzy, RViz2, and `xdotool` in under 3 minutes.

---

## 🏗️ System Highlights

- **Embedded Native RViz2**: Seamlessly docked into the desktop dashboard frame via X11 reparenting with strict PID and window-class isolation.
- **Web Dashboard**: Three.js twin at `http://localhost:8000`.
  - Light and dark themes; works at phone width.
  - Live status badges for the ROS bridge, Quard Bot and sensor node.
  - Runs UI-only without ROS 2.
- **Connections & consoles** (every tab, both UIs):
  - Live link state with the reason when something is wrong, ping round trip, ⟳ Reconnect, and port / IP changes without a restart.
  - Consoles with colour, filter, find, pause, copy, save and command history; raw commands to the arm ESP32 or the Quard Bot.
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
- **Dogu (robot dog)**:
  - Firmware `src/dogu.cpp` (ESP32 + PCA9685): walks with 8 servos, holds 4 body servos, starts its own Wi-Fi access point. See [docs/DOGU.md](docs/DOGU.md).
  - Its tab in both UIs connects to `http://192.168.4.1`: live status (walking, leg, phase) with **Start walking** / **Stop walking**; E-STOP sends Stop.
  - Its own tab in both UIs: Stand / Crouch / Sit poses and Walk, Turn, Wave and Idle animations, with a speed slider and live joint angles.
  - The model is built from `cad/dogu/Dogu.blend` by `scripts/dogu/`. Its poses and animations move the 3D model only.
- **Rover Telemetry & Sensors**:
  - Climate: DHT11 & BMP280 temperature, humidity, and barometric pressure.
  - Hazards: MQ Gas, water level, and optical flame detector with emergency fire alerts.
  - Motion: 6-DOF MPU6050 IMU accelerometer, gyroscope, and pitch/roll tilt.
  - Navigation: NEO-6M GPS coordinates and satellite lock telemetry.

---

## 📂 Project Structure

The full map, with "where do I find X" tables, data flow, topics, ports and pitfalls, is in **[PROJECT_MAP.md](PROJECT_MAP.md)**.

```text
.
├── launch_*.sh            # One-click launchers (dashboard, quard, dual, dogu, web, gesture teleop)
├── run_all.sh             # Older all-in-one launcher (sim / real menu)
├── servo_calibration.json # Arm calibration, read and written by both dashboards
├── ros2_ws/               # ROS 2 package servo_joint_publisher: dashboards, commanders, web twin
├── urdf/                  # Runtime robot models used by RViz and the web twin
│   ├── arm/  quard_bot/  dogu/  dual_robots.urdf
│   └── unnamed/           # Arm launch file (display_ros2.launch.py), sim + legacy Gazebo tools
├── src/  include/  lib/   # ESP32 / ESP8266 firmware (PlatformIO; envs in platformio.ini)
├── scripts/               # Model build tools (Dogu export/build, Quard/arm URDF generator, 3MF→STL)
├── cad/                   # Source CAD: Blender scenes and printable STLs (not loaded at runtime)
├── hardware/              # Carrier PCB (KiCad, Gerbers) and the BTS7960 motor controller
└── docs/                  # Guides, Quard Bot manual, project context, presentation, diagrams
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

### 3. Dogu (12-joint quadruped, model preview)

| Joints | Legs | Role | Zero |
|---|---|---|---|
| `{fl,fr,bl,br}_hip_joint` | front/back × left/right | Abduction (leg out to the side) | Standing |
| `{fl,fr,bl,br}_femur_joint` | | Upper leg pitch | Standing |
| `{fl,fr,bl,br}_tibia_joint` | | Knee | Standing |

Every joint turns about its own z axis and reads 0 in the standing pose. The ±90° limits are provisional until the servo mapping is confirmed. The firmware's own servo channels are in [docs/DOGU.md](docs/DOGU.md).

---

## 📚 Documentation Reference

- **[Project Map](PROJECT_MAP.md)**: directory map, where to find things, data flow, topics, ports and pitfalls. Start here.
- **[Agent Guide](AGENTS.md)**: rules and checks for AI coding agents (and humans) changing this repo.
- **[Quick Installation Guide](docs/QUICK_INSTALL_GUIDE.md)**: ROS 2 Jazzy, RViz2, and `xdotool` setup.
- **[Dashboard & RViz Docking Guide](docs/DASHBOARD_AND_RVIZ_GUIDE.md)**: Both UIs, control panels, connections & consoles, E-STOP and safety, keyboard shortcuts, web dashboard options, troubleshooting.
- **[Project Context & Handoff](docs/PROJECT_CONTEXT.md)**: Phobos patch details, gripper mimic joint math, and developer notes.
- **[Quard Bot Firmware & Hardware](docs/QUARD_BOT.md)**: Pinouts, OLED animations, Web AP instructions and the HTTP API.
- **[Dogu robot](docs/DOGU.md)**: hardware, servo channels, Wi-Fi access point, HTTP API and safety.
- **[Dogu model](urdf/dogu/README.md)**: how the Dogu URDF and animations are built from the Blender scene.
- **[Robotic Arm Carrier PCB](hardware/robotic_arm_motherboard/README.md)**: Custom KiCad carrier board and Gerbers.
- **[CAD sources](cad/README.md)**: Blender scenes and printable STLs for every robot.
- **[Project Presentation](docs/PROJECT_PRESENTATION.md)**: the long-form project write-up.

---

## 📄 License

This project is open-source under standard robotics educational licenses. See individual component directories for specific asset licensing.
