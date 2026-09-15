---
marp: true
theme: default
paginate: true
header: "🦾 Code for India — Autonomous Robotics Ecosystem"
footer: "Confidential & Proprietary • Project Presentation"
style: |
  section {
    font-family: 'Inter', system-ui, -apple-system, sans-serif;
    font-size: 19px;
    padding: 35px 50px;
    background-color: #0f172a;
    color: #f8fafc;
  }
  h1 {
    color: #38bdf8;
    font-size: 34px;
    margin-bottom: 12px;
  }
  h2 {
    color: #818cf8;
    font-size: 26px;
    border-bottom: 2px solid #334155;
    padding-bottom: 6px;
    margin-top: 10px;
    margin-bottom: 14px;
  }
  h3 {
    color: #34d399;
    font-size: 21px;
    margin-top: 10px;
    margin-bottom: 6px;
  }
  table {
    font-size: 14px;
    border-collapse: collapse;
    width: 100%;
    margin-top: 8px;
  }
  th {
    background-color: #1e293b;
    color: #38bdf8;
    padding: 7px 10px;
    border: 1px solid #334155;
  }
  td {
    padding: 6px 10px;
    border: 1px solid #334155;
    background-color: #0f172a;
  }
  code {
    background-color: #1e293b;
    color: #f43f5e;
    padding: 2px 6px;
    border-radius: 4px;
    font-family: 'JetBrains Mono', monospace;
    font-size: 14px;
  }
  pre code {
    background-color: transparent;
    color: #e2e8f0;
    font-size: 12px;
  }
  .highlight-box {
    background-color: #1e293b;
    border-left: 4px solid #38bdf8;
    padding: 10px 16px;
    border-radius: 0 8px 8px 0;
    margin: 10px 0;
  }
  .grid-2 {
    display: grid;
    grid-template-columns: 1fr 1fr;
    gap: 20px;
  }
  .badge {
    display: inline-block;
    padding: 3px 8px;
    border-radius: 12px;
    font-size: 12px;
    font-weight: bold;
    background: #0284c7;
    color: #fff;
  }
---

<!-- Slide 1: Title Slide -->
# 🦾 Code for India: Multi-Robot & Autonomous Hardware Ecosystem
### Real-Time Digital Twins, Embedded Robotics, AI Teleoperation & Visual SLAM

<br>

**Presenter:** Hardware & Robotics Engineering Team  
**Tech Stack:** ROS 2 Jazzy · Blender 5.2.0 (Phobos) · ESP32 · MediaPipe AI · Open3D · Three.js  
**Target Platforms:** 5-DOF Robotic Arm | 8-DOF Quadruped ("Quard Bot") | 43A Heavy Rover Base  

<div class="highlight-box">
  <b>Core Vision:</b> A unified, production-grade cyber-physical robotics platform integrating CAD-to-URDF digital twinning, real-time computer vision teleoperation, edge spatial SLAM mapping, and multi-robot command cockpits.
</div>

---

<!-- Slide 2: Executive Summary & Highlights -->
# 🌟 Executive Summary: What Makes This Project Unique?

<div class="grid-2">
<div>

### 🤖 4 Integrated Robotic Platforms
1. **5-DOF Articulated Robotic Arm**: Parallel geared linkage gripper with dual-synchronized shoulder servos.
2. **Quard Bot (Sesame Quadruped)**: 8-DOF biomimetic walking robot with PCA9685 driver & animated OLED face.
3. **Heavy-Duty Rover Base**: Dual BTS7960 43A motor driver with Bluetooth Classic teleoperation.
4. **Spatial Perception Node**: ESP32-CAM + VL53L1X ToF laser for real-time monocular Visual SLAM.

</div>
<div>

### 💡 Flagship Technical Innovations
- **AI Body Gesture Teleoperation**: MediaPipe upper-body skeletal tracking driving physical servos & 3D twin in real time.
- **Micro-Scale Visual SLAM**: Single-ray ToF laser sensor fusion solving the classic monocular scale ambiguity problem.
- **Anti-Brownout Firmware**: Staggered servo ignition engine and register-level brownout bypass for high-current loads.
- **Bi-Directional Digital Twin**: Synchronized RViz2 and WebGL 3D simulation with persistent calibration storage.

</div>
</div>

---

<!-- Slide 3: End-to-End System Architecture -->
# 🏗️ Complete System Architecture

```text
 ┌────────────────────────────────────────────────────────────────────────────────────────┐
 │                              HUMAN-ROBOT INTERACTION & AI                               │
 │   MediaPipe Webcam Teleop  │   PyQt5 Master Cockpit   │   WebGL Three.js 3D Web App    │
 └─────────────┬───────────────────────────┬───────────────────────────┬──────────────────┘
               │ Joint Angles              │ ROS 2 Messages            │ WebSockets / HTTP
               ▼                           ▼                           ▼
 ┌────────────────────────────────────────────────────────────────────────────────────────┐
 │                             ROS 2 JAZZY CORE MIDDLEWARE                                │
 │   /joint_states (50 Hz)    │   robot_state_publisher   │   RViz2 3D Interactive Twin   │
 │   /servo_calibration       │   calibrated_joint_gui    │   URDF Kinematic Trees        │
 └─────────────┬───────────────────────────────────────────────────────┬──────────────────┘
               │ Serial /dev/ttyUSB0 (115200 baud)                     │ UDP Wi-Fi (Port 5000)
               ▼                                                       ▼
 ┌───────────────────────────────────────────────┐ ┌──────────────────────────────────────┐
 │             PHYSICAL ROBOTS                   │ │          SPATIAL PERCEPTION          │
 │  • 5-DOF Arm (MG996R Servos + Mimic Gripper)  │ │  • ESP32-CAM (JPEG Slice Stream)     │
 │  • Quard Bot (8-Servo Quadruped + PCA9685)    │ │  • VL53L1X ToF (Distance mm)         │
 │  • Rover Base (Dual BTS7960 43A H-Bridges)    │ │  • Sparse 3D Triangulation / Open3D  │
 └───────────────────────────────────────────────┘ └──────────────────────────────────────┘
```

---

<!-- Slide 4: Subsystem 1 - 5-DOF Robotic Arm -->
# 🦾 Subsystem 1: 5-DOF Articulated Robotic Arm

<div class="grid-2">
<div>

### Kinematic Specifications
- **Base Turntable Yaw**: $\pm 180^\circ$ continuous rotational base.
- **Shoulder Pitch (Dual-Actuated)**: Two high-torque MG996R servos mechanically coupled on GPIO 19 & 21 with live synchronization trim.
- **Elbow 1 & 2 Pitch**: Upper & lower forearm pitch joints.
- **Wrist Roll/Twist**: Orientation joint (GPIO 27).
- **End-Effector Parallel Gripper**: Intermeshing geared linkage.

### Digital Twin & Calibration
- Full URDF with visual & collision STL meshes.
- Persistent configuration: `servo_calibration.json`.
- Smooth S-curve homing (`2.0s` duration).

</div>
<div>

### Kinematic Joint Mapping

| Joint ID | Real GPIO | Motion Role | Limits |
|:---|:---:|:---|:---:|
| `turntable_link_joint_dup` | GPIO 18 | Base Turntable Yaw | $0^\circ - 180^\circ$ |
| `turntable_link_joint` | GPIO 19/21 | Dual Shoulder Pitch | $9^\circ - 62^\circ$ |
| `turntable_link_joint_dup_1` | GPIO 22 | Elbow 1 Pitch | $57^\circ - 123^\circ$ |
| `turntable_link_joint_dup_2` | GPIO 23 | Elbow 2 Pitch | $0^\circ - 180^\circ$ |
| `wrist_twist_joint` | GPIO 27 | Wrist Twist | $0^\circ - 177^\circ$ |
| `grip` | GPIO 26 | Primary Gripper | $0^\circ - 180^\circ$ |

</div>
</div>

---

<!-- Slide 5: Parallel Geared Gripper Mechanism -->
# ⚙️ Gripper Kinematics: 5-Link Mimic Gear Train

<div class="highlight-box">
  <b>Problem:</b> Standard parallel grippers require bulky linear actuators or multiple motors, adding weight to the arm's end-effector.  
  <b>Solution:</b> A single MG996R servo drives a primary spur gear; five mechanical mimic links maintain perfectly parallel orientation.
</div>

### URDF Mimic Joint Multiplier Matrix

| Joint Name | Role in Assembly | Parent Joint | Mimic Multiplier | Kinematic Effect |
|:---|:---|:---|:---:|:---|
| `Gripper+Rod+Geared_left_link` | Primary Drive Gear | *Actuator* | **1.0 (Master)** | Direct servo rotation |
| `Gripper+Rod+Geared_right_link`| Opposite Spur Gear | Primary | **-1.0** | Counter-rotates right jaw |
| `left_parallel_link_joint` | Left 4-Bar Follower | Primary | **+1.0** | Parallels left linkage |
| `right_parallel_link_joint` | Right 4-Bar Follower | Primary | **-1.0** | Parallels right linkage |
| `Gripper+finger+Geared_left` | Left Finger Tip | Primary | **-1.0** | Counter-rotates to stay facing front |
| `Gripper+finger+Geared_right`| Right Finger Tip | Primary | **+1.0** | Counter-rotates to stay facing front |

**Result:** True parallel grasping without tip angular divergence across the entire stroke.

---

<!-- Slide 6: Subsystem 2 - Quard Bot (Sesame Quadruped) -->
# 🤖 Subsystem 2: Quard Bot (Sesame 8-Servo Quadruped)

<div class="grid-2">
<div>

### Biomimetic Walking Robot
- **8 Degrees of Freedom**: 2-DOF per leg (Hip + Foot/Knee across 4 legs: Front-Left, Front-Right, Rear-Left, Rear-Right).
- **Dedicated Co-Processor**: PCA9685 16-Channel 12-bit PWM I2C servo controller (reduces ESP32 jitter to zero).
- **Animated Expressive Face**: 0.96" SSD1306 OLED display displaying interactive emotional states (`Happy`, `Blinking`, `Walking`, `Wave`, `Sleepy`).
- **3D Printed Chassis**: 10 modular interlocking STL components (chassis frame, hip brackets, leg links, dust covers).

</div>
<div>

### PCA9685 8-Channel Channel Allocation

| Channel | Leg | Joint Description | Neutral Angle |
|:---:|:---:|:---|:---:|
| **Ch 0** | R1 | Right Front Hip | $90^\circ$ |
| **Ch 1** | R2 | Right Rear Hip | $0^\circ$ |
| **Ch 2** | L1 | Left Front Hip | $0^\circ$ |
| **Ch 3** | L2 | Left Rear Hip | $90^\circ$ |
| **Ch 4** | R4 | Right Rear Foot | $90^\circ$ |
| **Ch 5** | R3 | Right Front Foot | $0^\circ$ |
| **Ch 6** | L3 | Left Front Foot | $90^\circ$ |
| **Ch 7** | L4 | Left Rear Foot | $0^\circ$ |

</div>
</div>

---

<!-- Slide 7: Quadruped Embedded Engineering -->
# ⚡ Embedded Engineering: Anti-Brownout & Dual I2C

<div class="grid-2">
<div>

### 1. Dual Independent I2C Busses
To prevent bus contention and display refresh frame lag:
- **Wire 0 (GPIO 21 SDA / GPIO 22 SCL)**: Dedicated exclusively to high-rate PCA9685 PWM updates (`400 kHz`).
- **Wire 1 (GPIO 18 SDA / GPIO 19 SCL)**: Dedicated to SSD1306 animated OLED face graphics.

### 2. Brownout Mitigation Engine
Sudden simultaneous 8-servo movement triggers severe $V_{CC}$ voltage drops:
- **Staggered Soft-Start**: Servos initialize with a forced 35ms staggered interval.
- **Hardware Register Override**: ESP32 core brownout detector calibrated via `WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0)` during motor pulses.

</div>
<div>

### 3. Integrated Dual-Mode Control Server
- **STA Mode**: Auto-connects to primary operational Wi-Fi network (`Sabo`).
- **AP Fallback**: Broadcasts captive portal `Sesame-Robot-Control` (IP: `192.168.4.1`).
- **Interactive Web Interface**:
  - Full touch-friendly D-Pad (Forward, Reverse, Left/Right Turn, Neutral).
  - One-click macros: HI Wave, 45° Stance, 60° High Stance.
  - Real-time 8-slider independent calibration panel.

</div>
</div>

---

<!-- Slide 8: Subsystem 3 - Heavy-Duty Rover Base -->
# 🚀 Subsystem 3: Dual BTS7960 43A Heavy Rover

<div class="grid-2">
<div>

### High-Power Differential Drive Base
- **Dual BTS7960 H-Bridges**: Capable of supplying up to **43A peak current** per motor channel.
- **LEDC Hardware PWM**: ESP32 4-channel hardware PWM generating high-frequency motor drive signals for smooth low-speed torque.
- **Wireless Teleoperation**: Bluetooth Classic (`BluetoothSerial`) pairing as `ESP32_ROBOT` with sub-10ms command latency.

### Pinout & Control Mapping
- **Left Motor Forward/Rev**: GPIO 25 (`L_RPWM`) / GPIO 26 (`L_LPWM`)
- **Right Motor Forward/Rev**: GPIO 27 (`R_RPWM`) / GPIO 14 (`R_LPWM`)
- **Driver Enable Gates**: GPIO 32 (Left) / GPIO 33 (Right)

</div>
<div>

### Real-Time Command Protocol & Watchdog

```text
  [Joystick / Teleop Host]
             │ Bluetooth Classic /dev/rfcomm
             ▼
  ['F'] ──► Forward Drive (Both PWMs active)
  ['B'] ──► Reverse Drive
  ['L'] ──► Pivot Left (Differential Counter-Spin)
  ['R'] ──► Pivot Right
  ['G','I','H','J'] ──► Smooth Diagonal Curves
  ['0'..'9','q']    ──► Dynamic PWM Duty Scaling
             │
   [Safety Watchdog Timer]
             ▼
   Auto-Brakes motors if no packet received within 350ms!
```

</div>
</div>

---

<!-- Slide 9: Subsystem 4 - AI Gesture Teleoperation -->
# 👁️ Subsystem 4: AI Body Gesture Teleoperation

<div class="highlight-box">
  <b>Natural Human-Robot Interface:</b> No wearable sensors or joysticks required. A standard RGB webcam tracks human arm motion and mirrors it to the physical robot and digital twin in real time.
</div>

<div class="grid-2">
<div>

### Pipeline Steps
1. **Video Capture**: 30 FPS input from `/dev/video0`.
2. **Landmark Extraction**: Google MediaPipe Upper-Body Pose tracking (Shoulder, Elbow, Wrist, Index).
3. **Trigonometric Joint Mapping**:
   $$\theta_{\text{pitch}} = \arctan2(y_{\text{wrist}} - y_{\text{elbow}}, x_{\text{wrist}} - x_{\text{elbow}})$$
4. **Jitter Rejection**: Exponential moving average filtering ($\alpha = 0.35$) + deadzone thresholding.
5. **Simultaneous Dual-Stream Broadcast**:
   - Packets to physical ESP32 over serial.
   - Joint states to ROS 2 `/joint_states` topic.

</div>
<div>

### Live Visual Feedback
- **OpenCV HUD Window**: Renders human skeletal overlay, computed servo angles, and link vectors.
- **RViz2 3D Viewport**: Digital twin mirrors human pose simultaneously with zero noticeable lag.
- **Failsafe Grip Detection**: Hand distance triggers automatic open/close commands for the parallel gripper.

</div>
</div>

---

<!-- Slide 10: Subsystem 5 - Visual SLAM & 3D Mapping -->
# 🗺️ Subsystem 5: ESP32-CAM + ToF Visual SLAM

<div class="grid-2">
<div>

### Solving Monocular Scale Ambiguity
Monocular cameras cannot determine absolute metric distance without depth sensors.  
**Innovation:** We fuse an **AI Thinker ESP32-CAM** with a **VL53L1X Time-of-Flight (ToF) laser rangefinder** streaming over high-speed UDP Wi-Fi!

### Binary UDP Protocol (`<IHHH`, 10 Bytes)
- `uint32_t frameID`: Monotonic frame counter.
- `uint16_t packetIndex`: Chunk slice index within frame.
- `uint16_t totalPackets`: Total packets per frame.
- `uint16_t distanceMM`: Real-time laser reading in mm.

</div>
<div>

### Algorithmic Processing Chain
1. **JPEG Slice Reassembly**: Low-latency single-slot queue with frame dropping.
2. **Camera Undistortion**: Intrinsics ($f_x, f_y, c_x, c_y, k_1, k_2, p_1, p_2$).
3. **ORB Feature Tracking**: 2000 features + Lowe's ratio test ($0.75$) + RANSAC Essential Matrix.
4. **Sensor Fusion**: ToF metric laser filtering sets absolute metric translation scale.
5. **3D Reconstruction**: Multi-view triangulation (`triangulatePoints`), voxel grid filtering, and Open3D camera trajectory frustum.

</div>
</div>

---

<!-- Slide 11: Subsystem 6 - Unified Cockpit Dashboards -->
# 🖥️ Subsystem 6: Multi-Platform Command Cockpits

<div class="grid-2">
<div>

### 1. PyQt5 Desktop Cockpit (`calibrated_joint_gui`)
- **Embedded RViz2 Container**: 3D digital twin rendered directly inside the Qt window.
- **4 Dedicated Subsystem Tabs**:
  - `🦾 Robot Arm`: Slider limits, S-curve homing, sync offsets.
  - `🤖 Quard Bot`: Gait triggers & stance buttons.
  - `🚀 Rover`: Motor speeds & steering overrides.
  - `👁️ Visual SLAM`: Live camera stream & feature matches.
- **Live Terminal & Telemetry**: Serial & Wi-Fi console with connection heartbeat badges (🟢 Connected / 🔴 Disconnected).

</div>
<div>

### 2. WebGL 3D Web Dashboard (`web_dashboard`)
- **Zero-Install Browser Access**: Runs on `http://localhost:8000`.
- **Three.js 3D Viewport**: Interactive orbit/pan/zoom view of the robot models.
- **REST & WebSocket API**: Fast bi-directional communication between mobile phones/tablets and ROS 2 nodes.
- **Universal Cross-Platform**: Works across Linux, macOS, Android, and iOS.

</div>
</div>

---

<!-- Slide 12: CAD, Rigging & Digital Twin Toolchain -->
# 📐 CAD to Digital Twin Toolchain

<div class="highlight-box">
  <b>Flawless Asset Pipeline:</b> Automated toolchain from physical 3D print assets to simulation-ready kinematic trees.
</div>

```text
 [3MF / Blender CAD Assets] 
           │
           ├──► scripts/convert_3mf_to_stl.py ──► 29 Valid Binary STL Mesh Files
           │
           └──► Blender 5.2.0 LTS + Phobos Add-on (Custom Patched):
                 • deriveGeometry: Auto-defaults unassigned meshes
                 • deriveJoint: Fixes revolute type definitions
                 • add_aggregate: Eliminates duplicate joint name collisions
                       │
                       ▼
                 [unnamed.urdf] 
                       │
                       ├──► RViz2 Interactive Digital Twin (arm_view.rviz)
                       └──► Gazebo Physics Engine Ready (unnamed_gazebo.urdf)
```

---

<!-- Slide 13: Hardware Bill of Materials (BOM) -->
# 📋 Hardware Bill of Materials (BOM)

| Component | Specifications | Subsystem | Purpose |
|:---|:---|:---:|:---|
| **ESP32 NodeMCU-32S** | Dual-core 240MHz, Wi-Fi & Bluetooth Classic | Arm, Rover, Quard | Master microcontrollers |
| **ESP32-CAM (AI Thinker)** | OV2640 2MP Camera module + Wi-Fi | Visual SLAM | Video streaming & edge vision |
| **PCA9685 Driver** | 16-Channel 12-Bit PWM I2C Controller | Quard Bot | High-resolution servo actuation |
| **VL53L1X ToF Sensor** | 4-meter Class 1 laser ranging sensor (I2C) | Visual SLAM | Monocular metric scale anchor |
| **SSD1306 OLED** | 0.96" 128x64 I2C monochrome display | Quard Bot | Dynamic facial expressions |
| **BTS7960 Driver** | Dual 43A H-Bridge motor driver module | Rover Base | High-current DC drive motors |
| **MG996R Servos** | 11 kg·cm metal gear high-torque servos | Robot Arm | Shoulder, elbow, wrist & gripper |
| **SG90 / MG90S Servos** | Micro 9g / metal gear servos (8 units) | Quard Bot | Quadruped leg articulation |
| **5V / 10A UBEC** | High-efficiency buck power supply | Power Rail | Clean high-current servo power |

---

<!-- Slide 14: Software Stack & Dependencies -->
# 💻 Complete Software Stack

<div class="grid-2">
<div>

### Middleware & Robotics
- **ROS 2 Distribution**: ROS 2 Jazzy Jalisco (`Ubuntu 24.04 / Zorin OS 17`)
- **Robot Description**: URDF, Xacro, Robot State Publisher
- **Visualization**: RViz2, Gazebo Harmonic, Open3D, Three.js
- **Build Systems**: Colcon (`ros2_ws`), PlatformIO Core, CMake

### Embedded Firmware
- **Framework**: Arduino C++ on ESP32 / ESP8266
- **Drivers**: `Adafruit_PWMServoDriver`, `Adafruit_SSD1306`, `BluetoothSerial`, `VL53L1X`
- **Networking**: Asynchronous WebServer, UDP Sockets

</div>
<div>

### Computer Vision & AI
- **Pose Detection**: Google MediaPipe Pose Solution
- **Visual Odometry**: OpenCV (`cv2.ORB`, `recoverPose`, `findEssentialMat`)
- **Point Cloud / Mesh**: Open3D, Trimesh, NumPy, SciPy
- **GUI Framework**: PyQt5 with embedded native X11 window embedding (`QWindow.fromWinId`)

</div>
</div>

---

<!-- Slide 15: Live Demonstration Playbook -->
# 🎬 Live Demonstration Playbook (For Presenters)

### 1. Launch Robot Arm Digital Twin + RViz2
```bash
./launch_dashboard.sh --arm
# Launches PyQt5 Master Cockpit + embedded RViz2 viewport
```

### 2. Launch AI Body Gesture Teleoperation
```bash
./launch_gesture_teleop.sh
# Connects webcam, tracks human posture, drives arm servos & RViz twin
```

### 3. Launch WebGL 3D Browser Dashboard
```bash
./launch_web_dashboard.sh
# Open http://localhost:8000 on laptop or mobile browser
```

### 4. Launch ESP32-CAM Visual SLAM & 3D Point Cloud
```bash
cd lidr_slam && ./run.sh
# Streams video over UDP, estimates metric scale, displays 3D point cloud
```

---

<!-- Slide 16: Key Engineering Achievements -->
# 🏆 Key Engineering Achievements & Impact

<div class="grid-2">
<div>

### ✅ What We Built
- **Zero-Lag Teleoperation**: Real-time human kinematics translation to multi-axis physical servos.
- **True Parallel Gripping**: Designed and simulated complex 5-link mimic gear kinematics in pure URDF.
- **Scale-Aware Monocular SLAM**: Overcame standard monocular scale drift by pioneering laser ToF range fusion.
- **Rock-Solid Electrical Design**: Zero brownouts across 8 simultaneous servos via software-driven staggered starts.

</div>
<div>

### 🚀 Industrial & Research Relevance
- **Educational Robotics**: Accessible platform for teaching kinematics, digital twins, and ROS 2.
- **Hazardous Operations**: Contactless teleoperation allows operators to safely manipulate objects at distance.
- **Low-Cost Autonomy**: Demonstrates that high-end SLAM and quadruped walking are achievable on accessible microcontrollers ($<\$100$ total BOM).

</div>
</div>

---

<!-- Slide 17: Roadmap & Next Steps -->
# 🔮 Future Roadmap & Q&A

<div class="grid-2">
<div>

### Phase 2 Milestones
1. **Nav2 Mobile Manipulation**: Mount the 5-DOF robotic arm on the BTS7960 Rover base for autonomous pick-and-place navigation.
2. **Dense 3D Occupancy Grids**: Upgrade sparse ORB point cloud to dense RTAB-Map or Neural Radiance Fields (NeRF/Gaussian Splatting).
3. **Terrain-Adaptive Quadruped Gaits**: Implement IMU-driven closed-loop stabilization for walking on inclines and rough ground.

</div>
<div>

### ❓ Questions & Discussion

**Repository:** `Code-for-India / Hardware`  
**License:** Open Source (GPL / Apache 2.0)  
**Live Demo:** Ready for physical & virtual trials!

<br>

*Thank you for your time!* 🦾✨

</div>
</div>
