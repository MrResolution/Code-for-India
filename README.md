# 🦾 Code for India — Multi-DOF Robotic Arm

> ROS 2 Jazzy · Blender 5.2.0 · ESP32/Arduino · PlatformIO

A complete robotics project featuring a **multi-DOF robotic arm with a parallel geared gripper**, including 3D models, URDF descriptions, ROS 2 visualization & simulation nodes, and microcontroller firmware.

---

## 📂 Project Structure

```text
.
├── arm/                        # Blender 3D models & STL part files
│   ├── Arms.blend              # Main Blender project (robotic arm assembly)
│   └── *.stl                   # Individual 3D-printable gripper & arm parts
├── extracted_stls/             # STL files extracted from 3MF archives
├── urdf/                       # Robot descriptions & visualization configs
│   ├── meshes/                 # STL meshes for URDF links
│   └── unnamed/                # Phobos-exported complete robot model
│       ├── urdf/               # Full URDF with joints, mimic tags
│       └── meshes/stl/         # Exported mesh files
├── ros2_ws/                    # ROS 2 Colcon workspace
│   └── src/servo_joint_publisher/
│       ├── servo_serial_publisher.py   # USB serial joint state reader
│       └── servo_simulator.py          # Automated trajectory generator
├── src/                        # Microcontroller firmware (PlatformIO)
│   ├── main.cpp                # VL53L1X ToF sensor code
│   ├── main_servo.cpp          # Servo firmware (streams angle data)
│   ├── bts7960_motor_bt.cpp    # ESP32 Bluetooth BTS7960 motor driver firmware
│   └── quadruped_sesame.cpp    # Sesame 8-servo quadruped robot firmware
├── Quard bot/                  # Sesame Quadruped Robot STLs & firmware
│   ├── stl/                    # 10x 3D printable STL mesh parts (L1..L4, R1..R4, Frame)
│   ├── firmware/               # ESP32 firmware (PCA9685, SSD1306 OLED, WebServer)
│   └── README.md               # Pinout mapping, OLED graphics & Web AP guide
├── bts_motor_control/          # Dual BTS7960 43A H-Bridge Motor Controller
│   ├── firmware/               # Modular & single-file C++ driver source
│   └── README.md               # BTS7960 wiring pinout & Bluetooth protocol guide


├── scripts/                    # Utility scripts
│   └── convert_3mf_to_stl.py  # 3MF → binary STL converter
├── platformio.ini              # PlatformIO config (ESP32 NodeMCU-32S)
└── PROJECT_CONTEXT_README.md   # Detailed architecture & handoff docs
```

---

## 🚀 Quick Start

### Prerequisites
- **ROS 2 Jazzy** installed (`/opt/ros/jazzy`)
- **PlatformIO** (for firmware upload)
- **Blender 5.2.0** with **Phobos** add-on (for model editing)
- ESP32 or Arduino NodeMCU connected via USB

### 1. Launch Robot Arm in RViz2
```bash
source /opt/ros/jazzy/setup.bash
ros2 launch urdf_tutorial display.launch.py \
  model:=$(pwd)/urdf/unnamed/urdf/unnamed.urdf \
  rvizconfig:=$(pwd)/urdf/arm_view.rviz
```

### 2. Run Automated Motion Simulation
```bash
source /opt/ros/jazzy/setup.bash
cd ros2_ws && colcon build && source install/setup.bash && cd ..
ros2 run servo_joint_publisher servo_simulator
```

### 3. Run USB Serial Joint Publisher (ESP32/Arduino)
```bash
source /opt/ros/jazzy/setup.bash
source ros2_ws/install/setup.bash
ros2 run servo_joint_publisher servo_serial_publisher \
  --ros-args -p port:=/dev/ttyUSB0 -p baudrate:=115200
```

### 4. Flash Microcontroller Firmware
```bash
pio run -t upload
```

---

## 🦾 Kinematic Architecture

| Joint | Type | Axis | Description |
|-------|------|------|-------------|
| `turntable_link_joint` | Revolute | Z | Base rotation (±180°) |
| `Elbow_1` | Revolute | Y | Lower shoulder |
| `Elbow_2` | Revolute | Y | Upper forearm |
| `Wrist_pitch` | Revolute | Y | Wrist pitch |
| `Gripper (primary)` | Revolute | Z | Left gear arm (actuated) |
| `Gripper (mimic ×5)` | Mimic | — | Parallel geared linkage |

The gripper uses intermeshing spur gears with 4-bar parallel follower links. A single servo drives the primary gear arm; five mimic joints replicate the motion with appropriate sign inversions to achieve parallel open/close.

---

## 🛠️ Tech Stack

- **Embedded**: ESP32 NodeMCU-32S, PlatformIO, Arduino framework
- **3D Modeling**: Blender 5.2.0 LTS + Phobos add-on
- **Robot Simulation**: ROS 2 Jazzy, URDF, RViz2
- **Sensors**: VL53L1X Time-of-Flight, MG996R Servo Motors

---

## 📝 Detailed Documentation

See [PROJECT_CONTEXT_README.md](PROJECT_CONTEXT_README.md) for:
- Full environment & system overview
- Phobos patch details
- Gripper mimic joint sign conventions
- Agent handoff checklist

---

## 📄 License

This project is open source. See individual component licenses where applicable.
