# Complete Project Context & Handoff Guide: Robotic Arm Simulation (ROS 2 Jazzy & Blender 5.2.0)

This repository contains the complete codebase, URDF robot models, 3D STL mesh assets, Blender Phobos configuration, microcontoller serial joint publisher nodes, and ROS 2 launch pipelines for a **Multi-DOF Robotic Arm with a Geared Parallel Linkage Gripper**.

---

## 📌 1. Environment & System Overview

- **Operating System**: Ubuntu 24.04 LTS (Noble Numbat) / Zorin OS 17 (Noble-based).
- **ROS 2 Distribution**: **ROS 2 Jazzy Jalisco** (Installed in `/opt/ros/jazzy`).
- **3D Modeling & Robot Rigging**: **Blender 5.2.0 LTS** (Installed in `/opt/blender-5.2.0-linux-x64/` with `/usr/local/bin/blender` binary symlink).
- **Robot Modeling Add-on**: **Phobos** (Installed in `~/.config/blender/5.2/scripts/addons/phobos`). All Python dependencies (`pyyaml`, `scipy`, `numpy`, `pycollada`, `trimesh`, `lxml`, `networkx`, `Pillow`) are installed inside Blender 5.2's internal Python 3.13 site-packages.
- **Microcontroller**: ESP32 / Arduino NodeMCU streaming angle data over USB Serial (`/dev/ttyUSB0` @ 115200 baud).

---

## 📁 2. File & Directory Structure

```text
./
├── README.md                              # Main quick-start guide
├── PROJECT_CONTEXT_README.md              # Detailed project context & agent handoff documentation
├── platformio.ini                         # PlatformIO configuration for ESP32/Arduino firmware
├── extracted_stls/                        # 29 binary STL files extracted from 3MF archives
├── scripts/
│   └── convert_3mf_to_stl.py             # Python utility to convert 3MF archives into binary STL files
├── src/
│   ├── main.cpp                           # Original VL53L1X ToF sensor code
│   ├── main_servo.cpp                     # Microcontroller servo firmware (streams "ANGLE: 90.0")
│   ├── bts7960_motor_bt.cpp               # ESP32 Bluetooth BTS7960 motor driver firmware
│   └── quadruped_sesame.cpp               # Sesame 8-servo quadruped robot firmware
├── Quard bot/                             # Sesame Quadruped Robot CAD meshes & firmware
│   ├── README.md                          # PCA9685 pinout, OLED graphics & Web AP guide
│   ├── stl/                               # 10x 3D printable STL mesh parts (L1..L4, R1..R4, Frame, Covers)
│   └── firmware/                          # ESP32 firmware (PCA9685, SSD1306 OLED, WebServer)
├── bts_motor_control/                     # Dual BTS7960 43A H-Bridge Motor Controller
│   ├── README.md                          # BTS7960 wiring pinout & Bluetooth protocol guide
│   └── firmware/                          # Modular & single-file C++ driver source


├── urdf/
│   ├── single_servo.urdf                 # 1-DOF servo URDF model
│   ├── robot_arm.urdf                    # 5-DOF robotic arm URDF model
│   ├── servo_view.rviz                   # Tuned RViz display configuration for single servo
│   ├── arm_view.rviz                     # Tuned RViz display configuration for multi-DOF arm
│   ├── meshes/                           # Raw binary STL meshes for robot arm links
│   └── unnamed/                          # Phobos exported complete robot model
│       ├── urdf/unnamed.urdf             # Full exported URDF with links, revolute joints & mimic tags
│       └── meshes/stl/                   # Collada/STL meshes exported by Phobos
└── ros2_ws/                              # ROS 2 Colcon Workspace
    └── src/
        └── servo_joint_publisher/
            ├── package.xml
            ├── setup.py
            ├── setup.cfg
            └── servo_joint_publisher/
                ├── __init__.py
                ├── servo_serial_publisher.py # Microcontroller USB serial state reader node
                └── servo_simulator.py       # Automated 50 Hz sinusoidal trajectory generator node
```

---

## 🦾 3. Kinematic Architecture & Gripper Mechanism

### A. Arm Kinematic Chain
- **`basecase1_link`**: Fixed root base housing.
- **`turntable_link_joint`**: Revolute joint (Z-axis rotation, $-180^\circ$ to $+180^\circ$).
- **`Elbow_1_link`**: Lower shoulder link (Revolute Y-axis).
- **`Elbow_2_link`**: Upper forearm link (Revolute Y-axis).
- **`Wrist_pitch._link`**: Wrist pitch joint (Revolute Y-axis).
- **`gripper_base_link`**: Mount for the parallel gripper assembly.

### B. Geared Parallel Linkage Gripper Mechanism
The gripper consists of intermeshing spur gears and 4-bar parallel follower links:

- **Primary Servo Joint**: `Gripper+Rod+Geared_left_link` (Revolute Z-axis).
- **Mimic Joint 1 (Right Gear Arm)**: `Gripper+Rod+Geared_right_link`
  - `mimic/joint`: `Gripper+Rod+Geared_left_link`
  - `mimic/multiplier`: **`-1.0`** *(intermeshing gear turns in opposite direction)*
  - `mimic/offset`: `0.0`
- **Mimic Joint 2 (Left Parallel Link)**: `left_parallel_link_joint`
  - `mimic/joint`: `Gripper+Rod+Geared_left_link`
  - `mimic/multiplier`: **`1.0`** *(parallel link moves in same direction as left gear)*
  - `mimic/offset`: `0.0`
- **Mimic Joint 3 (Right Parallel Link)**: `right_parallel_link_joint`
  - `mimic/joint`: `Gripper+Rod+Geared_left_link`
  - `mimic/multiplier`: **`-1.0`**
  - `mimic/offset`: `0.0`
- **Mimic Joint 4 (Left Finger Tip)**: `Gripper+finger+Geared_left_link`
  - `mimic/joint`: `Gripper+Rod+Geared_left_link`
  - `mimic/multiplier`: **`-1.0`** *(counter-rotates to stay facing straight forward)*
  - `mimic/offset`: `0.0`
- **Mimic Joint 5 (Right Finger Tip)**: `Gripper+finger+Geared_right_link`
  - `mimic/joint`: `Gripper+Rod+Geared_left_link`
  - `mimic/multiplier`: **`1.0`** *(counter-rotates to stay facing straight forward)*
  - `mimic/offset`: `0.0`

---

## 🛠️ 4. Applied Patches to Phobos Add-on

The Phobos add-on inside `~/.config/blender/5.2/scripts/addons/phobos/` was patched to prevent export exceptions:

1. **`deriveGeometry` in `blender2phobos.py`**:
   ```python
   # Line 117
   if 'geometry/type' not in obj:
       obj['geometry/type'] = 'mesh'
   ```
   *Prevents `AttributeError: The geometry of object TrunTable has not yet been defined`.*

2. **`deriveJoint` in `blender2phobos.py`**:
   ```python
   # Line 491
   if "joint/type" not in values:
       values["joint/type"] = "revolute"
   ```
   *Prevents `KeyError: joint/type not defined`.*

3. **`add_aggregate` in `xmlrobot.py`**:
   ```python
   # Line 348 & Line 358
   if id(self.get_aggregate("joint", elem.name)) != id(elem):
       orig_name = elem.name
       idx = 1
       while elem.name in [str(j) for j in self.joints]:
           elem.name = f"{orig_name}_{idx}"
           idx += 1
   ```
   *Prevents `AssertionError` when duplicate joint names or duplicate child links are encountered.*

---

## 🚀 5. Execution & Simulation Commands

### A. Launch Full Exported Robot Arm in RViz2
```bash
source /opt/ros/jazzy/setup.bash
cd <project-root>

ros2 launch urdf_tutorial display.launch.py \
  model:=$(pwd)/urdf/unnamed/urdf/unnamed.urdf \
  rvizconfig:=$(pwd)/urdf/arm_view.rviz
```

### B. Run Automated Motion Simulator Node
```bash
source /opt/ros/jazzy/setup.bash
source ros2_ws/install/setup.bash

ros2 run servo_joint_publisher servo_simulator
```

### C. Run Microcontroller USB Serial Node
```bash
source /opt/ros/jazzy/setup.bash
source ros2_ws/install/setup.bash

ros2 run servo_joint_publisher servo_serial_publisher --ros-args -p port:=/dev/ttyUSB0 -p baudrate:=115200
```

### D. Convert 3MF Archives to Binary STLs
```bash
python3 scripts/convert_3mf_to_stl.py path/to/input.3mf extracted_stls
```

---

## 📝 6. Handoff Checklist for Future Agents

- [x] ROS 2 Jazzy workspace `ros2_ws` compiled cleanly with `colcon build`.
- [x] Phobos add-on installed and configured for Blender 5.2.0 LTS.
- [x] All 29 STL mesh files converted to valid binary STL format.
- [x] Phobos export bug fixes applied directly to `blender2phobos.py` and `xmlrobot.py`.
- [x] `unnamed.urdf` mesh paths updated to absolute `file://` URLs for RViz2 rendering.
- [x] Mimic joint sign conventions documented for parallel gear gripper.
