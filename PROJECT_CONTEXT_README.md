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
├── README.md                              # Main quick-start and system overview
├── PROJECT_CONTEXT_README.md              # Detailed project context & agent handoff documentation
├── launch_dashboard.sh                    # Main launcher (Arm by default, flags: --quard, --dual, --web)
├── launch_quard_dashboard.sh              # One-click launcher for Quard Bot (Quadruped)
├── launch_dual_dashboard.sh               # One-click launcher for Dual Robots simultaneous view
├── launch_web_dashboard.sh                # One-click launcher for Three.js WebGL twin
├── launch_gesture_teleop.sh               # MediaPipe hand gesture teleoperation launcher
├── docs/                                  # Documentation guides
│   ├── QUICK_INSTALL_GUIDE.md             # Fast step-by-step ROS 2 & RViz2 setup
│   └── DASHBOARD_AND_RVIZ_GUIDE.md        # Architecture, RViz docking & safety filtering guide
├── platformio.ini                         # PlatformIO configuration for ESP32/Arduino firmware
├── extracted_stls/                        # 29 binary STL files extracted from 3MF archives
├── scripts/
│   └── convert_3mf_to_stl.py              # Python utility to convert 3MF archives into binary STL files
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
│   ├── arm/                               # 6-DOF Arm URDF, meshes, and tuned rviz config
│   ├── quard_bot/                         # Quard Bot URDF, meshes, and tuned rviz config
│   └── dual_robots.urdf                   # Combined multi-robot URDF & rviz config
└── ros2_ws/                               # ROS 2 Colcon Workspace
    └── src/
        └── servo_joint_publisher/
            ├── package.xml
            ├── setup.py
            ├── setup.cfg
            └── servo_joint_publisher/
                ├── __init__.py
                ├── calibrated_joint_gui.py   # PyQt5 Dashboard with embedded RViz2 container
                ├── servo_serial_commander.py # /joint_states → arm ESP32 (serial or Wi-Fi UDP), owns the serial port
                ├── servo_serial_publisher.py # Microcontroller USB serial state reader node
                ├── servo_simulator.py        # Automated 50 Hz sinusoidal trajectory generator node
                └── web_dashboard/            # Three.js WebGL Digital Twin server & frontend
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

The Phobos add-on (installed in `~/.config/blender/4.2/scripts/addons/phobos/`, `~/.config/blender/5.2/scripts/addons/phobos/`, and Flatpak `~/.var/app/org.blender.Blender/config/blender/5.2/scripts/addons/phobos/`) has been patched to fix GUI crashes, undefined geometry errors, and export exceptions:

1. **`deriveGeometry` in `blender2phobos.py`**:
   ```python
   if 'geometry/type' not in obj:
       obj['geometry/type'] = 'mesh' if obj.type == 'MESH' else 'box'
   ```
   *Prevents `AttributeError: The geometry of object ... has not yet been defined` when clicking or deriving visual/collision meshes.*

2. **`deriveJoint` in `blender2phobos.py`**:
   ```python
   if "joint/type" not in values:
       values["joint/type"] = "revolute"
   ```
   *Prevents `KeyError: joint/type not defined` during derivation and export.*

3. **`deriveRepresentation` in `blender2phobos.py`**:
   ```python
   elif obj.phobostype == 'link':
       repr_instance = deriveLink(obj, logging=logging)
   ```
   *Properly derives link representations and catches derivation errors cleanly without crashing.*

4. **`PhobosPropertyInformationPanel.draw` in `phobosgui.py`**:
   - Wrapped derivation calls in `try...except` blocks so that panel redrawing never throws unhandled errors or freezes the 3D viewport.
   - Handled root links (`sUtils.isRoot(obj)`) cleanly without joint/motor errors.
   - Added `update_phobostype` callback to automatically assign `geometry/type` whenever an object's Phobostype is switched to `visual` or `collision`.

5. **`_MessageWithBox` in `phoboslog.py`**:
   - Added `not bpy.app.background` guard before `popup_menu` to prevent freezes in headless/batch execution.

6. **`add_aggregate` in `xmlrobot.py`**:
   - Automatic unique suffix incrementing (`elem.name = f"{orig_name}_{idx}"`) to prevent `AssertionError` on duplicate joints or child links.

7. **`export` in `robot.py`**:
   - Normalized `export_config` so list of strings (e.g. `['urdf']`) are parsed into configuration dicts without raising `AttributeError: 'str' object has no attribute 'get'`.
   - Fixed `TypeError: unsupported operand type(s) for |=: 'bool' and 'NoneType'` via `ros_pkg |= bool(export.get("ros_pathes", False))`.

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
- [x] ROS 2 Jazzy, RViz2, and `xdotool` installation documented in `docs/QUICK_INSTALL_GUIDE.md`.
- [x] PyQt5 Dashboard (`calibrated_joint_gui.py`) with side-by-side native RViz2 3D viewport embedding.
- [x] Strict PID and window-class (`WM_CLASS == "rviz2"`) isolation to safeguard host IDE (Antigravity/VS Code) windows.
- [x] Three.js WebGL Digital Twin fallback server running independently at `http://localhost:8000`.
- [x] Dual-robot simultaneous visualization support (`launch_dual_dashboard.sh`).
- [x] Phobos add-on installed and configured for Blender 5.2.0 LTS.
- [x] All 29 STL mesh files converted to valid binary STL format.
- [x] Phobos export bug fixes applied directly to `blender2phobos.py` and `xmlrobot.py`.
- [x] `unnamed.urdf` mesh paths updated to absolute `file://` URLs for RViz2 rendering.
- [x] Mimic joint sign conventions documented for parallel gear gripper.
- [x] Software E-STOP (button / Esc) in both dashboards: stops sim, homing, gesture mode and Quard gaits, sends `stand`, locks motion controls.
- [x] Desktop app theme centralised (`THEME`, `apply_app_theme`, `button_qss` in `calibrated_joint_gui.py`); window layout saved to `~/.config/CodeForIndia/RobotDashboard.conf`.
- [x] Web dashboard: light/dark + responsive layout, real status badges (`/api/state` reports `ros_bridge` and `comm_mode`), toasts instead of `alert()`, saved limits/home loaded on start-up.
- [x] Quard commands match the firmware: channel moves send `deg` (not `angle`); active gait is resent every 1 s to satisfy the 2 s `LOCOMOTION_WATCHDOG_MS`.
- [ ] Quard firmware has no `face` command; the OLED face buttons get `400 Bad Args`.
- [ ] Desktop app: arm **Start Sim** also starts the Quard Bot walking. Decide whether to decouple them.
- [ ] `launch_gesture_teleop.sh` and `launch_quard_rviz.sh` still hardcode `/home/sabo/Documents/learn_/Hardware`.

See [`docs/DASHBOARD_AND_RVIZ_GUIDE.md`](docs/DASHBOARD_AND_RVIZ_GUIDE.md) for the user-facing description of both dashboards.
