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

The directory map lives in one place, **[`PROJECT_MAP.md`](../PROJECT_MAP.md)** at the repo root, so it can't drift out of sync with copies here. It covers where to find each thing, the runtime data flow, topics and ports, and known pitfalls. Rules for changing the repo are in [`AGENTS.md`](../AGENTS.md).

In short: runtime code is in `ros2_ws/`, runtime robot models are in `urdf/`, firmware is in `src/` (PlatformIO), CAD sources are in `cad/`, PCBs are in `hardware/`, and documents are in `docs/`.

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

### A. Launch a Robot Model in RViz2
```bash
source /opt/ros/jazzy/setup.bash
cd <project-root>

# robot:= arm | quard | dual | dogu  (simulate:=true runs the arm simulator)
ros2 launch urdf/unnamed/display_ros2.launch.py robot:=arm simulate:=false
```
The launch file reads `urdf/<robot>/urdf/<robot>.urdf` and rewrites the absolute
`file:///home/sabo/Documents/learn_/Hardware` mesh prefix to wherever the repo lives.
`urdf/unnamed/urdf/unnamed.urdf` is no longer the arm: upstream commit `7944d8d`
replaced it with Dogu's raw Phobos export (also kept in `cad/dogu/phobos_export/`).

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
python3 scripts/convert_3mf_to_stl.py path/to/input.3mf cad/arm/extracted_stls
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
- [x] Link monitoring: `servo_serial_commander` publishes `/esp32_link_state` (JSON, transient local) and `/esp32_console`, and accepts `RAW:` / `LINK:` commands (raw joint CMDs clamped to limits).
- [x] Connection cards + shared console in every tab of both dashboards (`ConsoleWidget` in the Qt GUI; `ConsolePanel` + `/api/console` in the web UI). Late Quard motion commands (>2 s) are dropped, never replayed.
- [x] Dogu (12-joint quadruped) added as a model preview: `scripts/dogu/export_dogu_from_blender.py` exports `cad/dogu/Dogu.blend` to metric meshes and joint frames, `scripts/dogu/build_dogu.py` solves the standing pose (IK) and writes `urdf/dogu/` (URDF with zero = standing, `model.json`, `animations.json`). Both dashboards have a Dogu tab with poses and looping clips, and the desktop RViz switches to Dogu on that tab.
- [x] Repo reorganised: CAD sources in `cad/`, PCB and motor-driver hardware in `hardware/`, documents and diagrams in `docs/`, byte-identical duplicate STLs and firmware removed, `PROJECT_MAP.md` + `AGENTS.md` added.
- [x] No fake rover data: the simulated sensor source and every placeholder reading were removed from both dashboards and the web server. Unknown readings are `None` / "--", JSON replies only update the fields they contain, and changing source clears readings.
- [x] Desktop RViz follows the robot tab (Arm, Quard, Dogu) by setting `robot_state_publisher`'s `robot_description`; every model hangs off `world`. The launchers rebuild a stale `ros2_ws/install` (`scripts/ensure_ros_build.sh`).
- [ ] Check the desktop RViz model switch on the robot machine (ROS 2 Jazzy): Arm → Dogu → Quard → Arm, from `./launch_dashboard.sh`, `--quard`, `--dual` and `--dogu`. It is only tested offscreen with a stub node so far.
- [ ] The Dogu commits (`4a0cd8a`, `289b837` and later) are on sharveshgsarva-debug/Multifunctional-Bot `main`; MrResolution/Code-for-India `main` has no Dogu tab until `fix/remove-slam-local-dashboard` is merged there.
- [ ] Quard firmware has no `face` command; the OLED face buttons get `400 Bad Args`.
- [ ] Desktop app: arm **Start Sim** also starts the Quard Bot walking. Decide whether to decouple them.
- [ ] `launch_gesture_teleop.sh` and `launch_quard_rviz.sh` still hardcode `/home/sabo/Documents/learn_/Hardware`.
- [x] Dogu firmware added (`src/dogu.cpp`, env `dogu`): ESP32 + PCA9685, 8 walking servos + 4 held body servos, its own Wi-Fi AP, `GET /start`, `/stop`, `/status`. Both dashboards have a Robot link card (status, Start / Stop walking); E-STOP sends `/stop`. AP credentials moved to `include/secrets.h`. See `docs/DOGU.md`.
- [x] Web viewport **Twin Sync** on the Dogu tab used to fall through to the arm and re-zero its `sync_offset_deg` values, which `servo_serial_commander` applies to servo commands. It now does nothing there and says why.
- [ ] Dogu firmware has no link watchdog: if Wi-Fi drops while it walks, it keeps walking. Consider stopping after a few seconds without any request (the dashboards poll `/status` every 1 s while it walks, and the firmware's own page every 0.5 s).
- [ ] Dogu calibration is the firmware's starting values (hip 305, knee 320, body 305 ticks); the firmware warns the body values are not guaranteed safe. Measure them on the robot.
- [ ] Confirm Dogu's servo-to-model mapping: which body servo (CH8–CH11) sits on which leg, and whether firmware hip / knee are the model's femur / tibia. The model's ±90° limits are still placeholders.
- [ ] Dogu was only tested against a stand-in HTTP server (desktop offscreen and web); the firmware compiles (`pio run -e dogu`) but hasn't been run with the dashboards on the real robot.
- [ ] Upstream commit `7944d8d` raised every arm joint's `servo_max_deg` to 180 in `servo_calibration.json`. Check this against the physical end stops.
- [ ] `docs/QUARD_BOT.md` publishes the Quard Bot's Wi-Fi AP password.

See [`DASHBOARD_AND_RVIZ_GUIDE.md`](DASHBOARD_AND_RVIZ_GUIDE.md) for the user-facing description of both dashboards.
