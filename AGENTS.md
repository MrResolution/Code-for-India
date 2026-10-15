# Agent Guide

Instructions for AI coding agents (and people) working on this repository. Read [PROJECT_MAP.md](PROJECT_MAP.md) first: it says where everything is, how the pieces connect, and the pitfalls.

## The setup
- The robots and ROS 2 Jazzy run on **Linux** (Ubuntu 24.04). A Windows or macOS checkout is for editing and UI work only. There is no ROS there, and both dashboards run "UI only".
- The canonical remote is `https://github.com/MrResolution/Code-for-India`.
- Physical hardware may be attached when someone runs this code. Anything that sends motion must respect E-STOP and the calibrated limits.

## Fast orientation
| Task | Start at |
|---|---|
| Desktop app UI | `ros2_ws/src/servo_joint_publisher/servo_joint_publisher/calibrated_joint_gui.py` (search `class .*Widget`) |
| Web dashboard UI | `.../web_dashboard/static/{index.html,app.js,style.css}`, `.../web_dashboard/server.py` |
| A robot's firmware | `platformio.ini` env → `src/<file>.cpp` (table in PROJECT_MAP §3) |
| A robot model | `urdf/<robot>/`. Dogu is generated: change `scripts/dogu/build_dogu.py` and re-run it |
| Launch behaviour | `launch_dashboard.sh` → `urdf/unnamed/display_ros2.launch.py` |

## Rules
1. **Keep the runtime layout.** These paths are loaded by code or by launch scripts:
   - `servo_calibration.json` and `urdf/` at the root (repo-root markers)
   - `urdf/{arm,quard_bot,dogu,unnamed}`, `ros2_ws/`
   - PlatformIO's `src/`, `include/`, `lib/` and `test/`
   - the `launch_*.sh` scripts

   If you must move one, grep for every reference first and update this file and PROJECT_MAP.md.
2. **Source CAD goes in `cad/<robot>/`, and boards in `hardware/`.** Nothing in either folder may be loaded at runtime. Runtime meshes belong under `urdf/<robot>/meshes/`, with names unique across robots.
3. **Don't hand-edit generated files** (PROJECT_MAP §7). Change the generator and re-run it.
4. **Motion safety.** Every new way to move a robot (or the Dogu model) must:
   - stop when E-STOP engages (`engage_estop()` / `engageEstop()`)
   - refuse to start while it is engaged
   - respect `servo_calibration.json` limits for the arm
5. **No fake data.** Show `--` or "No data" until a real device answers. Simulators must say they are simulators.
6. **Same feature in both UIs.** The desktop app and the web dashboard mirror each other: tabs, link cards, consoles, E-STOP. Change both, or note the gap in `docs/PROJECT_CONTEXT.md` §6.
7. **Unique joint names.** All robots share one `/joint_states`.
8. **Line endings.** Shell scripts must be LF (`.gitattributes` enforces it). Don't convert files you didn't otherwise change.
9. **Secrets.** Wi-Fi credentials belong in `include/secrets.h` (gitignored; template `include/secrets.h.example`), never in docs or code.
10. **Docs travel with the change.**
    - User-visible behaviour goes in `docs/DASHBOARD_AND_RVIZ_GUIDE.md`.
    - Layout, topic, port or API changes go in `PROJECT_MAP.md`.
    - Open problems go in the `docs/PROJECT_CONTEXT.md` §6 checklist.
11. **Tests.** Tests must never write the real `servo_calibration.json` or Qt settings: point them at a temporary copy.

## Checks before you finish
Run what applies to the files you touched (from the repo root):

```bash
node --check ros2_ws/src/servo_joint_publisher/servo_joint_publisher/web_dashboard/static/app.js
python3 -m py_compile ros2_ws/src/servo_joint_publisher/servo_joint_publisher/*.py \
    ros2_ws/src/servo_joint_publisher/servo_joint_publisher/web_dashboard/server.py \
    urdf/unnamed/display_ros2.launch.py scripts/dogu/*.py
bash -n launch_*.sh run_all.sh
```

- **Web UI without ROS:**
  1. Run `python3 ros2_ws/src/servo_joint_publisher/servo_joint_publisher/web_dashboard/server.py --port 8765`.
  2. Open `http://127.0.0.1:8765` and check every tab at desktop width and at 375 px.
  3. Check the browser console for errors.
- **Desktop app without a display:**
  1. Set `QT_QPA_PLATFORM=offscreen`. On Windows, also set `QT_QPA_FONTDIR=C:/Windows/Fonts` so text renders.
  2. Construct `CalibratedJointPublisherGUI` with a stub node, then `grab()` screenshots.
- **Dogu model:** run `python3 scripts/dogu/build_dogu.py`. It checks the foot IK error (≤ 2 mm) and joint ranges (≤ 90°), and fails loudly.
- **Firmware:** `pio run -e <env>` (needs PlatformIO).
- **On the robot machine only:**
  - `cd ros2_ws && colcon build`
  - `./launch_dashboard.sh --<robot>`
