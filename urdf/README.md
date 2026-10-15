# urdf/: runtime robot models

Everything here is loaded at runtime by RViz2 (through `unnamed/display_ros2.launch.py`) or by the web twin (through `server.py`). Source CAD lives in `../cad/`, not here.

| Folder / file | Robot | Used by |
|---|---|---|
| `arm/urdf/arm.urdf`, `arm/meshes/stl/`, `arm/rviz/arm.rviz` | 🦾 Arm | `robot:=arm`, both dashboards |
| `quard_bot/urdf/quard_bot.urdf`, `quard_bot/meshes/`, `quard_bot/rviz/` | 🤖 Quard Bot | `robot:=quard`, both dashboards, `launch_quard_rviz.sh` |
| `dual_robots.urdf`, `dual_robots.rviz` | Arm + Quard | `robot:=dual` |
| `dogu/` | 🐕 Dogu | `robot:=dogu`, both dashboards. **Generated**: see `dogu/README.md` |
| `unnamed/` | launch + tools | see below |

`arm/urdf/arm_raw_phobos.urdf` is the arm's raw Phobos export, kept for reference (relative mesh paths).

## unnamed/ (the name is historical)
| File | Purpose |
|---|---|
| `display_ros2.launch.py` | **The** RViz launch for every robot (`robot:=arm\|quard\|dual\|dogu`, `simulate:=true\|false`). Rewrites the `/home/sabo/...` mesh prefix to the real repo path. |
| `arm_simulator.py` | Sine-wave arm motion, started when `simulate:=true` (arm and dual only) |
| `robot.rviz` | Identical copy of `arm/rviz/arm.rviz` |
| `unnamed/urdf/unnamed_gazebo.urdf` | Arm model used by `launch_gesture_teleop.sh` and the Gazebo launch |
| `unnamed/urdf/unnamed.urdf`, `unnamed/smurf/` | Dogu's raw Phobos export (upstream `7944d8d`). Not used at runtime; only the legacy `fix_*.py` tools read it |
| `unnamed/urdf/unnamed_scaled.urdf` | Output of `test_scale.py` |
| `unnamed/meshes/stl/` | Meshes for the URDFs above (many names repeat arm/Quard meshes) |
| `gazebo_ros2.launch.py`, `display_manual.launch.py`, `*.sh`, `fix_*.py`, `test_scale.py` | Legacy Gazebo / Phobos post-processing tools with hardcoded `/home/sabo/...` paths |

## Mesh paths
URDFs reference meshes as `file:///home/sabo/Documents/learn_/Hardware/urdf/...`. Keep that prefix in new URDFs: `display_ros2.launch.py` swaps it for the actual location, and the web server ignores the directory and looks the file name up in the mesh folders.
