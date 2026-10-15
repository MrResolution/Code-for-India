# scripts/: build tools

Run from the repository root.

| Script | What it does | Writes |
|---|---|---|
| `dogu/export_dogu_from_blender.py` | Runs **inside Blender**: exports Dogu's per-link meshes and joint frames from `cad/dogu/Dogu.blend` (read-only) | `urdf/dogu/meshes/`, `urdf/dogu/layout_model.json` |
| `dogu/build_dogu.py` | Plain Python: solves the standing pose, re-zeroes the joints, and builds poses and looping clips | `urdf/dogu/urdf/dogu.urdf`, `model.json`, `animations.json` |
| `build_configured_urdf.py` | Generates the Quard Bot and arm URDFs from built-in link tables | `urdf/quard_bot/urdf/quard_bot.urdf`, `urdf/arm/urdf/arm.urdf` (**hardcoded `/home/sabo/...` paths**; diff before regenerating) |
| `setup_r1_r3.py` | Runs **inside Blender**: rigs Quard Bot leg links and joints (Phobos armatures) | the open `.blend` |
| `convert_3mf_to_stl.py` | Extracts every object in a `.3mf` to binary STL | `cad/arm/extracted_stls/` by default |
| `ensure_ros_build.sh` | Sourced by `launch_dashboard.sh` and `launch_web_dashboard.sh`: rebuilds `servo_joint_publisher` when `ros2_ws/install` is missing or differs from the checkout (a `--symlink-install` build is always current) | `ros2_ws/{build,install,log}/` |

```bash
blender --background --factory-startup cad/dogu/Dogu.blend --python scripts/dogu/export_dogu_from_blender.py
python3 scripts/dogu/build_dogu.py
python3 scripts/convert_3mf_to_stl.py cad/arm/Robotic+Arm+with+Servo+\&+Arduino.3mf
```
