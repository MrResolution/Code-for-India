# urdf/dogu/: Dogu runtime model (generated)

Dogu is a 12-joint quadruped: a hip (abduction), femur and tibia per leg, legs `fl`, `fr`, `bl` and `br`. It has no firmware yet, so the dashboards only animate the model.

**Don't edit these files by hand.** They come from the Blender scene `cad/dogu/Dogu.blend`:

```bash
# 1. Blender → per-link meshes + joint frames (only needed when the .blend changes)
blender --background --factory-startup cad/dogu/Dogu.blend \
        --python scripts/dogu/export_dogu_from_blender.py
# 2. Frames → URDF, web model and animations (standard-library Python)
python3 scripts/dogu/build_dogu.py            # --stand-ratio --stride --lift --frames --mesh-prefix
```

| File | Made by | Read by |
|---|---|---|
| `meshes/dogu_<link>.stl` | export script (decimated, metres) | RViz (via the URDF), web twin (`/meshes/`) |
| `layout_model.json` | export script: frames with zero = the .blend's flat parts layout | `build_dogu.py` only |
| `urdf/dogu.urdf` | `build_dogu.py`: joint zero = **standing** | `display_ros2.launch.py robot:=dogu` |
| `model.json` | `build_dogu.py`: the same kinematics for Three.js | web dashboard (`/models/dogu/model.json`) |
| `animations.json` | `build_dogu.py`: poses `stand`, `crouch`, `sit`, and 24-frame looping clips `walk`, `turn_left`, `turn_right`, `wave`, `idle`, in radians from standing | desktop `DoguWidget`, web Dogu tab |
| `rviz/dogu.rviz` | hand-made (fixed frame `base_link`) | RViz |

How the build works:
- The standing pose is solved with inverse kinematics: feet 0.8 × leg reach below the femur joints, knees bent backwards (Spot style).
- That pose is folded into the joint origins, so every joint reads 0 when standing.
- The clips move the foot targets (trot gait, turn, paw wave, breathing), and each keyframe is solved with IK.
- The build fails if a foot misses its target by more than 2 mm, or if a joint goes past ±90° from standing.

The ±90° URDF limits are provisional until Dogu has servos.
