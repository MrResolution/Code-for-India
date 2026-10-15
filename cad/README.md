# cad/: source CAD (not loaded at runtime)

Blender scenes, printable parts and raw exports. Nothing in this folder is read by the dashboards, RViz or the launch scripts: runtime meshes live in `../urdf/<robot>/meshes/`. Large binaries (about 140 MB, mostly `.blend`).

| Path | Contents |
|---|---|
| `arm/Arms.blend` | Main arm scene (the `Arms_backup*.blend` files are older snapshots) |
| `arm/Mini_bot.blend`, `arm/Mini_bot_movi.blend` | Other Blender scenes (not used by any script) |
| `arm/Gripper+*.stl`, `Robot+Arm+Wrist.stl`, `mg996r-v17.stl`, `MG996R_servo.sldprt` | Gripper and servo parts |
| `arm/Robotic+Arm+with+Servo+&+Arduino.3mf` | Original arm model; `scripts/convert_3mf_to_stl.py` extracts it to `arm/extracted_stls/` |
| `arm/solidworks_meshes/` | Older SolidWorks arm export (`Base.STL`, `BaseArm.STL`, …), unused by the current URDF |
| `arm/assets/` | Blender materials and textures |
| `quard_bot/stl/` | 11 printable Quard Bot parts (L1–L4, R1–R4, frame, covers); mapping in `docs/QUARD_BOT.md` |
| `dogu/Dogu.blend` | Dogu scene, in decimetres, laid out flat. `scripts/dogu/export_dogu_from_blender.py` reads it |
| `dogu/stl/` | Dogu parts as supplied (`FemurLeft/Right`, `TibiaL/R`, `HipFrontRight-BackLeft`, `jd 1`, `jd 2`) |
| `dogu/phobos_export/` | Raw Phobos URDF/SMURF export of Dogu (10× scale, inconsistent names), kept for reference. The runtime model is `urdf/dogu/` |

To change a runtime model:
- **Dogu:** re-run the scripts in `scripts/dogu/` (see `urdf/dogu/README.md`).
- **Arm and Quard Bot:** export new meshes into `urdf/<robot>/meshes/`, then update the URDF.
