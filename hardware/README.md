# hardware/: electronics

| Folder | Contents |
|---|---|
| `robotic_arm_motherboard/` | Carrier PCB for the arm: an ESP32 + PCA9685 socket board with power protection, E-STOP gate and ToF headers. It holds the KiCad project, Gerbers (`production_gerbers/`, `*_gerbers.zip`), BOM, pick-and-place, STEP and PDF. `build_carrier_motherboard.py` generates the PCB, and `gen_3d_mount.py` generates the PCA9685 bracket (`.scad`/`.stl`). Both write next to themselves. Details are in its `README.md`. The workflow diagram is `../docs/diagrams/robotic_arm_motherboard.workflow.html`. |
| `bts_motor_control/` | Dual BTS7960 43 A H-bridge driver: wiring, Bluetooth protocol and a standalone firmware copy (`firmware/`). The firmware that PlatformIO builds is `../src/bts7960_motor_bt.cpp` (env `bts_motor_control`). |
