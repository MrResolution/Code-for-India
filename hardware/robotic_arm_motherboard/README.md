# 🦾 5-DOF Robotic Arm Modular Carrier Motherboard

> **Carrier PCB & Backplane** · KiCad 7 / 8 · ESP32 DevKit V1 Socket · PCA9685 Module Socket · Dual VL53L1X ToF · XT30 Power

A professional 100mm × 80mm 2-layer FR4 carrier / motherboard PCB designed to interconnect the electronic modules of the 5-DOF robotic arm without hard-soldering microcontrollers or driver ICs. All commercial off-the-shelf (COTS) modules plug directly into standard 2.54mm female pin headers.

---

## 📐 Board Overview & Specifications

| Parameter | Specification |
| :--- | :--- |
| **Dimensions** | 100.0 mm × 80.0 mm |
| **Layers** | 2-layer FR4, 1.6 mm thickness, 1 oz copper ($35\,\mu\text{m}$) |
| **Surface Finish** | HASL Lead-Free (or ENIG) |
| **Mounting Holes** | 4× M3 mounting holes at corners with 3.5 mm edge inset (93.0 mm × 73.0 mm spacing) |
| **Power Input** | XT30 connector (SERVO_V+: 5.0V – 6.0V external DC, 5A–10A capability) |
| **Protection** | 5A mini blade/SMD fuse (F1) + Schottky reverse polarity diode (D1) + 1000 µF low-ESR bulk electrolytic capacitor (C1) |
| **Logic Supply** | Onboard AMS1117-3.3 LDO regulator providing clean 3.3V rail from servo power |
| **Signal Integrity** | Ground planes on bottom layer, wide 2.0 mm power distribution traces, localized 0.1 µF & 10 µF decoupling |

---

## 🧩 Modular Socket Architecture

### 1. ESP32 DevKit V1 Socket
* Dual 1×15 female socket headers spaced **22.86 mm (0.9")** apart with standard 2.54 mm pitch.
* Sockets accept any standard 30-pin ESP32 DevKit V1 board.
* Dedicated antenna keepout zone ensuring zero Wi-Fi / Bluetooth attenuation.
* Dedicated breakout lines for I2C (GPIO21 SDA, GPIO22 SCL), UART (TX0, RX0), and ToF sensor shutdown (GPIO16 XSHUT1, GPIO17 XSHUT2).

### 2. PCA9685 16-Channel Servo Driver Module Socket & Mount
* **Direct Servo Connection**: RC servos (MG996R, SG90, DS3218, etc.) plug **directly** into the PCA9685 module's native 16×3 male header pins. No duplicate servo headers on the carrier motherboard.
* Module connects to the motherboard via a 6-pin control header: `GND`, `OE` (Active-Low Output Enable), `SCL`, `SDA`, `VCC` (3.3V), and `V+` (Servo Power).
* Accompanied by a custom 3D-printable elevated mounting bracket (`pca9685_mount_bracket.stl` and `pca9685_mount_bracket.scad`) that secures the module cleanly above the PCB.

### 3. Dual VL53L1X Time-of-Flight (ToF) Distance Sensors
* 2× 5-pin headers (`J_VL53_1`, `J_VL53_2`): `3V3`, `GND`, `SDA`, `SCL`, `XSHUT`.
* Allows dynamic I2C address reassignment at boot for dual-sensor LiDAR and obstacle ranging.

### 4. Expansion & Debug Headers
* **I2C Expansion (J_I2C_EXP)**: 4-pin (`3V3`, `GND`, `SDA`, `SCL`) with 4.7 kΩ pull-ups.
* **UART Debug (J_UART)**: 4-pin (`3V3`, `GND`, `TX`, `RX`).
* **GPIO Breakout (J_GPIO_EXP)**: 10-pin header exposing spare ESP32 GPIOs for limit switches, encoders, or buttons.
* **Safety Controls**: Emergency Stop 2-pin header (`J_ESTOP`) and hardware Servo Enable 2-pin header (`J_SRV_EN`).
* **Status LEDs**: 3.3V Power (LED1), Servo V+ Active (LED2), Arm Status (LED3), Fault/Disable (LED4).

---

## 📁 Included Files

* **KiCad Design Files**:
  * `robotic_arm_motherboard.kicad_pcb`: PCB layout (tracks, pours, footprints, silkscreen).
  * `robotic_arm_motherboard.kicad_sch`: Schematic with power, sockets, protection, and decoupling.
  * `robotic_arm_motherboard.kicad_pro`: KiCad 7/8 project configuration.
* **Fabrication & Assembly**:
  * `robotic_arm_motherboard_gerbers.zip`: Ready-to-upload ZIP for JLCPCB, PCBWay, or OSH Park.
  * `production_gerbers/`: Individual RS-274X Gerber layers and Excellon NC drill files.
  * `BOM.csv`: Complete bill of materials with references, values, and footprints.
  * `robotic_arm_motherboard-pos.csv`: Component pick-and-place centroid positions.
  * `robotic_arm_motherboard-pcb.pdf`: High-resolution vector schematic and layer plots.
* **3D Models**:
  * `robotic_arm_motherboard-3d.step`: Full 3D CAD model of the carrier board.
  * `pca9685_mount_bracket.stl`: 3D printable bracket for the PCA9685 module.
  * `pca9685_mount_bracket.scad`: Parametric OpenSCAD source file.
* **Visuals**:
  * `pcb_layout_diagram.svg`: Dimensioned topology and socket arrangement diagram.
  * `pcb_render.svg`: Top layer vector render.
