# 🤖 Quard Bot (Sesame Quadruped Robot)

This guide covers the **3D printable CAD mesh assets (STLs)** and **ESP32 firmware** for the **Sesame 8-Servo Quadruped Robot**, featuring an integrated PCA9685 servo driver, SSD1306 OLED animated face display, brownout-proof staggered servo startup, and a dark-mode Web AP dashboard.

---

## 📁 Where the Quard Bot files are

| What | Path |
|---|---|
| Firmware (PlatformIO env `quard_bot`) | `src/quadruped_sesame.cpp` (build: `pio run -e quard_bot -t upload`) |
| Printable STLs (11 parts) | `cad/quard_bot/stl/` |
| Runtime URDF + meshes (RViz, web twin) | `urdf/quard_bot/urdf/quard_bot.urdf`, `urdf/quard_bot/meshes/` |
| URDF generator | `scripts/build_configured_urdf.py` |
| Blender leg-rig helper | `scripts/setup_r1_r3.py` |
| Dashboard code | the Quard tab in `calibrated_joint_gui.py` (`QuardBotWidget`) and `web_dashboard/static/app.js` |

```text
cad/quard_bot/stl/
├── Bottom-Cover-v121.stl          # Lower chassis base cover
├── Internal-Frame-v121.stl        # Internal mounting frame for ESP32 & PCA9685
├── L1-v117.stl                    # Left Front Hip Link (Servo Ch 2)
├── L2-v117.stl                    # Left Rear Hip Link (Servo Ch 3)
├── L3-v117.stl                    # Left Front Foot Link (Servo Ch 6)
├── L4-v117.stl                    # Left Rear Foot Link (Servo Ch 7)
├── R1-v117.stl                    # Right Front Hip Link (Servo Ch 0)
├── R2-v117.stl                    # Right Rear Hip Link (Servo Ch 1)
├── R3-v117.stl                    # Right Front Foot Link (Servo Ch 5)
├── R4-v117.stl                    # Right Rear Foot Link (Servo Ch 4)
└── Top-Cover-Enclosed-v117.stl    # Top enclosure cover
```

---

## 🔌 Pinout & Wiring Specifications

### Hardware Components
1. **ESP32 NodeMCU-32S**
2. **PCA9685 16-Channel 12-Bit PWM I2C Servo Driver** (Address `0x40`)
3. **SSD1306 128x64 I2C OLED Display** (Address `0x3C` / `0x3D`)
4. **8x Micro Servos** (SG90 or MG90S)
5. **UBEC / High-Current 5V Power Supply** (for servos)

### Pin Connections

| ESP32 Pin | Connected Component | Function |
|-----------|---------------------|----------|
| **GPIO 21** | PCA9685 `SDA`      | I2C Bus 0 Data |
| **GPIO 22** | PCA9685 `SCL`      | I2C Bus 0 Clock |
| **GPIO 18** | SSD1306 OLED `SDA` | I2C Bus 1 Data |
| **GPIO 19** | SSD1306 OLED `SCL` | I2C Bus 1 Clock |
| **5V / VCC**| PCA9685 & OLED VCC | Logic Power |
| **GND**     | Common GND         | System Ground |

---

## 🦾 PCA9685 8-Channel Servo Mapping

| PCA9685 Channel | Leg / Joint Name | Function / Description | Base Zero |
|-----------------|------------------|------------------------|-----------|
| **Ch 0**        | `R1`             | Right Front Hip        | 90°       |
| **Ch 1**        | `R2`             | Right Rear Hip         | 0°        |
| **Ch 2**        | `L1`             | Left Front Hip         | 0°        |
| **Ch 3**        | `L2`             | Left Rear Hip          | 90°       |
| **Ch 4**        | `R4`             | Right Rear Foot        | 90°       |
| **Ch 5**        | `R3`             | Right Front Foot       | 0°        |
| **Ch 6**        | `L3`             | Left Front Foot        | 90°       |
| **Ch 7**        | `L4`             | Left Rear Foot         | 0°        |

---

## 🌐 Web Dashboard Control

- **Primary Wi-Fi Network (STA)**: Connects to Wi-Fi SSID `Sabo` (Password: `sandy0606`) — same network as the Robot Arm
- **Fallback Access Point (AP)**: `Sesame-Robot-Control` (Password: `12345678`)
- **Dashboard URL**: `http://<ESP32_IP_ADDRESS>` or `http://sesame-robot.local` (or `http://192.168.4.1` on AP fallback)

### Features
- **D-Pad Locomotion**: Forward, Backward, Turn Left, Turn Right, Stand
- **Quick Actions**: HI Wave animation, Stand (45°), Base Zero (0°), High Stance (60°)
- **Live Channel Sliders**: Individual 0°–180° adjustment for all 8 leg channels

### HTTP Command API (used by both project dashboards)

| Endpoint | Purpose |
|---|---|
| `GET /cmd?...` | Send a command (parameters below). Replies `200 OK`, or `400 Bad Args` if nothing was recognised. |
| `GET /status` (alias `/state`) | JSON telemetry |
| `GET /` | The built-in web page |

| Parameter | Values |
|---|---|
| `move` (or `mode`, `cmd`) | `w`/`forward`, `b`/`backward`, `l`/`left`, `r`/`right`, `s`/`stand`/`stop`, `z`/`zero`, `hi`/`wave`, `crouch`, `high` |
| `ch` + `deg` (or `val`) | One channel `0`–`7` to an angle, e.g. `/cmd?ch=3&deg=70`. The angle **must** be in `deg` or `val`; if neither is present it reads as 0°. |
| `all` | Every channel to one angle, e.g. `/cmd?all=45` |
| `speed` (or `delay`) | Gait step delay in ms, `50`–`800` |

**Locomotion watchdog:** while walking, the robot returns to stand if no `/cmd` request arrives for 2 s (`LOCOMOTION_WATCHDOG_MS`). Clients must repeat the active gait (the dashboards resend it every second). There is no OLED `face` command yet.

---

## 🛡️ Anti-Surge & Power Protection

1. **Staggered Startup**: Servos initialize sequentially with a 35ms delay between each channel to prevent high inrush current brownouts.
2. **Brownout Detector Bypass**: ESP32 Core v3 brownout control (`WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0)`).
3. **Animated OLED Expressions**: Cute interactive face animations (Happy, Blinking, Walking, Wave, Sleepy).
