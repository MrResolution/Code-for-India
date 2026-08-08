# 🤖 Quard Bot (Sesame Quadruped Robot)

This folder contains the **3D printable CAD mesh assets (STLs)** and **ESP32 firmware** for the **Sesame 8-Servo Quadruped Robot**, featuring an integrated PCA9685 servo driver, SSD1306 OLED animated face display, brownout-proof staggered servo startup, and a dark-mode Web AP dashboard.

---

## 📁 Directory Structure

```text
Quard bot/
├── README.md                          # Comprehensive technical documentation & pinout guide
├── stl/                               # 3D Printable STL files (Chassis, legs, covers)
│   ├── Bottom-Cover-v121.stl          # Lower chassis base cover
│   ├── Internal-Frame-v121.stl        # Internal mounting frame for ESP32 & PCA9685
│   ├── L1-v117.stl                    # Left Front Hip Link (Servo Ch 2)
│   ├── L3-v117.stl                    # Left Front Foot Link (Servo Ch 6)
│   ├── L4-v117.stl                    # Left Rear Foot Link (Servo Ch 7)
│   ├── R1-v117.stl                    # Right Front Hip Link (Servo Ch 0)
│   ├── R2-v117.stl                    # Right Rear Hip Link (Servo Ch 1)
│   ├── R3-v117.stl                    # Right Front Foot Link (Servo Ch 5)
│   ├── R4-v117.stl                    # Right Rear Foot Link (Servo Ch 4)
│   └── Top-Cover-Enclosed-v117.stl    # Top enclosure cover
└── firmware/                          # ESP32 Firmware Source
    └── src/
        └── main.cpp                    # Full Sesame Robot firmware (PCA9685, OLED, Web Server)
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

## 🌐 Web AP Dashboard Control

- **WiFi SSID**: `Sesame-Robot-Control`
- **WiFi Password**: `12345678`
- **Dashboard URL**: `http://192.168.4.1` or `http://sesame-robot.local`

### Features
- **D-Pad Locomotion**: Forward, Backward, Turn Left, Turn Right, Stand
- **Quick Actions**: HI Wave animation, Stand (45°), Base Zero (0°), High Stance (60°)
- **Live Channel Sliders**: Individual 0°–180° adjustment for all 8 leg channels

---

## 🛡️ Anti-Surge & Power Protection

1. **Staggered Startup**: Servos initialize sequentially with a 35ms delay between each channel to prevent high inrush current brownouts.
2. **Brownout Detector Bypass**: ESP32 Core v3 brownout control (`WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0)`).
3. **Animated OLED Expressions**: Cute interactive face animations (Happy, Blinking, Walking, Wave, Sleepy).
