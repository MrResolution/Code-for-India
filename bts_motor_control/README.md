# ⚡ BTS7960 Motor Controller (ESP32 Bluetooth Driver)

This standalone folder contains the **C++ firmware & Arduino codebase** for controlling dual high-power BTS7960 43A motor drivers using an ESP32 microcontroller over Bluetooth Classic (`BluetoothSerial`).

---

## 📁 Directory Structure

```text
bts_motor_control/
├── README.md                          # Pinout wiring table & Bluetooth protocol guide
└── firmware/                          # C++ firmware codebase
    ├── include/
    │   └── BTS7960MotorController.h   # Class header for BTS7960 driver
    └── src/
        ├── BTS7960MotorController.cpp # Motor control logic, LEDC PWM & safety watchdog
        ├── main.cpp                    # Main application
        └── single_file_main.cpp        # Standalone single-file sketch (Arduino IDE friendly)
```

---

## 🔌 Hardware & Wiring Specifications

| ESP32 Pin | Left BTS7960 | Right BTS7960 | Function |
|---|---|---|---|
| **GPIO 25** | `L_RPWM` | — | Left Motor Forward PWM (LEDC Channel 0) |
| **GPIO 26** | `L_LPWM` | — | Left Motor Reverse PWM (LEDC Channel 1) |
| **GPIO 32** | `R_EN` + `L_EN` | — | Left Driver Enable |
| **GPIO 27** | — | `R_RPWM` | Right Motor Forward PWM (LEDC Channel 2) |
| **GPIO 14** | — | `R_LPWM` | Right Motor Reverse PWM (LEDC Channel 3) |
| **GPIO 33** | — | `R_EN` + `L_EN` | Right Driver Enable |

---

## 📱 Bluetooth Command Protocol

- **Device Name**: `ESP32_ROBOT`
- **Commands**:
  - `'F'` / `'B'` / `'L'` / `'R'`: Forward, Backward, Turn Left, Turn Right
  - `'G'`, `'I'`, `'H'`, `'J'`: Curved diagonal directions
  - `'S'` / `'X'`: Emergency Stop
  - `'0'` .. `'9'`, `'q'`: Speed levels (0 - 255)
