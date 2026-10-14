# 🖥️ Robotics Control Center: Dashboards & Embedded RViz2 Guide

This guide covers the two control UIs, how the 3D viewport is docked, the safety controls, and common problems.

| UI | Launch | Best for |
|---|---|---|
| **Desktop app** (`calibrated_joint_gui.py`, PyQt5) | `./launch_dashboard.sh` (`--quard`, `--dual`) | Day-to-day control with the native RViz2 viewport |
| **Web dashboard** (`web_dashboard/`, Three.js) | `./launch_web_dashboard.sh` → http://localhost:8000 | Machines without a desktop session, a second screen, quick checks |

Both drive the same robots and read/write the same `servo_calibration.json`.

---

## 🏗️ 1. Desktop App Layout

```text
┌────────────────────────────────────────────────────────────────────────────────────────┐
│ 🦾 Robotics Control Center  [ROS 2 status]        Esc = E-STOP · Ctrl+S = save [E-STOP]│
├────────────────────────────────────────────────────────────────────────────────────────┤
│ (red banner while E-STOP is engaged)                                                   │
├─────────────────────────────────────────┬──────────────────────────────────────────────┤
│  [ 🦾 Robot Arm | 🤖 Quard Bot | 🚀 Rover ]│  🧊 Embedded RViz2 viewport                │
│                                         │                                              │
│  MOTION       Start Sim · Go Home ·     │   Arm / Quard / Dual robot models            │
│               Center All · Gesture      │                                              │
│  CALIBRATION  Set Home · Sync · Comm    │   (falls back to the Three.js WebGL twin     │
│               ························ Save│    when RViz2 is not installed)              │
│  Joint cards (scroll)                   │                                              │
│  ═════════ drag to resize ═════════     │                                              │
│  Serial / Wi-Fi monitor console         │                                              │
├─────────────────────────────────────────┴──────────────────────────────────────────────┤
│ status bar: save confirmations, E-STOP state                                           │
└────────────────────────────────────────────────────────────────────────────────────────┘
```

- **Header:** the ROS 2 badge shows whether the app is publishing `/joint_states`. "ROS 2 not found: UI only" means nothing reaches the robots.
- **Dividers:** both the left/right split and the joints/console split can be dragged.
- **Remembered layout:** window size and both divider positions are saved on exit to `~/.config/CodeForIndia/RobotDashboard.conf`. Delete that file to reset. The starting tab follows the launcher (`--quard` opens the Quard Bot tab).
- **Feedback:** saving shows a status-bar message instead of a popup. Errors still open a dialog.

---

## 🧊 2. Native RViz2 Window Docking Mechanics

The dashboard embeds the native **RViz2** 3D viewport side-by-side using an X11 reparenting pipeline:

### Process Lifecycle & Safety Isolation:
1. **Direct Child Process**: `calibrated_joint_gui` launches RViz2 directly using `QProcess` with the target robot's RViz config (`arm.rviz`, `quard_bot.rviz`, or `dual_robots.rviz`).
2. **PID-Locked Window Discovery**: The dashboard identifies the window strictly using the child's exact **Process ID (`_NET_WM_PID`)** and verifies `WM_CLASS == "rviz2"`.
3. **Host Window Isolation**: The search actively verifies window properties and **strictly excludes** host applications (`antigravity`, VS Code, browsers, or terminal emulators) from being touched or unmapped.
4. **Reparenting into Qt**: The X11 window is reparented into the PyQt frame (`self.rviz_frame`) using `xdotool windowreparent`.
5. **Qt Container Integration**: The native X11 window is wrapped in a `QWindow.fromWinId()` and embedded inside a `QWidget.createWindowContainer()`, allowing dynamic resizing alongside the control panel splitter.

---

## 🎛️ 3. Control Panels

### 🦾 Robot Arm
- **Joint cards:** each card is titled with the readable joint name (for example "Shoulder Pitch"). The URDF name and wiring hint are shown underneath.
  - Slider in degrees, with a degree/radian readout.
  - **Min / Max** safety limits. **📍 Set Low** / **📍 Set High** copy the current angle into them.
  - **Twin offset:** −5/−1/+1/+5/0 nudges that line the 3D model up with the physical servo.
- **Motion row:**
  - `▶️ Start Sim`: sweeps every joint through its limits.
  - `🏠 Go Home`: eased 2 s move to the saved home pose.
  - `⊕ Center All`: all joints to 90°.
  - `✋ Gesture Mode`: webcam teleoperation.
- **Calibration row:**
  - `📌 Set Home`: current pose becomes home.
  - `🎯 Sync to Physical`: sets twin offsets so the model shows the home pose.
  - `📡 Comm`: Wi-Fi / Serial link to the arm ESP32.
  - `💾 Save Config` (Ctrl+S).
- **Monitor console:** ESP32 connection badge, timestamped log, and a raw command box (for example `CMD:turntable_link_joint_dup=90.0`).

### 🤖 Quard Bot (Sesame Quadruped)
- **Network target:** host or IP (default `http://sesame-robot.local`), **Ping**, and live telemetry.
- **Locomotion:** D-pad or keyboard (W/A/S/D or arrows to walk and turn, Space to stand). Gait step-delay slider.
- **Stances:** Stand (45°), Base Zero (0°), Crouch (30°), High (60°), Wave. Sync current stance / reset trims.
- **PCA9685 channels:** a "Set All Servos" master slider plus one slider per channel with its trim offset (Δ).
- **More keys (desktop app):** C = crouch, H = high stance, 1–5 = OLED faces.

### 🚀 Rover (ESP32 Sensor Node)
- **Connection:** Wi-Fi host (`/data` JSON endpoint) or USB serial port/baud. Simulated data is used when neither is connected.
- **Climate:** DHT11 temperature/humidity, BMP280 temperature/pressure.
- **Hazards:** MQ gas, water level, flame sensor, with a fire alert banner.
- **IMU (MPU6050):** accelerometer, gyroscope, pitch/roll, **Tare Horizon**.
- **GPS & storage:** NEO-6M fix, coordinates, satellites, microSD status, uptime.
- **Telemetry log:** raw stream from the node.

---

## ⛔ 4. Safety Controls

### E-STOP (both UIs)
Press the **E-STOP** button in the header or press **Esc**. It:
1. stops the arm simulation, homing and gesture mode (the arm holds its current pose);
2. cancels Quard Bot gaits and animations and sends `stand` immediately;
3. locks the motion controls and shows a red banner until you press **Release**.

> ⚠️ This is a **software stop**. It does not cut servo power, and it can't help if the dashboard has lost contact with a robot. Keep a hardware power switch within reach.

### Quard Bot locomotion watchdog
The Quard firmware stands the robot up if it receives no `/cmd` request for 2 s (`LOCOMOTION_WATCHDOG_MS` in `src/quadruped_sesame.cpp`). Both UIs resend the active gait once a second while walking. If the dashboard or the network drops, the robot stops by itself within about 2 s. While the robot is unreachable, the web dashboard keeps only the newest walk command queued, so stale commands are not replayed when it reconnects.

### Safe-range indicators (web dashboard)
Each arm slider shows the saved Min–Max band in green. A joint outside its saved limits is outlined in amber. The desktop app clamps the slider to the limits instead.

---

## ⌨️ 5. Keyboard Shortcuts

| Key | Desktop app | Web dashboard |
|---|---|---|
| **Esc** | E-STOP | E-STOP |
| **Ctrl+S** | Save calibration | — |
| **W / A / S / D**, arrows | Quard walk / turn (Quard tab) | Quard walk / turn (Quard tab) |
| **Space** | Quard stand | Quard stand |
| **C / H** | Quard crouch / high stance | — |
| **1 – 5** | Quard OLED faces | — |
| **← / →** on a focused tab | — | Switch tabs |

Walk keys are ignored while E-STOP is engaged.

---

## 🌐 6. Web Dashboard

`./launch_web_dashboard.sh` sources ROS 2, starts `servo_serial_commander` (which owns the arm's serial port), and opens http://localhost:8000.

| Option | Effect |
|---|---|
| `--no-commander` | Dashboard only; don't start `servo_serial_commander` |
| `--host 0.0.0.0` | Allow other devices on the network (phones/tablets). **No authentication**: anyone on the network can move the robots. |
| `--port N` / `PORT=N` | Listen on another port |

**Layout:** robot tabs on the left and the Three.js twin on the right. Under 1100 px wide the twin moves on top; phone widths are supported. The last tab is remembered.

**Header badges:**
- **Arm:** shows `ROS 2 · Serial/Wi-Fi` when the ROS bridge is publishing. `UI only (no ROS)` means rclpy wasn't available, so nothing reaches the arm. `Server offline` means the browser lost the dashboard server.
- **Quard:** HTTP heartbeat (Connected / Offline).
- **Sensors:** data source (Wi-Fi, Serial, Simulated, Disconnected).

**Other features:**
- **Theme:** 🌓 switches light/dark (defaults to the system setting).
- **Arm link card:** arm commands go out on `/joint_states`, and `servo_serial_commander` forwards them to the ESP32. Change its port with `--ros-args -p port:=/dev/ttyACM0`.
- **Notifications:** saves, syncs and errors appear as toasts in the bottom-right corner.

The server is plain Python (`web_dashboard/server.py`) and also runs without ROS 2 (UI only), which is handy for working on the UI on another machine.

---

## 🛠️ 7. Troubleshooting & FAQ

### Q: Why does the 3D panel show WebGL instead of RViz?
- **Cause**: ROS 2 Jazzy or RViz2 is not installed or not in PATH.
- **Fix**: Run the one-line install commands in [`docs/QUICK_INSTALL_GUIDE.md`](QUICK_INSTALL_GUIDE.md).

### Q: The header says "ROS 2 not found" / "UI only (no ROS)".
ROS 2 wasn't sourced, so sliders move the twin but nothing is published. Use the launch scripts (they source `/opt/ros/jazzy/setup.bash`), or source it yourself before starting.

### Q: Can I run both the Robotic Arm and Quard Bot together?
- **Yes**: Run `./launch_dual_dashboard.sh`. The dashboard loads both robots simultaneously into RViz2.

### Q: How do I connect to physical hardware?
- **Arm:** `servo_serial_commander` owns the serial port (default `/dev/ttyUSB0` @ 115200). Select Serial or Wi-Fi with the **Comm** toggle.
- **Quard Bot:** enter the ESP32 IP or `http://sesame-robot.local` and press **Ping**.
- **Rover:** enter its Wi-Fi IP and press **Connect**, or pick the serial port and press **Connect Serial**.

### Q: A joint card is outlined in amber (web).
The joint's current angle is outside the Min/Max saved for it. Move it back into the green band, or adjust and save the limits.

### Q: The OLED face buttons say the command was rejected.
The Quard firmware does not implement a `face` command yet (it answers `400 Bad Args`). The robot is online; only that command is unsupported.

### Q: How do I reset the desktop window layout?
Delete `~/.config/CodeForIndia/RobotDashboard.conf`.

### Known issue
In the desktop app, **Start Sim** on the arm tab also starts the Quard Bot walking forward, and stopping the sim makes it stand. Don't press it with the quadruped powered unless you want both to move.
