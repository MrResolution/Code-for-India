# 🖥️ Robotics Control Center: Dashboards & Embedded RViz2 Guide

This guide covers the two control UIs, how the 3D viewport is docked, the safety controls, and common problems.

| UI | Launch | Best for |
|---|---|---|
| **Desktop app** (`calibrated_joint_gui.py`, PyQt5) | `./launch_dashboard.sh` (`--quard`, `--dual`, `--dogu`) | Day-to-day control with the native RViz2 viewport |
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
│  [ 🦾 Arm | 🤖 Quard | 🚀 Rover | 🐕 Dogu ]│  🧊 Embedded RViz2 viewport                │
│                                         │                                              │
│  MOTION       Start Sim · Go Home ·     │   Arm / Quard / Dual / Dogu robot models     │
│               Center All · Gesture      │                                              │
│  CALIBRATION  Set Home · Sync · Comm    │   (falls back to the Three.js WebGL twin     │
│               ························ Save│    when RViz2 is not installed)              │
│  Joint cards (scroll)                   │                                              │
│  ═════════ drag to resize ═════════     │                                              │
│  Arm link card + console                │                                              │
├─────────────────────────────────────────┴──────────────────────────────────────────────┤
│ status bar: save confirmations, E-STOP state                                           │
└────────────────────────────────────────────────────────────────────────────────────────┘
```

- **Header:** the ROS 2 badge shows whether the app is publishing `/joint_states`. "ROS 2 not found: UI only" means nothing reaches the robots.
- **Dividers:** both the left/right split and the joints/console split can be dragged.
- **Connections and consoles:** every tab has a link card and a console; see [section 4](#-4-connections--consoles).
- **Remembered layout:** window size and both divider positions are saved on exit to `~/.config/CodeForIndia/RobotDashboard.conf`. Delete that file to reset. The starting tab follows the launcher (`--quard` opens the Quard Bot tab, `--dogu` the Dogu tab).
- **RViz follows the tab:** the Arm, Quard Bot and Dogu tabs each load their own robot into the embedded RViz, whatever the launcher picked. The badge over the viewport (`RViz: 🐕 Dogu`) names the model it shows, and the arm console logs each switch. The Rover tab has no model, so it keeps the last one. After `--dual`, the Arm and Quard tabs show both bots.
- **Feedback:** saving shows a status-bar message instead of a popup. Errors still open a dialog.

---

## 🧊 2. Native RViz2 Window Docking Mechanics

The dashboard embeds the native **RViz2** 3D viewport side-by-side using an X11 reparenting pipeline:

### Process Lifecycle & Safety Isolation:
1. **Direct Child Process**: `calibrated_joint_gui` launches RViz2 directly using `QProcess` with the target robot's RViz config (`arm.rviz`, `quard_bot.rviz`, `dual_robots.rviz` or `dogu.rviz`).
2. **PID-Locked Window Discovery**: The dashboard identifies the window strictly using the child's exact **Process ID (`_NET_WM_PID`)** and verifies `WM_CLASS == "rviz2"`.
3. **Host Window Isolation**: The search actively verifies window properties and **strictly excludes** host applications (`antigravity`, VS Code, browsers, or terminal emulators) from being touched or unmapped.
4. **Reparenting into Qt**: The X11 window is reparented into the PyQt frame (`self.rviz_frame`) using `xdotool windowreparent`.
5. **Qt Container Integration**: The native X11 window is wrapped in a `QWindow.fromWinId()` and embedded inside a `QWidget.createWindowContainer()`, allowing dynamic resizing alongside the control panel splitter.

### Switching robots
RViz is never restarted. When the tab changes, the app sets the `robot_description` parameter of `/robot_state_publisher`. It republishes `/robot_description`, and RViz reloads the model from that topic. Every model hangs off a `world` link (the launch file and the app add one when a URDF starts elsewhere), so RViz keeps a valid Fixed Frame for every robot. The camera stays where the launcher's RViz config put it: zoom or pan if a robot looks small.

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
- **Arm link card and console:** link state, ping, ⟳ Reconnect, ⚙ Link settings and a command box (see [section 4](#-4-connections--consoles)).

### 🤖 Quard Bot (Sesame Quadruped)
- **Connection:** host or IP (default `http://sesame-robot.local`, recent ones remembered), **Ping**, **Auto** check, round trip and RSSI, live telemetry. The console takes `/cmd` queries.
- **Locomotion:** D-pad or keyboard (W/A/S/D or arrows to walk and turn, Space to stand). Gait step-delay slider.
- **Stances:** Stand (45°), Base Zero (0°), Crouch (30°), High (60°), Wave. Sync current stance / reset trims.
- **PCA9685 channels:** a "Set All Servos" master slider plus one slider per channel with its trim offset (Δ).
- **More keys (desktop app):** C = crouch, H = high stance, 1–5 = OLED faces.

### 🚀 Rover (ESP32 Sensor Node)
- **Connection:** one source at a time: Wi-Fi host (`/data` JSON endpoint) or USB serial port/baud, with its live state.
- **Readings are only ever real:** every value shows `--` and every badge `No data` until the rover sends it. Fields a reply leaves out stay empty, and changing or disconnecting the source clears the readings. **Tare Horizon** needs a live IMU reading.
- **Climate:** DHT11 temperature/humidity, BMP280 temperature/pressure.
- **Hazards:** MQ gas, water level, flame sensor, with a fire alert banner.
- **IMU (MPU6050):** accelerometer, gyroscope, pitch/roll, **Tare Horizon**.
- **GPS & storage:** NEO-6M fix, coordinates, satellites, microSD status, uptime.
- **Console:** the raw stream from the node plus connection events.

### 🐕 Dogu (robot dog)
The tab has two parts: **Robot link**, which drives the real robot (firmware `src/dogu.cpp`, see [`DOGU.md`](DOGU.md)), and **Model preview**, which only moves the 3D model.

**Robot link**
- Join Dogu's own Wi-Fi network, then use the default address `http://192.168.4.1`. Link details are in [section 4](#-4-connections--consoles).
- **Robot** shows what the dog reports: `Walking · Front Left · Lift` or `Stopped`. It reads `--` until the robot answers.
- **Start walking** / **Stop walking**: the firmware's single gait, one leg at a time, until stopped. E-STOP sends Stop and locks Start walking.
- ⚠️ The firmware has no link watchdog: if Wi-Fi drops while it walks, it keeps walking. Stop it from its own page at the same address, or cut its power.

**Model preview**
The robot reports no joint angles, so the model never mirrors it. In the desktop app the model's angles go out on `/joint_states`, and the embedded RViz switches to Dogu while this tab is open, from any launcher. `./launch_dogu_dashboard.sh` (or `./launch_dashboard.sh --dogu`) starts on this tab.
- **Poses:** Stand, Crouch, Sit. The model eases into each pose in about 0.6 s and holds it.
- **Moves:** Walk (trot), Turn left, Turn right, Wave (front-right paw), Idle (breathing). These loop until you press **Stop model** or pick something else.
- **Speed:** 0.25× to 2×.
- **Joint angles:** a live table of all 12 joints (hip, femur, tibia per leg), in degrees from standing.
- **Console:** robot link events and replies, what was played, and load errors (for example a missing `urdf/dogu/animations.json`).
- **E-STOP** stops the animation and blocks new ones until released.
- In the web twin the body is levelled and lowered so the lowest feet touch the ground.

The poses and clips are generated, not hand-edited: see [`urdf/dogu/README.md`](../urdf/dogu/README.md).

---

## 🔗 4. Connections & Consoles

Each tab, in both UIs, has a **connection card** (a coloured status pill plus details) and a **console**.

### Arm (servo_serial_commander)
The commander owns the arm's serial port / UDP socket and reports on `/esp32_link_state`; both dashboards show that.

| Pill | Meaning | What to do |
|---|---|---|
| **Online** | The ESP32 replied within 4 s. Details: link, ping round trip, last reply, RX/TX counts, the ESP32's own IP (from its heartbeat). | — |
| **No reply** | Port or socket is open, the ESP32 is silent. | Power, cable, right port / IP. |
| **Mode mismatch** | The ESP32 replies but listens on the other link (`ERR:WIFI_MODE_ACTIVE`), so commands are ignored. | Press **Switch ESP32 to serial / Wi-Fi** in the warning. |
| **Port error** | The port can't be opened. The reason is spelled out: permission denied → add yourself to `dialout`; not found → plug in / rescan; busy. | Fix, then **⟳ Reconnect**. The **⚙ Link** settings open by themselves. |
| **Commander not running** | No link state for 3.5 s. | `ros2 run servo_joint_publisher servo_serial_commander` (the launch scripts start it). |
| **ROS 2 not available** | The UI isn't connected to ROS 2. | Start from the launch scripts. |

- **⟳ Reconnect** closes and reopens the link now.
- **⚙ Link** changes the serial port (with a ⟳ rescan of connected ports), the baud rate or the Wi-Fi IP while everything keeps running. The commander validates the values and reports the result in the console.
- The **Comm** toggle always shows the link the commander is really using.

### Quard Bot (HTTP)
- **Online · N ms** is the round trip of `GET /status`, plus RSSI. **Unreachable** gives the reason: host not found (a `.local` name needs mDNS; try the IP), timed out, refused, or network unreachable.
- **Auto** checks every 3 s while online and backs off 2 → 4 → 8 → 10 s while unreachable ("retry in N s"). **Ping** checks at once and replaces a check that is stuck on a slow name lookup.
- Recent addresses are remembered (drop-down). After a good ping, commands go to the resolved IP, which avoids repeated slow `.local` lookups.
- A motion command that waited more than 2 s behind a slow request is **dropped, not sent late** (Stand is always sent). It is logged as such. Changing the address drops commands still queued for the old one.

### Dogu (HTTP)
- **Online · N ms** is the round trip of `GET /status`. **Unreachable** gives the reason, the time it was last seen, and when it retries. The Robot line then shows the last report and its age.
- **Auto** checks every 3 s while online, every 1 s while it walks, and backs off 2 → 4 → 8 → 10 s while unreachable. **Ping** checks at once.
- Commands go one at a time and in order. A newer press replaces one not yet sent, and a Start that waited more than 2 s behind a slow request is **dropped, not sent late**.
- A failed Stop says the dog may still be walking.
- The web dashboard's server makes the requests, so the computer running `server.py` must be on Dogu's network.

### Rover
- One source at a time: **Wi-Fi** (`http://<ip>/data` JSON) or **Serial**, or **Not connected**. There is no simulated data.
- States:
  - **Live**: shows the rate and the age of the last data.
  - **No data**: nothing for 5 s.
  - **Waiting for data**: the port is open but silent; usually the wrong baud rate.
  - **Error**: shows the reason. Serial retries every 2 s; Wi-Fi backs off up to 5 s.
- Opening the arm commander's serial port for the rover would reset that ESP32. The desktop app asks first; the web dashboard needs a second click within 10 s.

### Consoles
- Every line is timestamped and coloured: `→` sent, `←` received, `•` event, `✓` OK, `!` warning, `✗` error; raw sensor data in grey.
- Identical lines collapse into one with `×N`, and a slider drag updates a single line instead of adding hundreds.
- **Filter** (All / Sent / Received / Events / Problems), **Find**, **Pause** (new lines are kept and appear on resume), **Copy**, **Save** (`.log` file) and **Clear**.
- Command box with **↑ / ↓ history**, kept between runs:
  - **Arm:** text goes to the ESP32 as typed, for example `PING` or `MODE:SERIAL`. Lines starting with `MODE:`, `CMD:`, `CALIB:`, `SYNC:` or `LINK:` go to the commander's own handlers. Joint commands such as `CMD:turntable_link_joint=90` are **clamped to that joint's limits**, are blocked during E-STOP, and the next slider move overrides them. **Joint traffic** also shows every joint command and acknowledgement (busy).
  - **Quard Bot:** a `/cmd` query (`move=w`, `ch=3&deg=70`, `all=45`, `speed=180`) or `status`. Blocked during E-STOP except stand.
  - **Rover:** read-only.

### For other tools
| Topic / endpoint | Content |
|---|---|
| `/esp32_link_state` | `std_msgs/String` JSON (`state`, `transport`, `target`, `rtt_ms`, `last_rx_age`, `rx`, `tx`, `last_error`, `mismatch_mode`, …). 1 Hz and on change; reliable, transient local. Treat it as stale after ~3.5 s. |
| `/esp32_console` | `std_msgs/String`, `RX:` / `TX:` / `SYS:` / `ERR:` + text; reliable, volatile. Repeated ESP32 lines are throttled to one per 5 s. |
| `/esp32_connection_status` | Legacy `STATUS:CONNECTED:…` / `STATUS:DISCONNECTED:…` strings. |
| `/servo_calibration` | Also accepts `RAW:<line>`, `LINK:RECONNECT`, `LINK:STATUS`, `LINK:VERBOSE=0/1`, `LINK:PORT=/dev/ttyACM0`, `LINK:BAUD=115200`, `LINK:IP=192.168.1.50`. |
| Web API | `/api/state` → `links`; `GET /api/console?since=N`; `GET /api/serial_ports`; `POST /api/arm/console {"line"}`; `POST /api/arm/link {"cmd"}`. |

```bash
ros2 topic echo /esp32_link_state
```

```bash
ros2 topic pub --once /servo_calibration std_msgs/msg/String "{data: 'LINK:PORT=/dev/ttyACM0'}"
```

---

## ⛔ 5. Safety Controls

### E-STOP (both UIs)
Press the **E-STOP** button in the header or press **Esc**. It:
1. stops the arm simulation, homing and gesture mode (the arm holds its current pose);
2. cancels Quard Bot gaits and animations and sends `stand` immediately;
3. sends `/stop` to Dogu (and locks its Start walking) and stops the Dogu model animation;
4. locks the motion controls and shows a red banner until you press **Release**.

> ⚠️ This is a **software stop**. It does not cut servo power, and it can't help if the dashboard has lost contact with a robot. Keep a hardware power switch within reach.

### Quard Bot locomotion watchdog
The Quard firmware stands the robot up if it receives no `/cmd` request for 2 s (`LOCOMOTION_WATCHDOG_MS` in `src/quadruped_sesame.cpp`). Both UIs resend the active gait once a second while walking. If the dashboard or the network drops, the robot stops by itself within about 2 s. Neither UI replays motion late: a command that waited more than 2 s behind a slow request is dropped (Stand is always sent), and the web dashboard also keeps only the newest walk command queued.

### Safe-range indicators (web dashboard)
Each arm slider shows the saved Min–Max band in green. A joint outside its saved limits is outlined in amber. The desktop app clamps the slider to the limits instead.

---

## ⌨️ 6. Keyboard Shortcuts

| Key | Desktop app | Web dashboard |
|---|---|---|
| **Esc** | E-STOP | E-STOP |
| **Ctrl+S** | Save calibration | — |
| **W / A / S / D**, arrows | Quard walk / turn (Quard tab) | Quard walk / turn (Quard tab) |
| **Space** | Quard stand | Quard stand |
| **C / H** | Quard crouch / high stance | — |
| **1 – 5** | Quard OLED faces | — |
| **← / →** on a focused tab | — | Switch tabs |
| **↑ / ↓** in a console command box | Command history | Command history |

Walk keys are ignored while E-STOP is engaged.

---

## 🌐 7. Web Dashboard

`./launch_web_dashboard.sh` sources ROS 2, starts `servo_serial_commander` (which owns the arm's serial port), and opens http://localhost:8000.

| Option | Effect |
|---|---|
| `--no-commander` | Dashboard only; don't start `servo_serial_commander` |
| `--host 0.0.0.0` | Allow other devices on the network (phones/tablets). **No authentication**: anyone on the network can move the robots. |
| `--port N` / `PORT=N` | Listen on another port |

**Layout:** robot tabs (Arm, Quard Bot, Sensor Node, Dogu) on the left and the Three.js twin on the right. The twin's **Model** menu follows the tab, or can be pinned to one robot (Dogu included). Under 1100 px wide the twin moves on top; phone widths are supported. The last tab is remembered.

**Header badges:**
- **Arm:** the commander's link state: `Online · Serial`, `No reply`, `Mode mismatch`, `Port error`, `No commander`, or `UI only (no ROS)` when rclpy wasn't available.
- **Quard:** `Online · N ms` or `Unreachable`.
- **Sensors:** `Wi-Fi`, `Serial`, `No data`, `Error` or `Off`.
- Every badge reads `Server offline` when the browser loses the dashboard server.

**Other features:**
- **Theme:** 🌓 switches light/dark (defaults to the system setting).
- **Arm link card:** arm commands go out on `/joint_states`, and `servo_serial_commander` forwards them to the ESP32. Change its port live with **⚙ Link**, or at start-up with `--ros-args -p port:=/dev/ttyACM0`. The console can be hidden with **Console**.
- **Notifications:** saves, syncs and errors appear as toasts in the bottom-right corner.

The server is plain Python (`web_dashboard/server.py`) and also runs without ROS 2 (UI only), which is handy for working on the UI on another machine.

---

## 🛠️ 8. Troubleshooting & FAQ

### Q: Why does the 3D panel show WebGL instead of RViz?
- **Cause**: ROS 2 Jazzy or RViz2 is not installed or not in PATH.
- **Fix**: Run the one-line install commands in [`docs/QUICK_INSTALL_GUIDE.md`](QUICK_INSTALL_GUIDE.md).

### Q: The header says "ROS 2 not found" / "UI only (no ROS)".
ROS 2 wasn't sourced, so sliders move the twin but nothing is published. Use the launch scripts (they source `/opt/ros/jazzy/setup.bash`), or source it yourself before starting.

### Q: I pulled the latest code but a new tab (for example Dogu) is missing.
- **Desktop app:** with ROS 2, `./launch_dashboard.sh` starts the copy of the app in `ros2_ws/install`. The launcher rebuilds it (`colcon build --packages-select servo_joint_publisher`) when it no longer matches the checkout, so start it through the launcher. If you run `ros2 run` or `ros2 launch` yourself, run `cd ros2_ws && colcon build` first.
- **Check the branch:** `grep -c "class DoguWidget" ros2_ws/src/servo_joint_publisher/servo_joint_publisher/calibrated_joint_gui.py` must print `1`. If it prints `0`, the branch you pulled doesn't have the Dogu tab yet.
- **Web dashboard:** it runs from the checkout, so a reload is enough. Restart `./launch_web_dashboard.sh` if the old server is still running.

### Q: Can I run both the Robotic Arm and Quard Bot together?
- **Yes**: Run `./launch_dual_dashboard.sh`. The dashboard loads both robots simultaneously into RViz2.

### Q: How do I connect to physical hardware?
- **Arm:** `servo_serial_commander` owns the serial port (default `/dev/ttyUSB0` @ 115200). Select Serial or Wi-Fi with the **Comm** toggle; change the port or IP with **⚙ Link**.
- **Quard Bot:** enter the ESP32 IP or `http://sesame-robot.local` and press **Ping**.
- **Dogu:** join its Wi-Fi access point, keep `http://192.168.4.1`, and press **Ping**.
- **Rover:** enter its Wi-Fi IP and press **Connect**, or pick the serial port and press **Connect Serial**.

### Q: The arm card says "No reply", "Mode mismatch" or "Port error".
See the table in [section 4](#-4-connections--consoles). The detail line names the port or IP and the reason; for a port error it includes the fix (for example the `dialout` group). Type `PING` in the arm console to test the link by hand.

### Q: The Quard Bot is "Unreachable: host name not found".
`sesame-robot.local` needs mDNS (Avahi on Linux). Use the robot's IP instead: it is shown in the telemetry row whenever it was reachable, and on the router.

### Q: The rover says "Waiting for data".
The serial port opened but nothing arrives: usually the wrong baud rate, or a different board on that port.

### Q: A joint card is outlined in amber (web).
The joint's current angle is outside the Min/Max saved for it. Move it back into the green band, or adjust and save the limits.

### Q: The OLED face buttons say the command was rejected.
The Quard firmware does not implement a `face` command yet (it answers `400 Bad Args`). The robot is online; only that command is unsupported.

### Q: How do I reset the desktop window layout?
Delete `~/.config/CodeForIndia/RobotDashboard.conf`.

### Known issue
In the desktop app, **Start Sim** on the arm tab also starts the Quard Bot walking forward, and stopping the sim makes it stand. Don't press it with the quadruped powered unless you want both to move.
