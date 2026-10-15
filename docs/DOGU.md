# 🐕 Dogu (robot dog)

Dogu is an ESP32 robot dog with 12 servos on a PCA9685 driver. The firmware walks with 8 of them (hip and knee on each leg) and holds the other 4 ("body" servos) still. It starts its own Wi-Fi access point and answers three HTTP commands, which both dashboards use.

---

## 📁 Where the Dogu files are

| What | Path |
|---|---|
| Firmware (PlatformIO env `dogu`) | `src/dogu.cpp` (build: `pio run -e dogu -t upload`) |
| Wi-Fi name and password | `include/secrets.h` (gitignored; template `include/secrets.h.example`) |
| Runtime URDF, meshes, poses (RViz, web twin) | `urdf/dogu/` (generated: see [`urdf/dogu/README.md`](../urdf/dogu/README.md)) |
| Source model | `cad/dogu/Dogu.blend` |
| Dashboard code | the Dogu tab: `DoguWidget` in `calibrated_joint_gui.py`, `/api/dogu/*` in `web_dashboard/server.py`, the Dogu section of `web_dashboard/static/app.js` |

---

## 🔌 Hardware

| Part | Connection |
|---|---|
| ESP32 dev board (`nodemcu-32s`) | I²C SDA = GPIO 21, SCL = GPIO 22 |
| PCA9685 servo driver | address `0x40`, 50 Hz |

If the PCA9685 doesn't answer at boot, the firmware prints `ERROR: PCA9685 not detected` on serial (115200 baud) and stops there.

### Servo channels

| Leg | Hip (swings forward / back) | Knee (lifts the foot) | Body servo (held still) |
|---|---|---|---|
| Front left | CH0 | CH1 | one of CH8–CH11 |
| Front right | CH2 | CH3 | one of CH8–CH11 |
| Back left | CH4 | CH5 | one of CH8–CH11 |
| Back right | CH6 | CH7 | one of CH8–CH11 |

The firmware doesn't say which body servo belongs to which leg. In the 12-joint model, the hip (abduction) joints are the likely match for the body servos, the firmware's hip for the femur and its knee for the tibia. That mapping is unconfirmed, so the dashboards never drive the model from the robot.

### Calibration (PCA9685 ticks, out of 4096 per 20 ms)

| Value | Default | Meaning |
|---|---|---|
| `hipCenter[4]` | 305 | Hip neutral |
| `kneeDown[4]` | 320 | Knee with the foot on the ground |
| `hipDirection[4]` | −1, 1, −1, 1 | Flip a leg whose servo is mounted the other way |
| `kneeDirection[4]` | 1, 1, 1, 1 | Same for the knees |
| `HIP_STEP` | 20 | Hip swing either side of neutral |
| `KNEE_LIFT` | 25 | How far the knee lifts the foot |
| `bodyHoldPWM[4]` | 305 | Body servo hold positions |

> ⚠️ These are the firmware's starting values, not measured ones. The firmware itself warns that the body servo values are examples and **not guaranteed safe**: calibrate each servo before connecting its load.

---

## 📶 Wi-Fi

Dogu always starts its own access point; it never joins another network.

1. Copy `include/secrets.h.example` to `include/secrets.h` and set `DOGU_AP_SSID` and `DOGU_AP_PASSWORD`. The password must be 8 to 63 characters, or the access point won't start. Without these two values the build stops with an error that names the file.
2. Flash: `pio run -e dogu -t upload`.
3. Join that network from the computer running the dashboard, then use **`http://192.168.4.1`**.

At boot the firmware prints the network name, its password and the address on serial.

---

## 🌐 HTTP API (port 80)

| Request | Reply | What it does |
|---|---|---|
| `GET /` | HTML | The firmware's own control page (Start / Stop and live status) |
| `GET /start` | `Walking started` | Starts walking from the home pose. Does nothing if it is already walking. |
| `GET /stop` | `Walking stopped; hold positions commanded` | Stops walking, puts all legs at home and re-sends the body servo holds |
| `GET /status` | `{"walking": true, "leg": "Front Left", "phase": "Lift"}` | `phase` is `Stopped` while it isn't walking |

There are no joint angles in `/status`, and no other commands.

### Gait

It moves one leg at a time: front left, front right, back left, back right. Each leg runs five phases of 500 ms: **Lift → Swing forward → Lower → Push backward → Return**. That makes 2.5 s per leg and 10 s per full cycle. Each phase eases in and out (smoothstep), with an update every 20 ms.

---

## 🖥️ In the dashboards

Both UIs have the same Dogu tab:

- **Robot link:** the address (default `http://192.168.4.1`), **Ping**, and **Auto**. Auto checks `/status` every 3 s while online, every 1 s while it walks, and backs off to 10 s while it is unreachable.
- **Robot:** what `/status` last reported, for example `Walking · Front Left · Lift`. It shows `--` until the robot answers. While it is unreachable, it shows the last report and its age.
- **Start walking** / **Stop walking** send `/start` and `/stop`, one at a time and in order. A newer press replaces one that hasn't been sent yet. A Start that waited more than 2 s behind a slow request is dropped, never sent late.
- **E-STOP** (button or Esc) sends `/stop` and locks Start walking until you release it. Stop walking always works.
- **Model preview:** the poses and moves underneath animate the 3D model only. Nothing is sent to the robot.

The web dashboard reaches the robot through its server (`POST /api/dogu/ping`, `POST /api/dogu/cmd`), so the browser doesn't need to be on Dogu's network; the computer running `server.py` does.

---

## ⚠️ Safety

- **No link watchdog.** Once started, Dogu walks until it gets `/stop` or loses power. If Wi-Fi drops, the dashboards can't stop it (a failed Stop says so in the console). Use the firmware's own page at the same address from another device, or cut its power. The Quard Bot, by contrast, stands up by itself 2 s after its last command.
- E-STOP is a software stop: it only works while the dashboard can reach the robot.
- `/start` and `/stop` have no authentication. Anyone on Dogu's network can send them, so keep the access point password private.
