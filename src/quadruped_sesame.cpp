// ======================================================================
//   Sesame Quadruped Robot — Clean Real-Time Firmware (No OLED)
//   • Core 0: Wi-Fi UDP (Port 8888), WebServer (Port 80), Watchdog
//   • Core 1: Dedicated Locomotion & Gait Task (PCA9685 I2C 50Hz)
// ======================================================================

#include <WiFi.h>
#include <WebServer.h>
#include <ESPmDNS.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// 🛡️ ESP32 Power System Control (ESP32 Core v3.x API)
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"

// 🌐 Unified Robot Communication Protocol
#include "robot_protocol.h"

RobotProtocol robotComm("Sesame_Quadruped");

// ── PCA9685 I2C Pin Definitions ─────────────────────────────────────────
#define PCA_SDA  21
#define PCA_SCL  22

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);
WebServer server(80);

// ── Kinematics & Calibration ────────────────────────────────────────────
// Base zero offset for each of the 8 servo channels:
const int baseZero[8] = {90, 0, 0, 90, 90, 0, 90, 0};
int currentInputAngles[8] = {45, 45, 45, 45, 45, 45, 45, 45};

enum GaitMode {
  MODE_STAND,
  MODE_WALK_FORWARD,
  MODE_WALK_BACKWARD,
  MODE_TURN_LEFT,
  MODE_TURN_RIGHT
};

volatile GaitMode currentMode = MODE_STAND;
volatile GaitMode requestedMode = MODE_STAND;
volatile int stepDelay = 220; // Gait step phase delay (ms)

// Locomotion safety watchdog: stop walking if no command received for 2 seconds
const unsigned long LOCOMOTION_WATCHDOG_MS = 2000;
unsigned long lastMotionCommandMs = 0;

// Mutex for PCA9685 I2C bus transactions
SemaphoreHandle_t pcaMutex = NULL;

// ── Servo Kinematics ────────────────────────────────────────────────────
int calculateFinalAngle(int channel, int inputAngle) {
  int finalAngle = (baseZero[channel] == 90) ? (90 - inputAngle) : (0 + inputAngle);
  return constrain(finalAngle, 0, 180);
}

void setServoInputAngle(uint8_t channel, int inputAngle) {
  if (channel < 8) currentInputAngles[channel] = inputAngle;
  int finalAngle = calculateFinalAngle(channel, inputAngle);
  int uS = map(finalAngle, 0, 180, 732, 2929);

  if (pcaMutex != NULL && xSemaphoreTake(pcaMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
    pwm.writeMicroseconds(channel, uS);
    xSemaphoreGive(pcaMutex);
  }
}

void setAllInputAngles(int targetInputAngle) {
  bool moving = true;
  while (moving) {
    moving = false;
    for (int i = 0; i < 8; i++) {
      if (currentInputAngles[i] < targetInputAngle) {
        currentInputAngles[i] = min(currentInputAngles[i] + 3, targetInputAngle);
        int finalAngle = calculateFinalAngle(i, currentInputAngles[i]);
        int uS = map(finalAngle, 0, 180, 732, 2929);
        if (pcaMutex != NULL && xSemaphoreTake(pcaMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
          pwm.writeMicroseconds(i, uS);
          xSemaphoreGive(pcaMutex);
        }
        moving = true;
      } else if (currentInputAngles[i] > targetInputAngle) {
        currentInputAngles[i] = max(currentInputAngles[i] - 3, targetInputAngle);
        int finalAngle = calculateFinalAngle(i, currentInputAngles[i]);
        int uS = map(finalAngle, 0, 180, 732, 2929);
        if (pcaMutex != NULL && xSemaphoreTake(pcaMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
          pwm.writeMicroseconds(i, uS);
          xSemaphoreGive(pcaMutex);
        }
        moving = true;
      }
    }
    vTaskDelay(pdMS_TO_TICKS(15));
  }
}

// Non-blocking interruptible phase delay: returns false if mode changed
bool gaitPhaseDelay(int ms) {
  int elapsed = 0;
  while (elapsed < ms) {
    if (currentMode != requestedMode || currentMode == MODE_STAND) {
      return false; // Preempt immediately!
    }
    vTaskDelay(pdMS_TO_TICKS(10));
    elapsed += 10;
  }
  return true;
}

void poseStand()  { currentMode = MODE_STAND; requestedMode = MODE_STAND; setAllInputAngles(45); }
void poseZero()   { currentMode = MODE_STAND; requestedMode = MODE_STAND; setAllInputAngles(0); }
void poseCrouch() { currentMode = MODE_STAND; requestedMode = MODE_STAND; setAllInputAngles(30); }
void poseHigh()   { currentMode = MODE_STAND; requestedMode = MODE_STAND; setAllInputAngles(60); }

void animHiAction() {
  currentMode = MODE_STAND; requestedMode = MODE_STAND;
  setAllInputAngles(60);
  vTaskDelay(pdMS_TO_TICKS(250));
  for (int w = 0; w < 3; w++) {
    setServoInputAngle(5, 0);  vTaskDelay(pdMS_TO_TICKS(140));
    setServoInputAngle(5, 35); vTaskDelay(pdMS_TO_TICKS(140));
  }
  poseStand();
}

// ── Locomotion Steps (Interruptible) ────────────────────────────────────
bool stepWalkForward() {
  // Phase 1: Lift Diag A & swing hips forward
  setServoInputAngle(5, 25); setServoInputAngle(7, 65);
  setServoInputAngle(0, 70); setServoInputAngle(3, 70);
  if (!gaitPhaseDelay(stepDelay)) return false;

  // Phase 2: Ground Diag A
  setServoInputAngle(5, 45); setServoInputAngle(7, 45);
  setServoInputAngle(0, 45); setServoInputAngle(3, 45);
  if (!gaitPhaseDelay(stepDelay / 2)) return false;

  // Phase 3: Lift Diag B & swing hips forward
  setServoInputAngle(4, 65); setServoInputAngle(6, 25);
  setServoInputAngle(1, 20); setServoInputAngle(2, 20);
  if (!gaitPhaseDelay(stepDelay)) return false;

  // Phase 4: Ground Diag B
  setServoInputAngle(4, 45); setServoInputAngle(6, 45);
  setServoInputAngle(1, 45); setServoInputAngle(2, 45);
  return gaitPhaseDelay(stepDelay / 2);
}

bool stepWalkBackward() {
  setServoInputAngle(4, 25); setServoInputAngle(6, 65);
  setServoInputAngle(1, 70); setServoInputAngle(2, 70);
  if (!gaitPhaseDelay(stepDelay)) return false;

  setServoInputAngle(4, 45); setServoInputAngle(6, 45);
  setServoInputAngle(1, 45); setServoInputAngle(2, 45);
  if (!gaitPhaseDelay(stepDelay / 2)) return false;

  setServoInputAngle(5, 65); setServoInputAngle(7, 25);
  setServoInputAngle(0, 20); setServoInputAngle(3, 20);
  if (!gaitPhaseDelay(stepDelay)) return false;

  setServoInputAngle(5, 45); setServoInputAngle(7, 45);
  setServoInputAngle(0, 45); setServoInputAngle(3, 45);
  return gaitPhaseDelay(stepDelay / 2);
}

bool stepTurnLeft() {
  setServoInputAngle(5, 30); setServoInputAngle(4, 60);
  setServoInputAngle(0, 70); setServoInputAngle(1, 70);
  if (!gaitPhaseDelay(stepDelay)) return false;

  setServoInputAngle(5, 45); setServoInputAngle(4, 45);
  setServoInputAngle(0, 45); setServoInputAngle(1, 45);
  if (!gaitPhaseDelay(stepDelay / 2)) return false;

  setServoInputAngle(6, 60); setServoInputAngle(7, 30);
  setServoInputAngle(2, 20); setServoInputAngle(3, 20);
  if (!gaitPhaseDelay(stepDelay)) return false;

  setServoInputAngle(6, 45); setServoInputAngle(7, 45);
  setServoInputAngle(2, 45); setServoInputAngle(3, 45);
  return gaitPhaseDelay(stepDelay / 2);
}

bool stepTurnRight() {
  setServoInputAngle(6, 30); setServoInputAngle(7, 60);
  setServoInputAngle(2, 70); setServoInputAngle(3, 70);
  if (!gaitPhaseDelay(stepDelay)) return false;

  setServoInputAngle(6, 45); setServoInputAngle(7, 45);
  setServoInputAngle(2, 45); setServoInputAngle(3, 45);
  if (!gaitPhaseDelay(stepDelay / 2)) return false;

  setServoInputAngle(5, 60); setServoInputAngle(4, 30);
  setServoInputAngle(0, 20); setServoInputAngle(1, 20);
  if (!gaitPhaseDelay(stepDelay)) return false;

  setServoInputAngle(5, 45); setServoInputAngle(4, 45);
  setServoInputAngle(0, 45); setServoInputAngle(1, 45);
  return gaitPhaseDelay(stepDelay / 2);
}

/**
 * FreeRTOS Task: Dedicated Locomotion & Gait Engine running on Core 1.
 * Completely isolates servo timing from Wi-Fi/HTTP/UDP network traffic.
 */
void locomotionTask(void* pvParameters) {
  (void)pvParameters;

  for (;;) {
    currentMode = requestedMode;

    switch (currentMode) {
      case MODE_WALK_FORWARD:
        stepWalkForward();
        break;
      case MODE_WALK_BACKWARD:
        stepWalkBackward();
        break;
      case MODE_TURN_LEFT:
        stepTurnLeft();
        break;
      case MODE_TURN_RIGHT:
        stepTurnRight();
        break;
      case MODE_STAND:
      default:
        vTaskDelay(pdMS_TO_TICKS(30));
        break;
    }
  }
}

// ── Web Dashboard & Status JSON ─────────────────────────────────────────
String getSesameStatusJson() {
  String json = "{";
  json += "\"name\":\"Sesame Robot\",";
  json += "\"online\":true,";
  json += "\"mode\":";
  switch(currentMode) {
    case MODE_WALK_FORWARD:  json += "\"forward\","; break;
    case MODE_WALK_BACKWARD: json += "\"backward\","; break;
    case MODE_TURN_LEFT:     json += "\"left\","; break;
    case MODE_TURN_RIGHT:    json += "\"right\","; break;
    default:                 json += "\"stand\","; break;
  }
  json += "\"step_delay\":" + String(stepDelay) + ",";
  json += "\"ip\":\"" + robotComm.getIP().toString() + "\",";
  json += "\"rssi\":" + String(WiFi.status() == WL_CONNECTED ? String(WiFi.RSSI()) : "0") + ",";
  json += "\"uptime_ms\":" + String(millis());
  json += "}";
  return json;
}

const char DASHBOARD_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1.0">
  <title>Sesame Robot AP Dashboard</title>
  <link href="https://fonts.googleapis.com/css2?family=Outfit:wght@300;400;600;800&display=swap" rel="stylesheet">
  <style>
    :root { --bg: #0b0f19; --card-bg: rgba(22, 30, 48, 0.75); --card-border: rgba(255, 255, 255, 0.1); --accent: #6366f1; --accent-glow: rgba(99, 102, 241, 0.4); --danger: #ef4444; --success: #10b981; --text: #f3f4f6; --text-muted: #9ca3af; }
    * { box-sizing: border-box; margin: 0; padding: 0; font-family: 'Outfit', sans-serif; user-select: none; }
    body { background: radial-gradient(circle at top, #1e1b4b 0%, var(--bg) 100%); color: var(--text); min-height: 100vh; padding: 20px; display: flex; flex-direction: column; align-items: center; }
    header { text-align: center; margin-bottom: 24px; }
    header h1 { font-size: 2rem; font-weight: 800; background: linear-gradient(135deg, #a855f7, #6366f1, #3b82f6); -webkit-background-clip: text; -webkit-text-fill-color: transparent; }
    header p { color: var(--text-muted); font-size: 0.9rem; margin-top: 4px; }
    .status-badge { display: inline-flex; align-items: center; gap: 6px; padding: 4px 12px; border-radius: 999px; background: rgba(16, 185, 129, 0.15); border: 1px solid rgba(16, 185, 129, 0.3); color: var(--success); font-size: 0.8rem; font-weight: 600; margin-top: 8px; }
    .status-dot { width: 8px; height: 8px; background: var(--success); border-radius: 50%; box-shadow: 0 0 8px var(--success); }
    .container { width: 100%; max-width: 900px; display: grid; grid-template-columns: 1fr 1fr; gap: 20px; }
    @media (max-width: 768px) { .container { grid-template-columns: 1fr; } }
    .card { background: var(--card-bg); backdrop-filter: blur(16px); border: 1px solid var(--card-border); border-radius: 20px; padding: 24px; box-shadow: 0 20px 40px rgba(0, 0, 0, 0.4); }
    .card h2 { font-size: 1.1rem; font-weight: 600; margin-bottom: 16px; color: var(--text-muted); text-transform: uppercase; letter-spacing: 1px; }
    .dpad-grid { display: grid; grid-template-columns: repeat(3, 1fr); grid-template-rows: repeat(3, 1fr); gap: 12px; width: 220px; height: 220px; margin: 0 auto; }
    .btn-dpad { background: rgba(255, 255, 255, 0.05); border: 1px solid rgba(255, 255, 255, 0.15); border-radius: 16px; color: var(--text); font-size: 1.5rem; display: flex; align-items: center; justify-content: center; cursor: pointer; transition: all 0.15s ease; box-shadow: 0 4px 12px rgba(0, 0, 0, 0.2); }
    .btn-dpad:hover { background: var(--accent); border-color: var(--accent); box-shadow: 0 0 20px var(--accent-glow); }
    .btn-dpad.stop { background: rgba(239, 68, 68, 0.2); border-color: rgba(239, 68, 68, 0.4); color: var(--danger); font-size: 0.9rem; font-weight: 700; }
    .actions-grid { display: grid; grid-template-columns: repeat(2, 1fr); gap: 12px; margin-top: 20px; }
    .btn-action { background: rgba(255, 255, 255, 0.05); border: 1px solid rgba(255, 255, 255, 0.15); border-radius: 14px; padding: 14px; color: var(--text); font-size: 0.95rem; font-weight: 600; cursor: pointer; display: flex; align-items: center; justify-content: center; gap: 8px; transition: all 0.2s ease; }
    .btn-action:hover { background: var(--accent); border-color: var(--accent); box-shadow: 0 0 16px var(--accent-glow); }
    .slider-group { display: flex; flex-direction: column; gap: 14px; }
    .slider-row { display: flex; flex-direction: column; gap: 4px; }
    .slider-header { display: flex; justify-content: space-between; font-size: 0.85rem; color: var(--text-muted); }
    .slider-header span.val { color: var(--accent); font-weight: 700; }
    input[type=range] { -webkit-appearance: none; width: 100%; height: 6px; background: rgba(255, 255, 255, 0.1); border-radius: 4px; outline: none; }
    input[type=range]::-webkit-slider-thumb { -webkit-appearance: none; width: 18px; height: 18px; border-radius: 50%; background: var(--accent); cursor: pointer; }
  </style>
</head>
<body>
  <header>
    <h1>SESAME ROBOT</h1>
    <p>Dual-Core Real-Time Locomotion + Ultra-Low Latency UDP</p>
    <div class="status-badge"><div class="status-dot"></div><span id="ip-badge">Connecting...</span></div>
  </header>
  <div class="container">
    <div class="card">
      <h2>Locomotion Control</h2>
      <div class="dpad-grid">
        <div></div><button class="btn-dpad" onclick="sendCmd('move', 'w')">FORWARD</button><div></div>
        <button class="btn-dpad" onclick="sendCmd('move', 'l')">LEFT</button>
        <button class="btn-dpad stop" onclick="sendCmd('move', 's')">STAND</button>
        <button class="btn-dpad" onclick="sendCmd('move', 'r')">RIGHT</button>
        <div></div><button class="btn-dpad" onclick="sendCmd('move', 'b')">BACK</button><div></div>
      </div>
      <h2 style="margin-top: 24px;">Quick Actions</h2>
      <div class="actions-grid">
        <button class="btn-action" onclick="sendCmd('move', 'hi')">HI Wave</button>
        <button class="btn-action" onclick="sendCmd('move', 's')">Stand (45deg)</button>
        <button class="btn-action" onclick="sendCmd('move', 'z')">Base Zero (0deg)</button>
        <button class="btn-action" onclick="sendAngleAll(60)">High Stance (60deg)</button>
      </div>
    </div>
    <div class="card">
      <h2>Live Channel Sliders</h2>
      <div class="slider-group">
        <div class="slider-row"><div class="slider-header"><span>Ch 0: R1 (Right Front Hip)</span><span class="val" id="val-0">45deg</span></div><input type="range" min="0" max="180" value="45" oninput="updateSlider(0, this.value)"></div>
        <div class="slider-row"><div class="slider-header"><span>Ch 1: R2 (Right Rear Hip)</span><span class="val" id="val-1">45deg</span></div><input type="range" min="0" max="180" value="45" oninput="updateSlider(1, this.value)"></div>
        <div class="slider-row"><div class="slider-header"><span>Ch 2: L1 (Left Front Hip)</span><span class="val" id="val-2">45deg</span></div><input type="range" min="0" max="180" value="45" oninput="updateSlider(2, this.value)"></div>
        <div class="slider-row"><div class="slider-header"><span>Ch 3: L2 (Left Rear Hip)</span><span class="val" id="val-3">45deg</span></div><input type="range" min="0" max="180" value="45" oninput="updateSlider(3, this.value)"></div>
        <div class="slider-row"><div class="slider-header"><span>Ch 4: R4 (Right Rear Foot)</span><span class="val" id="val-4">45deg</span></div><input type="range" min="0" max="180" value="45" oninput="updateSlider(4, this.value)"></div>
        <div class="slider-row"><div class="slider-header"><span>Ch 5: R3 (Right Front Foot)</span><span class="val" id="val-5">45deg</span></div><input type="range" min="0" max="180" value="45" oninput="updateSlider(5, this.value)"></div>
        <div class="slider-row"><div class="slider-header"><span>Ch 6: L3 (Left Front Foot)</span><span class="val" id="val-6">45deg</span></div><input type="range" min="0" max="180" value="45" oninput="updateSlider(6, this.value)"></div>
        <div class="slider-row"><div class="slider-header"><span>Ch 7: L4 (Left Rear Foot)</span><span class="val" id="val-7">45deg</span></div><input type="range" min="0" max="180" value="45" oninput="updateSlider(7, this.value)"></div>
      </div>
    </div>
  </div>
  <script>
    const sendCmd = (type, val) => fetch('/cmd?' + type + '=' + val).catch(err => console.error(err));
    let sliderDebounce = {};
    const updateSlider = (ch, val) => {
      document.getElementById('val-' + ch).innerText = val + 'deg';
      clearTimeout(sliderDebounce[ch]);
      sliderDebounce[ch] = setTimeout(() => fetch('/cmd?ch=' + ch + '&deg=' + val).catch(err => console.error(err)), 50);
    };
    const sendAngleAll = (val) => fetch('/cmd?all=' + val).catch(err => console.error(err));
    fetch('/status').then(r=>r.json()).then(d=>{ document.getElementById('ip-badge').innerText = 'Online (' + d.ip + ')'; });
  </script>
</body>
</html>
)rawliteral";

void handleRoot() { server.send(200, "text/html", DASHBOARD_HTML); }

void handleStatus() {
  server.sendHeader("Connection", "close");
  server.sendHeader("Access-Control-Allow-Origin", "*");
  server.send(200, "application/json", getSesameStatusJson());
}

void handleCommand() {
  server.sendHeader("Connection", "close");
  server.sendHeader("Access-Control-Allow-Origin", "*");
  lastMotionCommandMs = millis();

  bool handled = false;
  String m = "";
  if (server.hasArg("move")) m = server.arg("move");
  else if (server.hasArg("mode")) m = server.arg("mode");
  else if (server.hasArg("cmd")) m = server.arg("cmd");

  if (m.length() > 0) {
    m.toLowerCase();
    if (m == "w" || m == "forward") { requestedMode = MODE_WALK_FORWARD; handled = true; }
    else if (m == "b" || m == "backward") { requestedMode = MODE_WALK_BACKWARD; handled = true; }
    else if (m == "l" || m == "left") { requestedMode = MODE_TURN_LEFT; handled = true; }
    else if (m == "r" || m == "right") { requestedMode = MODE_TURN_RIGHT; handled = true; }
    else if (m == "s" || m == "stand" || m == "stop") { requestedMode = MODE_STAND; poseStand(); handled = true; }
    else if (m == "z" || m == "zero") { poseZero(); handled = true; }
    else if (m == "hi" || m == "wave") { animHiAction(); handled = true; }
    else if (m == "crouch") { poseCrouch(); handled = true; }
    else if (m == "high") { poseHigh(); handled = true; }
  }

  if (server.hasArg("ch")) {
    int ch = server.arg("ch").toInt();
    int angle = server.hasArg("deg") ? server.arg("deg").toInt() : server.arg("val").toInt();
    if (angle >= 0 && ch >= 0 && ch < 8) {
      requestedMode = MODE_STAND;
      setServoInputAngle(ch, angle);
      handled = true;
    }
  }

  if (server.hasArg("all")) {
    requestedMode = MODE_STAND;
    setAllInputAngles(server.arg("all").toInt());
    handled = true;
  }

  if (server.hasArg("speed") || server.hasArg("delay")) {
    int s = server.hasArg("speed") ? server.arg("speed").toInt() : server.arg("delay").toInt();
    if (s >= 50 && s <= 800) { stepDelay = s; handled = true; }
  }

  server.send(handled ? 200 : 400, "text/plain", handled ? "OK" : "Bad Args");
}

/**
 * High-Speed Unified UDP & Serial Command Processor
 */
void processSesameCommand(const String& prefix, const String& target, const String& payload, String& reply) {
  lastMotionCommandMs = millis();

  if (prefix == "CMD") {
    String tgt = target; tgt.toLowerCase();
    String val = payload; val.toLowerCase();

    // Movement / Locomotion
    if (tgt == "move" || tgt == "gait" || tgt == "drive") {
      if (val == "w" || val == "forward" || val == "walk_forward") {
        requestedMode = MODE_WALK_FORWARD;
        reply = "ACK:move=forward";
      } else if (val == "b" || val == "backward" || val == "walk_backward") {
        requestedMode = MODE_WALK_BACKWARD;
        reply = "ACK:move=backward";
      } else if (val == "l" || val == "left" || val == "turn_left") {
        requestedMode = MODE_TURN_LEFT;
        reply = "ACK:move=left";
      } else if (val == "r" || val == "right" || val == "turn_right") {
        requestedMode = MODE_TURN_RIGHT;
        reply = "ACK:move=right";
      } else if (val == "s" || val == "stand" || val == "stop") {
        requestedMode = MODE_STAND;
        poseStand();
        reply = "ACK:move=stand";
      } else if (val == "z" || val == "zero") {
        poseZero();
        reply = "ACK:move=zero";
      } else if (val == "hi" || val == "wave") {
        animHiAction();
        reply = "ACK:move=wave";
      } else if (val == "crouch") {
        poseCrouch();
        reply = "ACK:move=crouch";
      } else if (val == "high") {
        poseHigh();
        reply = "ACK:move=high";
      } else {
        reply = "ERR:UNKNOWN_MOVE:" + payload;
      }
    }
    // Direct Channel (e.g. CMD:ch0=45 or CMD:ch=0,45)
    else if (tgt.startsWith("ch")) {
      int ch = -1, angle = -1;
      if (tgt == "ch") {
        int commaIdx = payload.indexOf(',');
        if (commaIdx > 0) {
          ch = payload.substring(0, commaIdx).toInt();
          angle = payload.substring(commaIdx + 1).toInt();
        }
      } else {
        ch = tgt.substring(2).toInt();
        angle = payload.toInt();
      }
      if (ch >= 0 && ch < 8 && angle >= 0 && angle <= 180) {
        requestedMode = MODE_STAND;
        setServoInputAngle(ch, angle);
        reply = "ACK:ch" + String(ch) + "=" + String(angle);
      } else {
        reply = "ERR:INVALID_CHANNEL_OR_ANGLE";
      }
    }
    // Set All Servos
    else if (tgt == "all") {
      requestedMode = MODE_STAND;
      setAllInputAngles(payload.toInt());
      reply = "ACK:all=" + payload;
    }
    // Gait Step Delay / Speed
    else if (tgt == "speed" || tgt == "delay") {
      int s = payload.toInt();
      if (s >= 50 && s <= 800) {
        stepDelay = s;
        reply = "ACK:speed=" + String(s);
      } else {
        reply = "ERR:SPEED_RANGE_50_800";
      }
    }
    // Direct Actions
    else if (tgt == "stand" || tgt == "stop") {
      requestedMode = MODE_STAND;
      poseStand();
      reply = "ACK:stand";
    } else if (tgt == "zero") {
      poseZero();
      reply = "ACK:zero";
    } else if (tgt == "wave" || tgt == "hi") {
      animHiAction();
      reply = "ACK:wave";
    } else if (tgt == "crouch") {
      poseCrouch();
      reply = "ACK:crouch";
    } else if (tgt == "high") {
      poseHigh();
      reply = "ACK:high";
    } else {
      reply = "ERR:UNKNOWN_TARGET:" + target;
    }
  } else if (prefix == "CALIB") {
    reply = "ACK:CALIB:" + target + "=" + payload;
  } else {
    reply = "ERR:UNKNOWN_PREFIX:" + prefix;
  }
}

void setup() {
  // 🛡️ Disable ESP32 Brownout Detector (Core v3 syntax)
  #ifdef RTC_CNTL_BROWN_OUT_REG
    WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);
  #endif

  Serial.begin(115200);
  delay(500);

  Serial.println("\n==================================================");
  Serial.println(" 🤖 SESAME ROBOT - REAL-TIME DUAL-CORE FIRMWARE");
  Serial.println("==================================================");

  pcaMutex = xSemaphoreCreateMutex();

  // 1. Initialize PCA9685 I2C (GPIO 21 & 22)
  Wire.begin(PCA_SDA, PCA_SCL);
  Wire.setClock(400000);
  pwm.begin();
  pwm.setPWMFreq(50);
  Serial.println(" ✅ PCA9685 I2C initialized on GPIO 21 (SDA) / GPIO 22 (SCL)");

  // 2. Launch Dedicated Locomotion Task on Core 1 (Isolated from Wi-Fi stack)
  xTaskCreatePinnedToCore(
    locomotionTask,
    "LocomotionEngine",
    4096,
    NULL,
    2,
    NULL,
    1 // Core 1
  );
  Serial.println("🎯 Locomotion Engine task launched on Core 1 (Real-Time)");

  // 3. Initialize Unified Communication Protocol on Core 0
  RobotWiFiConfig wifiCfg(
    "123", "12345678",       // Primary Wi-Fi (Active Router network)
    "Sabo", "sandy0606",     // Secondary Wi-Fi (Hotspot)
    "Sesame_AP", "sesame123", // SoftAP Fallback
    8888,                    // UDP Port
    "sesame-robot"           // mDNS Hostname (http://sesame-robot.local)
  );

  robotComm.setConfig(wifiCfg);
  robotComm.setCommandHandler(processSesameCommand);
  robotComm.setStatusHandler(getSesameStatusJson);
  robotComm.begin();

  // ⚡ Wi-Fi Power & Latency Optimization
  WiFi.setSleep(false);                 // Sub-2ms low-latency UDP response
  WiFi.setTxPower(WIFI_POWER_17dBm);    // Mitigates RF current spikes to prevent servo brownout

  // 4. Start Web Dashboard Server (Port 80)
  server.on("/", handleRoot);
  server.on("/cmd", handleCommand);
  server.on("/status", handleStatus);
  server.on("/state", handleStatus);
  server.begin();

  lastMotionCommandMs = millis();
  poseStand();
}

void loop() {
  // Main Communication & Network Loop (Runs isolated on Core 0)
  robotComm.update();
  server.handleClient();

  // 🛡️ Safety Watchdog: If walking and connection goes silent, return to STAND
  if (currentMode != MODE_STAND) {
    if (millis() - lastMotionCommandMs > LOCOMOTION_WATCHDOG_MS) {
      Serial.println("⚠️ Locomotion Watchdog Timeout (>2s silent) -> Auto-Standing");
      poseStand();
    }
  }

  delay(1);
}
