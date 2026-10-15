// Dogu (robot dog) firmware: ESP32 + PCA9685, 8 walking servos + 4 fixed body servos.
// Build/flash: pio run -e dogu -t upload      Docs: docs/DOGU.md
//
// Starts its own Wi-Fi access point (name and password in include/secrets.h)
// and serves a small control page plus GET /start, /stop and /status on port 80.
// Both dashboards use those three endpoints (Dogu tab).

#include <Wire.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Adafruit_PWMServoDriver.h>

#if __has_include("secrets.h")
#include "secrets.h"
#endif
#if !defined(DOGU_AP_SSID) || !defined(DOGU_AP_PASSWORD)
#error "Define DOGU_AP_SSID and DOGU_AP_PASSWORD in include/secrets.h (copy include/secrets.h.example)"
#endif

// ==================================================
// ESP32 + PCA9685 ROBOTIC DOG
// 8 walking servos + 4 fixed body servos
// ==================================================

Adafruit_PWMServoDriver pwm(0x40);
WebServer server(80);

const char* AP_NAME = DOGU_AP_SSID;
const char* AP_PASS = DOGU_AP_PASSWORD;

// PCA9685 channels for the 4 legs
// Front Left:  CH0 hip, CH1 knee
// Front Right: CH2 hip, CH3 knee
// Back Left:   CH4 hip, CH5 knee
// Back Right:  CH6 hip, CH7 knee
const int hipCh[4]  = {0, 2, 4, 6};
const int kneeCh[4] = {1, 3, 5, 7};

const char* legNames[4] = {
  "Front Left", "Front Right",
  "Back Left", "Back Right"
};

// ==================================================
// LEG CALIBRATION
// Replace with your actual safe calibration values.
// ==================================================

int hipCenter[4] = {305, 305, 305, 305};
int kneeDown[4]  = {320, 320, 320, 320};

// Reverse a direction if the physical servo is mounted
// in the opposite orientation.
int hipDirection[4]  = {-1, 1, -1, 1};
int kneeDirection[4] = {1, 1, 1, 1};

const int HIP_STEP = 20;
const int KNEE_LIFT = 25;

const unsigned long PHASE_MS = 500;
const unsigned long UPDATE_MS = 20;

// ==================================================
// BODY SERVOS: CH8, CH9, CH10, CH11
// These PWM values are examples, NOT guaranteed safe.
// Calibrate each body servo before connecting its load.
// ==================================================

const int bodyChannels[4] = {8, 9, 10, 11};

int bodyHoldPWM[4] = {
  305, 305, 305, 305
};

// ==================================================
// WALKING STATE
// ==================================================

bool walking = false;
int activeLeg = 0;
int phase = 0;

unsigned long phaseStarted = 0;
unsigned long lastUpdate = 0;

int currentHip[4];
int currentKnee[4];

int fromHip, fromKnee;
int toHip, toKnee;

const char* phaseNames[5] = {
  "Lift", "Swing forward", "Lower",
  "Push backward", "Return"
};

// ==================================================
// SERVO FUNCTIONS
// ==================================================

void writeLeg(int leg, int hip, int knee) {
  pwm.setPWM(hipCh[leg], 0, hip);
  pwm.setPWM(kneeCh[leg], 0, knee);

  currentHip[leg] = hip;
  currentKnee[leg] = knee;
}

void setLegsHome() {
  for (int i = 0; i < 4; i++) {
    writeLeg(i, hipCenter[i], kneeDown[i]);
  }
}

// Keep the four body servos at their fixed positions.
// Do not call this repeatedly during walking unless
// you have confirmed that your servos tolerate it.
void lockBodyServos() {
  for (int i = 0; i < 4; i++) {
    pwm.setPWM(
      bodyChannels[i],
      0,
      bodyHoldPWM[i]
    );
  }
}

// ==================================================
// WALKING SEQUENCE
// ==================================================

void beginPhase(int newPhase) {
  phase = newPhase;
  phaseStarted = millis();

  fromHip = currentHip[activeLeg];
  fromKnee = currentKnee[activeLeg];

  toHip = hipCenter[activeLeg];
  toKnee = kneeDown[activeLeg];

  switch (phase) {
    case 0: // Lift foot
      toKnee = kneeDown[activeLeg]
             + kneeDirection[activeLeg] * KNEE_LIFT;
      break;

    case 1: // Swing forward
      toHip = hipCenter[activeLeg]
            + hipDirection[activeLeg] * HIP_STEP;
      toKnee = kneeDown[activeLeg]
             + kneeDirection[activeLeg] * KNEE_LIFT;
      break;

    case 2: // Lower foot
      toHip = hipCenter[activeLeg]
            + hipDirection[activeLeg] * HIP_STEP;
      toKnee = kneeDown[activeLeg];
      break;

    case 3: // Push backward
      toHip = hipCenter[activeLeg]
            - hipDirection[activeLeg] * HIP_STEP;
      toKnee = kneeDown[activeLeg];
      break;

    case 4: // Return to center
      toHip = hipCenter[activeLeg];
      toKnee = kneeDown[activeLeg];
      break;
  }
}

void updateWalking() {
  if (!walking) return;

  unsigned long now = millis();

  if (now - lastUpdate < UPDATE_MS) return;
  lastUpdate = now;

  unsigned long elapsed = now - phaseStarted;

  float t = (float)elapsed / PHASE_MS;
  if (t > 1.0f) t = 1.0f;

  // Smooth transition between servo positions
  float smooth = t * t * (3.0f - 2.0f * t);

  int hip = fromHip
          + (int)((toHip - fromHip) * smooth);

  int knee = fromKnee
           + (int)((toKnee - fromKnee) * smooth);

  writeLeg(activeLeg, hip, knee);

  if (elapsed >= PHASE_MS) {
    if (phase < 4) {
      beginPhase(phase + 1);
    } else {
      activeLeg = (activeLeg + 1) % 4;
      beginPhase(0);
    }
  }
}

// ==================================================
// WEB DASHBOARD
// ==================================================

String dashboardPage() {
  return R"HTML(
<!DOCTYPE html>
<html>
<head>
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Robotic Dog Control</title>
<style>
body {
  font-family: Arial, sans-serif;
  background: #101827;
  color: white;
  padding: 18px;
}
h1 { color: #61dafb; }
.card {
  background: #1d293b;
  padding: 18px;
  border-radius: 12px;
  margin: 12px 0;
}
button {
  padding: 13px 18px;
  border: 0;
  border-radius: 8px;
  margin: 5px;
  cursor: pointer;
  font-weight: bold;
}
.start { background: #22c55e; }
.stop { background: #ef4444; color: white; }
.small { color: #cbd5e1; }
</style>
</head>
<body>
<h1>ROBOTIC DOG</h1>

<div class="card">
  <h3>Walking control</h3>
  <p class="small">8 leg servos + 4 fixed body servos</p>
  <button class="start" onclick="sendCommand('/start')">
    START WALKING
  </button>
  <button class="stop" onclick="sendCommand('/stop')">
    STOP WALKING
  </button>
  <p id="status">Connecting...</p>
</div>

<div class="card">
  <h3>Current movement</h3>
  <p id="leg">Waiting</p>
  <p id="phase">Waiting</p>
  <p>Body servos: CH8–CH11 held at configured positions</p>
</div>

<script>
async function sendCommand(path) {
  try {
    const response = await fetch(path, {cache: 'no-store'});
    const message = await response.text();
    document.getElementById('status').textContent = message;
    await refreshStatus();
  } catch (error) {
    document.getElementById('status').textContent =
      'Connection error: ' + error.message;
  }
}

async function refreshStatus() {
  try {
    const response = await fetch('/status', {cache: 'no-store'});
    if (!response.ok) throw new Error('HTTP ' + response.status);

    const data = await response.json();

    document.getElementById('status').textContent =
      data.walking ? 'Walking' : 'Stopped';

    document.getElementById('leg').textContent =
      'Active leg: ' + data.leg;

    document.getElementById('phase').textContent =
      'Phase: ' + data.phase;
  } catch (error) {
    document.getElementById('status').textContent =
      'Controller connection error';
  }
}

setInterval(refreshStatus, 500);
refreshStatus();
</script>
</body>
</html>
)HTML";
}

void handleRoot() {
  server.send(200, "text/html", dashboardPage());
}

void handleStart() {
  if (!walking) {
    walking = true;
    activeLeg = 0;

    // Establish the initial leg and body positions
    setLegsHome();
    lockBodyServos();

    beginPhase(0);

    Serial.println("Walking started");
  }

  server.send(200, "text/plain", "Walking started");
}

void handleStop() {
  walking = false;

  setLegsHome();
  lockBodyServos();

  Serial.println("Walking stopped; hold positions commanded");

  server.send(
    200,
    "text/plain",
    "Walking stopped; hold positions commanded"
  );
}

void handleStatus() {
  String json = "{\"walking\":";
  json += walking ? "true" : "false";

  json += ",\"leg\":\"";
  json += legNames[activeLeg];

  json += "\",\"phase\":\"";
  json += walking ? phaseNames[phase] : "Stopped";

  json += "\"}";

  server.send(200, "application/json", json);
}

// ==================================================
// SETUP
// ==================================================

void setup() {
  Serial.begin(115200);
  delay(300);

  Wire.begin(21, 22);

  if (!pwm.begin()) {
    Serial.println("ERROR: PCA9685 not detected");
    while (true) {
      delay(1000);
    }
  }

  pwm.setPWMFreq(50);
  delay(500);

  // Initialize the leg software state
  for (int i = 0; i < 4; i++) {
    currentHip[i] = hipCenter[i];
    currentKnee[i] = kneeDown[i];
  }

  // Command all leg and body servos to their initial positions
  setLegsHome();
  lockBodyServos();

  WiFi.mode(WIFI_AP);
  bool apStarted = WiFi.softAP(AP_NAME, AP_PASS);

  if (!apStarted) {
    Serial.println("ERROR: Wi-Fi AP failed to start");
  }

  server.on("/", HTTP_GET, handleRoot);
  server.on("/start", HTTP_GET, handleStart);
  server.on("/stop", HTTP_GET, handleStop);
  server.on("/status", HTTP_GET, handleStatus);

  server.begin();

  Serial.println();
  Serial.println("Robotic dog dashboard ready");
  Serial.print("Wi-Fi: ");
  Serial.println(AP_NAME);
  Serial.print("Password: ");
  Serial.println(AP_PASS);
  Serial.print("Dashboard: http://");
  Serial.println(WiFi.softAPIP());
}

void loop() {
  server.handleClient();
  updateWalking();
}
