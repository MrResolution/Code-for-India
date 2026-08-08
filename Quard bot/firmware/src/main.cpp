// ======================================================================
//   Sesame Robot — Staggered Startup & Brownout-Proof Firmware
//   ESP32 Core v3.x Compatible (GPIO 21/22 + SSD1306 OLED GPIO 18/19)
// ======================================================================

#include <WiFi.h>
#include <WebServer.h>
#include <ESPmDNS.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// 🛡️ ESP32 Power System Control (ESP32 Core v3.x API)
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"

#define AP_SSID   "Sesame-Robot-Control"
#define AP_PASS   "12345678"

#define PCA_SDA  21
#define PCA_SCL  22

#define OLED_SDA 18
#define OLED_SCL 19

TwoWire I2C_OLED = TwoWire(1);

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET    -1

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &I2C_OLED, OLED_RESET);

WebServer server(80);

const int baseZero[8] = {90, 0, 0, 90, 90, 0, 90, 0};

enum GaitMode {
  MODE_STAND,
  MODE_WALK_FORWARD,
  MODE_WALK_BACKWARD,
  MODE_TURN_LEFT,
  MODE_TURN_RIGHT
};

GaitMode currentMode = MODE_STAND;
int stepDelay = 250;

enum FaceState {
  FACE_HAPPY,
  FACE_WALK,
  FACE_WAVE,
  FACE_SLEEPY,
  FACE_CUTE
};

FaceState currentFace = FACE_HAPPY;
unsigned long lastEyeBlink = 0;
bool isBlinking = false;
int walkEyeOffset = 0;
bool oledReady = false;

// 🎨 OLED CUTE FACE GRAPHICS
void drawCuteFace(FaceState state) {
  if (!oledReady) return;
  display.clearDisplay();

  switch (state) {
    case FACE_HAPPY: {
      if (isBlinking) {
        display.drawCircle(40, 28, 12, SSD1306_WHITE);
        display.fillRect(28, 28, 25, 14, SSD1306_BLACK);
        display.drawCircle(88, 28, 12, SSD1306_WHITE);
        display.fillRect(76, 28, 25, 14, SSD1306_BLACK);
      } else {
        display.fillCircle(40, 26, 14, SSD1306_WHITE);
        display.fillCircle(88, 26, 14, SSD1306_WHITE);
        display.fillCircle(35, 21, 5, SSD1306_BLACK);
        display.fillCircle(44, 30, 3, SSD1306_BLACK);
        display.fillCircle(83, 21, 5, SSD1306_BLACK);
        display.fillCircle(92, 30, 3, SSD1306_BLACK);
      }
      for (int x = 20; x <= 30; x += 4) display.drawLine(x, 44, x + 2, 40, SSD1306_WHITE);
      for (int x = 98; x <= 108; x += 4) display.drawLine(x, 44, x + 2, 40, SSD1306_WHITE);
      display.drawCircle(64, 42, 8, SSD1306_WHITE);
      display.fillRect(54, 34, 20, 9, SSD1306_BLACK);
      break;
    }
    case FACE_WALK: {
      int eyeShiftX = (walkEyeOffset % 2 == 0) ? 4 : -4;
      display.fillCircle(40 + eyeShiftX, 26, 12, SSD1306_WHITE);
      display.fillCircle(88 + eyeShiftX, 26, 12, SSD1306_WHITE);
      display.fillCircle(37 + eyeShiftX, 23, 4, SSD1306_BLACK);
      display.fillCircle(85 + eyeShiftX, 23, 4, SSD1306_BLACK);
      display.fillCircle(64, 46, 5, SSD1306_WHITE);
      display.fillCircle(64, 46, 3, SSD1306_BLACK);
      break;
    }
    case FACE_WAVE: {
      display.drawCircle(40, 28, 14, SSD1306_WHITE);
      display.fillRect(24, 28, 32, 16, SSD1306_BLACK);
      display.fillCircle(88, 26, 14, SSD1306_WHITE);
      display.fillCircle(83, 21, 5, SSD1306_BLACK);
      display.fillCircle(92, 30, 3, SSD1306_BLACK);
      display.drawCircle(59, 44, 5, SSD1306_WHITE);
      display.drawCircle(69, 44, 5, SSD1306_WHITE);
      display.fillRect(52, 36, 24, 8, SSD1306_BLACK);
      break;
    }
    case FACE_SLEEPY: {
      display.drawCircle(40, 24, 12, SSD1306_WHITE);
      display.fillRect(26, 12, 28, 14, SSD1306_BLACK);
      display.drawCircle(88, 24, 12, SSD1306_WHITE);
      display.fillRect(74, 12, 28, 14, SSD1306_BLACK);
      display.drawLine(60, 44, 68, 44, SSD1306_WHITE);
      display.setTextSize(1);
      display.setTextColor(SSD1306_WHITE);
      display.setCursor(102, 10); display.print("Z");
      display.setCursor(110, 18); display.print("z");
      display.setCursor(116, 25); display.print("z");
      break;
    }
    case FACE_CUTE: {
      display.fillCircle(40, 26, 13, SSD1306_WHITE);
      display.fillCircle(88, 26, 13, SSD1306_WHITE);
      display.fillCircle(36, 22, 4, SSD1306_BLACK);
      display.fillCircle(84, 22, 4, SSD1306_BLACK);
      display.fillCircle(64, 42, 7, SSD1306_WHITE);
      display.fillRect(54, 35, 20, 7, SSD1306_BLACK);
      break;
    }
  }

  display.display();
}

void updateOLEDAnimation() {
  if (currentFace == FACE_HAPPY) {
    if (millis() - lastEyeBlink > 3500) {
      isBlinking = true;
      drawCuteFace(FACE_HAPPY);
      delay(150);
      isBlinking = false;
      drawCuteFace(FACE_HAPPY);
      lastEyeBlink = millis();
    }
  }
}

// SERVO KINEMATICS WITH STAGGERED ANTI-SURGE POWER PROTECTION
int calculateFinalAngle(int channel, int inputAngle) {
  int finalAngle = (baseZero[channel] == 90) ? (90 - inputAngle) : (0 + inputAngle);
  return constrain(finalAngle, 0, 180);
}

void setServoInputAngle(uint8_t channel, int inputAngle) {
  int finalAngle = calculateFinalAngle(channel, inputAngle);
  int uS = map(finalAngle, 0, 180, 732, 2929);
  pwm.writeMicroseconds(channel, uS);
  delay(15);
}

// Move servos one by one with a 35ms delay to prevent power surge
void setAllInputAngles(int inputAngle) {
  for (int i = 0; i < 8; i++) {
    setServoInputAngle(i, inputAngle);
    delay(35);
  }
}

void poseStand() { currentMode = MODE_STAND; currentFace = FACE_HAPPY; drawCuteFace(FACE_HAPPY); setAllInputAngles(45); }
void poseZero()  { currentMode = MODE_STAND; currentFace = FACE_SLEEPY; drawCuteFace(FACE_SLEEPY); setAllInputAngles(0); }

void animHiAction() {
  currentMode = MODE_STAND; currentFace = FACE_WAVE; drawCuteFace(FACE_WAVE);
  setAllInputAngles(60); delay(400);
  setServoInputAngle(5, 0);  delay(200);
  setServoInputAngle(5, 30); delay(200);
  setServoInputAngle(5, 0);  delay(200);
  setServoInputAngle(5, 30); delay(200);
  setServoInputAngle(5, 0);  delay(200);
  poseStand();
}

void stepWalkForward()  { currentFace = FACE_WALK; walkEyeOffset++; drawCuteFace(FACE_WALK); setServoInputAngle(0, 75); setServoInputAngle(3, 75); delay(stepDelay); setServoInputAngle(0, 45); setServoInputAngle(3, 45); delay(stepDelay); setServoInputAngle(1, 15); setServoInputAngle(2, 15); delay(stepDelay); setServoInputAngle(1, 45); setServoInputAngle(2, 45); delay(stepDelay); }
void stepWalkBackward() { currentFace = FACE_WALK; walkEyeOffset++; drawCuteFace(FACE_WALK); setServoInputAngle(1, 75); setServoInputAngle(2, 75); delay(stepDelay); setServoInputAngle(1, 45); setServoInputAngle(2, 45); delay(stepDelay); setServoInputAngle(0, 15); setServoInputAngle(3, 15); delay(stepDelay); setServoInputAngle(0, 45); setServoInputAngle(3, 45); delay(stepDelay); }
void stepTurnLeft()     { currentFace = FACE_WALK; walkEyeOffset++; drawCuteFace(FACE_WALK); setServoInputAngle(0, 75); setServoInputAngle(1, 75); delay(stepDelay); setServoInputAngle(0, 45); setServoInputAngle(1, 45); delay(stepDelay); setServoInputAngle(2, 15); setServoInputAngle(3, 15); delay(stepDelay); setServoInputAngle(2, 45); setServoInputAngle(3, 45); delay(stepDelay); }
void stepTurnRight()    { currentFace = FACE_WALK; walkEyeOffset++; drawCuteFace(FACE_WALK); setServoInputAngle(2, 75); setServoInputAngle(3, 75); delay(stepDelay); setServoInputAngle(2, 45); setServoInputAngle(3, 45); delay(stepDelay); setServoInputAngle(0, 15); setServoInputAngle(1, 15); delay(stepDelay); setServoInputAngle(0, 45); setServoInputAngle(1, 45); delay(stepDelay); }

// WEB DASHBOARD HTML
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
    .card { background: var(--card-bg); backdrop-filter: blur(16px); -webkit-backdrop-filter: blur(16px); border: 1px solid var(--card-border); border-radius: 20px; padding: 24px; box-shadow: 0 20px 40px rgba(0, 0, 0, 0.4); }
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
    <h1>SESAME ROBOT AP</h1>
    <p>Direct WiFi Access Point + Cute OLED Expressions</p>
    <div class="status-badge"><div class="status-dot"></div><span>AP Connected (192.168.4.1)</span></div>
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
  </script>
</body>
</html>
)rawliteral";

void handleRoot() { server.send(200, "text/html", DASHBOARD_HTML); }
void handleCommand() {
  if (server.hasArg("move")) {
    String m = server.arg("move");
    if (m == "w") currentMode = MODE_WALK_FORWARD;
    else if (m == "b") currentMode = MODE_WALK_BACKWARD;
    else if (m == "l") currentMode = MODE_TURN_LEFT;
    else if (m == "r") currentMode = MODE_TURN_RIGHT;
    else if (m == "s") poseStand();
    else if (m == "z") poseZero();
    else if (m == "hi") animHiAction();
    server.send(200, "text/plain", "OK");
  }
  else if (server.hasArg("ch") && server.hasArg("deg")) {
    currentMode = MODE_STAND;
    setServoInputAngle(server.arg("ch").toInt(), server.arg("deg").toInt());
    server.send(200, "text/plain", "OK");
  }
  else if (server.hasArg("all")) {
    currentMode = MODE_STAND;
    setAllInputAngles(server.arg("all").toInt());
    server.send(200, "text/plain", "OK");
  } else { server.send(400, "text/plain", "Bad Args"); }
}

void setup() {
  // 🛡️ Disable ESP32 Brownout Detector (Core v3 syntax)
  #ifdef RTC_CNTL_BROWN_OUT_REG
    WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);
  #endif

  Serial.begin(115200);
  delay(1000);
  Serial.println("\n==================================================");
  Serial.println(" 🤖 SESAME ROBOT - STAGGERED STARTUP FIRMWARE");
  Serial.println("==================================================");

  // 1. Initialize PCA9685 (GPIO 21 & 22)
  Wire.begin(PCA_SDA, PCA_SCL);
  pwm.begin();
  pwm.setPWMFreq(50);

  // 2. Initialize OLED I2C Bus on GPIO 18 & 19
  I2C_OLED.begin(OLED_SDA, OLED_SCL);

  if (display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println(" ✅ OLED Found at 0x3C!");
    oledReady = true;
  } else if (display.begin(SSD1306_SWITCHCAPVCC, 0x3D)) {
    Serial.println(" ✅ OLED Found at 0x3D!");
    oledReady = true;
  } else {
    Serial.println(" ❌ OLED display not detected on 18/19.");
  }

  if (oledReady) {
    display.clearDisplay();
    display.setRotation(0);
    display.dim(false);
    drawCuteFace(FACE_HAPPY);
  }

  // 3. Configure Access Point
  WiFi.mode(WIFI_AP);
  WiFi.softAP(AP_SSID, AP_PASS);
  Serial.print(" 🌐 Dashboard URL: http://"); Serial.println(WiFi.softAPIP());
  if (MDNS.begin("sesame-robot")) Serial.println(" 🌐 mDNS Hostname: http://sesame-robot.local");

  server.on("/", handleRoot);
  server.on("/cmd", handleCommand);
  server.begin();

  Serial.println(" 🧍 Standing up smoothly...");
  poseStand();
}

void loop() {
  server.handleClient();
  updateOLEDAnimation();
  switch (currentMode) {
    case MODE_WALK_FORWARD:  stepWalkForward();  break;
    case MODE_WALK_BACKWARD: stepWalkBackward(); break;
    case MODE_TURN_LEFT:     stepTurnLeft();     break;
    case MODE_TURN_RIGHT:    stepTurnRight();    break;
    case MODE_STAND:         break;
  }
}
