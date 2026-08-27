#include <Arduino.h>

#if defined(ESP8266)
  #include <ESP8266WiFi.h>
  #include <WiFiUdp.h>
  #include <Servo.h>
#else
  #include <Servo.h>
#endif

// ── Wi-Fi Configuration ─────────────────────────────────────────────────
const char* WIFI_SSID = "123";     // User Wi-Fi SSID
const char* WIFI_PASS = "12345678"; // User Wi-Fi Password

// Fallback SoftAP settings if home Wi-Fi is unavailable:
const char* AP_SSID   = "RobotArm_AP";
const char* AP_PASS   = "robotarm123";

const uint16_t UDP_PORT = 8888;
WiFiUDP udp;

// Structure to define each joint, its pin, current angle, and safety calibration bounds
struct ServoJoint {
  const char* jointName;
  int pin;
  Servo servoObj;
  float currentAngleDeg;
  float targetAngleDeg;  // Desired position (interpolated toward by servo task)
  float minDeg;     // Minimum physical safety limit (e.g. 0.0°)
  float maxDeg;     // Maximum physical safety limit (e.g. 180.0°)
  float trimOffset; // Servo horn trim offset (-20.0° to +20.0°)
  bool isMirroredSlave; // If true, automatically mirrors master joint angle (180.0 - deg)
};

// Define all 6 physical robotic arm servos mapped to ESP8266 GPIO pins:
// Note: ESP8266 has limited pins. Using D1-D6 (GPIO 5, 4, 0, 2, 14, 12)
ServoJoint joints[] = {
  { "turntable_link_joint_dup",   5,  Servo(), 90.0, 90.0, 0.0, 180.0, 0.0, false }, // D1
  { "turntable_link_joint",       4,  Servo(), 90.0, 90.0, 0.0, 180.0, 0.0, false }, // D2, Primary Shoulder
  { "turntable_link_joint_slave", 0,  Servo(), 90.0, 90.0, 0.0, 180.0, 0.0, true  }, // D3, Opposing Mirrored Shoulder
  { "turntable_link_joint_dup_1", 2,  Servo(), 90.0, 90.0, 0.0, 180.0, 0.0, false }, // D4
  { "turntable_link_joint_dup_2", 14, Servo(), 90.0, 90.0, 0.0, 180.0, 0.0, false }, // D5
  { "wrist_twist_joint",          12, Servo(), 90.0, 90.0, 0.0, 180.0, 0.0, false }  // D6
};

const int NUM_JOINTS = sizeof(joints) / sizeof(joints[0]);

unsigned long lastHeartbeatMs = 0;
const unsigned long HEARTBEAT_INTERVAL_MS = 2000;
unsigned long lastCommandMs = 0;

// ── Communication Mode Toggle ───────────────────────────────────────────
enum CommMode { COMM_WIFI, COMM_SERIAL };
CommMode commMode = COMM_WIFI;  // Default: accept commands over WiFi UDP

// ── Smooth Servo Interpolation Configuration ────────────────────────────
const float EASE_FACTOR       = 0.12;  // Exponential smoothing (0.0–1.0); higher = faster response
const float EASE_THRESHOLD    = 0.1;   // Snap-to-target threshold in degrees (avoids micro-jitter)
const int   SERVO_UPDATE_MS   = 20;    // Interpolation interval (50 Hz update rate)
const int   SERVO_MIN_US      = 500;   // Minimum pulse width (microseconds)
const int   SERVO_MAX_US      = 2500;  // Maximum pulse width (microseconds)
unsigned long lastServoUpdateMs = 0;

/**
 * Convert angle in degrees (0–180) to pulse width in microseconds (500–2500).
 */
int degToMicroseconds(float deg) {
  return SERVO_MIN_US + (int)((deg / 180.0f) * (float)(SERVO_MAX_US - SERVO_MIN_US));
}

/**
 * Updates servo positions to smoothly interpolate toward their targets.
 * Called continuously in the main loop for ESP8266 (no RTOS tasks).
 */
void updateServos() {
  unsigned long now = millis();
  if (now - lastServoUpdateMs >= SERVO_UPDATE_MS) {
    for (int i = 0; i < NUM_JOINTS; i++) {
      float diff = joints[i].targetAngleDeg - joints[i].currentAngleDeg;

      if (fabs(diff) < EASE_THRESHOLD) {
        joints[i].currentAngleDeg = joints[i].targetAngleDeg;
      } else {
        joints[i].currentAngleDeg += diff * EASE_FACTOR;
      }

      joints[i].servoObj.writeMicroseconds(degToMicroseconds(joints[i].currentAngleDeg));
    }
    lastServoUpdateMs = now;
  }
}

/**
 * Parse a command line:
 *   1. Motion command:  "CMD:<joint_name>=<degrees>"
 *   2. Calib command:   "CALIB:<joint_name>=<min_deg>,<max_deg>"
 */
bool parseCmd(const String& line, String& prefix, String& jointName, String& payload) {
  int colonIdx = line.indexOf(':');
  if (colonIdx < 1) return false;

  prefix = line.substring(0, colonIdx);
  prefix.trim();

  String rest = line.substring(colonIdx + 1);
  int eqIdx = rest.indexOf('=');
  if (eqIdx < 1) return false;

  jointName = rest.substring(0, eqIdx);
  jointName.trim();

  payload = rest.substring(eqIdx + 1);
  payload.trim();

  return true;
}

void processCommandString(const String& line, String& replyString) {
  if (line.equalsIgnoreCase("PING")) {
    replyString = "ACK:PONG";
    return;
  }

  String prefix, jointName, payload;
  if (!parseCmd(line, prefix, jointName, payload)) {
    replyString = "ERR:UNKNOWN_CMD:" + line;
    return;
  }

  bool found = false;
  for (int i = 0; i < NUM_JOINTS; i++) {
    if (jointName.equals(joints[i].jointName)) {
      found = true;

      if (prefix.equals("CMD")) {
        float requestedDeg = payload.toFloat();
        float trimmedDeg = requestedDeg + joints[i].trimOffset;
        float clampedDeg = constrain(trimmedDeg, joints[i].minDeg, joints[i].maxDeg);

        joints[i].targetAngleDeg = clampedDeg;

        // If commanding primary Shoulder Pitch (turntable_link_joint), drive opposing slave servo in mirrored sync
        if (jointName.equals("turntable_link_joint")) {
          for (int j = 0; j < NUM_JOINTS; j++) {
            if (joints[j].isMirroredSlave) {
              float mirroredDeg = 180.0 - clampedDeg;
              float clampedSlave = constrain(mirroredDeg, joints[j].minDeg, joints[j].maxDeg);
              joints[j].targetAngleDeg = clampedSlave;
            }
          }
        }

        replyString = "ACK:" + jointName + "=" + String(clampedDeg, 1);

      } else if (prefix.equals("CALIB")) {
        int commaIdx = payload.indexOf(',');
        if (commaIdx > 0) {
          float newMin = payload.substring(0, commaIdx).toFloat();
          float newMax = payload.substring(commaIdx + 1).toFloat();

          joints[i].minDeg = constrain(newMin, 0.0, 180.0);
          joints[i].maxDeg = constrain(newMax, joints[i].minDeg, 180.0);

          replyString = "ACK:CALIB:" + jointName + "=[" + String(joints[i].minDeg, 1) + "," + String(joints[i].maxDeg, 1) + "]";
        }
      }
      break;
    }
  }

  if (!found) {
    replyString = "ERR:UNKNOWN_JOINT:" + jointName;
  }
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println("\n==================================================");
  Serial.println("  ESP8266 6-Servo Dual Mirrored Arm Controller     ");
  Serial.println("==================================================");

  // Initialize all 6 physical servos
  for (int i = 0; i < NUM_JOINTS; i++) {
    joints[i].servoObj.attach(joints[i].pin, 500, 2500);
    float safeAngle = constrain(joints[i].currentAngleDeg + joints[i].trimOffset, joints[i].minDeg, joints[i].maxDeg);
    joints[i].servoObj.writeMicroseconds(degToMicroseconds(safeAngle));
    joints[i].currentAngleDeg = safeAngle;
    joints[i].targetAngleDeg = safeAngle;
  }
  
  Serial.println("🎯 Servos initialized (Using loop update on ESP8266)");

  // ── Wi-Fi Connection Setup ───────────────────────────────────────────
  WiFi.mode(WIFI_STA);
  Serial.print("Connecting to Wi-Fi SSID '");
  Serial.print(WIFI_SSID);
  Serial.print("'...");

  WiFi.begin(WIFI_SSID, WIFI_PASS);

  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 20) {
    delay(500);
    Serial.print(".");
    attempts++;
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\n✅ Connected to Wi-Fi Network!");
    Serial.print("📡 ESP8266 Wireless IP Address: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("\n⚠️ Wi-Fi Network not found. Starting Access Point fallback...");
    WiFi.mode(WIFI_AP);
    WiFi.softAP(AP_SSID, AP_PASS);
    Serial.print("📡 ESP8266 AP SSID: ");
    Serial.println(AP_SSID);
    Serial.print("📡 ESP8266 AP IP Address: ");
    Serial.println(WiFi.softAPIP());
  }

  // Start UDP listener on port 8888
  udp.begin(UDP_PORT);
  Serial.print("🚀 Listening for Wireless UDP Packets on port ");
  Serial.println(UDP_PORT);
  Serial.println("Ready for wireless control commands...\n");

  lastHeartbeatMs = millis();
  lastCommandMs = millis();
}

/**
 * Handle MODE commands (always processed regardless of active channel).
 */
bool handleModeCommand(const String& line, String& replyString) {
  if (line.equalsIgnoreCase("MODE:SERIAL")) {
    commMode = COMM_SERIAL;
    replyString = "ACK:MODE=SERIAL";
    Serial.println("📡 Switched to SERIAL control mode");
    return true;
  } else if (line.equalsIgnoreCase("MODE:WIFI")) {
    commMode = COMM_WIFI;
    replyString = "ACK:MODE=WIFI";
    Serial.println("📡 Switched to WIFI control mode");
    return true;
  } else if (line.equalsIgnoreCase("MODE:TOGGLE")) {
    commMode = (commMode == COMM_WIFI) ? COMM_SERIAL : COMM_WIFI;
    replyString = "ACK:MODE=" + String(commMode == COMM_WIFI ? "WIFI" : "SERIAL");
    Serial.println("📡 Toggled to " + String(commMode == COMM_WIFI ? "WIFI" : "SERIAL") + " control mode");
    return true;
  } else if (line.equalsIgnoreCase("MODE")) {
    replyString = "ACK:MODE=" + String(commMode == COMM_WIFI ? "WIFI" : "SERIAL");
    return true;
  }
  return false;
}

void loop() {
  // Always update servos first to keep animation smooth
  updateServos();

  // ── 1. Read Wi-Fi UDP Packets ─────────────────────────────────────────
  int packetSize = udp.parsePacket();
  if (packetSize > 0) {
    char packetBuffer[255];
    int len = udp.read(packetBuffer, 254);
    if (len > 0) packetBuffer[len] = 0;

    String line = String(packetBuffer);
    line.trim();

    if (line.length() > 0) {
      String replyString;

      if (!handleModeCommand(line, replyString)) {
        if (commMode == COMM_WIFI) {
          processCommandString(line, replyString);
        } else {
          replyString = "ERR:SERIAL_MODE_ACTIVE";
        }
      }

      if (replyString.length() > 0) {
        udp.beginPacket(udp.remoteIP(), udp.remotePort());
        udp.print(replyString);
        udp.endPacket();
      }
      lastCommandMs = millis();
    }
  }

  // ── 2. Read Serial Commands (USB Backup) ──────────────────────────────
  while (Serial.available() > 0) {
    String line = Serial.readStringUntil('\n');
    line.trim();

    if (line.length() > 0) {
      String replyString;

      if (!handleModeCommand(line, replyString)) {
        if (commMode == COMM_SERIAL) {
          processCommandString(line, replyString);
        } else {
          replyString = "ERR:WIFI_MODE_ACTIVE";
        }
      }

      if (replyString.length() > 0) {
        Serial.println(replyString);
      }
      lastCommandMs = millis();
    }
  }

  // ── 3. Periodic Heartbeat ─────────────────────────────────────────────
  unsigned long now = millis();
  if (now - lastHeartbeatMs >= HEARTBEAT_INTERVAL_MS) {
    if (now - lastCommandMs > HEARTBEAT_INTERVAL_MS) {
      Serial.print("HEARTBEAT:JOINTS=");
      Serial.print(NUM_JOINTS);
      Serial.print(" | MODE=");
      Serial.print(commMode == COMM_WIFI ? "WIFI" : "SERIAL");
      Serial.print(" | IP=");
      if (WiFi.status() == WL_CONNECTED) {
        Serial.println(WiFi.localIP());
      } else {
        Serial.println(WiFi.softAPIP());
      }
    }
    lastHeartbeatMs = now;
  }
}
