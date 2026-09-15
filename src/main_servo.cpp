#include <Arduino.h>

#if defined(ESP32)
  #include <WiFi.h>
  #include <WiFiUdp.h>
  #include <ESP32Servo.h>
  #include "soc/soc.h"
  #include "soc/rtc_cntl_reg.h"
#else
  #include <Servo.h>
#endif

// 🌐 Unified Robot Communication Protocol
#include "robot_protocol.h"

// Instantiate Unified Protocol Handler for Robot Arm
RobotProtocol robotComm("Robot_Arm");

// ── Structure to define each joint, its pin, angles, and safety bounds ──────
struct ServoJoint {
  const char* jointName;
  int pin;
  Servo servoObj;
  float currentAngleDeg;
  float targetAngleDeg;  // Desired position (interpolated toward by servo task)
  float minDeg;          // Minimum physical safety limit (e.g. 0.0°)
  float maxDeg;          // Maximum physical safety limit (e.g. 180.0°)
  float trimOffset;      // Servo horn trim offset (-20.0° to +20.0°)
  bool isMirroredSlave;  // If true, automatically mirrors master joint angle (180.0 - deg)
};

// ── 7 Physical Robotic Arm Servos mapped to ESP32 GPIO pins ─────────────────
// 1. Servo 1: GPIO 18 (Base Turntable - turntable_link_joint_dup)
// 2. Servo 2: GPIO 19 (Shoulder Pitch Primary - turntable_link_joint)
// 3. Servo 3: GPIO 21 (Shoulder Pitch Opposing Mirrored Slave - turntable_link_joint_slave)
// 4. Servo 4: GPIO 22 (Elbow 1 Pitch - turntable_link_joint_dup_1)
// 5. Servo 5: GPIO 23 (Elbow 2 Pitch - turntable_link_joint_dup_2)
// 6. Servo 6: GPIO 27 (Wrist Twist - wrist_twist_joint)
// 7. Servo 7: GPIO 26 (Gripper - grip)
ServoJoint joints[] = {
  { "turntable_link_joint_dup",   18, Servo(), 90.0, 90.0, 0.0, 180.0, 0.0, false },
  { "turntable_link_joint",       19, Servo(), 90.0, 90.0, 0.0, 180.0, 0.0, false }, // Primary Shoulder
  { "turntable_link_joint_slave", 21, Servo(), 90.0, 90.0, 0.0, 180.0, 0.0, true  }, // Opposing Mirrored Shoulder
  { "turntable_link_joint_dup_1", 22, Servo(), 90.0, 90.0, 0.0, 180.0, 0.0, false },
  { "turntable_link_joint_dup_2", 23, Servo(), 90.0, 90.0, 0.0, 180.0, 0.0, false },
  { "wrist_twist_joint",          27, Servo(), 90.0, 90.0, 0.0, 180.0, 0.0, false }, // Wrist Twist Servo
  { "grip",                       26, Servo(), 90.0, 90.0, 0.0, 180.0, 0.0, false }  // Gripper Actuator
};

const int NUM_JOINTS = sizeof(joints) / sizeof(joints[0]);

// FreeRTOS Mutex for thread-safe target angle updates between Core 0 and Core 1
SemaphoreHandle_t jointMutex = NULL;

// ── Smooth Servo Interpolation Configuration ────────────────────────────────
const float EASE_FACTOR       = 0.12;  // Exponential smoothing (0.0–1.0)
const float EASE_THRESHOLD    = 0.1;   // Snap-to-target threshold in degrees
const int   SERVO_UPDATE_MS   = 20;    // Interpolation interval (50 Hz rate)
const int   SERVO_MIN_US      = 500;   // Minimum pulse width (microseconds)
const int   SERVO_MAX_US      = 2500;  // Maximum pulse width (microseconds)

int degToMicroseconds(float deg) {
  return SERVO_MIN_US + (int)((deg / 180.0f) * (float)(SERVO_MAX_US - SERVO_MIN_US));
}

/**
 * FreeRTOS task: Smoothly interpolates all servo positions toward their targets
 * at a fixed 50 Hz rate using exponential easing. Runs isolated on Core 1.
 */
void servoInterpolationTask(void* pvParameters) {
  (void)pvParameters;

  for (;;) {
    if (xSemaphoreTake(jointMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
      for (int i = 0; i < NUM_JOINTS; i++) {
        float diff = joints[i].targetAngleDeg - joints[i].currentAngleDeg;

        if (fabs(diff) < EASE_THRESHOLD) {
          joints[i].currentAngleDeg = joints[i].targetAngleDeg;
        } else {
          joints[i].currentAngleDeg += diff * EASE_FACTOR;
        }

        joints[i].servoObj.writeMicroseconds(degToMicroseconds(joints[i].currentAngleDeg));
      }
      xSemaphoreGive(jointMutex);
    }

    vTaskDelay(pdMS_TO_TICKS(SERVO_UPDATE_MS));
  }
}

/**
 * Return JSON status of the robot arm
 */
String getArmStatusJson() {
  String json = "{";
  json += "\"name\":\"Robot Arm\",";
  json += "\"online\":true,";
  json += "\"mode\":\"" + String(robotComm.getMode() == ROBOT_COMM_WIFI ? "WIFI" : "SERIAL") + "\",";
  json += "\"ip\":\"" + robotComm.getIP().toString() + "\",";
  json += "\"joints\":{";

  if (jointMutex != NULL && xSemaphoreTake(jointMutex, pdMS_TO_TICKS(20)) == pdTRUE) {
    for (int i = 0; i < NUM_JOINTS; i++) {
      json += "\"" + String(joints[i].jointName) + "\":{\"cur\":" + String(joints[i].currentAngleDeg, 1) + ",\"target\":" + String(joints[i].targetAngleDeg, 1) + "}";
      if (i < NUM_JOINTS - 1) json += ",";
    }
    xSemaphoreGive(jointMutex);
  } else {
    for (int i = 0; i < NUM_JOINTS; i++) {
      json += "\"" + String(joints[i].jointName) + "\":{\"cur\":" + String(joints[i].currentAngleDeg, 1) + "}";
      if (i < NUM_JOINTS - 1) json += ",";
    }
  }

  json += "},\"uptime_ms\":" + String(millis()) + "}";
  return json;
}

/**
 * Unified Command Handler for Robotic Arm
 */
void processArmCommand(const String& prefix, const String& target, const String& payload, String& reply) {
  if (prefix == "CMD") {
    bool found = false;

    if (jointMutex != NULL && xSemaphoreTake(jointMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
      for (int i = 0; i < NUM_JOINTS; i++) {
        bool isMatch = target.equalsIgnoreCase(joints[i].jointName);
        if (!isMatch && (target.equalsIgnoreCase("gripper") || target.equalsIgnoreCase("gripper_joint"))) {
          isMatch = String(joints[i].jointName).equals("grip");
        }

        if (isMatch) {
          found = true;
          float requestedDeg = 90.0;
          if (payload.equalsIgnoreCase("open")) {
            requestedDeg = 180.0;
          } else if (payload.equalsIgnoreCase("close")) {
            requestedDeg = 0.0;
          } else if (payload.equalsIgnoreCase("half")) {
            requestedDeg = 90.0;
          } else {
            requestedDeg = payload.toFloat();
          }

          float trimmedDeg = requestedDeg + joints[i].trimOffset;
          float clampedDeg = constrain(trimmedDeg, joints[i].minDeg, joints[i].maxDeg);

          joints[i].targetAngleDeg = clampedDeg;

          // If commanding primary Shoulder Pitch, drive opposing mirrored slave
          if (target.equalsIgnoreCase("turntable_link_joint")) {
            for (int j = 0; j < NUM_JOINTS; j++) {
              if (joints[j].isMirroredSlave) {
                float mirroredDeg = 180.0 - clampedDeg;
                joints[j].targetAngleDeg = constrain(mirroredDeg, joints[j].minDeg, joints[j].maxDeg);
              }
            }
          }

          reply = "ACK:" + target + "=" + String(clampedDeg, 1);
          break;
        }
      }
      xSemaphoreGive(jointMutex);
    }

    if (!found) {
      reply = "ERR:UNKNOWN_JOINT:" + target;
    }

  } else if (prefix == "CALIB") {
    int commaIdx = payload.indexOf(',');
    if (commaIdx > 0) {
      float newMin = payload.substring(0, commaIdx).toFloat();
      float newMax = payload.substring(commaIdx + 1).toFloat();

      if (jointMutex != NULL && xSemaphoreTake(jointMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        for (int i = 0; i < NUM_JOINTS; i++) {
          if (target.equalsIgnoreCase(joints[i].jointName)) {
            joints[i].minDeg = constrain(newMin, 0.0, 180.0);
            joints[i].maxDeg = constrain(newMax, joints[i].minDeg, 180.0);
            reply = "ACK:CALIB:" + target + "=[" + String(joints[i].minDeg, 1) + "," + String(joints[i].maxDeg, 1) + "]";
            break;
          }
        }
        xSemaphoreGive(jointMutex);
      }
    } else {
      reply = "ERR:CALIB_FORMAT:" + payload;
    }
  } else {
    reply = "ERR:UNKNOWN_PREFIX:" + prefix;
  }
}

void setup() {
  #ifdef RTC_CNTL_BROWN_OUT_REG
    WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);
  #endif

  Serial.begin(115200);
  delay(500);

  Serial.println("\n==================================================");
  Serial.println("  🤖 ESP32 6-Servo Robotic Arm (Unified Comm)");
  Serial.println("==================================================");

  // Create Mutex for thread-safe cross-core servo data access
  jointMutex = xSemaphoreCreateMutex();

  // Initialize all 6 physical servos
  for (int i = 0; i < NUM_JOINTS; i++) {
#if defined(ESP32)
    joints[i].servoObj.setPeriodHertz(50);
    joints[i].servoObj.attach(joints[i].pin, 500, 2500);
#else
    joints[i].servoObj.attach(joints[i].pin, 500, 2500);
#endif
    float safeAngle = constrain(joints[i].currentAngleDeg + joints[i].trimOffset, joints[i].minDeg, joints[i].maxDeg);
    joints[i].servoObj.writeMicroseconds(degToMicroseconds(safeAngle));
    joints[i].currentAngleDeg = safeAngle;
    joints[i].targetAngleDeg = safeAngle;
  }

  // Launch Servo Interpolation Task on Core 1 (leaving Core 0 dedicated to Wi-Fi)
  xTaskCreatePinnedToCore(
    servoInterpolationTask,
    "ServoEase",
    4096,
    NULL,
    2,
    NULL,
    1 // Core 1
  );
  Serial.println("🎯 Servo interpolation task launched on Core 1 (50Hz)");

  // Initialize Unified Communication Protocol with Dual-Network & SoftAP
  RobotWiFiConfig wifiCfg(
    "123", "12345678",       // Primary Wi-Fi (Active Router)
    "Sabo", "sandy0606",     // Secondary Wi-Fi (Hotspot)
    "RobotArm_AP", "robotarm123", // SoftAP Fallback
    8888,                    // UDP Port
    "robot-arm"              // mDNS Hostname (http://robot-arm.local)
  );

  robotComm.setConfig(wifiCfg);
  robotComm.setCommandHandler(processArmCommand);
  robotComm.setStatusHandler(getArmStatusJson);
  robotComm.begin();

  // ⚡ Low-Latency Wi-Fi Optimization: Disable power-save modem sleep
  WiFi.setSleep(false);
  Serial.println("⚡ Wi-Fi modem sleep disabled for sub-2ms latency.");
}

void loop() {
  // Main communication loop (Runs on Core 0)
  robotComm.update();
  delay(1);
}
