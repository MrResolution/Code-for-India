#include <Arduino.h>
#include "BluetoothSerial.h"
#include "robot_protocol.h"

BluetoothSerial SerialBT;
RobotProtocol robotComm("BTS_Rover");

// Left BTS7960 Motor Driver Pins
#define L_RPWM 25
#define L_LPWM 26
#define L_EN   32  // Connect to R_EN + L_EN on Left BTS7960

// Right BTS7960 Motor Driver Pins
#define R_RPWM 27
#define R_LPWM 14
#define R_EN   33  // Connect to R_EN + L_EN on Right BTS7960

// PWM Configurations for ESP32 LEDC (20 kHz to eliminate motor noise)
#define PWM_FREQ 20000
#define PWM_RES  8

#define L_RPWM_CHAN 0
#define L_LPWM_CHAN 1
#define R_RPWM_CHAN 2
#define R_LPWM_CHAN 3

int speedValue = 200; // Default speed (0 - 255)
String currentDriveState = "STOP";
unsigned long lastCommandTime = 0;
const unsigned long SAFETY_TIMEOUT_MS = 1500; // Stop robot if command stream drops

void stopRobot();

void setLeftMotor(int speed) {
  speed = constrain(speed, -255, 255);
  if (speed >= 0) {
    ledcWrite(L_RPWM_CHAN, speed);
    ledcWrite(L_LPWM_CHAN, 0);
  } else {
    ledcWrite(L_RPWM_CHAN, 0);
    ledcWrite(L_LPWM_CHAN, -speed);
  }
}

void setRightMotor(int speed) {
  speed = constrain(speed, -255, 255);
  if (speed >= 0) {
    ledcWrite(R_RPWM_CHAN, speed);
    ledcWrite(R_LPWM_CHAN, 0);
  } else {
    ledcWrite(R_RPWM_CHAN, 0);
    ledcWrite(R_LPWM_CHAN, -speed);
  }
}

void forward() {
  lastCommandTime = millis();
  currentDriveState = "FORWARD";
  setLeftMotor(speedValue);
  setRightMotor(speedValue);
}

void backward() {
  lastCommandTime = millis();
  currentDriveState = "BACKWARD";
  setLeftMotor(-speedValue);
  setRightMotor(-speedValue);
}

void left() {
  lastCommandTime = millis();
  currentDriveState = "LEFT";
  setLeftMotor(-speedValue);
  setRightMotor(speedValue);
}

void right() {
  lastCommandTime = millis();
  currentDriveState = "RIGHT";
  setLeftMotor(speedValue);
  setRightMotor(-speedValue);
}

void stopRobot() {
  lastCommandTime = millis();
  currentDriveState = "STOP";
  setLeftMotor(0);
  setRightMotor(0);
}

String getRoverStatusJson() {
  String json = "{";
  json += "\"name\":\"BTS Rover\",";
  json += "\"online\":true,";
  json += "\"drive_state\":\"" + currentDriveState + "\",";
  json += "\"speed\":" + String(speedValue) + ",";
  json += "\"ip\":\"" + robotComm.getIP().toString() + "\",";
  json += "\"bt_name\":\"ESP32_ROBOT\",";
  json += "\"uptime_ms\":" + String(millis());
  json += "}";
  return json;
}

void processRoverCommand(const String& prefix, const String& target, const String& payload, String& reply) {
  lastCommandTime = millis();

  if (prefix == "CMD") {
    String tgt = target; tgt.toLowerCase();
    String val = payload; val.toLowerCase();

    if (tgt == "drive" || tgt == "move") {
      if (val == "forward" || val == "f") {
        forward();
        reply = "ACK:drive=forward";
      } else if (val == "backward" || val == "b") {
        backward();
        reply = "ACK:drive=backward";
      } else if (val == "left" || val == "l") {
        left();
        reply = "ACK:drive=left";
      } else if (val == "right" || val == "r") {
        right();
        reply = "ACK:drive=right";
      } else if (val == "stop" || val == "s") {
        stopRobot();
        reply = "ACK:drive=stop";
      } else {
        reply = "ERR:UNKNOWN_DRIVE_VAL:" + payload;
      }
    } else if (tgt == "stop") {
      stopRobot();
      reply = "ACK:stop";
    } else if (tgt == "speed") {
      int s = payload.toInt();
      speedValue = constrain(s, 0, 255);
      reply = "ACK:speed=" + String(speedValue);
    } else if (tgt == "left_motor") {
      int s = payload.toInt();
      setLeftMotor(s);
      reply = "ACK:left_motor=" + String(s);
    } else if (tgt == "right_motor") {
      int s = payload.toInt();
      setRightMotor(s);
      reply = "ACK:right_motor=" + String(s);
    } else if (tgt == "motors") {
      // CMD:motors=200,200 (left,right)
      int comma = payload.indexOf(',');
      if (comma > 0) {
        int l = payload.substring(0, comma).toInt();
        int r = payload.substring(comma + 1).toInt();
        setLeftMotor(l);
        setRightMotor(r);
        reply = "ACK:motors=" + String(l) + "," + String(r);
      } else {
        reply = "ERR:INVALID_MOTORS_PAYLOAD";
      }
    } else {
      reply = "ERR:UNKNOWN_TARGET:" + target;
    }
  } else {
    reply = "ERR:UNKNOWN_PREFIX:" + prefix;
  }
}

void setup() {
  Serial.begin(115200);

  // Initialize Bluetooth
  if (!SerialBT.begin("ESP32_ROBOT")) {
    Serial.println("⚠️ Bluetooth initialization failed!");
  } else {
    Serial.println("📶 ESP32 Bluetooth Ready: Device Name 'ESP32_ROBOT'");
  }

  // Configure ESP32 LEDC PWM Channels
  ledcSetup(L_RPWM_CHAN, PWM_FREQ, PWM_RES);
  ledcAttachPin(L_RPWM, L_RPWM_CHAN);

  ledcSetup(L_LPWM_CHAN, PWM_FREQ, PWM_RES);
  ledcAttachPin(L_LPWM, L_LPWM_CHAN);

  ledcSetup(R_RPWM_CHAN, PWM_FREQ, PWM_RES);
  ledcAttachPin(R_RPWM, R_RPWM_CHAN);

  ledcSetup(R_LPWM_CHAN, PWM_FREQ, PWM_RES);
  ledcAttachPin(R_LPWM, R_LPWM_CHAN);

  pinMode(L_EN, OUTPUT);
  pinMode(R_EN, OUTPUT);
  digitalWrite(L_EN, HIGH);
  digitalWrite(R_EN, HIGH);

  stopRobot();

  // Initialize Unified Robot Communication Protocol (WiFi UDP + Serial)
  RobotWiFiConfig wifiCfg(
    "Sabo", "sandy0606",
    "123", "12345678",
    "BTS_Rover_AP", "rover1234",
    8888, "bts-rover"
  );
  robotComm.setConfig(wifiCfg);
  robotComm.setCommandHandler(processRoverCommand);
  robotComm.setStatusHandler(getRoverStatusJson);
  robotComm.begin();
}

void loop() {
  // 1. Process Unified Protocol (Wi-Fi UDP & Serial)
  robotComm.update();

  // 2. Process Bluetooth Serial (Supports single chars & line commands)
  while (SerialBT.available()) {
    char c = SerialBT.peek();
    if (c == 'C' || c == 'P' || c == 'S' && SerialBT.available() > 3) {
      // Looks like a protocol string
      String line = SerialBT.readStringUntil('\n');
      String reply;
      robotComm.processCommandLine(line, reply);
      if (reply.length() > 0) {
        SerialBT.println(reply);
      }
    } else {
      // Traditional single-character command
      char cmd = SerialBT.read();
      lastCommandTime = millis();
      switch (cmd) {
        case 'F': case 'f': forward(); break;
        case 'B': case 'b': backward(); break;
        case 'L': case 'l': left(); break;
        case 'R': case 'r': right(); break;
        case 'S': case 's': case 'X': case 'x': stopRobot(); break;

        // Speed levels 0-9
        case '0': speedValue = 0; break;
        case '1': speedValue = 30; break;
        case '2': speedValue = 60; break;
        case '3': speedValue = 90; break;
        case '4': speedValue = 120; break;
        case '5': speedValue = 150; break;
        case '6': speedValue = 180; break;
        case '7': speedValue = 200; break;
        case '8': speedValue = 230; break;
        case '9': speedValue = 255; break;
        case 'q': speedValue = 255; break;
      }
    }
  }

  // 3. Safety Watchdog across all channels
  if (millis() - lastCommandTime > SAFETY_TIMEOUT_MS) {
    if (currentDriveState != "STOP") {
      stopRobot();
    }
  }
}
