#include <Arduino.h>
#include "BluetoothSerial.h"

BluetoothSerial SerialBT;

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
unsigned long lastCommandTime = 0;
const unsigned long SAFETY_TIMEOUT_MS = 1500; // Stop robot if Bluetooth command drops

void stopRobot();

void setup() {
  Serial.begin(115200);
  
  if (!SerialBT.begin("ESP32_ROBOT")) {
    Serial.println("Bluetooth initialization failed!");
    while (1);
  }
  Serial.println("ESP32 Quad Bot Bluetooth Ready. Device Name: ESP32_ROBOT");

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
}

void forward() {
  lastCommandTime = millis();
  ledcWrite(L_RPWM_CHAN, speedValue);
  ledcWrite(L_LPWM_CHAN, 0);

  ledcWrite(R_RPWM_CHAN, speedValue);
  ledcWrite(R_LPWM_CHAN, 0);
}

void backward() {
  lastCommandTime = millis();
  ledcWrite(L_RPWM_CHAN, 0);
  ledcWrite(L_LPWM_CHAN, speedValue);

  ledcWrite(R_RPWM_CHAN, 0);
  ledcWrite(R_LPWM_CHAN, speedValue);
}

void left() {
  lastCommandTime = millis();
  ledcWrite(L_RPWM_CHAN, 0);
  ledcWrite(L_LPWM_CHAN, speedValue);

  ledcWrite(R_RPWM_CHAN, speedValue);
  ledcWrite(R_LPWM_CHAN, 0);
}

void right() {
  lastCommandTime = millis();
  ledcWrite(L_RPWM_CHAN, speedValue);
  ledcWrite(L_LPWM_CHAN, 0);

  ledcWrite(R_RPWM_CHAN, 0);
  ledcWrite(R_LPWM_CHAN, speedValue);
}

void stopRobot() {
  lastCommandTime = millis();
  ledcWrite(L_RPWM_CHAN, 0);
  ledcWrite(L_LPWM_CHAN, 0);

  ledcWrite(R_RPWM_CHAN, 0);
  ledcWrite(R_LPWM_CHAN, 0);
}

void loop() {
  if (SerialBT.available()) {
    char cmd = SerialBT.read();

    switch (cmd) {
      case 'F': case 'f': forward(); break;
      case 'B': case 'b': backward(); break;
      case 'L': case 'l': left(); break;
      case 'R': case 'r': right(); break;
      case 'S': case 's': case 'X': case 'x': stopRobot(); break;

      // Speed levels
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

  // Safety Watchdog
  if (millis() - lastCommandTime > SAFETY_TIMEOUT_MS) {
    stopRobot();
  }
}
