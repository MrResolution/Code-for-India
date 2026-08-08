#include <Arduino.h>
#include "BluetoothSerial.h"
#include "BTS7960MotorController.h"

BluetoothSerial SerialBT;
BTS7960MotorController motorBot;

void setup() {
  Serial.begin(115200);
  Serial.println("\n==========================================");
  Serial.println("  ESP32 BTS7960 Bluetooth Motor Driver");
  Serial.println("==========================================");

  if (!SerialBT.begin("ESP32_ROBOT")) {
    Serial.println("[ERROR] Bluetooth initialization failed!");
    while (1);
  }

  Serial.println("[OK] Bluetooth device 'ESP32_ROBOT' active.");
  motorBot.begin();
  motorBot.setSafetyTimeout(1500);
}

void loop() {
  if (SerialBT.available()) {
    char cmd = SerialBT.read();
    Serial.print("[BT CMD] ");
    Serial.println(cmd);
    motorBot.processCommand(cmd);
  }

  motorBot.update();
}
