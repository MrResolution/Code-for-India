#include <Arduino.h>
#include <Wire.h>
#include <VL53L1X.h>

#define SDA_PIN 21
#define SCL_PIN 22
#define LED_PIN 2

VL53L1X sensor;

void setup() {
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  Serial.begin(115200);
  while (!Serial && millis() < 3000); // Wait for Serial Monitor

  Serial.println("\n=================================");
  Serial.println("  NodeMCU ESP-32S + VL53L1X ToF  ");
  Serial.println("=================================");

  // Initialize I2C bus on GPIO21 (SDA) and GPIO22 (SCL)
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000); // Fast I2C (400kHz)

  sensor.setTimeout(500);
  if (!sensor.init()) {
    Serial.println("❌ ERROR: Failed to detect and initialize VL53L1X sensor!");
    Serial.println("Please check your wiring:");
    Serial.println("  - VCC -> 3V3");
    Serial.println("  - GND -> GND");
    Serial.println("  - SDA -> GPIO 21 (P21)");
    Serial.println("  - SCL -> GPIO 22 (P22)");
    while (1) {
      digitalWrite(LED_PIN, HIGH);
      delay(200);
      digitalWrite(LED_PIN, LOW);
      delay(200);
    }
  }

  // Use Long distance mode and set measurement timing budget
  sensor.setDistanceMode(VL53L1X::Long);
  sensor.setMeasurementTimingBudget(50000); // 50 ms budget

  // Start continuous ranging with a period of 50 ms
  sensor.startContinuous(50);

  Serial.println("✅ VL53L1X Initialized successfully! Ranging started...\n");
}

void loop() {
  sensor.read();

  if (sensor.timeoutOccurred()) {
    Serial.println("⚠️ Warning: Sensor read timeout!");
  } else {
    uint16_t distance_mm = sensor.read();
    float distance_cm = distance_mm / 10.0;

    Serial.print("Distance: ");
    Serial.print(distance_mm);
    Serial.print(" mm  |  ");
    Serial.print(distance_cm, 1);
    Serial.println(" cm");

    // Toggle onboard LED on each reading
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
  }

  delay(100);
}
