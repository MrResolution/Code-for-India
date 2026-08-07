#include <Arduino.h>

#if defined(ESP32)
  #include <ESP32Servo.h>
  #define SERVO_PIN 18  // GPIO 18 on ESP32
#else
  #include <Servo.h>
  #define SERVO_PIN 9   // Pin 9 on Arduino Uno / Nano
#endif

Servo myServo;

int currentAngle = 0;
int sweepDirection = 1; // 1 = increasing angle, -1 = decreasing angle

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000); // Wait for Serial connection

  Serial.println("\n==================================================");
  Serial.println("  Microcontroller Servo ROS 2 Joint State Driver  ");
  Serial.println("==================================================");

  // Attach servo to control pin
  myServo.attach(SERVO_PIN);
  myServo.write(currentAngle);
  
  Serial.println("✅ Servo attached successfully. Streaming angle data over Serial...\n");
}

void loop() {
  // Update servo angle (simulated sweep or sensor-driven position)
  myServo.write(currentAngle);

  // Output format understood by ROS 2 Python Node: e.g. "ANGLE: 45.0"
  Serial.print("ANGLE: ");
  Serial.println(currentAngle);

  // Sweep from 0 to 180 degrees and back
  currentAngle += (sweepDirection * 2);
  if (currentAngle >= 180) {
    currentAngle = 180;
    sweepDirection = -1;
  } else if (currentAngle <= 0) {
    currentAngle = 0;
    sweepDirection = 1;
  }

  // 20 ms delay -> 50 Hz streaming rate
  delay(20);
}
