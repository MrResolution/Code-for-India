#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <DHT.h>
#include "robot_protocol.h"

// ==================== PIN DEFINITIONS ====================
#define DHT_PIN     4
#define DHT_TYPE    DHT11
#define MQ5_PIN     34
#define SDA_PIN     21
#define SCL_PIN     22
#define LED_PIN     2    // Onboard LED

// ==================== GLOBALS ====================
DHT dht(DHT_PIN, DHT_TYPE);
Adafruit_MPU6050 mpu;
RobotProtocol robotComm("Sensor_Rover");

unsigned long lastReadTime = 0;
unsigned long sensorReadIntervalMs = 2000;
uint32_t readingCount = 0;
bool streamTelemetry = false;
bool mpuReady = false;

// Cached latest sensor readings
float curTemp = 0.0;
float curHumidity = 0.0;
float curAccelX = 0.0, curAccelY = 0.0, curAccelZ = 0.0;
float curGyroX = 0.0, curGyroY = 0.0, curGyroZ = 0.0;
int curGas = 0;

void readSensors() {
  readingCount++;

  // DHT11
  float t = dht.readTemperature();
  float h = dht.readHumidity();
  if (!isnan(t)) curTemp = t;
  if (!isnan(h)) curHumidity = h;

  // MPU6050
  if (mpuReady) {
    sensors_event_t accel, gyro, temp;
    mpu.getEvent(&accel, &gyro, &temp);
    curAccelX = accel.acceleration.x;
    curAccelY = accel.acceleration.y;
    curAccelZ = accel.acceleration.z;
    curGyroX = gyro.gyro.x;
    curGyroY = gyro.gyro.y;
    curGyroZ = gyro.gyro.z;
  }

  // MQ-5
  curGas = analogRead(MQ5_PIN);
}

String getSensorJson() {
  String json = "{";
  json += "\"name\":\"Sensor Rover\",";
  json += "\"count\":" + String(readingCount) + ",";
  json += "\"temp\":" + String(curTemp, 1) + ",";
  json += "\"humidity\":" + String(curHumidity, 1) + ",";
  json += "\"accel\":[" + String(curAccelX, 2) + "," + String(curAccelY, 2) + "," + String(curAccelZ, 2) + "],";
  json += "\"gyro\":[" + String(curGyroX, 2) + "," + String(curGyroY, 2) + "," + String(curGyroZ, 2) + "],";
  json += "\"gas\":" + String(curGas) + ",";
  json += "\"streaming\":" + String(streamTelemetry ? "true" : "false") + ",";
  json += "\"interval_ms\":" + String(sensorReadIntervalMs) + ",";
  json += "\"ip\":\"" + robotComm.getIP().toString() + "\",";
  json += "\"uptime_ms\":" + String(millis());
  json += "}";
  return json;
}

void processSensorCommand(const String& prefix, const String& target, const String& payload, String& reply) {
  if (prefix == "CMD") {
    String tgt = target; tgt.toLowerCase();
    String val = payload; val.toLowerCase();

    if (tgt == "read") {
      readSensors();
      reply = "ACK:SENSORS:" + getSensorJson();
    } else if (tgt == "stream") {
      if (val == "on" || val == "1" || val == "true") {
        streamTelemetry = true;
        reply = "ACK:stream=on";
      } else if (val == "off" || val == "0" || val == "false") {
        streamTelemetry = false;
        reply = "ACK:stream=off";
      } else if (val == "toggle") {
        streamTelemetry = !streamTelemetry;
        reply = "ACK:stream=" + String(streamTelemetry ? "on" : "off");
      } else {
        reply = "ERR:STREAM_ARG_ON_OFF";
      }
    } else if (tgt == "interval") {
      int ms = payload.toInt();
      if (ms >= 100 && ms <= 60000) {
        sensorReadIntervalMs = ms;
        reply = "ACK:interval=" + String(ms);
      } else {
        reply = "ERR:INTERVAL_RANGE_100_60000";
      }
    } else if (tgt == "led") {
      if (val == "on" || val == "1") {
        digitalWrite(LED_PIN, HIGH);
        reply = "ACK:led=on";
      } else if (val == "off" || val == "0") {
        digitalWrite(LED_PIN, LOW);
        reply = "ACK:led=off";
      } else if (val == "toggle") {
        int newState = !digitalRead(LED_PIN);
        digitalWrite(LED_PIN, newState);
        reply = "ACK:led=" + String(newState ? "on" : "off");
      }
    } else {
      reply = "ERR:UNKNOWN_TARGET:" + target;
    }
  } else {
    reply = "ERR:UNKNOWN_PREFIX:" + prefix;
  }
}

void setup() {
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  Serial.begin(115200);
  delay(1000);

  Serial.println("\n=========================================");
  Serial.println("   ROVER - ESP32 Sensor Node (Unified)   ");
  Serial.println("=========================================");

  // Initialize I2C Bus
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);

  // Initialize DHT11
  dht.begin();
  Serial.println("[OK] DHT11 initialized");

  // Initialize MPU6050
  if (mpu.begin()) {
    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
    mpuReady = true;
    Serial.println("[OK] MPU6050 initialized");
  } else {
    Serial.println("[WARN] MPU6050 not found on SDA 21 / SCL 22");
    mpuReady = false;
  }

  Serial.println("[OK] MQ-5 ready (GPIO 34 Analog)");

  // Initialize Unified Communication Protocol
  RobotWiFiConfig wifiCfg(
    "Sabo", "sandy0606",
    "123", "12345678",
    "Rover_AP", "rover1234",
    8888, "rover-sensors"
  );
  robotComm.setConfig(wifiCfg);
  robotComm.setCommandHandler(processSensorCommand);
  robotComm.setStatusHandler(getSensorJson);
  robotComm.begin();
}

void loop() {
  // 1. Process incoming commands across WiFi UDP & Serial
  robotComm.update();

  // 2. Periodic Sensor Sampling & Telemetry
  unsigned long now = millis();
  if (now - lastReadTime >= sensorReadIntervalMs) {
    lastReadTime = now;
    readSensors();

    // Blink onboard LED
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));

    // Stream telemetry over UDP if requested
    if (streamTelemetry) {
      robotComm.broadcastUDP("TELEMETRY:" + getSensorJson());
    }

    // Print to Serial if in Serial mode or monitoring
    if (robotComm.getMode() == ROBOT_COMM_SERIAL) {
      Serial.println("TELEMETRY:" + getSensorJson());
    }
  }
}
