#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <DHT.h>

#define DHT_PIN 4
#define DHT_TYPE DHT11
#define MQ5_PIN 34

DHT dht(DHT_PIN, DHT_TYPE);
Adafruit_MPU6050 mpu;

void setup() {
  Serial.begin(115200);

  // I2C
  Wire.begin(21, 22);

  // DHT11
  dht.begin();

  // MPU6050
  if (!mpu.begin()) {
    Serial.println("MPU6050 NOT FOUND!");
    while (1) {
      delay(1000);
    }
  }

  Serial.println("MPU6050 OK");
  Serial.println("DHT11 OK");
  Serial.println("MQ-5 OK");
  Serial.println("-----------------------");
}

void loop() {

  // ===== DHT11 =====
  float temperature = dht.readTemperature();
  float humidity = dht.readHumidity();

  // ===== MPU6050 =====
  sensors_event_t accel, gyro, temp;
  mpu.getEvent(&accel, &gyro, &temp);

  // ===== MQ-5 =====
  int gasValue = analogRead(MQ5_PIN);

  // ===== PRINT =====
  Serial.println("========== SENSOR DATA ==========");

  if (isnan(temperature) || isnan(humidity)) {
    Serial.println("DHT11: ERROR");
  } else {
    Serial.print("Temperature: ");
    Serial.print(temperature);
    Serial.println(" °C");

    Serial.print("Humidity: ");
    Serial.print(humidity);
    Serial.println(" %");
  }

  Serial.println("MPU6050:");

  Serial.print("Accel X: ");
  Serial.print(accel.acceleration.x);
  Serial.print(" | Y: ");
  Serial.print(accel.acceleration.y);
  Serial.print(" | Z: ");
  Serial.print(accel.acceleration.z);
  Serial.println(" m/s^2");

  Serial.print("Gyro X: ");
  Serial.print(gyro.gyro.x);
  Serial.print(" | Y: ");
  Serial.print(gyro.gyro.y);
  Serial.print(" | Z: ");
  Serial.print(gyro.gyro.z);
  Serial.println(" rad/s");

  Serial.print("MQ-5 Analog: ");
  Serial.println(gasValue);

  Serial.println("=================================\n");

  delay(2000);
}
