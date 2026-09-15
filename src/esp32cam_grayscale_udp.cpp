/**
 * ESP32-CAM Grayscale UDP + VL53L1X ToF Streamer
 *
 * Board   : AI Thinker ESP32-CAM
 * Stream  : Raw QQVGA (160x120) grayscale pixels over UDP
 * Sensor  : VL53L1X ToF (I2C on GPIO13=SDA, GPIO14=SCL)
 *
 * Packet layout (per UDP datagram):
 *   [14-byte header] + [up to 1200 bytes of raw pixel data]
 *
 * Header (little-endian, packed):
 *   uint32_t frameID       4 bytes
 *   uint16_t packetIndex   2 bytes
 *   uint16_t totalPackets  2 bytes
 *   uint16_t distanceMM    2 bytes
 *   uint16_t width         2 bytes
 *   uint16_t height        2 bytes
 *   ─────────────────────────────
 *   TOTAL                 14 bytes
 */

#include "esp_camera.h"
#include <WiFi.h>
#include <WiFiUdp.h>
#include <Wire.h>
#include <VL53L1X.h>

// ============================================================
// WIFI CONFIGURATION
// ============================================================

#define WIFI_SSID     "123"
#define WIFI_PASSWORD "12345678"
#define RECEIVER_IP   "10.35.234.59"

// ============================================================
// RECEIVER
// ============================================================

const uint16_t UDP_PORT = 5000;

WiFiUDP udp;

// ============================================================
// VL53L1X I2C PINS
// NOTE: Do not use the SD-card while using these pins.
// ============================================================

#define I2C_SDA 13
#define I2C_SCL 14

VL53L1X sensor;

// ============================================================
// AI THINKER ESP32-CAM CAMERA PINS
// ============================================================

#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0

#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27

#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5

#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22

// ============================================================
// STREAM SETTINGS
// ============================================================

// QQVGA = 160 x 120 — small enough to fit several packets
#define FRAME_WIDTH  160
#define FRAME_HEIGHT 120

// Maximum image bytes per UDP datagram (keep well under MTU 1500)
#define PACKET_PAYLOAD_SIZE 1200

uint32_t frameID = 0;

// ============================================================
// UDP PACKET HEADER  (14 bytes, packed, little-endian)
// ============================================================

struct __attribute__((packed)) PacketHeader
{
    uint32_t frameID;
    uint16_t packetIndex;
    uint16_t totalPackets;
    uint16_t distanceMM;
    uint16_t width;
    uint16_t height;
};

// ============================================================
// WIFI CONNECTION
// ============================================================

void connectWiFi()
{
    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

    Serial.println();
    Serial.println("Connecting to WiFi...");

    while (WiFi.status() != WL_CONNECTED)
    {
        delay(500);
        Serial.print(".");
    }

    Serial.println();
    Serial.println("================================");
    Serial.println("WIFI CONNECTED");

    Serial.print("ESP32-CAM IP: ");
    Serial.println(WiFi.localIP());

    Serial.print("Sending UDP to: ");
    Serial.print(RECEIVER_IP);
    Serial.print(":");
    Serial.println(UDP_PORT);

    Serial.println("================================");
}

// ============================================================
// CAMERA SETUP
// ============================================================

bool setupCamera()
{
    camera_config_t config;

    config.ledc_channel = LEDC_CHANNEL_0;
    config.ledc_timer   = LEDC_TIMER_0;

    config.pin_d0 = Y2_GPIO_NUM;
    config.pin_d1 = Y3_GPIO_NUM;
    config.pin_d2 = Y4_GPIO_NUM;
    config.pin_d3 = Y5_GPIO_NUM;
    config.pin_d4 = Y6_GPIO_NUM;
    config.pin_d5 = Y7_GPIO_NUM;
    config.pin_d6 = Y8_GPIO_NUM;
    config.pin_d7 = Y9_GPIO_NUM;

    config.pin_xclk  = XCLK_GPIO_NUM;
    config.pin_pclk  = PCLK_GPIO_NUM;
    config.pin_vsync = VSYNC_GPIO_NUM;
    config.pin_href  = HREF_GPIO_NUM;

    config.pin_sccb_sda = SIOD_GPIO_NUM;
    config.pin_sccb_scl = SIOC_GPIO_NUM;

    config.pin_pwdn  = PWDN_GPIO_NUM;
    config.pin_reset = RESET_GPIO_NUM;

    config.xclk_freq_hz = 20000000;

    // Raw grayscale — no JPEG encoding, lowest latency
    config.pixel_format = PIXFORMAT_GRAYSCALE;
    config.frame_size   = FRAMESIZE_QQVGA;   // 160 x 120
    config.jpeg_quality = 12;                 // unused for GRAYSCALE

    if (psramFound())
    {
        config.fb_count    = 2;
        config.fb_location = CAMERA_FB_IN_PSRAM;
    }
    else
    {
        config.fb_count    = 1;
        config.fb_location = CAMERA_FB_IN_DRAM;
    }

    esp_err_t err = esp_camera_init(&config);

    if (err != ESP_OK)
    {
        Serial.printf("Camera init failed: 0x%x\n", err);
        return false;
    }

    Serial.println("CAMERA READY  [160x120 GRAYSCALE]");
    return true;
}

// ============================================================
// VL53L1X SETUP
// ============================================================

bool setupToF()
{
    Wire.begin(I2C_SDA, I2C_SCL);
    Wire.setClock(400000);    // Fast I2C (400 kHz)

    sensor.setTimeout(500);

    if (!sensor.init())
    {
        Serial.println("WARNING: VL53L1X NOT DETECTED — streaming without ToF distance.");
        return false;
    }

    // Short mode: up to ~1.3 m, better ambient rejection
    sensor.setDistanceMode(VL53L1X::Short);

    // 20 ms budget → fast measurements, less precision at distance
    sensor.setMeasurementTimingBudget(20000);

    // Continuous ranging every 25 ms
    sensor.startContinuous(25);

    Serial.println("VL53L1X READY  [Short mode, 25 ms]");
    return true;
}


// ============================================================
// ============================================================
// TOF READY FLAG  (set false if sensor is absent/unwired)
// ============================================================

bool tof_ready = false;

// ============================================================
// READ TOF DISTANCE  (returns 0 safely if sensor not present)
// ============================================================

uint16_t getDistance()
{
    if (!tof_ready) return 0;

    uint16_t d = sensor.read();

    if (sensor.timeoutOccurred())
    {
        return 0;
    }

    return d;
}


// ============================================================
// CAPTURE + SEND ONE FRAME
// ============================================================

void sendFrame()
{
    // 1. Read ToF before capturing the frame so timestamps align
    uint16_t distanceMM = getDistance();

    // 2. Capture raw grayscale frame from OV2640
    camera_fb_t* fb = esp_camera_fb_get();

    if (!fb)
    {
        Serial.println("Camera capture failed");
        return;
    }

    // 3. Compute required packet count
    uint16_t totalPackets =
        (fb->len + PACKET_PAYLOAD_SIZE - 1) / PACKET_PAYLOAD_SIZE;

    // 4. Slice and send each packet
    for (uint16_t pktIdx = 0; pktIdx < totalPackets; pktIdx++)
    {
        int offset      = pktIdx * PACKET_PAYLOAD_SIZE;
        int remaining   = (int)fb->len - offset;
        int bytesToSend = (remaining > PACKET_PAYLOAD_SIZE)
                          ? PACKET_PAYLOAD_SIZE
                          : remaining;

        PacketHeader hdr;
        hdr.frameID      = frameID;
        hdr.packetIndex  = pktIdx;
        hdr.totalPackets = totalPackets;
        hdr.distanceMM   = distanceMM;
        hdr.width        = (uint16_t)fb->width;
        hdr.height       = (uint16_t)fb->height;

        udp.beginPacket(RECEIVER_IP, UDP_PORT);
        udp.write((uint8_t*)&hdr,         sizeof(PacketHeader));
        udp.write(fb->buf + offset,       bytesToSend);
        udp.endPacket();
    }

    // 5. Release frame buffer back to driver
    esp_camera_fb_return(fb);

    frameID++;

    // Periodic serial status (every 30 frames ≈ ~1 s at 30 fps)
    if (frameID % 30 == 0)
    {
        Serial.printf(
            "Frame %-6u | ToF: %4u mm | Pkts: %u\n",
            frameID, distanceMM, totalPackets
        );
    }
}

// ============================================================
// SETUP
// ============================================================

void setup()
{
    Serial.begin(115200);
    delay(1000);

    Serial.println();
    Serial.println("========================================");
    Serial.println(" ESP32-CAM  GRAYSCALE + VL53L1X  UDP  ");
    Serial.println("========================================");

    if (!setupCamera())
    {
        Serial.println("FATAL: CAMERA FAILED — halting.");
        while (true) { delay(1000); }
    }

    // ToF is optional — camera streams even without it
    tof_ready = setupToF();

    connectWiFi();

    if (tof_ready)
        Serial.println("SYSTEM READY — camera + ToF streaming...");
    else
        Serial.println("SYSTEM READY — camera only (no ToF)...");
}


// ============================================================
// MAIN LOOP
// ============================================================

void loop()
{
    if (WiFi.status() != WL_CONNECTED)
    {
        Serial.println("WiFi lost — reconnecting...");
        connectWiFi();
    }

    sendFrame();
}
