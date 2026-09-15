#ifndef ROBOT_PROTOCOL_H
#define ROBOT_PROTOCOL_H

#include <Arduino.h>
#include <functional>

#if defined(ESP32)
  #include <WiFi.h>
  #include <WiFiUdp.h>
  #include <ESPmDNS.h>
#elif defined(ESP8266)
  #include <ESP8266WiFi.h>
  #include <WiFiUdp.h>
  #include <ESP8266mDNS.h>
#endif

/**
 * Robot Communication Modes
 */
enum RobotCommMode {
  ROBOT_COMM_WIFI,
  ROBOT_COMM_SERIAL,
  ROBOT_COMM_AUTO   // Accepts from any interface, replies to sender
};

/**
 * WiFi Configuration helper
 */
struct RobotWiFiConfig {
  const char* primarySsid;
  const char* primaryPass;
  const char* secondarySsid;
  const char* secondaryPass;
  const char* apSsid;
  const char* apPass;
  uint16_t udpPort;
  const char* mdnsName;

  RobotWiFiConfig(
    const char* pSsid = "Sabo",
    const char* pPass = "sandy0606",
    const char* sSsid = "123",
    const char* sPass = "12345678",
    const char* apS = "Robot_AP",
    const char* apP = "robot1234",
    uint16_t port = 8888,
    const char* mdns = "robot"
  ) : primarySsid(pSsid), primaryPass(pPass),
      secondarySsid(sSsid), secondaryPass(sPass),
      apSsid(apS), apPass(apP),
      udpPort(port), mdnsName(mdns) {}
};

/**
 * Unified Robot Communication System
 */
class RobotProtocol {
public:
  typedef std::function<void(const String& prefix, const String& target, const String& payload, String& reply)> CommandCallback;
  typedef std::function<String()> StatusCallback;

  RobotProtocol(const char* robotName = "Robot")
    : name(robotName),
      commMode(ROBOT_COMM_AUTO),
      extraStream(nullptr),
      lastHeartbeatMs(0),
      heartbeatIntervalMs(2000),
      lastCommandMs(0),
      cmdCallback(nullptr),
      statusCallback(nullptr),
      lastRemotePort(0)
  {
  }

  void setConfig(const RobotWiFiConfig& cfg) {
    config = cfg;
  }

  void setCommandHandler(CommandCallback cb) {
    cmdCallback = cb;
  }

  void setStatusHandler(StatusCallback cb) {
    statusCallback = cb;
  }

  void setExtraStream(Stream* stream) {
    extraStream = stream;
  }

  void setHeartbeatInterval(unsigned long intervalMs) {
    heartbeatIntervalMs = intervalMs;
  }

  RobotCommMode getMode() const { return commMode; }
  void setMode(RobotCommMode mode) { commMode = mode; }
  unsigned long getLastCommandTime() const { return lastCommandMs; }
  bool isTimedOut(unsigned long timeoutMs) const {
    return (millis() - lastCommandMs > timeoutMs);
  }

  IPAddress getIP() const {
    if (WiFi.status() == WL_CONNECTED) return WiFi.localIP();
    return WiFi.softAPIP();
  }

  bool isConnectedWiFi() const {
    return (WiFi.status() == WL_CONNECTED);
  }

  void begin() {
    Serial.println("\n==================================================");
    Serial.print(" 🚀 Initializing Unified Comm Protocol: ");
    Serial.println(name);
    Serial.println("==================================================");

    // 1. Connect to Wi-Fi
    WiFi.mode(WIFI_STA);
    bool connected = false;

    // Try Primary Wi-Fi
    if (config.primarySsid && strlen(config.primarySsid) > 0) {
      Serial.printf("Connecting to Primary Wi-Fi '%s'...", config.primarySsid);
      WiFi.begin(config.primarySsid, config.primaryPass);
      int attempts = 0;
      while (WiFi.status() != WL_CONNECTED && attempts < 25) {
        delay(400);
        Serial.print(".");
        attempts++;
      }
      if (WiFi.status() == WL_CONNECTED) {
        connected = true;
        Serial.println("\n✅ Connected to Primary Wi-Fi!");
      } else {
        Serial.println("\n⚠️ Primary Wi-Fi connection timed out.");
        WiFi.disconnect(true);
        delay(300);
      }
    }

    // Try Secondary Wi-Fi if primary failed
    if (!connected && config.secondarySsid && strlen(config.secondarySsid) > 0) {
      Serial.printf("Connecting to Secondary Wi-Fi '%s'...", config.secondarySsid);
      WiFi.begin(config.secondarySsid, config.secondaryPass);
      int attempts = 0;
      while (WiFi.status() != WL_CONNECTED && attempts < 25) {
        delay(400);
        Serial.print(".");
        attempts++;
      }
      if (WiFi.status() == WL_CONNECTED) {
        connected = true;
        Serial.println("\n✅ Connected to Secondary Wi-Fi!");
      } else {
        Serial.println("\n⚠️ Secondary Wi-Fi connection timed out.");
        WiFi.disconnect(true);
        delay(300);
      }
    }

    // Fallback to SoftAP
    if (!connected) {
      Serial.println("⚠️ Starting Access Point fallback...");
      WiFi.mode(WIFI_AP);
      WiFi.softAP(config.apSsid, config.apPass);
      Serial.printf("📡 AP SSID: %s | IP: %s\n", config.apSsid, WiFi.softAPIP().toString().c_str());
    } else {
      Serial.printf("📡 Local IP Address: %s\n", WiFi.localIP().toString().c_str());
    }

    // 2. Start mDNS
    if (config.mdnsName && strlen(config.mdnsName) > 0) {
      if (MDNS.begin(config.mdnsName)) {
        Serial.printf("🌐 mDNS responder started: http://%s.local\n", config.mdnsName);
      }
    }

    // 3. Start UDP Listener
    udp.begin(config.udpPort);
    Serial.printf("🚀 Listening on UDP Port %d\n", config.udpPort);
    Serial.println("Unified protocol ready.\n");

    lastHeartbeatMs = millis();
    lastCommandMs = millis();
  }

  /**
   * Main update loop — call in Arduino loop()
   */
  void update() {
    readUDP();
    readSerial();
    readExtraStream();
    checkHeartbeat();
  }

  /**
   * Broadcast telemetry or notification over UDP
   */
  void broadcastUDP(const String& message) {
    IPAddress bcast = IPAddress(255, 255, 255, 255);
    udp.beginPacket(bcast, config.udpPort);
    udp.print(message);
    udp.endPacket();
  }

  /**
   * Send UDP message to specific or last known remote client
   */
  void sendUDP(const String& message, IPAddress targetIP = IPAddress(0,0,0,0), uint16_t targetPort = 0) {
    if (targetIP == IPAddress(0,0,0,0)) targetIP = lastRemoteIP;
    if (targetPort == 0) targetPort = lastRemotePort;

    if (targetIP != IPAddress(0,0,0,0) && targetPort != 0) {
      udp.beginPacket(targetIP, targetPort);
      udp.print(message);
      udp.endPacket();
    }
  }

  /**
   * Parse standard command string:
   * 1. Motion/Control command: "PREFIX:TARGET=PAYLOAD" (e.g. CMD:drive=forward)
   * 2. Simple command:        "PREFIX:TARGET"         (e.g. MODE:SERIAL)
   * 3. Bare keyword:          "KEYWORD"               (e.g. PING, STATUS)
   */
  static bool parseCommand(const String& rawLine, String& prefix, String& target, String& payload) {
    String line = rawLine;
    line.trim();
    if (line.length() == 0) return false;

    int colonIdx = line.indexOf(':');
    if (colonIdx < 0) {
      prefix = line;
      target = "";
      payload = "";
      return true;
    }

    prefix = line.substring(0, colonIdx);
    prefix.trim();

    String rest = line.substring(colonIdx + 1);
    rest.trim();

    int eqIdx = rest.indexOf('=');
    if (eqIdx < 0) {
      target = rest;
      payload = "";
    } else {
      target = rest.substring(0, eqIdx);
      target.trim();
      payload = rest.substring(eqIdx + 1);
      payload.trim();
    }

    return true;
  }

  /**
   * Process a single command line and generate appropriate response
   */
  void processCommandLine(const String& line, String& reply) {
    if (line.equalsIgnoreCase("PING")) {
      reply = "ACK:PONG:" + name;
      lastCommandMs = millis();
      return;
    }

    if (line.equalsIgnoreCase("STATUS")) {
      if (statusCallback) {
        reply = "STATUS:" + statusCallback();
      } else {
        reply = "STATUS:{\"name\":\"" + name + "\",\"mode\":\"" + getModeString() + "\",\"ip\":\"" + getIP().toString() + "\"}";
      }
      lastCommandMs = millis();
      return;
    }

    // Check MODE commands
    if (handleModeCommand(line, reply)) {
      lastCommandMs = millis();
      return;
    }

    String prefix, target, payload;
    if (!parseCommand(line, prefix, target, payload)) {
      reply = "ERR:MALFORMED_CMD:" + line;
      return;
    }

    // Forward to custom robot handler
    if (cmdCallback) {
      cmdCallback(prefix, target, payload, reply);
      lastCommandMs = millis();
    } else {
      reply = "ERR:NO_HANDLER:" + line;
    }
  }

private:
  String name;
  RobotWiFiConfig config;
  RobotCommMode commMode;
  WiFiUDP udp;
  Stream* extraStream;

  unsigned long lastHeartbeatMs;
  unsigned long heartbeatIntervalMs;
  unsigned long lastCommandMs;

  CommandCallback cmdCallback;
  StatusCallback statusCallback;

  IPAddress lastRemoteIP;
  uint16_t lastRemotePort;

  String getModeString() const {
    switch (commMode) {
      case ROBOT_COMM_SERIAL: return "SERIAL";
      case ROBOT_COMM_WIFI:   return "WIFI";
      default:                return "AUTO";
    }
  }

  bool handleModeCommand(const String& line, String& reply) {
    if (line.equalsIgnoreCase("MODE:SERIAL")) {
      commMode = ROBOT_COMM_SERIAL;
      reply = "ACK:MODE=SERIAL";
      Serial.println("📡 Comm switched to SERIAL mode");
      return true;
    } else if (line.equalsIgnoreCase("MODE:WIFI")) {
      commMode = ROBOT_COMM_WIFI;
      reply = "ACK:MODE=WIFI";
      Serial.println("📡 Comm switched to WIFI mode");
      return true;
    } else if (line.equalsIgnoreCase("MODE:AUTO")) {
      commMode = ROBOT_COMM_AUTO;
      reply = "ACK:MODE=AUTO";
      Serial.println("📡 Comm switched to AUTO mode");
      return true;
    } else if (line.equalsIgnoreCase("MODE:TOGGLE")) {
      commMode = (commMode == ROBOT_COMM_WIFI) ? ROBOT_COMM_SERIAL : ROBOT_COMM_WIFI;
      reply = "ACK:MODE=" + getModeString();
      Serial.println("📡 Comm toggled to " + getModeString() + " mode");
      return true;
    } else if (line.equalsIgnoreCase("MODE")) {
      reply = "ACK:MODE=" + getModeString();
      return true;
    }
    return false;
  }

  void readUDP() {
    int packetSize = udp.parsePacket();
    if (packetSize > 0) {
      char packetBuffer[512];
      int len = udp.read(packetBuffer, sizeof(packetBuffer) - 1);
      if (len > 0) packetBuffer[len] = 0;

      lastRemoteIP = udp.remoteIP();
      lastRemotePort = udp.remotePort();

      String line = String(packetBuffer);
      line.trim();

      if (line.length() > 0) {
        String reply;
        processCommandLine(line, reply);

        if (reply.length() > 0) {
          udp.beginPacket(lastRemoteIP, lastRemotePort);
          udp.print(reply);
          udp.endPacket();
        }
      }
    }
  }

  void readSerial() {
    while (Serial.available() > 0) {
      String line = Serial.readStringUntil('\n');
      line.trim();

      if (line.length() > 0) {
        String reply;
        processCommandLine(line, reply);

        if (reply.length() > 0) {
          Serial.println(reply);
        }
      }
    }
  }

  void readExtraStream() {
    if (!extraStream) return;

    while (extraStream->available() > 0) {
      String line = extraStream->readStringUntil('\n');
      line.trim();

      if (line.length() > 0) {
        String reply;
        processCommandLine(line, reply);

        if (reply.length() > 0) {
          extraStream->println(reply);
        }
      }
    }
  }

  void checkHeartbeat() {
    unsigned long now = millis();
    if (now - lastHeartbeatMs >= heartbeatIntervalMs) {
      if (now - lastCommandMs > heartbeatIntervalMs) {
        String hb = "HEARTBEAT:NAME=" + name +
                    " | MODE=" + getModeString() +
                    " | IP=" + getIP().toString() +
                    " | RSSI=" + (WiFi.status() == WL_CONNECTED ? String(WiFi.RSSI()) : "AP");
        Serial.println(hb);
      }
      lastHeartbeatMs = now;
    }
  }
};

#endif // ROBOT_PROTOCOL_H
