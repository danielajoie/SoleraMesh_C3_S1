/*
Project name: SoleraMesh_C3_S1 - Meshtastic-integrated ESP32-C3 Controller
Board: seeed_xiao_esp32c3
Hardware/pins:
- Servo1: GPIO 5 (PWM channel 0)
- Servo2: GPIO 6 (PWM channel 1)
- LED: GPIO 7 (PWM via analogWrite)
- Motor AIN1: GPIO 0
- Motor AIN2: GPIO 1
- PIR: GPIO 3 (digital input)
- UART: GPIO 4 (RX) / GPIO 9 (TX) for Meshtastic
Libraries: WiFi, ArduinoOTA, Preferences
Date: March 2026
*/

#include <Arduino.h>
#include <ESP32Servo.h>
#include <WiFi.h>
#include <ArduinoOTA.h>
#include <Preferences.h>

// Function prototypes
void attemptWiFiConnect();
void saveDeviceIdentifiers();
void saveWiFiCredentials();
void saveLightSettings();
void setupOTA();
void setServoAngle(int channel, int angle);

// Hardware configuration
const int SERVO1_PIN = 5;
const int SERVO2_PIN = 6;
const int LED_PIN    = 7;

const int MOTOR_AIN1 = 0;
const int MOTOR_AIN2 = 1;

const int UART_RX_PIN = 4;   // ESP32-C3 RX ← Meshtastic TX
const int UART_TX_PIN = 9;   // ESP32-C3 TX → Meshtastic RX

const int PIR_PIN = 3;       // PIR motion sensor

// Updatable variables (via mesh commands)
String deviceName;           // Customizable device name
String groupName;            // Customizable group name
String wifi_ssid;            // WiFi SSID
String wifi_password;        // WiFi password
unsigned long holdTime = 5000UL;  // ms: full power after no motion
unsigned long dimTime  = 10000UL; // ms: dim level before off
int dimLevel           = 50;      // %: dim brightness

// Defaults
const char* DEFAULT_SSID     = "YourWiFiSSID";
const char* DEFAULT_PASSWORD = "YourWiFiPassword";
const char* ota_password     = "ota_pass";

// Firmware version (hardcoded at compile time)
const char* FIRMWARE_VERSION = "SoleraMesh_C3_S1_Status_T260321";

// Runtime variables
String deviceUID;            // Unique fixed UID
unsigned long lastMotionTime = 0;
unsigned long lastBroadcastTime = 0;
unsigned long broadcastCooldown = 5000UL;  // Configurable broadcast cooldown
bool motionDetected = false;
int currentMotorSpeed = 0;   // Track motor for status
int currentLedPwm = 0;       // Track LED PWM for status
int currentServo1Angle = 90; // Track servo angles
int currentServo2Angle = 90;
unsigned long lastWiFiReconnectAttempt = 0;
const unsigned long WIFI_RECONNECT_INTERVAL = 30000UL; // 30 seconds
bool wifiEnabled = false;    // WiFi power saving flag

// UART & Preferences
HardwareSerial MeshSerial(1);
Servo servo1;
Servo servo2;
bool otaInitialized = false;
Preferences prefs;

void setup() {
  Serial.begin(115200);
  MeshSerial.begin(115200, SERIAL_8N1, UART_RX_PIN, UART_TX_PIN);

  // Setup servos
  servo1.setPeriodHertz(50); servo1.attach(SERVO1_PIN, 500, 2400);
  servo2.setPeriodHertz(50); servo2.attach(SERVO2_PIN, 500, 2400);

  prefs.begin("device", false);
  deviceUID = prefs.getString("uid", "");
  if (deviceUID == "") {
    uint64_t mac = ESP.getEfuseMac();
    deviceUID = String((uint32_t)(mac >> 32), HEX) + String((uint32_t)mac, HEX);
    prefs.putString("uid", deviceUID);
  }
  deviceName = prefs.getString("name", "");
  if (deviceName == "") {
    deviceName = "esp-" + deviceUID.substring(0,8);
    prefs.putString("name", deviceName);
  }
  groupName = prefs.getString("group", "");
  prefs.end();

  Serial.println("Device name loaded: " + deviceName);

  prefs.begin("wifi", false);
  wifi_ssid     = prefs.getString("ssid",     DEFAULT_SSID);
  wifi_password = prefs.getString("password", DEFAULT_PASSWORD);
  prefs.end();

  prefs.begin("light", false);
  holdTime = prefs.getULong("holdtime", 5000UL);
  dimTime  = prefs.getULong("dimtime",  10000UL);
  dimLevel = prefs.getInt("dimlevel",   50);
  broadcastCooldown = prefs.getULong("broadcast", 5000UL);
  prefs.end();

  MeshSerial.printf("C3 started | UID:%s | Name:%s | Group:%s\n",
                    deviceUID.c_str(), deviceName.c_str(), groupName.c_str());

  // Broadcast device identification on startup/reset
  MeshSerial.printf("device: %s\n", deviceName.c_str());
  Serial.println("Identify broadcast sent over mesh");

  // WiFi disabled by default for power savings

  pinMode(LED_PIN, OUTPUT);
  pinMode(MOTOR_AIN1, OUTPUT);
  pinMode(MOTOR_AIN2, OUTPUT);
  pinMode(PIR_PIN, INPUT);

  analogWrite(LED_PIN, 0);
  analogWrite(MOTOR_AIN1, 0);
  analogWrite(MOTOR_AIN2, 0);

  setServoAngle(0, 90);
  setServoAngle(1, 90);
}

void setServoAngle(int channel, int angle) {
  Serial.println("Setting servo " + String(channel) + " to angle " + String(angle));
  if (channel == 0) {
    servo1.write(angle);
  } else if (channel == 1) {
    servo2.write(angle);
  }
}

void attemptWiFiConnect() {
  WiFi.mode(WIFI_STA);
  WiFi.setAutoReconnect(true);
  MeshSerial.printf("Attempting to connect to WiFi: %s\n", wifi_ssid.c_str());
  WiFi.begin(wifi_ssid.c_str(), wifi_password.c_str());

  unsigned long t = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t < 15000) {
    delay(500);
    MeshSerial.print(".");
  }
  MeshSerial.println();

  if (WiFi.status() == WL_CONNECTED) {
    MeshSerial.println("WiFi connected successfully");
    MeshSerial.printf("IP: %s\n", WiFi.localIP().toString().c_str());
    lastWiFiReconnectAttempt = 0; // Reset timer
    setupOTA();
  } else {
    MeshSerial.printf("WiFi connection failed. Status: %d\n", WiFi.status());
  }
}

void saveDeviceIdentifiers() {
  prefs.begin("device", false);
  prefs.putString("name", deviceName);
  prefs.putString("group", groupName);
  prefs.end();
}

void saveWiFiCredentials() {
  prefs.begin("wifi", false);
  prefs.putString("ssid", wifi_ssid);
  prefs.putString("password", wifi_password);
  prefs.end();
}

void saveLightSettings() {
  prefs.begin("light", false);
  prefs.putULong("holdtime", holdTime);
  prefs.putULong("dimtime", dimTime);
  prefs.putInt("dimlevel", dimLevel);
  prefs.putULong("broadcast", broadcastCooldown);
  prefs.end();
}

void setupOTA() {
  ArduinoOTA.setHostname(deviceName.c_str());
  ArduinoOTA.setPassword(ota_password);

  ArduinoOTA.onStart([]() {
    String type;
    if (ArduinoOTA.getCommand() == U_FLASH) {
      type = "sketch";
    } else {
      type = "filesystem";
    }
    Serial.println("OTA Start updating " + type);
  });

  ArduinoOTA.onEnd([]() {
    Serial.println("\nOTA End");
  });

  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("OTA Progress: %u%%\r", (progress / (total / 100)));
  });

  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("OTA Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
    else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
    else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
    else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
    else if (error == OTA_END_ERROR) Serial.println("End Failed");
  });

  ArduinoOTA.begin();
  Serial.println("OTA initialized");
  otaInitialized = true;
}

void loop() {
  if (WiFi.status() == WL_CONNECTED) ArduinoOTA.handle();

  // PIR + LED logic
  bool motionNow = digitalRead(PIR_PIN) == HIGH;
  if (motionNow) {
    lastMotionTime = millis();
    analogWrite(LED_PIN, 255);
    currentLedPwm = 255;
    if (groupName != "" && millis() - lastBroadcastTime >= broadcastCooldown) {
      MeshSerial.printf("%s:light:on\n", groupName.c_str());
      lastBroadcastTime = millis();
    }
  } else {
    unsigned long elapsed = millis() - lastMotionTime;
    if (elapsed > holdTime + dimTime) {
      analogWrite(LED_PIN, 0);
      currentLedPwm = 0;
    } else if (elapsed > holdTime) {
      int dim = (dimLevel * 255) / 100;
      analogWrite(LED_PIN, dim);
      currentLedPwm = dim;
    } else {
      currentLedPwm = 255;
    }
  }

  // WiFi reconnection logic (only if WiFi is enabled)
  if (wifiEnabled && WiFi.status() != WL_CONNECTED && millis() - lastWiFiReconnectAttempt > WIFI_RECONNECT_INTERVAL) {
    MeshSerial.println("WiFi disconnected, attempting reconnection...");
    lastWiFiReconnectAttempt = millis();
    otaInitialized = false;
    attemptWiFiConnect();
  }

  // Mesh command parser
  if (MeshSerial.available()) {
    String line = MeshSerial.readStringUntil('\n');
    line.trim();

    Serial.println("Received line: " + line);

    int colon = line.indexOf(':');
    String content;
    if (colon == -1) {
      content = line;
    } else {
      content = line.substring(colon + 1);
    }
    content.trim();

    // Special broadcast identify command
    if (content == "identify") {
      Serial.println("Identify command received, responding");
      MeshSerial.printf("device: %s\n", deviceName.c_str());
      return;
    }

    if (!content.startsWith(deviceName + ":") && !content.startsWith(groupName + ":")) return;

    int offset = content.startsWith(deviceName + ":") ? deviceName.length() + 1 : groupName.length() + 1;
    String cmd = content.substring(offset);
    cmd.trim();

    if (cmd.startsWith("servo1:")) {
      int angle = cmd.substring(7).toInt();
      if (angle >= 0 && angle <= 180) {
        setServoAngle(0, angle);
        currentServo1Angle = angle;
        MeshSerial.printf("servo1 → %d\n", angle);
      }
    }
    else if (cmd.startsWith("servo2:")) {
      int angle = cmd.substring(7).toInt();
      if (angle >= 0 && angle <= 180) {
        setServoAngle(1, angle);
        currentServo2Angle = angle;
        MeshSerial.printf("servo2 → %d\n", angle);
      }
    }
    else if (cmd.startsWith("light:")) {
      String state = cmd.substring(6);
      state.toLowerCase();
      int val = (state=="on"||state=="1") ? 255 : (state=="off"||state=="0") ? 0 : state.toInt();
      if (val>=0 && val<=255) {
        analogWrite(LED_PIN, val);
        currentLedPwm = val;
        MeshSerial.printf("light → %d\n", val);
        lastMotionTime = millis();  // reset motion timer
      }
    }
    else if (cmd.startsWith("motor:")) {
      int speed = cmd.substring(6).toInt();
      if (speed >= -255 && speed <= 255) {
        if (speed > 0) {
          analogWrite(MOTOR_AIN1, speed);
          analogWrite(MOTOR_AIN2, 0);
        } else if (speed < 0) {
          analogWrite(MOTOR_AIN1, 0);
          analogWrite(MOTOR_AIN2, -speed);
        } else {
          analogWrite(MOTOR_AIN1, 0);
          analogWrite(MOTOR_AIN2, 0);
        }
        currentMotorSpeed = speed;
        MeshSerial.printf("motor → %d\n", speed);
      }
    }
    else if (cmd.startsWith("wifi:")) {
      String sub = cmd.substring(5);
      sub.trim();
      if (sub.startsWith("ssid:")) {
        wifi_ssid = sub.substring(5); wifi_ssid.trim();
        saveWiFiCredentials();
        MeshSerial.printf("ssid → %s\n", wifi_ssid.c_str());
      }
      else if (sub.startsWith("pass:")) {
        wifi_password = sub.substring(5); wifi_password.trim();
        saveWiFiCredentials();
        MeshSerial.println("pass updated");
      }
      else if (sub == "connect") {
        wifiEnabled = true;
        MeshSerial.println("Reconnecting...");
        WiFi.disconnect(true); delay(100);
        otaInitialized = false;
        attemptWiFiConnect();
      }
      else if (sub == "on") {
        wifiEnabled = true;
        if (WiFi.status() != WL_CONNECTED) {
          attemptWiFiConnect();
        } else {
          MeshSerial.println("WiFi already on");
        }
      }
      else if (sub == "off") {
        wifiEnabled = false;
        WiFi.disconnect(true);
        otaInitialized = false;
        MeshSerial.println("WiFi off");
      }
    }
    else if (cmd.startsWith("name:")) {
      String newName = cmd.substring(5); newName.trim();
      if (newName.length() > 0) {
        deviceName = newName;
        saveDeviceIdentifiers();
        MeshSerial.printf("name → %s\n", deviceName.c_str());
      }
    }
    else if (cmd.startsWith("group:")) {
      String newGroup = cmd.substring(6); newGroup.trim();
      groupName = newGroup;
      saveDeviceIdentifiers();
      MeshSerial.printf("group → %s\n", groupName.c_str());
    }
    else if (cmd.startsWith("holdtime:")) {
      unsigned long val = cmd.substring(9).toInt();
      if (val >= 1000 && val <= 600000) {
        holdTime = val;
        saveLightSettings();
        MeshSerial.printf("holdtime → %lu\n", holdTime);
      }
    }
    else if (cmd.startsWith("dimtime:")) {
      unsigned long val = cmd.substring(8).toInt();
      if (val >= 1000 && val <= 600000) {
        dimTime = val;
        saveLightSettings();
        MeshSerial.printf("dimtime → %lu\n", dimTime);
      }
    }
    else if (cmd.startsWith("dimlevel:")) {
      int val = cmd.substring(9).toInt();
      if (val >= 0 && val <= 100) {
        dimLevel = val;
        saveLightSettings();
        MeshSerial.printf("dimlevel → %d\n", dimLevel);
      }
    }
    else if (cmd.startsWith("broadcast:")) {
      unsigned long val = cmd.substring(10).toInt();
      if (val >= 1000 && val <= 30000) {
        broadcastCooldown = val;
        saveLightSettings();
        MeshSerial.printf("broadcast → %lu\n", broadcastCooldown);
      }
    }
    // parameter: config dump
    else if (cmd == "parameter") {
      MeshSerial.printf("name: %s\n", deviceName.c_str());
      MeshSerial.printf("group: %s\n", groupName.c_str());
      MeshSerial.printf("ssid: %s\n", wifi_ssid.c_str());
      MeshSerial.printf("holdtime: %lu\n", holdTime);
      MeshSerial.printf("dimtime: %lu\n", dimTime);
      MeshSerial.printf("dimlevel: %d\n", dimLevel);
      MeshSerial.printf("broadcast: %lu\n", broadcastCooldown);
      MeshSerial.printf("version: %s\n", FIRMWARE_VERSION);
    }
    // status: runtime dump
    else if (cmd == "status") {
      MeshSerial.printf("name: %s\n", deviceName.c_str());
      MeshSerial.printf("group: %s\n", groupName.c_str());

      String wifiState;
      if (!wifiEnabled) {
        wifiState = "disabled";
      } else if (WiFi.status() == WL_CONNECTED) {
        wifiState = "connected " + wifi_ssid + " " + WiFi.localIP().toString();
      } else {
        wifiState = "off";
      }
      MeshSerial.printf("wifi: %s\n", wifiState.c_str());

      String lightState = currentLedPwm == 255 ? "on" : currentLedPwm > 0 ? "dim" : "off";
      MeshSerial.printf("light: %s (%d)\n", lightState.c_str(), currentLedPwm);

      MeshSerial.printf("motor: %d\n", currentMotorSpeed);
      MeshSerial.printf("servo1: %d\n", currentServo1Angle);
      MeshSerial.printf("servo2: %d\n", currentServo2Angle);
    }
  }
}