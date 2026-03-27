/*
Project name: SoleraMesh_C3_S1 - Meshtastic-integrated ESP32-C3 Controller
Board: seeed_xiao_esp32c3
Hardware/pins:
- Servo1: GPIO 5 (PWM channel 0)
- Servo2: GPIO 6 (PWM channel 1)
- LED: GPIO 7 (PWM via analogWrite)
- Motor AIN1: GPIO 0
- Motor AIN2: GPIO 1
- PIR: GPIO 4 (digital input)
- UART: GPIO 3 (RX) / GPIO 2 (TX) for Meshtastic
- I2C: GPIO 8 (SDA) / GPIO 9 (SCL) for INA3221
Libraries: WiFi, ArduinoOTA, Preferences, Wire
Date: March 2026
*/

#include <Arduino.h>
#include <ESP32Servo.h>
#include <WiFi.h>
#include <ArduinoOTA.h>
#include <Preferences.h>
#include <Wire.h>

// Function prototypes
void attemptWiFiConnect();
void saveDeviceIdentifiers();
void saveWiFiCredentials();
void saveLightSettings();
void setupOTA();
void setServoAngle(int channel, int angle);
int16_t readReg(uint8_t reg);
uint8_t readDS1307(uint8_t reg);
void writeDS1307(uint8_t reg, uint8_t data);
void readTime();

// Hardware configuration
const int SERVO1_PIN = 5;
const int SERVO2_PIN = 6;
const int LED_PIN    = 7;
const int LEDC_CHANNEL = 0;
const int LEDC_FREQ = 5000;
const int LEDC_RES = 8;

const int MOTOR_AIN1 = 0;
const int MOTOR_AIN2 = 1;

const int UART_RX_PIN = 3;   // ESP32-C3 RX ← Meshtastic TX
const int UART_TX_PIN = 2;   // ESP32-C3 TX → Meshtastic RX

const int PIR_PIN = 4;       // PIR motion sensor

// INA3221 configuration
#define SHUNT_RESISTOR 0.1  // Ohms
#define SDA_PIN 8
#define SCL_PIN 9
#define I2C_ADDR_INA 0x40

// DS1307 configuration
#define DS1307_ADDR 0x68

// INA3221 runtime variables
float inaBusV[3] = {0, 0, 0};    // Bus voltage for Ch1,2,3
float inaShuntV[3] = {0, 0, 0};  // Shunt voltage
float inaCurrent[3] = {0, 0, 0}; // Current
float inaPower[3] = {0, 0, 0};   // Power

// DS1307 time structure
struct TimeStruct {
  uint8_t second;
  uint8_t minute;
  uint8_t hour;
  uint8_t dayOfWeek;
  uint8_t dayOfMonth;
  uint8_t month;
  uint8_t year;
};

// DS1307 runtime variables
TimeStruct currentTime = {0, 0, 0, 0, 0, 0, 0};
bool ds1307Present = false;

// Updatable variables (via mesh commands)
String deviceName;           // Customizable device name
String groupName;            // Customizable group name
String wifi_ssid;            // WiFi SSID
String wifi_password;        // WiFi password
int8_t utcOffset = 0;        // UTC offset in hours (-12 to +12)
bool observeDST = false;     // Observe daylight saving time

// Load Power Set variables
float sunsetThreshold = 4.0;     // Voltage threshold for sunset detection
uint16_t motionWindow = 10000;   // Motion detection window in milliseconds (default 10 seconds)
uint8_t startMode = 0;           // 0=dusk (voltage), 1=fixed time (RTC)
uint8_t startHour = 18;          // Fixed start hour (0-23) for RTC mode
uint8_t startMinute = 0;         // Fixed start minute (0-59) for RTC mode
uint8_t sTime[9] = {2, 1, 0, 0, 0, 0, 0, 0, 0};  // Hours per segment (S-Time-1 to S-Time-9)
uint8_t sCPow[9] = {80, 80, 0, 0, 0, 0, 0, 0, 0}; // Motion power % (S-C-Pow-1 to S-C-Pow-9)
uint8_t sLPow[9] = {10, 20, 0, 0, 0, 0, 0, 0, 0}; // Idle power % (S-L-Pow-1 to S-L-Pow-9)

// Load Power Set state machine
enum LightMode { MODE_DAY, MODE_NIGHT };
LightMode currentMode = MODE_DAY;
uint8_t currentSegment = 0;       // 0-8 (segment index)
unsigned long segmentStartTime = 0; // millis() when current segment started
unsigned long sunsetDetectTime = 0; // millis() when sunset first detected
unsigned long dawnDetectTime = 0;   // millis() when dawn first detected
bool sunsetDetected = false;
bool dawnDetected = false;

// Defaults
const char* DEFAULT_SSID     = "YourWiFiSSID";
const char* DEFAULT_PASSWORD = "YourWiFiPassword";
const char* ota_password     = "ota_pass";

// Firmware version (hardcoded at compile time)
const char* FIRMWARE_VERSION = "SoleraMesh_RTCtD260326A";

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
  broadcastCooldown = prefs.getULong("broadcast", 5000UL);
  prefs.end();

  prefs.begin("timezone", false);
  utcOffset = prefs.getChar("offset", 0);
  observeDST = prefs.getBool("dst", false);
  prefs.end();

  // Load Load Power Set settings
  prefs.begin("loadpower", false);
  sunsetThreshold = prefs.getFloat("threshold", 4.0);
  motionWindow = prefs.getUShort("motionwin", 10000);
  startMode = prefs.getUChar("startmode", 0);
  startHour = prefs.getUChar("starthour", 18);
  startMinute = prefs.getUChar("startmin", 0);
  for (uint8_t i = 0; i < 9; i++) {
    sTime[i] = prefs.getUChar(("time" + String(i+1)).c_str(), (i < 2) ? ((i == 0) ? 2 : 1) : 0);
    sCPow[i] = prefs.getUChar(("cpow" + String(i+1)).c_str(), (i < 2) ? 80 : 0);
    sLPow[i] = prefs.getUChar(("lpow" + String(i+1)).c_str(), (i == 0) ? 10 : (i == 1) ? 20 : 0);
  }
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

  // Setup LEDC for LED PWM
  ledcSetup(LEDC_CHANNEL, LEDC_FREQ, LEDC_RES);
  ledcAttachPin(LED_PIN, LEDC_CHANNEL);
  ledcWrite(LEDC_CHANNEL, 0);

  analogWrite(MOTOR_AIN1, 0);
  analogWrite(MOTOR_AIN2, 0);

  setServoAngle(0, 90);
  setServoAngle(1, 90);

  // Initialize INA3221
  Wire.begin(SDA_PIN, SCL_PIN);

  // Verify chip: Manu ID 0xFE=0x5449 (TI)
  uint16_t manu = readReg(0xFE);
  if (manu != 0x5449) {
    Serial.print("INA3221 not found (Manu ID: 0x"); Serial.print(manu, HEX); Serial.println(")");
  } else {
    Serial.println("INA3221 found");

    // Config
    uint16_t config = readReg(0x00);
    Serial.print("Initial config: 0x"); Serial.println(config, HEX);

    config |= 0x7000;  // Enable Ch1-3 (bits 14-12)
    config &= ~0x003F; // Clear avg (5-3), mode (2-0)
    config |= 0x0020;  // Avg 128 (bits 5-3=100)
    config |= 0x0007;  // Mode shunt+bus cont (bits 2-0=111)

    Wire.beginTransmission(I2C_ADDR_INA);
    Wire.write(0x00);
    Wire.write(config >> 8);
    Wire.write(config & 0xFF);
    Wire.endTransmission();

    config = readReg(0x00);
    Serial.print("Updated config: 0x"); Serial.println(config, HEX);
    Serial.println("INA3221 Initialized");
  }

  // Initialize DS1307
  // Check if DS1307 is present by reading a register
  Wire.beginTransmission(DS1307_ADDR);
  Wire.write(0x00);  // Seconds register
  if (Wire.endTransmission() == 0) {
    ds1307Present = true;
    Serial.println("DS1307 RTC found");

    // Enable oscillator if not already running
    uint8_t seconds = readDS1307(0x00);
    if (seconds & 0x80) {  // CH bit set (oscillator stopped)
      writeDS1307(0x00, seconds & 0x7F);  // Clear CH bit to start oscillator
      Serial.println("DS1307 oscillator started");
    }

    // Read initial time
    readTime();
    Serial.println("DS1307 Initialized");
  } else {
    Serial.println("DS1307 RTC not found");
  }
}

void setServoAngle(int channel, int angle) {
  Serial.println("Setting servo " + String(channel) + " to angle " + String(angle));
  if (channel == 0) {
    servo1.write(angle);
  } else if (channel == 1) {
    servo2.write(angle);
  }
}

// INA3221 functions
int16_t readReg(uint8_t reg) {
  Wire.beginTransmission(I2C_ADDR_INA);
  Wire.write(reg);
  Wire.endTransmission();
  Wire.requestFrom(I2C_ADDR_INA, 2);
  if (Wire.available() < 2) return 0;
  return (Wire.read() << 8) | Wire.read();
}

float getBusVoltage(uint8_t ch) {
  if (ch < 1 || ch > 3) return NAN;
  int16_t raw = readReg(0x02 + (ch-1)*2) >> 3;
  return raw * 0.008f;
}

float getShuntVoltage(uint8_t ch) {
  if (ch < 1 || ch > 3) return NAN;
  int16_t raw = readReg(0x01 + (ch-1)*2);
  raw >>= 3;  // 13-bit
  if (raw & 0x1000) raw |= 0xF000;  // Sign extend
  return raw * 0.00004f;
}

void readINA3221() {
  for (uint8_t ch = 1; ch <= 3; ch++) {
    inaBusV[ch-1] = getBusVoltage(ch);
    inaShuntV[ch-1] = getShuntVoltage(ch);
    inaCurrent[ch-1] = inaShuntV[ch-1] / SHUNT_RESISTOR;
    inaPower[ch-1] = inaBusV[ch-1] * inaCurrent[ch-1];
  }
}

// DS1307 functions
uint8_t decToBcd(uint8_t val) {
  return ((val / 10 * 16) + (val % 10));
}

uint8_t bcdToDec(uint8_t val) {
  return ((val / 16 * 10) + (val % 16));
}

void writeDS1307(uint8_t reg, uint8_t data) {
  Wire.beginTransmission(DS1307_ADDR);
  Wire.write(reg);
  Wire.write(data);
  Wire.endTransmission();
}

uint8_t readDS1307(uint8_t reg) {
  Wire.beginTransmission(DS1307_ADDR);
  Wire.write(reg);
  Wire.endTransmission();
  Wire.requestFrom(DS1307_ADDR, 1);
  if (Wire.available()) {
    return Wire.read();
  }
  return 0;
}

void readTime() {
  if (!ds1307Present) return;

  currentTime.second = bcdToDec(readDS1307(0x00) & 0x7F);
  currentTime.minute = bcdToDec(readDS1307(0x01));
  currentTime.hour = bcdToDec(readDS1307(0x02) & 0x3F);
  currentTime.dayOfWeek = bcdToDec(readDS1307(0x03));
  currentTime.dayOfMonth = bcdToDec(readDS1307(0x04));
  currentTime.month = bcdToDec(readDS1307(0x05));
  currentTime.year = bcdToDec(readDS1307(0x06));
}

void setTime(uint8_t second, uint8_t minute, uint8_t hour, uint8_t dayOfWeek, uint8_t dayOfMonth, uint8_t month, uint8_t year) {
  if (!ds1307Present) return;

  writeDS1307(0x00, decToBcd(second));
  writeDS1307(0x01, decToBcd(minute));
  writeDS1307(0x02, decToBcd(hour));
  writeDS1307(0x03, decToBcd(dayOfWeek));
  writeDS1307(0x04, decToBcd(dayOfMonth));
  writeDS1307(0x05, decToBcd(month));
  writeDS1307(0x06, decToBcd(year));

  // Update current time
  currentTime.second = second;
  currentTime.minute = minute;
  currentTime.hour = hour;
  currentTime.dayOfWeek = dayOfWeek;
  currentTime.dayOfMonth = dayOfMonth;
  currentTime.month = month;
  currentTime.year = year;
}

void syncNTP() {
  if (!wifiEnabled || WiFi.status() != WL_CONNECTED) {
    MeshSerial.println("wifi OFF");
    return;
  }

  configTime(0, 0, "pool.ntp.org");  // UTC time

  struct tm timeinfo;
  if (getLocalTime(&timeinfo, 10000)) {  // 10 second timeout
    setTime(timeinfo.tm_sec, timeinfo.tm_min, timeinfo.tm_hour,
            timeinfo.tm_wday + 1, timeinfo.tm_mday, timeinfo.tm_mon + 1,
            timeinfo.tm_year - 100);  // tm_year is years since 1900, DS1307 uses 00-99
    MeshSerial.println("NTP sync successful");
  } else {
    MeshSerial.println("NTP sync failed");
  }
}

bool isDST() {
  if (!observeDST) return false;

  // Simple US DST: March to November (2nd Sunday in March to 1st Sunday in November)
  int year = currentTime.year + 2000;
  int month = currentTime.month;
  int day = currentTime.dayOfMonth;

  if (month < 3 || month > 11) return false;
  if (month > 3 && month < 11) return true;

  // March: DST starts on 2nd Sunday
  if (month == 3) {
    int firstSunday = (1 + (7 - ((2 + year/4 - year/100 + year/400) % 7)) % 7);
    int secondSunday = firstSunday + 7;
    return day >= secondSunday;
  }

  // November: DST ends on 1st Sunday
  if (month == 11) {
    int firstSunday = (1 + (7 - ((2 + year/4 - year/100 + year/400) % 7)) % 7);
    return day < firstSunday;
  }

  return false;
}

String getTimeString() {
  if (!ds1307Present) return "RTC not found";

  int8_t totalOffset = utcOffset;
  if (isDST()) totalOffset += 1;

  // Apply timezone offset to get local time
  int localHour = currentTime.hour + totalOffset;
  int localDay = currentTime.dayOfMonth;
  int localMonth = currentTime.month;
  int localYear = currentTime.year + 2000;

  // Handle day/month/year rollover
  if (localHour >= 24) {
    localHour -= 24;
    localDay++;
    // Simple day increment (not accounting for month lengths)
    if (localDay > 31) {
      localDay = 1;
      localMonth++;
      if (localMonth > 12) {
        localMonth = 1;
        localYear++;
      }
    }
  } else if (localHour < 0) {
    localHour += 24;
    localDay--;
    if (localDay < 1) {
      localMonth--;
      if (localMonth < 1) {
        localMonth = 12;
        localYear--;
      }
      // Simple day decrement (not accounting for month lengths)
      localDay = 31; // Approximation
    }
  }

  char buf[20];
  sprintf(buf, "%04d-%02d-%02d %02d:%02d:%02d",
          localYear, localMonth, localDay,
          localHour, currentTime.minute, currentTime.second);
  return String(buf);
}

String getUTCTimeString() {
  if (!ds1307Present) return "RTC not found";

  char buf[20];
  sprintf(buf, "%04d-%02d-%02d %02d:%02d:%02d",
          currentTime.year + 2000, currentTime.month, currentTime.dayOfMonth,
          currentTime.hour, currentTime.minute, currentTime.second);
  return String(buf);
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
  prefs.putULong("broadcast", broadcastCooldown);
  prefs.end();
}

void saveTimezoneSettings() {
  prefs.begin("timezone", false);
  prefs.putChar("offset", utcOffset);
  prefs.putBool("dst", observeDST);
  prefs.end();
}

void saveLoadPowerSettings() {
  prefs.begin("loadpower", false);
  prefs.putFloat("threshold", sunsetThreshold);
  prefs.putUShort("motionwin", motionWindow);
  prefs.putUChar("startmode", startMode);
  prefs.putUChar("starthour", startHour);
  prefs.putUChar("startmin", startMinute);
  for (uint8_t i = 0; i < 9; i++) {
    prefs.putUChar(("time" + String(i+1)).c_str(), sTime[i]);
    prefs.putUChar(("cpow" + String(i+1)).c_str(), sCPow[i]);
    prefs.putUChar(("lpow" + String(i+1)).c_str(), sLPow[i]);
  }
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
  static unsigned long lastINARead = 0;
  static unsigned long lastTimeRead = 0;

  if (millis() - lastINARead >= 5000) {  // Read every 5 seconds
    readINA3221();
    lastINARead = millis();
  }

  if (millis() - lastTimeRead >= 1000) {  // Read time every second
    readTime();
    lastTimeRead = millis();
  }

  if (WiFi.status() == WL_CONNECTED) ArduinoOTA.handle();

  // Load Power Set logic
  bool motionNow = digitalRead(PIR_PIN) == HIGH;
  if (motionNow) {
    lastMotionTime = millis();
    if (groupName != "" && millis() - lastBroadcastTime >= broadcastCooldown) {
      MeshSerial.printf("%s:light:on\n", groupName.c_str());
      lastBroadcastTime = millis();
    }
  }

  // Load Power Set DAY/NIGHT transition logic
  if (startMode == 0) {  // Dusk mode - solar voltage detection
    if (inaBusV[0] < sunsetThreshold) {  // Solar voltage below threshold = sunset
      if (!sunsetDetected) {
        if (sunsetDetectTime == 0) {
          sunsetDetectTime = millis();
        } else if (millis() - sunsetDetectTime >= 30000) {  // 30 seconds
          sunsetDetected = true;
          dawnDetected = false;
          currentMode = MODE_NIGHT;
          currentSegment = 0;
          segmentStartTime = millis();
          Serial.println("Sunset detected - starting Load Power Set");
        }
      }
    } else {  // Solar voltage above threshold = dawn
      if (!dawnDetected && currentMode == MODE_NIGHT) {
        if (dawnDetectTime == 0) {
          dawnDetectTime = millis();
        } else if (millis() - dawnDetectTime >= 30000) {  // 30 seconds
          dawnDetected = true;
          sunsetDetected = false;
          currentMode = MODE_DAY;
          Serial.println("Dawn detected - stopping Load Power Set");
        }
      }
    }

    // Reset detection timers if voltage goes back above/below threshold
    if (inaBusV[0] >= sunsetThreshold && sunsetDetectTime > 0) {
      sunsetDetectTime = 0;
    }
    if (inaBusV[0] < sunsetThreshold && dawnDetectTime > 0) {
      dawnDetectTime = 0;
    }
  } else {  // Fixed time mode - RTC based
    if (ds1307Present) {
      // Calculate local time
      int8_t totalOffset = utcOffset;
      if (isDST()) totalOffset += 1;
      int localHour = currentTime.hour + totalOffset;
      int localMinute = currentTime.minute;

      // Handle hour rollover
      if (localHour >= 24) {
        localHour -= 24;
      } else if (localHour < 0) {
        localHour += 24;
      }

      // Check if current local time >= start time
      bool isStartTime = (localHour > startHour) ||
                        (localHour == startHour && localMinute >= startMinute);

      if (isStartTime && currentMode == MODE_DAY) {
        // Start NIGHT mode
        currentMode = MODE_NIGHT;
        currentSegment = 0;
        segmentStartTime = millis();
        Serial.println("Fixed time reached - starting Load Power Set");
      } else if (!isStartTime && currentMode == MODE_NIGHT) {
        // End NIGHT mode (next day)
        currentMode = MODE_DAY;
        Serial.println("Fixed time cycle completed - stopping Load Power Set");
      }
    }
  }

  // Load Power Set control logic
  if (currentMode == MODE_NIGHT) {
    // Check if current segment has expired
    unsigned long segmentElapsed = millis() - segmentStartTime;
    unsigned long segmentDurationMs = (unsigned long)sTime[currentSegment] * 3600000UL; // Convert hours to ms

    if (segmentElapsed >= segmentDurationMs || sTime[currentSegment] == 0) {
      // Move to next segment or end schedule
      currentSegment++;
      if (currentSegment >= 9 || sTime[currentSegment] == 0) {
        // End of schedule - turn off light
        currentMode = MODE_DAY;
        ledcWrite(LEDC_CHANNEL, 0);
        currentLedPwm = 0;
        Serial.println("Load Power Set schedule completed");
      } else {
        // Start next segment
        segmentStartTime = millis();
        Serial.printf("Starting segment %d\n", currentSegment + 1);
      }
    }

    if (currentMode == MODE_NIGHT) {
      // Control LED based on motion and current segment
      bool recentMotion = (millis() - lastMotionTime) < motionWindow;
      uint8_t targetPower = recentMotion ? sCPow[currentSegment] : sLPow[currentSegment];
      int pwmValue = (targetPower * 255) / 100;

      // Debug output for LED power changes
      static int lastPwmValue = -1;
      if (pwmValue != lastPwmValue) {
        Serial.printf("LED Power: %d%% (%d PWM) - %s\n",
                     targetPower, pwmValue, recentMotion ? "motion" : "idle");
        lastPwmValue = pwmValue;
      }

      ledcWrite(LEDC_CHANNEL, pwmValue);
      currentLedPwm = pwmValue;
    }
  } else {
    // Day mode - light off
    ledcWrite(LEDC_CHANNEL, 0);
    currentLedPwm = 0;
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
        ledcWrite(LEDC_CHANNEL, val);
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
      MeshSerial.printf("broadcast: %lu\n", broadcastCooldown);
      MeshSerial.printf("version: %s\n", FIRMWARE_VERSION);
    }
    // statusG: general status (name, group, wifi, light, time)
    else if (cmd == "statusG") {
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

      // Time - show local time, with UTC if different
      if (utcOffset != 0 || observeDST) {
        MeshSerial.printf("time: %s (UTC: %s)\n", getTimeString().c_str(), getUTCTimeString().c_str());
      } else {
        MeshSerial.printf("time: %s\n", getTimeString().c_str());
      }

      // Timezone info
      char tzSign = (utcOffset >= 0) ? '+' : '-';
      int tzAbs = abs(utcOffset);
      MeshSerial.printf("timezone: UTC%c%d, DST=%s (%s)\n", tzSign, tzAbs,
                       observeDST ? "on" : "off", isDST() ? "active" : "inactive");
    }
    // statusP: power status (name, solar, battery, led_load)
    else if (cmd == "statusP") {
      MeshSerial.printf("name: %s\n", deviceName.c_str());
      MeshSerial.printf("solar: %.2fV %.1fmA %.3fW\n", inaBusV[0], inaCurrent[0]*1000, inaPower[0]);
      MeshSerial.printf("battery: %.2fV %.1fmA %.3fW\n", inaBusV[1], inaCurrent[1]*1000, inaPower[1]);
      MeshSerial.printf("led_load: %.2fV %.1fmA %.3fW\n", inaBusV[2], inaCurrent[2]*1000, inaPower[2]);
    }
    // statusM: motor status (name, motor, servo1, servo2, light)
    else if (cmd == "statusM") {
      MeshSerial.printf("name: %s\n", deviceName.c_str());
      MeshSerial.printf("motor: %d\n", currentMotorSpeed);
      MeshSerial.printf("servo1: %d\n", currentServo1Angle);
      MeshSerial.printf("servo2: %d\n", currentServo2Angle);

      String lightState = currentLedPwm == 255 ? "on" : currentLedPwm > 0 ? "dim" : "off";
      MeshSerial.printf("light: %s (%d)\n", lightState.c_str(), currentLedPwm);
    }
    // ina: detailed power readings
    else if (cmd == "ina") {
      for (uint8_t ch = 0; ch < 3; ch++) {
        String chName = (ch == 0) ? "Solar" : (ch == 1) ? "Battery" : "LED Load";
        MeshSerial.printf("Ch%d (%s): BusV=%.3fV ShuntV=%.1fmV I=%.1fmA P=%.3fW\n",
                          ch+1, chName.c_str(), inaBusV[ch], inaShuntV[ch]*1000, inaCurrent[ch]*1000, inaPower[ch]);
      }
    }
    // time: get current UTC time
    else if (cmd == "time") {
      MeshSerial.printf("time: %s\n", getTimeString().c_str());
    }
    // time:set:YYYY-MM-DD HH:MM:SS - set local time manually (converts to UTC for storage)
    else if (cmd.startsWith("time:set:")) {
      String timeStr = cmd.substring(9);
      int y, m, d, h, min, s;
      if (sscanf(timeStr.c_str(), "%d-%d-%d %d:%d:%d", &y, &m, &d, &h, &min, &s) == 6) {
        if (y >= 2000 && y <= 2099 && m >= 1 && m <= 12 && d >= 1 && d <= 31 &&
            h >= 0 && h <= 23 && min >= 0 && min <= 59 && s >= 0 && s <= 59) {
          // Convert local time to UTC for storage
          int8_t totalOffset = utcOffset;
          if (observeDST && isDST()) totalOffset += 1;

          int utcHour = h - totalOffset;
          int utcDay = d;
          int utcMonth = m;
          int utcYear = y;

          // Handle day/month/year rollover for UTC conversion
          if (utcHour >= 24) {
            utcHour -= 24;
            utcDay++;
            // Simple day increment
            if (utcDay > 31) {
              utcDay = 1;
              utcMonth++;
              if (utcMonth > 12) {
                utcMonth = 1;
                utcYear++;
              }
            }
          } else if (utcHour < 0) {
            utcHour += 24;
            utcDay--;
            if (utcDay < 1) {
              utcMonth--;
              if (utcMonth < 1) {
                utcMonth = 12;
                utcYear--;
              }
              utcDay = 31; // Approximation
            }
          }

          setTime(s, min, utcHour, 1, utcDay, utcMonth, utcYear - 2000);  // Default dayOfWeek=1 (Monday)
          MeshSerial.printf("time set: %s\n", getTimeString().c_str());
        } else {
          MeshSerial.println("Invalid time format");
        }
      } else {
        MeshSerial.println("Invalid time format (use: YYYY-MM-DD HH:MM:SS)");
      }
    }
    // time:sync - NTP sync (only if WiFi enabled)
    else if (cmd == "time:sync") {
      syncNTP();
    }
    // timezone:offset:X - Set UTC offset in hours (-12 to +12)
    else if (cmd.startsWith("timezone:offset:")) {
      int offset = cmd.substring(16).toInt();
      if (offset >= -12 && offset <= 12) {
        utcOffset = offset;
        saveTimezoneSettings();
        MeshSerial.printf("timezone offset → %d\n", utcOffset);
      } else {
        MeshSerial.println("Invalid offset (-12 to +12)");
      }
    }
    // timezone:dst:on/off - Enable/disable DST observation
    else if (cmd.startsWith("timezone:dst:")) {
      String state = cmd.substring(13);
      state.toLowerCase();
      if (state == "on" || state == "1") {
        observeDST = true;
        saveTimezoneSettings();
        MeshSerial.println("DST observation → on");
      } else if (state == "off" || state == "0") {
        observeDST = false;
        saveTimezoneSettings();
        MeshSerial.println("DST observation → off");
      } else {
        MeshSerial.println("Use 'on' or 'off'");
      }
    }
    // timezone - Display current timezone settings
    else if (cmd == "timezone") {
      MeshSerial.printf("timezone offset: %d hours\n", utcOffset);
      MeshSerial.printf("DST observation: %s\n", observeDST ? "on" : "off");
      MeshSerial.printf("current DST: %s\n", isDST() ? "active" : "inactive");
    }
    // statusL: Load Power Set status (mode, segment, power levels, timers)
    else if (cmd == "statusL") {
      MeshSerial.printf("name: %s\n", deviceName.c_str());
      MeshSerial.printf("mode: %s\n", currentMode == MODE_DAY ? "DAY" : "NIGHT");
      if (currentMode == MODE_NIGHT) {
        MeshSerial.printf("segment: %d\n", currentSegment + 1);
        unsigned long elapsed = millis() - segmentStartTime;
        unsigned long remaining = (unsigned long)sTime[currentSegment] * 3600000UL - elapsed;
        MeshSerial.printf("remaining: %lu min\n", remaining / 60000UL);
        bool hasRecentMotion = (millis() - lastMotionTime) < motionWindow;
        unsigned long timeSinceMotion = millis() - lastMotionTime;
        MeshSerial.printf("motion: %s (%lu ms ago)\n", hasRecentMotion ? "YES" : "NO", timeSinceMotion);
        MeshSerial.printf("power: %d%% (%s)\n", (currentLedPwm * 100) / 255,
                         hasRecentMotion ? "motion" : "idle");
      }
      MeshSerial.printf("start mode: %s\n", startMode == 0 ? "dusk" : "fixed time");
      if (startMode == 1) {
        MeshSerial.printf("start time: %02d:%02d\n", startHour, startMinute);
      }
      MeshSerial.printf("solar: %.2fV\n", inaBusV[0]);
      MeshSerial.printf("threshold: %.1fV\n", sunsetThreshold);
      MeshSerial.printf("motion window: %dms\n", motionWindow);
    }
    // parameterL: Load Power Set parameters
    else if (cmd == "parameterL") {
      MeshSerial.printf("start mode: %s\n", startMode == 0 ? "dusk" : "fixed time");
      if (startMode == 1) {
        MeshSerial.printf("start time: %02d:%02d\n", startHour, startMinute);
      }
      MeshSerial.printf("threshold: %.1fV\n", sunsetThreshold);
      MeshSerial.printf("motion window: %dms\n", motionWindow);
      for (uint8_t i = 0; i < 9; i++) {
        if (sTime[i] > 0 || sCPow[i] > 0 || sLPow[i] > 0) {
          MeshSerial.printf("S-Time-%d: %dH S-C-Pow%d: %d%% S-L-Pow%d: %d%%\n",
                           i+1, sTime[i], i+1, sCPow[i], i+1, sLPow[i]);
        }
      }
    }
    // loadpower:set:S-Time-1:2 - Set segment parameters
    else if (cmd.startsWith("loadpower:set:")) {
      String paramStr = cmd.substring(14);
      int colon1 = paramStr.indexOf(':');
      if (colon1 > 0) {
        String param = paramStr.substring(0, colon1);
        int value = paramStr.substring(colon1 + 1).toInt();

        bool valid = false;
        if (param.startsWith("S-Time-")) {
          int seg = param.substring(7).toInt() - 1;
          if (seg >= 0 && seg < 9 && value >= 0 && value <= 24) {
            sTime[seg] = value;
            valid = true;
          }
        } else if (param.startsWith("S-C-Pow")) {
          int seg = param.substring(8).toInt() - 1;
          if (seg >= 0 && seg < 9 && value >= 0 && value <= 100) {
            sCPow[seg] = value;
            valid = true;
          }
        } else if (param.startsWith("S-L-Pow")) {
          int seg = param.substring(8).toInt() - 1;
          if (seg >= 0 && seg < 9 && value >= 0 && value <= 100) {
            sLPow[seg] = value;
            valid = true;
          }
        }

        if (valid) {
          saveLoadPowerSettings();
          MeshSerial.printf("%s → %d\n", param.c_str(), value);
        } else {
          MeshSerial.println("Invalid parameter or value");
        }
      }
    }
    // loadpower - Display Load Power Set settings (compact, <200 bytes)
    else if (cmd == "loadpower") {
      MeshSerial.printf("LPS: %.1fV", sunsetThreshold);
      for (uint8_t i = 0; i < 9; i++) {
        if (sTime[i] > 0 || sCPow[i] > 0 || sLPow[i] > 0) {
          MeshSerial.printf(" S%d:%dH-%d%%-%d%%", i+1, sTime[i], sCPow[i], sLPow[i]);
        }
      }
      MeshSerial.printf("\n");
    }
    // sunset:threshold:4.0 - Set voltage threshold
    else if (cmd.startsWith("sunset:threshold:")) {
      float threshold = cmd.substring(17).toFloat();
      if (threshold >= 1.0 && threshold <= 10.0) {
        sunsetThreshold = threshold;
        saveLoadPowerSettings();
        MeshSerial.printf("sunset threshold → %.1fV\n", sunsetThreshold);
      } else {
        MeshSerial.println("Invalid threshold (1.0-10.0V)");
      }
    }
    // motion:window:5000 - Set motion detection window in milliseconds
    else if (cmd.startsWith("motion:window:")) {
      uint16_t window = cmd.substring(13).toInt();
      if (window >= 1000 && window <= 30000) {  // 1-30 seconds
        motionWindow = window;
        saveLoadPowerSettings();
        MeshSerial.printf("motion window → %dms\n", motionWindow);
      } else {
        MeshSerial.println("Invalid window (1000-30000ms)");
      }
    }
    // startmode:0 or startmode:1 - Set start mode (0=dusk, 1=fixed time)
    else if (cmd.startsWith("startmode:")) {
      uint8_t mode = cmd.substring(10).toInt();
      if (mode == 0 || mode == 1) {
        startMode = mode;
        saveLoadPowerSettings();
        MeshSerial.printf("start mode → %s\n", startMode == 0 ? "dusk" : "fixed time");
      } else {
        MeshSerial.println("Invalid mode (0=dusk, 1=fixed time)");
      }
    }
    // starttime:18:30 - Set fixed start time (HH:MM)
    else if (cmd.startsWith("starttime:")) {
      String timeStr = cmd.substring(10);
      int colon = timeStr.indexOf(':');
      if (colon > 0) {
        int hour = timeStr.substring(0, colon).toInt();
        int minute = timeStr.substring(colon + 1).toInt();
        if (hour >= 0 && hour <= 23 && minute >= 0 && minute <= 59) {
          startHour = hour;
          startMinute = minute;
          saveLoadPowerSettings();
          MeshSerial.printf("start time → %02d:%02d\n", startHour, startMinute);
        } else {
          MeshSerial.println("Invalid time (00:00-23:59)");
        }
      } else {
        MeshSerial.println("Invalid format (use HH:MM)");
      }
    }
    // startmode - Get current start mode
    else if (cmd == "startmode") {
      MeshSerial.printf("start mode: %s\n", startMode == 0 ? "dusk" : "fixed time");
    }
    // starttime - Get current start time
    else if (cmd == "starttime") {
      MeshSerial.printf("start time: %02d:%02d\n", startHour, startMinute);
    }
    // debug:lps - Debug Load Power Set state
    else if (cmd == "debug:lps") {
      MeshSerial.printf("LPS Debug:\n");
      MeshSerial.printf("Mode: %s\n", currentMode == MODE_DAY ? "DAY" : "NIGHT");
      MeshSerial.printf("Segment: %d\n", currentSegment + 1);
      MeshSerial.printf("Motion now: %s\n", digitalRead(PIR_PIN) == HIGH ? "YES" : "NO");
      MeshSerial.printf("Last motion: %lu ms ago\n", millis() - lastMotionTime);
      MeshSerial.printf("Motion window: %d ms\n", motionWindow);
      MeshSerial.printf("Recent motion: %s\n", (millis() - lastMotionTime) < motionWindow ? "YES" : "NO");
      MeshSerial.printf("Target power: %d%% (%d PWM)\n",
                       (millis() - lastMotionTime) < motionWindow ? sCPow[currentSegment] : sLPow[currentSegment],
                       ((millis() - lastMotionTime) < motionWindow ? sCPow[currentSegment] : sLPow[currentSegment]) * 255 / 100);
      MeshSerial.printf("Current PWM: %d\n", currentLedPwm);
      MeshSerial.printf("Solar voltage: %.2fV\n", inaBusV[0]);
      MeshSerial.printf("Threshold: %.1fV\n", sunsetThreshold);
      MeshSerial.printf("Sunset detected: %s\n", sunsetDetected ? "YES" : "NO");
      MeshSerial.printf("Dawn detected: %s\n", dawnDetected ? "YES" : "NO");
    }
  }
}