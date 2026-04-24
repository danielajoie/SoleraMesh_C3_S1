/*
Project name: MeshTruck_C3_S1 - Meshtastic-integrated ESP32-C3 RC vehicle Controller
Board: seeed_xiao_esp32c3
Hardware/pins:
- Servo1: GPIO 2 (ESP32Servo library) <center = Right; >center = Left
- Servo2: GPIO 3 (ESP32Servo library)
- LED: GPIO 7 (LEDC channel 2)
- Motor AIN1: GPIO 0
- Motor AIN2: GPIO 1
- PIR: GPIO 4 (digital input)
- UART: GPIO 21 (RX) / GPIO 20 (TX) for Meshtastic
- I2C: GPIO 8 (SDA) / GPIO 9 (SCL) for INA3221
Libraries: WiFi, ArduinoOTA, Preferences, Wire
Date: April 2026
*/

#include <Arduino.h>
#include <WiFi.h>
#include <ArduinoOTA.h>
#include <Preferences.h>
#include <Wire.h>
#include <Adafruit_AHTX0.h>
#include <QMC5883LCompass.h>
#include <ESP32Servo.h>

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
const int SERVO1_PIN = 2;
const int SERVO2_PIN = 3;
const int LED_PIN    = 7;
const int LEDC_CHANNEL = 2;
const int LEDC_FREQ = 5000;
const int LEDC_RES = 8;

Servo servo1;
Servo servo2;

const int MOTOR_AIN1 = 0;
const int MOTOR_AIN2 = 1;

const int UART_RX_PIN = 21;   // ESP32-C3 RX ← Meshtastic TX 21
const int UART_TX_PIN = 20;   // ESP32-C3 TX → Meshtastic RX 19

const int PIR_PIN = 4;       // PIR motion sensor
const int SHAFT_SENSOR_PIN = 5; // TCRT5000 shaft rotation sensor
const int RELAY_PIN = 10;    // Solar charging relay control

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

// Enhanced S-Time structure supporting both timer and RTC modes
struct SegmentTime {
  bool isRTCTime;      // true = HH:MM format, false = hours timer
  union {
    uint8_t hours;     // 0-24 for timer mode
    struct {
      uint8_t hour;    // 0-23 for RTC mode
      uint8_t minute;  // 0-59 for RTC mode
    } rtcTime;
  } value;
};

// Initialize with default values (timer mode)
SegmentTime sTime[9] = {
  {false, {2}},   // Segment 1: 2 hours timer
  {false, {1}},   // Segment 2: 1 hour timer
  {false, {0}}, {false, {0}}, {false, {0}}, {false, {0}}, {false, {0}}, {false, {0}}, {false, {0}}
};

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
const char* FIRMWARE_VERSION = "MT_C3_260421_Compass_Nav-rpm";

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

// Servo configuration variables
int servo1Min = 0;           // Servo1 minimum angle
int servo1Max = 180;         // Servo1 maximum angle
int servo1Center = 90;       // Servo1 center/default position
bool servo1Invert = false;   // Servo1 direction inversion
int servo2Min = 0;           // Servo2 minimum angle
int servo2Max = 180;         // Servo2 maximum angle
int servo2Center = 90;       // Servo2 center/default position
bool servo2Invert = false;   // Servo2 direction inversion
unsigned long lastWiFiReconnectAttempt = 0;
const unsigned long WIFI_RECONNECT_INTERVAL = 30000UL; // 30 seconds
bool wifiEnabled = false;    // WiFi power saving flag

// UART & Preferences
HardwareSerial MeshSerial(1);
bool otaInitialized = false;
Preferences prefs;

// AHT30 temperature/humidity sensor
Adafruit_AHTX0 aht30;
float currentTemperature = 0.0;
float currentHumidity = 0.0;
bool aht30Present = false;

// QMC5883L compass sensor
QMC5883LCompass compass;
float currentHeading = 0.0;
bool compassPresent = false;
bool compassCalibrating = false;
int compassCalMinX = 32767, compassCalMaxX = -32768;
int compassCalMinY = 32767, compassCalMaxY = -32768;
int compassCalMinZ = 32767, compassCalMaxZ = -32768;
unsigned long compassReadIntervalNav = 500;   // Fast refresh during navigation (ms)
unsigned long compassReadIntervalIdle = 5000; // Slow refresh when idle (ms)

// Compass Navigation mode
bool navigationMode = false;
float targetHeading = 0.0;
int navigationSteeringGain = 2;
float navigationDeadband = 2.0;
unsigned long navigationUpdateInterval = 25; // milliseconds (default 25ms for fast response)

// Battery charging control
enum BatteryChemistry { BAT_LI_ION, BAT_LEAD_ACID };
BatteryChemistry batteryChemistry = BAT_LI_ION;  // Default to Li-ion
float chargeTempMin = 0.0;    // Minimum temperature for charging (°C)
float chargeTempMax = 45.0;   // Maximum temperature for charging (°C)
bool chargingEnabled = false; // Current charging state
enum ChargingStatus { CHG_DISABLED, CHG_ENABLED, CHG_ERROR_TEMP_LOW, CHG_ERROR_TEMP_HIGH, CHG_ERROR_SENSOR };
ChargingStatus chargingStatus = CHG_DISABLED;
unsigned long lastChargingStatusChange = 0;

// Shaft rotation sensor (TCRT5000) - RPM and motion detection
volatile unsigned long shaftPulseCount = 0;  // Interrupt counter for pulses
unsigned long lastShaftPulseTime = 0;        // Timestamp of last pulse
float currentRPM = 0.0;                      // Calculated RPM
bool vehicleMoving = false;                  // Motion detection state
unsigned long lastRPMCalculation = 0;        // When RPM was last calculated
const unsigned long RPM_CALCULATION_INTERVAL = 1000; // Calculate RPM every second
const unsigned long MOTION_TIMEOUT = 3000;   // Consider stopped if no pulses for 3 seconds
int pulsesPerRevolution = 2;                 // Pulses per revolution (2 white lines)

// Interrupt service routine for shaft sensor
void IRAM_ATTR shaftSensorISR() {
  shaftPulseCount++;
  lastShaftPulseTime = millis();
}

void setup() {
  Serial.begin(115200);
  MeshSerial.begin(115200, SERIAL_8N1, UART_RX_PIN, UART_TX_PIN);



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
  // For now, load as simple timer values (backward compatibility)
  // TODO: Implement proper SegmentTime serialization
  for (uint8_t i = 0; i < 9; i++) {
    uint8_t hours = prefs.getUChar(("time" + String(i+1)).c_str(), (i < 2) ? ((i == 0) ? 2 : 1) : 0);
    sTime[i].isRTCTime = false;  // Default to timer mode
    sTime[i].value.hours = hours;
    sCPow[i] = prefs.getUChar(("cpow" + String(i+1)).c_str(), (i < 2) ? 80 : 0);
    sLPow[i] = prefs.getUChar(("lpow" + String(i+1)).c_str(), (i == 0) ? 10 : (i == 1) ? 20 : 0);
  }
  prefs.end();

  // Load charging settings
  prefs.begin("charging", false);
  batteryChemistry = (BatteryChemistry)prefs.getUChar("chemistry", BAT_LI_ION);
  chargeTempMin = prefs.getFloat("tempmin", 0.0);
  chargeTempMax = prefs.getFloat("tempmax", 45.0);
  prefs.end();

  // Load servo configuration settings
  prefs.begin("servo", false);
  servo1Min = prefs.getInt("s1min", 0);
  servo1Max = prefs.getInt("s1max", 180);
  servo1Center = prefs.getInt("s1center", 90);
  servo1Invert = prefs.getBool("s1invert", false);
  servo2Min = prefs.getInt("s2min", 0);
  servo2Max = prefs.getInt("s2max", 180);
  servo2Center = prefs.getInt("s2center", 90);
  servo2Invert = prefs.getBool("s2invert", false);
  prefs.end();

  // Load navigation and compass settings
  prefs.begin("navigation", false);
  compassReadIntervalNav = prefs.getULong("compass_nav", 500);
  compassReadIntervalIdle = prefs.getULong("compass_idle", 5000);
  navigationSteeringGain = prefs.getInt("gain", 2);
  navigationDeadband = prefs.getFloat("deadband", 2.0);
  navigationUpdateInterval = prefs.getULong("interval", 25);
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
  pinMode(SHAFT_SENSOR_PIN, INPUT);  // Shaft rotation sensor input
  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, LOW);  // Start with charging disabled

  // Setup interrupt for shaft sensor (TCRT5000)
  attachInterrupt(digitalPinToInterrupt(SHAFT_SENSOR_PIN), shaftSensorISR, RISING);

  // Initialize servos first (should get LEDC channels 0,1)
  servo1.attach(SERVO1_PIN);
  servo2.attach(SERVO2_PIN);
  servo1.write(servo1Center);
  servo2.write(servo2Center);
  currentServo1Angle = servo1Center;
  currentServo2Angle = servo2Center;

  // Setup LEDC for LED PWM (should get channel 2)
  ledcSetup(LEDC_CHANNEL, LEDC_FREQ, LEDC_RES);
  ledcAttachPin(LED_PIN, LEDC_CHANNEL);
  ledcWrite(LEDC_CHANNEL, 0);

  analogWrite(MOTOR_AIN1, 0);
  analogWrite(MOTOR_AIN2, 0);

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

  // Initialize AHT30 temperature/humidity sensor
  if (aht30.begin()) {
    aht30Present = true;
    Serial.println("AHT30 sensor found");

    // Initial reading
    sensors_event_t humidity, temp;
    aht30.getEvent(&humidity, &temp);
    currentTemperature = temp.temperature;
    currentHumidity = humidity.relative_humidity;
    Serial.printf("AHT30 Initial: %.1f°C, %.1f%%\n", currentTemperature, currentHumidity);
  } else {
    Serial.println("AHT30 sensor not found");
  }

  // Initialize QMC5883L compass sensor
  compass.init();
  compassPresent = true;
  Serial.println("QMC5883L compass initialized");

  // Initial reading
  compass.read();
  currentHeading = compass.getAzimuth();
  // Normalize to 0-360° range
  if (currentHeading < 0) currentHeading += 360.0;
  Serial.printf("Compass Initial: %.1f°\n", currentHeading);
}

void setServoAngle(int channel, int angle) {
  // Apply direction inversion if enabled
  int actualAngle = angle;
  if (channel == 0 && servo1Invert) {
    actualAngle = servo1Center - (angle - servo1Center);
  } else if (channel == 1 && servo2Invert) {
    actualAngle = servo2Center - (angle - servo2Center);
  }

  Serial.println("Setting servo " + String(channel) + " to angle " + String(angle) + " (actual: " + String(actualAngle) + ")");
  if (channel == 0) {
    servo1.write(actualAngle);
  } else if (channel == 1) {
    servo2.write(actualAngle);
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
  // For now, save as simple timer values (backward compatibility)
  // TODO: Implement proper SegmentTime serialization
  for (uint8_t i = 0; i < 9; i++) {
    uint8_t hours = sTime[i].isRTCTime ? 0 : sTime[i].value.hours;  // Save 0 for RTC times
    prefs.putUChar(("time" + String(i+1)).c_str(), hours);
    prefs.putUChar(("cpow" + String(i+1)).c_str(), sCPow[i]);
    prefs.putUChar(("lpow" + String(i+1)).c_str(), sLPow[i]);
  }
  prefs.end();
}

void saveServoSettings() {
  prefs.begin("servo", false);
  prefs.putInt("s1min", servo1Min);
  prefs.putInt("s1max", servo1Max);
  prefs.putInt("s1center", servo1Center);
  prefs.putBool("s1invert", servo1Invert);
  prefs.putInt("s2min", servo2Min);
  prefs.putInt("s2max", servo2Max);
  prefs.putInt("s2center", servo2Center);
  prefs.putBool("s2invert", servo2Invert);
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
  static unsigned long lastAHTRead = 0;
  static unsigned long lastCompassRead = 0;

  if (millis() - lastINARead >= 5000) {  // Read every 5 seconds
    readINA3221();
    lastINARead = millis();
  }

  if (millis() - lastTimeRead >= 1000) {  // Read time every second
    readTime();
    lastTimeRead = millis();
  }

  if (millis() - lastAHTRead >= 10000 && aht30Present) {  // Read every 10 seconds
    sensors_event_t humidity, temp;
    aht30.getEvent(&humidity, &temp);
    currentTemperature = temp.temperature;
    currentHumidity = humidity.relative_humidity;
    Serial.printf("AHT30: %.1f°C, %.1f%%\n", currentTemperature, currentHumidity);

    // Temperature-aware charging control
    ChargingStatus newStatus = CHG_DISABLED;
    if (!aht30Present) {
      newStatus = CHG_ERROR_SENSOR;
    } else if (currentTemperature < chargeTempMin) {
      newStatus = CHG_ERROR_TEMP_LOW;
    } else if (currentTemperature > chargeTempMax) {
      newStatus = CHG_ERROR_TEMP_HIGH;
    } else {
      newStatus = CHG_ENABLED;
    }

    // Update charging state if status changed
    if (newStatus != chargingStatus) {
      chargingStatus = newStatus;
      lastChargingStatusChange = millis();

      switch (chargingStatus) {
        case CHG_ENABLED:
          digitalWrite(RELAY_PIN, HIGH);
          chargingEnabled = true;
          Serial.printf("Charging ENABLED (Temp: %.1f°C in range %.1f-%.1f°C)\n",
                       currentTemperature, chargeTempMin, chargeTempMax);
          break;
        case CHG_ERROR_TEMP_LOW:
          digitalWrite(RELAY_PIN, LOW);
          chargingEnabled = false;
          Serial.printf("Charging DISABLED - Temperature too LOW (%.1f°C < %.1f°C)\n",
                       currentTemperature, chargeTempMin);
          break;
        case CHG_ERROR_TEMP_HIGH:
          digitalWrite(RELAY_PIN, LOW);
          chargingEnabled = false;
          Serial.printf("Charging DISABLED - Temperature too HIGH (%.1f°C > %.1f°C)\n",
                       currentTemperature, chargeTempMax);
          break;
        case CHG_ERROR_SENSOR:
          digitalWrite(RELAY_PIN, LOW);
          chargingEnabled = false;
          Serial.println("Charging DISABLED - Temperature sensor not available");
          break;
        default:
          digitalWrite(RELAY_PIN, LOW);
          chargingEnabled = false;
          Serial.println("Charging DISABLED - Unknown error");
          break;
      }
    }

    lastAHTRead = millis();
  }

  // Dynamic compass refresh rate based on navigation mode
  unsigned long currentCompassInterval = navigationMode ? compassReadIntervalNav : compassReadIntervalIdle;
  if (millis() - lastCompassRead >= currentCompassInterval && compassPresent) {
    compass.read();
    currentHeading = compass.getAzimuth();
    // Normalize to 0-360° range
    if (currentHeading < 0) currentHeading += 360.0;

    if (compassCalibrating) {
      // Collect min/max values for calibration
      int x = compass.getX();
      int y = compass.getY();
      int z = compass.getZ();
      if (x < compassCalMinX) compassCalMinX = x;
      if (x > compassCalMaxX) compassCalMaxX = x;
      if (y < compassCalMinY) compassCalMinY = y;
      if (y > compassCalMaxY) compassCalMaxY = y;
      if (z < compassCalMinZ) compassCalMinZ = z;
      if (z > compassCalMaxZ) compassCalMaxZ = z;
      Serial.printf("Calibrating: X:%d Y:%d Z:%d (Min:%d,%d,%d Max:%d,%d,%d)\n",
                   x, y, z, compassCalMinX, compassCalMinY, compassCalMinZ,
                   compassCalMaxX, compassCalMaxY, compassCalMaxZ);
    } else {
      Serial.printf("Compass: %.1f° X:%d Y:%d Z:%d\n", currentHeading, compass.getX(), compass.getY(), compass.getZ());
    }

    lastCompassRead = millis();
  }

  // Navigation steering control (runs every navigationUpdateInterval ms when active)
  static unsigned long lastNavigationUpdate = 0;
  if (millis() - lastNavigationUpdate >= navigationUpdateInterval && navigationMode) {
    // Check compass availability
    if (!compassPresent) {
      // Emergency stop - compass failed during navigation
      analogWrite(MOTOR_AIN1, 0);
      analogWrite(MOTOR_AIN2, 0);
      currentMotorSpeed = 0;
      navigationMode = false;
      MeshSerial.println("NAVIGATION ERROR: Compass not available - motor stopped!");
      Serial.println("Navigation emergency stop: compass unavailable");
    } else {
      // Calculate shortest angular error (handles 360° wrap-around)
      float error = targetHeading - currentHeading;
      if (error > 180.0) error -= 360.0;
      if (error < -180.0) error += 360.0;

      // Apply deadband
      if (abs(error) < navigationDeadband) {
        // Within deadband - steer straight
        setServoAngle(0, servo1Center);
        currentServo1Angle = servo1Center;
      } else {
        // Outside deadband - apply proportional steering with direction correction
        // Invert steering when motor is in reverse (front-wheel steering vehicle)
        int steeringDirection = (currentMotorSpeed >= 0) ? 1 : -1;
        int steeringAngle = servo1Center + (error * navigationSteeringGain * steeringDirection);
        // Clamp to servo limits
        steeringAngle = constrain(steeringAngle, servo1Min, servo1Max);
        setServoAngle(0, steeringAngle);
        currentServo1Angle = steeringAngle;
      }
    }

    lastNavigationUpdate = millis();
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
    bool segmentExpired = false;

    if (sTime[currentSegment].isRTCTime) {
      // RTC time mode - check if current time >= target time
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

      // Check if current time >= target time
      if (localHour > sTime[currentSegment].value.rtcTime.hour ||
          (localHour == sTime[currentSegment].value.rtcTime.hour &&
           localMinute >= sTime[currentSegment].value.rtcTime.minute)) {
        segmentExpired = true;
      }
    } else {
      // Timer mode - check elapsed time
      unsigned long segmentElapsed = millis() - segmentStartTime;
      unsigned long segmentDurationMs = (unsigned long)sTime[currentSegment].value.hours * 3600000UL;
      if (segmentElapsed >= segmentDurationMs) {
        segmentExpired = true;
      }
    }

    // Check for zero/empty segment (end of schedule)
    bool isEmptySegment = (!sTime[currentSegment].isRTCTime && sTime[currentSegment].value.hours == 0);

    if (segmentExpired || isEmptySegment) {
      // Move to next segment or end schedule
      currentSegment++;
      if (currentSegment >= 9 || (!sTime[currentSegment].isRTCTime && sTime[currentSegment].value.hours == 0)) {
        // End of schedule - turn off light and send completion message
        currentMode = MODE_DAY;
        ledcWrite(LEDC_CHANNEL, 0);
        currentLedPwm = 0;
        Serial.println("Load Power Set schedule completed");
        MeshSerial.println("All segments complete");
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

  // Shaft sensor RPM calculation and motion detection
  if (millis() - lastRPMCalculation >= RPM_CALCULATION_INTERVAL) {
    // Calculate RPM: pulses per second * 60 / pulses per revolution
    // We have 1 pulse per revolution, so RPM = (pulses in last second) * 60
    noInterrupts();
    unsigned long pulseCount = shaftPulseCount;
    shaftPulseCount = 0;  // Reset counter
    interrupts();

    // Calculate RPM: pulses per second * 60 / pulses per revolution
    currentRPM = (pulseCount * 60.0) / (RPM_CALCULATION_INTERVAL / 1000.0) / pulsesPerRevolution;

    // Motion detection: vehicle is moving if we got pulses recently
    vehicleMoving = (millis() - lastShaftPulseTime) < MOTION_TIMEOUT;

    Serial.printf("Shaft: %.1f RPM, %s\n", currentRPM, vehicleMoving ? "MOVING" : "STOPPED");

    lastRPMCalculation = millis();
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
      String subCmd = cmd.substring(7);
      if (subCmd.startsWith("min:")) {
        int newMin = subCmd.substring(4).toInt();
        if (newMin >= 0 && newMin <= 180 && newMin < servo1Max) {
          servo1Min = newMin;
          if (servo1Center < servo1Min) servo1Center = servo1Min;
          saveServoSettings();
          MeshSerial.printf("servo1 min → %d\n", servo1Min);
        } else {
          MeshSerial.println("Invalid servo1 min (0-180, < max)");
        }
      } else if (subCmd.startsWith("max:")) {
        int newMax = subCmd.substring(4).toInt();
        if (newMax >= 0 && newMax <= 180 && newMax > servo1Min) {
          servo1Max = newMax;
          if (servo1Center > servo1Max) servo1Center = servo1Max;
          saveServoSettings();
          MeshSerial.printf("servo1 max → %d\n", servo1Max);
        } else {
          MeshSerial.println("Invalid servo1 max (0-180, > min)");
        }
      } else if (subCmd.startsWith("center:")) {
        int newCenter = subCmd.substring(7).toInt();
        if (newCenter >= servo1Min && newCenter <= servo1Max) {
          servo1Center = newCenter;
          saveServoSettings();
          MeshSerial.printf("servo1 center → %d\n", servo1Center);
        } else {
          MeshSerial.printf("Invalid servo1 center (%d-%d)\n", servo1Min, servo1Max);
        }
      } else if (subCmd.startsWith("invert:")) {
        String state = subCmd.substring(7);
        state.toLowerCase();
        if (state == "on" || state == "1") {
          servo1Invert = true;
          saveServoSettings();
          MeshSerial.println("servo1 invert → on");
        } else if (state == "off" || state == "0") {
          servo1Invert = false;
          saveServoSettings();
          MeshSerial.println("servo1 invert → off");
        } else {
          MeshSerial.println("Use 'on' or 'off'");
        }
      } else if (subCmd == "invert") {
        MeshSerial.printf("servo1 invert: %s\n", servo1Invert ? "on" : "off");
      } else if (subCmd == "center") {
        // Return servo to center position
        setServoAngle(0, servo1Center);
        currentServo1Angle = servo1Center;
        MeshSerial.printf("servo1 → center (%d)\n", servo1Center);
      } else {
        // Direct angle control with clamping
        int angle = subCmd.toInt();
        int clampedAngle = constrain(angle, servo1Min, servo1Max);
        setServoAngle(0, clampedAngle);
        currentServo1Angle = clampedAngle;
        if (clampedAngle != angle) {
          MeshSerial.printf("servo1 → %d*\n", clampedAngle);
        } else {
          MeshSerial.printf("servo1 → %d\n", clampedAngle);
        }
      }
    }
    else if (cmd.startsWith("servo2:")) {
      String subCmd = cmd.substring(7);
      if (subCmd.startsWith("min:")) {
        int newMin = subCmd.substring(4).toInt();
        if (newMin >= 0 && newMin <= 180 && newMin < servo2Max) {
          servo2Min = newMin;
          if (servo2Center < servo2Min) servo2Center = servo2Min;
          saveServoSettings();
          MeshSerial.printf("servo2 min → %d\n", servo2Min);
        } else {
          MeshSerial.println("Invalid servo2 min (0-180, < max)");
        }
      } else if (subCmd.startsWith("max:")) {
        int newMax = subCmd.substring(4).toInt();
        if (newMax >= 0 && newMax <= 180 && newMax > servo2Min) {
          servo2Max = newMax;
          if (servo2Center > servo2Max) servo2Center = servo2Max;
          saveServoSettings();
          MeshSerial.printf("servo2 max → %d\n", servo2Max);
        } else {
          MeshSerial.println("Invalid servo2 max (0-180, > min)");
        }
      } else if (subCmd.startsWith("center:")) {
        int newCenter = subCmd.substring(7).toInt();
        if (newCenter >= servo2Min && newCenter <= servo2Max) {
          servo2Center = newCenter;
          saveServoSettings();
          MeshSerial.printf("servo2 center → %d\n", servo2Center);
        } else {
          MeshSerial.printf("Invalid servo2 center (%d-%d)\n", servo2Min, servo2Max);
        }
      } else if (subCmd.startsWith("invert:")) {
        String state = subCmd.substring(7);
        state.toLowerCase();
        if (state == "on" || state == "1") {
          servo2Invert = true;
          saveServoSettings();
          MeshSerial.println("servo2 invert → on");
        } else if (state == "off" || state == "0") {
          servo2Invert = false;
          saveServoSettings();
          MeshSerial.println("servo2 invert → off");
        } else {
          MeshSerial.println("Use 'on' or 'off'");
        }
      } else if (subCmd == "invert") {
        MeshSerial.printf("servo2 invert: %s\n", servo2Invert ? "on" : "off");
      } else if (subCmd == "center") {
        // Return servo to center position
        setServoAngle(1, servo2Center);
        currentServo2Angle = servo2Center;
        MeshSerial.printf("servo2 → center (%d)\n", servo2Center);
      } else {
        // Direct angle control with clamping
        int angle = subCmd.toInt();
        int clampedAngle = constrain(angle, servo2Min, servo2Max);
        setServoAngle(1, clampedAngle);
        currentServo2Angle = clampedAngle;
        if (clampedAngle != angle) {
          MeshSerial.printf("servo2 → %d*\n", clampedAngle);
        } else {
          MeshSerial.printf("servo2 → %d\n", clampedAngle);
        }
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

      // Charging status
      String chgStatus;
      switch (chargingStatus) {
        case CHG_ENABLED: chgStatus = "ON"; break;
        case CHG_ERROR_TEMP_LOW: chgStatus = "TEMP_LOW"; break;
        case CHG_ERROR_TEMP_HIGH: chgStatus = "TEMP_HIGH"; break;
        case CHG_ERROR_SENSOR: chgStatus = "SENSOR_ERROR"; break;
        default: chgStatus = "OFF"; break;
      }
      MeshSerial.printf("charging: %s %.1f°C\n", chgStatus.c_str(), currentTemperature);
    }
    // statusP: power status (name, solar, battery, led_load)
    else if (cmd == "statusP") {
      MeshSerial.printf("name: %s\n", deviceName.c_str());
      MeshSerial.printf("solar: %.2fV %.1fmA %.3fW\n", inaBusV[0], inaCurrent[0]*1000, inaPower[0]);
      MeshSerial.printf("battery: %.2fV %.1fmA %.3fW\n", inaBusV[1], inaCurrent[1]*1000, inaPower[1]);
      MeshSerial.printf("led_load: %.2fV %.1fmA %.3fW\n", inaBusV[2], inaCurrent[2]*1000, inaPower[2]);
    }
    // statusM: motor status (name, motor, servo1, servo2, light, rpm, motion)
    else if (cmd == "statusM") {
      MeshSerial.printf("name: %s\n", deviceName.c_str());
      MeshSerial.printf("motor: %d\n", currentMotorSpeed);
      MeshSerial.printf("servo1: %d (%d-%d, center:%d, invert:%s)\n", currentServo1Angle, servo1Min, servo1Max, servo1Center, servo1Invert ? "on" : "off");
      MeshSerial.printf("servo2: %d (%d-%d, center:%d, invert:%s)\n", currentServo2Angle, servo2Min, servo2Max, servo2Center, servo2Invert ? "on" : "off");

      String lightState = currentLedPwm == 255 ? "on" : currentLedPwm > 0 ? "dim" : "off";
      MeshSerial.printf("light: %s (%d)\n", lightState.c_str(), currentLedPwm);

      MeshSerial.printf("rpm: %.1f\n", currentRPM);
      MeshSerial.printf("motion: %s\n", vehicleMoving ? "MOVING" : "STOPPED");
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

        // Calculate remaining time based on segment type
        String remainingStr;
        if (sTime[currentSegment].isRTCTime) {
          // RTC mode - show target time
          remainingStr = String("until ") + (sTime[currentSegment].value.rtcTime.hour < 10 ? "0" : "") +
                        String(sTime[currentSegment].value.rtcTime.hour) + ":" +
                        (sTime[currentSegment].value.rtcTime.minute < 10 ? "0" : "") +
                        String(sTime[currentSegment].value.rtcTime.minute);
        } else {
          // Timer mode - show remaining minutes
          unsigned long elapsed = millis() - segmentStartTime;
          unsigned long segmentDurationMs = (unsigned long)sTime[currentSegment].value.hours * 3600000UL;
          unsigned long remaining = segmentDurationMs - elapsed;
          remainingStr = String(remaining / 60000UL) + " min";
        }
        MeshSerial.printf("remaining: %s\n", remainingStr.c_str());

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
        bool hasData = false;
        if (sTime[i].isRTCTime) {
          hasData = true;
        } else if (sTime[i].value.hours > 0) {
          hasData = true;
        }
        if (hasData || sCPow[i] > 0 || sLPow[i] > 0) {
          String timeStr;
          if (sTime[i].isRTCTime) {
            timeStr = String(sTime[i].value.rtcTime.hour < 10 ? "0" : "") +
                     String(sTime[i].value.rtcTime.hour) + ":" +
                     String(sTime[i].value.rtcTime.minute < 10 ? "0" : "") +
                     String(sTime[i].value.rtcTime.minute);
          } else {
            timeStr = String(sTime[i].value.hours) + "H";
          }
          MeshSerial.printf("S-Time-%d: %s S-C-Pow%d: %d%% S-L-Pow%d: %d%%\n",
                           i+1, timeStr.c_str(), i+1, sCPow[i], i+1, sLPow[i]);
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
          if (seg >= 0 && seg < 9) {
            // Check if value contains ':' (HH:MM format) or is plain number (hours)
            String valStr = paramStr.substring(colon1 + 1);
            int colonPos = valStr.indexOf(':');
            if (colonPos > 0) {
              // RTC time format HH:MM
              int hour = valStr.substring(0, colonPos).toInt();
              int minute = valStr.substring(colonPos + 1).toInt();
              if (hour >= 0 && hour <= 23 && minute >= 0 && minute <= 59) {
                sTime[seg].isRTCTime = true;
                sTime[seg].value.rtcTime.hour = hour;
                sTime[seg].value.rtcTime.minute = minute;
                valid = true;
                MeshSerial.printf("S-Time-%d → %02d:%02d\n", seg+1, hour, minute);
              }
            } else {
              // Timer format (hours)
              int hours = valStr.toInt();
              if (hours >= 0 && hours <= 24) {
                sTime[seg].isRTCTime = false;
                sTime[seg].value.hours = hours;
                valid = true;
                MeshSerial.printf("S-Time-%d → %dH\n", seg+1, hours);
              }
            }
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
        bool hasData = false;
        if (sTime[i].isRTCTime) {
          hasData = true;
        } else if (sTime[i].value.hours > 0) {
          hasData = true;
        }
        if (hasData || sCPow[i] > 0 || sLPow[i] > 0) {
          String timeStr;
          if (sTime[i].isRTCTime) {
            timeStr = String(sTime[i].value.rtcTime.hour < 10 ? "0" : "") +
                     String(sTime[i].value.rtcTime.hour) + ":" +
                     String(sTime[i].value.rtcTime.minute < 10 ? "0" : "") +
                     String(sTime[i].value.rtcTime.minute);
          } else {
            timeStr = String(sTime[i].value.hours) + "H";
          }
          MeshSerial.printf(" S%d:%s-%d%%-%d%%", i+1, timeStr.c_str(), sCPow[i], sLPow[i]);
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
    // charging:chemistry:li-ion or charging:chemistry:lead-acid - Set battery chemistry
    else if (cmd.startsWith("charging:chemistry:")) {
      String chemistry = cmd.substring(19);
      chemistry.toLowerCase();
      if (chemistry == "li-ion" || chemistry == "li_ion") {
        batteryChemistry = BAT_LI_ION;
        chargeTempMin = 0.0;
        chargeTempMax = 45.0;
        // Save to preferences
        prefs.begin("charging", false);
        prefs.putUChar("chemistry", batteryChemistry);
        prefs.putFloat("tempmin", chargeTempMin);
        prefs.putFloat("tempmax", chargeTempMax);
        prefs.end();
        MeshSerial.printf("battery chemistry → Li-ion (0-45°C)\n");
      } else if (chemistry == "lead-acid" || chemistry == "lead_acid") {
        batteryChemistry = BAT_LEAD_ACID;
        chargeTempMin = -20.0;
        chargeTempMax = 50.0;
        // Save to preferences
        prefs.begin("charging", false);
        prefs.putUChar("chemistry", batteryChemistry);
        prefs.putFloat("tempmin", chargeTempMin);
        prefs.putFloat("tempmax", chargeTempMax);
        prefs.end();
        MeshSerial.printf("battery chemistry → Lead-acid (-20-50°C)\n");
      } else {
        MeshSerial.println("Use 'li-ion' or 'lead-acid'");
      }
    }
    // charging:chemistry - Get current battery chemistry
    else if (cmd == "charging:chemistry") {
      String chemName = (batteryChemistry == BAT_LI_ION) ? "Li-ion" : "Lead-acid";
      MeshSerial.printf("battery chemistry: %s\n", chemName.c_str());
    }
    // charging:tempmin:VALUE - Set minimum charging temperature
    else if (cmd.startsWith("charging:tempmin:")) {
      float temp = cmd.substring(18).toFloat();
      if (temp >= -50.0 && temp <= 50.0 && temp < chargeTempMax) {
        chargeTempMin = temp;
        prefs.begin("charging", false);
        prefs.putFloat("tempmin", chargeTempMin);
        prefs.end();
        MeshSerial.printf("charge temp min → %.1f°C\n", chargeTempMin);
      } else {
        MeshSerial.println("Invalid temp (-50 to 50°C, < max)");
      }
    }
    // charging:tempmax:VALUE - Set maximum charging temperature
    else if (cmd.startsWith("charging:tempmax:")) {
      float temp = cmd.substring(18).toFloat();
      if (temp >= -50.0 && temp <= 80.0 && temp > chargeTempMin) {
        chargeTempMax = temp;
        prefs.begin("charging", false);
        prefs.putFloat("tempmax", chargeTempMax);
        prefs.end();
        MeshSerial.printf("charge temp max → %.1f°C\n", chargeTempMax);
      } else {
        MeshSerial.println("Invalid temp (-50 to 80°C, > min)");
      }
    }
    // charging:status - Get current charging status
    else if (cmd == "charging:status") {
      String statusStr;
      switch (chargingStatus) {
        case CHG_ENABLED: statusStr = "ENABLED"; break;
        case CHG_ERROR_TEMP_LOW: statusStr = "TEMP_LOW"; break;
        case CHG_ERROR_TEMP_HIGH: statusStr = "TEMP_HIGH"; break;
        case CHG_ERROR_SENSOR: statusStr = "SENSOR_ERROR"; break;
        default: statusStr = "DISABLED"; break;
      }
      MeshSerial.printf("charging: %s %.1f°C relay:%s\n",
                       statusStr.c_str(), currentTemperature,
                       chargingEnabled ? "ON" : "OFF");
    }
    // charging:parameter - Get all charging parameters
    else if (cmd == "charging:parameter") {
      String chemName = (batteryChemistry == BAT_LI_ION) ? "Li-ion" : "Lead-acid";
      MeshSerial.printf("chemistry:%s temp:%.1f-%.1f°C\n",
                       chemName.c_str(), chargeTempMin, chargeTempMax);
    }
    // compass - Get current compass heading and refresh mode
    else if (cmd == "compass") {
      if (compassPresent) {
        String modeStr = navigationMode ? "nav" : "idle";
        unsigned long currentInterval = navigationMode ? compassReadIntervalNav : compassReadIntervalIdle;
        MeshSerial.printf("compass: %.1f° (%s mode, %lu ms interval)\n",
                         currentHeading, modeStr.c_str(), currentInterval);
      } else {
        MeshSerial.println("compass: not available");
      }
    }
    // compass:raw - Get raw X,Y,Z magnetic field values
    else if (cmd == "compass:raw") {
      if (compassPresent) {
        compass.read();
        MeshSerial.printf("compass raw: X:%d Y:%d Z:%d\n", compass.getX(), compass.getY(), compass.getZ());
      } else {
        MeshSerial.println("compass: not available");
      }
    }
    // compass:calibrate - Start compass calibration process
    else if (cmd == "compass:calibrate") {
      if (compassPresent) {
        compassCalibrating = true;
        compassCalMinX = 32767; compassCalMaxX = -32768;
        compassCalMinY = 32767; compassCalMaxY = -32768;
        compassCalMinZ = 32767; compassCalMaxZ = -32768;
        MeshSerial.println("Compass calibration started:");
        MeshSerial.println("1. Rotate device 360° slowly in all axes");
        MeshSerial.println("2. Send 'compass:calibrate:done' when finished");
        MeshSerial.println("3. Calibration data will be collected and applied");
      } else {
        MeshSerial.println("compass: not available");
      }
    }
    // compass:calibrate:done - Finish calibration and apply offsets
    else if (cmd == "compass:calibrate:done") {
      if (compassPresent && compassCalibrating) {
        compassCalibrating = false;
        // Calculate offsets (center of range)
        int offsetX = (compassCalMaxX + compassCalMinX) / 2;
        int offsetY = (compassCalMaxY + compassCalMinY) / 2;
        int offsetZ = (compassCalMaxZ + compassCalMinZ) / 2;
        // Calculate scales (assuming spherical calibration)
        int rangeX = compassCalMaxX - compassCalMinX;
        int rangeY = compassCalMaxY - compassCalMinY;
        int rangeZ = compassCalMaxZ - compassCalMinZ;
        float scaleX = 1000.0 / rangeX;
        float scaleY = 1000.0 / rangeY;
        float scaleZ = 1000.0 / rangeZ;

        compass.setCalibrationOffsets(offsetX, offsetY, offsetZ);
        compass.setCalibrationScales(scaleX, scaleY, scaleZ);

        MeshSerial.printf("Calibration applied:\n");
        MeshSerial.printf("Offsets: X:%d Y:%d Z:%d\n", offsetX, offsetY, offsetZ);
        MeshSerial.printf("Scales: X:%.2f Y:%.2f Z:%.2f\n", scaleX, scaleY, scaleZ);
        MeshSerial.printf("Ranges: X:%d Y:%d Z:%d\n", rangeX, rangeY, rangeZ);
      } else {
        MeshSerial.println("Calibration not in progress or compass not available");
      }
    }
    // compass:interval:nav:XXX - Set navigation mode refresh interval (100-2000ms)
    else if (cmd.startsWith("compass:interval:nav:")) {
      unsigned long interval = cmd.substring(21).toInt();
      if (interval >= 100 && interval <= 2000) {
        compassReadIntervalNav = interval;
        prefs.begin("navigation", false);
        prefs.putULong("compass_nav", compassReadIntervalNav);
        prefs.end();
        MeshSerial.printf("Compass nav interval → %lu ms\n", compassReadIntervalNav);
      } else {
        MeshSerial.println("Invalid nav interval (100-2000 ms)");
      }
    }
    // compass:interval:idle:XXX - Set idle mode refresh interval (1000-10000ms)
    else if (cmd.startsWith("compass:interval:idle:")) {
      unsigned long interval = cmd.substring(22).toInt();
      if (interval >= 1000 && interval <= 10000) {
        compassReadIntervalIdle = interval;
        prefs.begin("navigation", false);
        prefs.putULong("compass_idle", compassReadIntervalIdle);
        prefs.end();
        MeshSerial.printf("Compass idle interval → %lu ms\n", compassReadIntervalIdle);
      } else {
        MeshSerial.println("Invalid idle interval (1000-10000 ms)");
      }
    }
    // compass:interval - Show current compass refresh intervals
    else if (cmd == "compass:interval") {
      MeshSerial.printf("compass intervals: nav=%lu ms, idle=%lu ms\n",
                       compassReadIntervalNav, compassReadIntervalIdle);
    }
    // shaft:pulses:X - Set pulses per revolution for RPM calculation (1-10)
    else if (cmd.startsWith("shaft:pulses:")) {
      int pulses = cmd.substring(13).toInt();
      if (pulses >= 1 && pulses <= 10) {
        pulsesPerRevolution = pulses;
        MeshSerial.printf("Shaft pulses per revolution → %d\n", pulsesPerRevolution);
      } else {
        MeshSerial.println("Invalid pulses per revolution (1-10)");
      }
    }
    // navigation:on - Enable navigation mode
    else if (cmd == "navigation:on") {
      if (compassPresent) {
        navigationMode = true;
        MeshSerial.println("Navigation mode → on");
      } else {
        MeshSerial.println("Navigation: compass not available");
      }
    }
    // navigation:off - Disable navigation mode
    else if (cmd == "navigation:off") {
      navigationMode = false;
      // Return servo to center when disabling navigation
      setServoAngle(0, servo1Center);
      currentServo1Angle = servo1Center;
      MeshSerial.println("Navigation mode → off");
    }
    // navigation:heading:XXX - Set target heading (0-360°)
    else if (cmd.startsWith("navigation:heading:")) {
      float heading = cmd.substring(19).toFloat();
      if (heading >= 0.0 && heading <= 360.0) {
        targetHeading = heading;
        MeshSerial.printf("Navigation heading → %.1f°\n", targetHeading);
      } else {
        MeshSerial.println("Invalid heading (0-360°)");
      }
    }
    // statusN - Get navigation status and settings
    else if (cmd == "statusN") {
      MeshSerial.printf("navigation: %s\n", navigationMode ? "ON" : "OFF");
      MeshSerial.printf("target: %.1f°\n", targetHeading);
      MeshSerial.printf("current: %.1f°\n", currentHeading);
      MeshSerial.printf("motor: %d (%s)\n", currentMotorSpeed,
                       currentMotorSpeed >= 0 ? "forward" : "reverse");
      MeshSerial.printf("servo1: %d\n", currentServo1Angle);
      MeshSerial.printf("gain: %d\n", navigationSteeringGain);
      MeshSerial.printf("deadband: %.1f°\n", navigationDeadband);
      MeshSerial.printf("interval: %lu ms\n", navigationUpdateInterval);
    }
    // navigation:gain:X - Set steering gain (1-10)
    else if (cmd.startsWith("navigation:gain:")) {
      int gain = cmd.substring(16).toInt();
      if (gain >= 1 && gain <= 10) {
        navigationSteeringGain = gain;
        prefs.begin("navigation", false);
        prefs.putInt("gain", navigationSteeringGain);
        prefs.end();
        MeshSerial.printf("Navigation gain → %d\n", navigationSteeringGain);
      } else {
        MeshSerial.println("Invalid gain (1-10)");
      }
    }
    // navigation:deadband:X.X - Set deadband (0-10°)
    else if (cmd.startsWith("navigation:deadband:")) {
      float deadband = cmd.substring(20).toFloat();
      if (deadband >= 0.0 && deadband <= 10.0) {
        navigationDeadband = deadband;
        prefs.begin("navigation", false);
        prefs.putFloat("deadband", navigationDeadband);
        prefs.end();
        MeshSerial.printf("Navigation deadband → %.1f°\n", navigationDeadband);
      } else {
        MeshSerial.println("Invalid deadband (0-10°)");
      }
    }
    // navigation:interval:XX - Set update interval in milliseconds (10-200)
    else if (cmd.startsWith("navigation:interval:")) {
      unsigned long interval = cmd.substring(20).toInt();
      if (interval >= 10 && interval <= 200) {
        navigationUpdateInterval = interval;
        prefs.begin("navigation", false);
        prefs.putULong("interval", navigationUpdateInterval);
        prefs.end();
        MeshSerial.printf("Navigation interval → %lu ms\n", navigationUpdateInterval);
      } else {
        MeshSerial.println("Invalid interval (10-200 ms)");
      }
    }
  }
}