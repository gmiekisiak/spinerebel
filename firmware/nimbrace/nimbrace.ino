// ============================== nimbrace.ino (MAIN) ==============================
// NimBrace Firmware v2.2.6
// (c) 2025-2026 Grzegorz Miekisiak / SpineRebel Technology
// Academic and non-commercial use only. See LICENSE.
// Zero-Cycle principle: PCT patent pending.
//
// CHANGES IN v2.2.6:
// - Cradle.ino: transmitFile() - removed periodic flush (fixed 99% stall)
// - Cradle.ino: moveToArchive() - rename to uploaded_ instead of copy (instant)
// - Cradle.ino: listFiles() - filters uploaded_ prefix
// - Logging.ino: listFiles() and cleanupSD() removed (now only in Cradle.ino)
//
// Previous fixes retained from v2.2.5:
// - Green LED breathing ONLY when BOTH IMUs are healthy and delivering data
// - Yellow LED fast blink when degraded (one or both IMUs failed)
// - areIMUsHealthy() requires BOTH IMUs
// - IMU boot wait - waits up to 10s for IMUs to report
// - Differentiated buzz pattern: 2=both OK, 3=one dead, 5=both dead
// - Disabled aggressive IMU health monitoring that caused false stuck detection
// - PWM conflict between green LED breathing and motor resolved
// - Double header bug fixed
// - Voltage monitoring for brownout detection

#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <FS.h>
#include <RTClib.h>
#include <SparkFun_BNO080_Arduino_Library.h>
#include <math.h>

// ---------------- Firmware Version ----------------
#define FIRMWARE_VERSION "2.2.6"
#define FIRMWARE_DATE __DATE__

// ---------------- Pins ----------------
#define I2C1_SDA 26
#define I2C1_SCL 27
#define BATT_SDA 4
#define BATT_SCL 5
#define SD_CS    13

#define RED_LED_PIN      14
#define MOVEMENT_LED_PIN 15  // Yellow LED
#define GREEN_LED_PIN    22
#define MOTOR_PIN_1      6
#define MOTOR_PIN_2      7
#define PAIN_BUTTON_PIN  3

// --- Constants ---
const char* CONFIG_FILE = "CONFIG.TXT";
const unsigned long IMU_TIMEOUT_MS = 2000;
const unsigned long IMU_BOOT_TIMEOUT_MS = 10000;
const unsigned long WRITE_INTERVAL_MS = 10;
const unsigned long PRINT_INTERVAL_MS = 100;
const unsigned long BATT_READ_INTERVAL_MS = 2000;
const unsigned long MODE_DEBOUNCE_MS = 1500;

// --- Event Flags ---
#define EVT_FLAG_PAIN_BUTTON 0x01
#define EVT_FLAG_MOTOR_1     0x02
#define EVT_FLAG_MOTOR_2     0x04

// --------------- Objects ---------------
BNO080 imu1, imu2;
TwoWire BatteryWire(i2c0, BATT_SDA, BATT_SCL);
RTC_PCF8523 rtc;
File dataFile;

// ------------- Types ------------
struct DataPacket {
  uint32_t timestamp;
  float q1w, q1x, q1y, q1z;
  float a1x, a1y, a1z;
  float g1x, g1y, g1z;
  float q2w, q2x, q2y, q2z;
  float a2x, a2y, a2z;
  float g2x, g2y, g2z;
  uint16_t voltage_mV;
  int16_t  current_mA;
  uint8_t  event_flags;
};

struct __attribute__((packed)) DataFileHeader {
  char     magic[8];
  uint16_t version;
  uint16_t header_size;
  uint16_t packet_size;
  char     basename[20];
  char     brace_id[32];
  uint32_t start_unix;
  uint8_t  reserved[16];
};

// IMU Health Monitor
struct IMUHealthMonitor {
  float prevGyroX, prevGyroY, prevGyroZ;
  float prevQuatW;
  uint16_t stuckCount;
  uint16_t resetCount;
  unsigned long lastGoodData;
  bool wasStuck;
};

IMUHealthMonitor imu1Health = {0, 0, 0, 0, 0, 0, 0, false};
IMUHealthMonitor imu2Health = {0, 0, 0, 0, 0, 0, 0, false};

// ------------- State ------------
bool sdReady = false;
bool imu1_ready = false, imu2_ready = false;
bool rtcAvailable = false;

unsigned long sessionId = 0;
char baseName[32] = {0};
char currentFileName[40] = {0};

unsigned long lastIMU1Data = 0, lastIMU2Data = 0;
unsigned long lastWriteTime = 0, lastPrintTime = 0;
unsigned long lastLedUpdate = 0, lastBattRead = 0;
unsigned long lastHzReport = 0, samplesSinceHz = 0;
unsigned long lastModeSwitch = 0;

DataPacket packet = {0};
uint16_t battVoltage_mV = 0;
int16_t  battCurrent_mA = 0;

float prevPitch1 = 0, prevRoll1 = 0, prevPitch2 = 0, prevRoll2 = 0;

String commandBuffer;

enum Mode { MODE_STARTUP, MODE_LOGGING, MODE_TRANSFER };
Mode mode = MODE_STARTUP;

enum UploadState { UL_IDLE, UL_NEGOTIATING, UL_UPLOADING, UL_COMPLETE };
volatile UploadState uploadState = UL_IDLE;

volatile bool cradleLocked = false;

// Pain button
unsigned long painButtonPressTime = 0;
bool painButtonActive = false;
bool painButtonBuzzed = false;

// Motor flags
volatile bool motor1Active_flag = false;
volatile bool motor2Active_flag = false;

volatile unsigned long motor1LastActiveTime = 0;
const unsigned long MOTOR_PWM_COOLDOWN_MS = 100;

// File splitting
uint8_t currentFileHour = 255;

// Debug
bool debugEuler = false;

// ---------------------------------------------------------------------
// PROTOTYPES
// NOTE: listFiles() and cleanupSD() are in Cradle.ino only (v2.2.6)
// ---------------------------------------------------------------------
void createSessionBaseName();
void computeBaseNameOnce();
void initSD();
void openDataFile();
void sd_write_packet(const DataPacket &pkt);
void transmitFile(const String& filename);
void moveToArchive(const String& filename);
void listFiles();
void listArchiveFiles();
void cleanupSD();
void writeDataFileHeader();
void readBatteryStatus();
bool usbPresent();
bool wantTransfer();
void switchTo(Mode newMode);
void handleSerialCommands();
void setRTCUnix(uint32_t unixTime);
String readBraceID();
void printStartupBanner();
void checkVoltage();
void checkIMUHealth();
bool checkIMU1Stuck(float gx, float gy, float gz, float qw);
bool checkIMU2Stuck(float gx, float gy, float gz, float qw);
bool reinitIMU1();
bool reinitIMU2();
void resetI2CBus();
void performZeroReferencing();
void confirm_zero_buzz();

// ---------------------------------------------------------------------
// STARTUP BANNER
// ---------------------------------------------------------------------
void printStartupBanner() {
  Serial.println();
  Serial.println("========================================");
  Serial.println("         N I M B R A C E");
  Serial.println("   Spine Motion Analysis System");
  Serial.println("========================================");
  Serial.print("Firmware:  v"); Serial.println(FIRMWARE_VERSION);
  Serial.print("Built:     "); Serial.println(FIRMWARE_DATE);
  Serial.println("----------------------------------------");
  Serial.print("IMU1:      "); Serial.println(imu1_ready ? "OK" : "FAIL");
  Serial.print("IMU2:      "); Serial.println(imu2_ready ? "OK" : "FAIL");
  Serial.print("SD Card:   "); Serial.println(sdReady ? "OK" : "FAIL");
  Serial.print("RTC:       "); Serial.println(rtcAvailable ? "OK" : "FAIL");
  Serial.print("Brace ID:  "); Serial.println(readBraceID());
  Serial.println("----------------------------------------");
  Serial.println("LED Guide:");
  Serial.println("  Green breathing = BOTH IMUs OK");
  Serial.println("  Yellow blink    = DEGRADED - IMU down!");
  Serial.println("  Red = Charging/USB connected");
  Serial.println("----------------------------------------");
  Serial.println("Buzz Guide:");
  Serial.println("  2 buzzes = Both IMUs OK");
  Serial.println("  3 buzzes = One IMU failed!");
  Serial.println("  5 buzzes = Both IMUs failed!");
  Serial.println("----------------------------------------");
  Serial.println("(c) 2025-2026 SpineRebel Technology");
  Serial.println("========================================");
  Serial.println();
}

// ---------------------------------------------------------------------
// MOTOR & LED HELPERS
// ---------------------------------------------------------------------

void stopMotor1() {
  analogWrite(MOTOR_PIN_1, 0);
  digitalWrite(MOTOR_PIN_1, LOW);
  motor1LastActiveTime = millis();
}

void stopMotor2() {
  digitalWrite(MOTOR_PIN_2, LOW);
}

inline void setMotor1(int pwm) {
  if (pwm > 0) {
    motor1Active_flag = true;
    motor1LastActiveTime = millis();
    analogWrite(MOTOR_PIN_1, min(pwm, 255));
  } else {
    stopMotor1();
  }
}

inline void setMotor2(bool state) {
  if (state) motor2Active_flag = true;
  digitalWrite(MOTOR_PIN_2, state ? HIGH : LOW);
}

bool canUseLedPWM() {
  return (millis() - motor1LastActiveTime) > MOTOR_PWM_COOLDOWN_MS;
}

void buzzMotor(int count) {
  for (int i = 0; i < count; i++) {
    setMotor1(200);
    delay(150);
    stopMotor1();
    if (i < count - 1) delay(150);
  }
  stopMotor1();
  delay(10);
}

void flickerAllLeds() {
  static unsigned long lastFlicker = 0;
  static bool state = false;
  if (millis() - lastFlicker > 50) {
    state = !state;
    digitalWrite(RED_LED_PIN, state);
    digitalWrite(MOVEMENT_LED_PIN, state);
    digitalWrite(GREEN_LED_PIN, state);
    lastFlicker = millis();
  }
}

// ---------------------------------------------------------------------
// GREEN LED BREATHING - PWM-SAFE
// ---------------------------------------------------------------------
void updateGreenBreathing(unsigned long now) {
  float breathPhase = (sin(now / 1000.0f * 2.0f * PI) + 1.0f) * 0.5f;
  int breathLevel = (int)(breathPhase * 255);
  
  if (canUseLedPWM()) {
    analogWrite(GREEN_LED_PIN, breathLevel);
  } else {
    digitalWrite(GREEN_LED_PIN, breathLevel > 127 ? HIGH : LOW);
  }
}

// ---------------------------------------------------------------------
// IMU HEALTH CHECK - v2.2.5+: REQUIRES BOTH IMUs
// ---------------------------------------------------------------------
bool areIMUsHealthy() {
  unsigned long now = millis();
  
  if (!imu1_ready || !imu2_ready) return false;
  if (now - lastIMU1Data > IMU_TIMEOUT_MS) return false;
  if (now - lastIMU2Data > IMU_TIMEOUT_MS) return false;
  
  return true;
}

// ---------------------------------------------------------------------
// SETUP
// ---------------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  delay(500);

  pinMode(RED_LED_PIN, OUTPUT);
  pinMode(MOVEMENT_LED_PIN, OUTPUT);
  pinMode(GREEN_LED_PIN, OUTPUT);
  digitalWrite(RED_LED_PIN, LOW);
  digitalWrite(MOVEMENT_LED_PIN, LOW);
  digitalWrite(GREEN_LED_PIN, LOW);

  pinMode(MOTOR_PIN_1, OUTPUT);
  pinMode(MOTOR_PIN_2, OUTPUT);
  digitalWrite(MOTOR_PIN_1, LOW);
  digitalWrite(MOTOR_PIN_2, LOW);
  pinMode(PAIN_BUTTON_PIN, INPUT_PULLUP);
  
  motor1LastActiveTime = 0;

  Wire1.setSDA(I2C1_SDA);
  Wire1.setSCL(I2C1_SCL);
  Wire1.begin();
  Wire1.setClock(400000);

  BatteryWire.begin();

  flickerAllLeds();

  // RTC
  if (rtc.begin(&BatteryWire)) {
    if (!rtc.initialized() || rtc.lostPower()) {
      rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
    }
    rtcAvailable = true;
  }
  flickerAllLeds();

  // IMU1
  if (imu1.begin(0x4A, Wire1)) {
    imu1.enableGameRotationVector(10);
    imu1.enableLinearAccelerometer(10);
    imu1.enableGyro(10);
    imu1_ready = true;
    lastIMU1Data = millis();
    Serial.println("IMU1: I2C OK");
  } else {
    Serial.println("IMU1: I2C FAIL");
  }
  flickerAllLeds();

  // IMU2
  if (imu2.begin(0x4B, Wire1)) {
    imu2.enableGameRotationVector(10);
    imu2.enableLinearAccelerometer(10);
    imu2.enableGyro(10);
    imu2_ready = true;
    lastIMU2Data = millis();
    Serial.println("IMU2: I2C OK");
  } else {
    Serial.println("IMU2: I2C FAIL");
  }
  flickerAllLeds();

  // SD
  sessionId = millis();
  initSD();
  flickerAllLeds();

  // Wait for IMUs to actually deliver data (up to 10s)
  bool got1 = !imu1_ready;
  bool got2 = !imu2_ready;
  unsigned long startWait = millis();
  
  Serial.println("Waiting for IMU data...");
  
  while (!got1 || !got2) {
    flickerAllLeds();
    
    if (!got1 && imu1.dataAvailable()) {
      got1 = true;
      Serial.print("IMU1: Data OK (");
      Serial.print(millis() - startWait);
      Serial.println("ms)");
    }
    if (!got2 && imu2.dataAvailable()) {
      got2 = true;
      Serial.print("IMU2: Data OK (");
      Serial.print(millis() - startWait);
      Serial.println("ms)");
    }
    
    if (millis() - startWait > IMU_BOOT_TIMEOUT_MS) {
      if (!got1) {
        imu1_ready = false;
        Serial.println("ERROR:IMU1_NO_DATA_AT_BOOT");
      }
      if (!got2) {
        imu2_ready = false;
        Serial.println("ERROR:IMU2_NO_DATA_AT_BOOT");
      }
      break;
    }
    delay(10);
  }

  if (imu1_ready && imu2_ready) {
    buzzMotor(2);
    Serial.println("BOOT:DUAL_IMU_OK");
  } else if (imu1_ready || imu2_ready) {
    buzzMotor(3);
    Serial.println("WARN:SINGLE_IMU_MODE");
  } else {
    buzzMotor(5);
    Serial.println("ERROR:NO_IMU_DATA");
  }

  if (rtcAvailable) currentFileHour = rtc.now().hour();

  digitalWrite(RED_LED_PIN, LOW);
  digitalWrite(MOVEMENT_LED_PIN, LOW);
  digitalWrite(GREEN_LED_PIN, LOW);

  lastHzReport = millis();
  lastModeSwitch = millis();

  printStartupBanner();
}

// ---------------------------------------------------------------------
// LOOP
// ---------------------------------------------------------------------
void loop() {
  unsigned long now = millis();

  handleSerialCommands();

  // Pain button (3s hold)
  if (digitalRead(PAIN_BUTTON_PIN) == LOW) {
    if (painButtonPressTime == 0) {
      painButtonPressTime = now;
    } else if (!painButtonBuzzed && (now - painButtonPressTime >= 3000)) {
      buzzMotor(3);
      painButtonActive = true;
      painButtonBuzzed = true;
    }
  } else {
    painButtonPressTime = 0;
    painButtonBuzzed = false;
  }

  // IMU reading
  if (imu1_ready && imu1.dataAvailable()) {
    packet.q1w = imu1.getQuatReal(); packet.q1x = imu1.getQuatI();
    packet.q1y = imu1.getQuatJ();   packet.q1z = imu1.getQuatK();
    packet.a1x = imu1.getLinAccelX(); packet.a1y = imu1.getLinAccelY(); packet.a1z = imu1.getLinAccelZ();
    packet.g1x = imu1.getGyroX();     packet.g1y = imu1.getGyroY();     packet.g1z = imu1.getGyroZ();
    lastIMU1Data = now;
  }
  if (imu2_ready && imu2.dataAvailable()) {
    packet.q2w = imu2.getQuatReal(); packet.q2x = imu2.getQuatI();
    packet.q2y = imu2.getQuatJ();   packet.q2z = imu2.getQuatK();
    packet.a2x = imu2.getLinAccelX(); packet.a2y = imu2.getLinAccelY(); packet.a2z = imu2.getLinAccelZ();
    packet.g2x = imu2.getGyroX();     packet.g2y = imu2.getGyroY();     packet.g2z = imu2.getGyroZ();
    lastIMU2Data = now;
  }

  // Mode switching
  if (wantTransfer()) {
    switchTo(MODE_TRANSFER);
  } else if (!cradleLocked) {
    switchTo(MODE_LOGGING);
  }

  // =========================================================
  // LOGGING MODE
  // =========================================================
  if (mode == MODE_LOGGING) {
    
    checkIMUHealth();
    checkVoltage();
    
    // Hourly file split
    if (sdReady && rtcAvailable) {
      uint8_t h = rtc.now().hour();
      if (h != currentFileHour) {
        currentFileHour = h;
        baseName[0] = '\0';
        openDataFile();
      }
    }

    // Write packet
    if (sdReady && (now - lastWriteTime >= WRITE_INTERVAL_MS)) {
      packet.timestamp = now;
      packet.voltage_mV = battVoltage_mV;
      packet.current_mA = battCurrent_mA;

      uint8_t flags = 0;
      if (painButtonActive) { flags |= EVT_FLAG_PAIN_BUTTON; painButtonActive = false; }
      if (motor1Active_flag) { flags |= EVT_FLAG_MOTOR_1; motor1Active_flag = false; }
      if (motor2Active_flag) { flags |= EVT_FLAG_MOTOR_2; motor2Active_flag = false; }
      packet.event_flags = flags;

      sd_write_packet(packet);
      samplesSinceHz++;
      lastWriteTime = now;
    }

    // Hz report
    if (now - lastHzReport >= 1000) {
      float hz = samplesSinceHz * 1000.0f / (now - lastHzReport);
      Serial.print("LOG_HZ:"); Serial.println(hz, 1);
      samplesSinceHz = 0;
      lastHzReport = now;
    }
  }

  // =========================================================
  // TRANSFER MODE
  // =========================================================
  if (mode == MODE_TRANSFER && cradleLocked) {
    static unsigned long lastLockMsg = 0;
    if (now - lastLockMsg >= 10000) {
      Serial.println("INFO:CRADLE_LOCKED");
      lastLockMsg = now;
    }
  }

  // Battery read
  if (now - lastBattRead >= BATT_READ_INTERVAL_MS) {
    readBatteryStatus();
    lastBattRead = now;
  }

  // Debug Euler
  if (debugEuler && mode == MODE_TRANSFER && (now - lastPrintTime >= PRINT_INTERVAL_MS)) {
    float r1 = imu1.getRoll() * RAD_TO_DEG, p1 = imu1.getPitch() * RAD_TO_DEG, y1 = imu1.getYaw() * RAD_TO_DEG;
    float r2 = imu2.getRoll() * RAD_TO_DEG, p2 = imu2.getPitch() * RAD_TO_DEG, y2 = imu2.getYaw() * RAD_TO_DEG;
    Serial.printf("E1:%6.1f %6.1f %6.1f | E2:%6.1f %6.1f %6.1f\n", r1, p1, y1, r2, p2, y2);
    lastPrintTime = now;
  }

  // =========================================================
  // LED UPDATES
  // =========================================================
  if (now - lastLedUpdate >= 20) {
    if (mode == MODE_STARTUP) {
      flickerAllLeds();
    } else {
      // RED: cradle/charging
      if (cradleLocked) {
        digitalWrite(RED_LED_PIN, (now / 500) % 2);
      } else {
        digitalWrite(RED_LED_PIN, (battCurrent_mA > 20 || Serial) ? HIGH : LOW);
      }

      // GREEN + YELLOW: health-dependent
      if (mode == MODE_LOGGING) {
        if (areIMUsHealthy()) {
          updateGreenBreathing(now);
          digitalWrite(MOVEMENT_LED_PIN, LOW);
        } else {
          digitalWrite(GREEN_LED_PIN, LOW);
          digitalWrite(MOVEMENT_LED_PIN, (now / 250) % 2);
        }
      } else {
        // TRANSFER MODE
        digitalWrite(MOVEMENT_LED_PIN, LOW);
        
        static bool greenOn = false;
        static unsigned long lastBlink = 0;
        unsigned long period = (uploadState == UL_UPLOADING) ? 100 : (uploadState == UL_NEGOTIATING) ? 500 : 0;
        if (uploadState == UL_COMPLETE) {
          digitalWrite(GREEN_LED_PIN, HIGH);
        } else if (period && (now - lastBlink >= period)) {
          greenOn = !greenOn;
          digitalWrite(GREEN_LED_PIN, greenOn);
          lastBlink = now;
        } else if (!period) {
          digitalWrite(GREEN_LED_PIN, LOW);
        }
      }
    }
    lastLedUpdate = now;
  }
}
