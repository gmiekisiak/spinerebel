// ============================== Logging.ino ==============================
// NimBrace v2.2.6 - Data logging and SD card operations
// (c) 2025-2026 Grzegorz Miekisiak / SpineRebel Technology
// Academic and non-commercial use only. See LICENSE.
//
// CHANGES IN v2.2.6:
// - listFiles() REMOVED - now lives in Cradle.ino (no more redefinition error)
// - cleanupSD() REMOVED - now lives in Cradle.ino
// - Version bumped to match Cradle.ino v2.2.6
//
// Previous fixes retained:
// - Relaxed stuck detection thresholds (was way too aggressive)
// - ALL BUZZES REMOVED from health monitoring - patient is naive
// - Silent logging for debugging when connected to cradle

// ZeroReferencePacket struct
struct ZeroReferencePacket {
  float q1w, q1x, q1y, q1z;
  float a1x, a1y, a1z;
  float g1x, g1y, g1z;
  float q2w, q2x, q2y, q2z;
  float a2x, a2y, a2z;
  float g2x, g2y, g2z;
};

// ==================== IMU STUCK DETECTION ====================
// A WORKING IMU at rest shows random noise - values always fluctuate slightly
// A STUCK IMU shows IDENTICAL values - exact same float repeated
// So we check for EXACT EQUALITY, not "small change"
// NO BUZZES - completely silent to patient

const uint16_t STUCK_COUNT_THRESHOLD = 300;    // 3 seconds @ 100Hz
const uint16_t MAX_RESETS_PER_SESSION = 5;
const unsigned long I2C_RESET_COOLDOWN_MS = 10000;  // 10s between resets

static unsigned long lastI2CReset = 0;

// ==================== I2C BUS RESET ====================
void resetI2CBus() {
  unsigned long now = millis();
  if (now - lastI2CReset < I2C_RESET_COOLDOWN_MS) return;
  lastI2CReset = now;
  
  Serial.println("WARN:I2C_BUS_RESET");
  
  Wire1.end();
  pinMode(I2C1_SCL, OUTPUT);
  for (int i = 0; i < 10; i++) {
    digitalWrite(I2C1_SCL, HIGH);
    delayMicroseconds(5);
    digitalWrite(I2C1_SCL, LOW);
    delayMicroseconds(5);
  }
  digitalWrite(I2C1_SCL, HIGH);
  delay(10);
  
  Wire1.setSDA(I2C1_SDA);
  Wire1.setSCL(I2C1_SCL);
  Wire1.begin();
  Wire1.setClock(400000);
  delay(50);
}

// ==================== IMU REINITIALIZATION (SILENT) ====================
bool reinitIMU1() {
  if (imu1Health.resetCount >= MAX_RESETS_PER_SESSION) {
    Serial.println("ERROR:IMU1_MAX_RESETS");
    return false;
  }
  
  Serial.print("WARN:IMU1_REINIT:");
  Serial.println(imu1Health.resetCount + 1);
  
  if (imu1.begin(0x4A, Wire1)) {
    imu1.enableGameRotationVector(10);
    imu1.enableLinearAccelerometer(10);
    imu1.enableGyro(10);
    
    imu1Health.stuckCount = 0;
    imu1Health.resetCount++;
    imu1Health.lastGoodData = millis();
    imu1Health.wasStuck = false;
    
    Serial.println("OK:IMU1_RECOVERED");
    return true;
  }
  
  resetI2CBus();
  delay(100);
  
  if (imu1.begin(0x4A, Wire1)) {
    imu1.enableGameRotationVector(10);
    imu1.enableLinearAccelerometer(10);
    imu1.enableGyro(10);
    
    imu1Health.stuckCount = 0;
    imu1Health.resetCount++;
    imu1Health.lastGoodData = millis();
    imu1Health.wasStuck = false;
    
    Serial.println("OK:IMU1_RECOVERED_I2C");
    return true;
  }
  
  Serial.println("ERROR:IMU1_REINIT_FAILED");
  imu1_ready = false;
  return false;
}

bool reinitIMU2() {
  if (imu2Health.resetCount >= MAX_RESETS_PER_SESSION) {
    Serial.println("ERROR:IMU2_MAX_RESETS");
    return false;
  }
  
  Serial.print("WARN:IMU2_REINIT:");
  Serial.println(imu2Health.resetCount + 1);
  
  if (imu2.begin(0x4B, Wire1)) {
    imu2.enableGameRotationVector(10);
    imu2.enableLinearAccelerometer(10);
    imu2.enableGyro(10);
    
    imu2Health.stuckCount = 0;
    imu2Health.resetCount++;
    imu2Health.lastGoodData = millis();
    imu2Health.wasStuck = false;
    
    Serial.println("OK:IMU2_RECOVERED");
    return true;
  }
  
  resetI2CBus();
  delay(100);
  
  if (imu2.begin(0x4B, Wire1)) {
    imu2.enableGameRotationVector(10);
    imu2.enableLinearAccelerometer(10);
    imu2.enableGyro(10);
    
    imu2Health.stuckCount = 0;
    imu2Health.resetCount++;
    imu2Health.lastGoodData = millis();
    imu2Health.wasStuck = false;
    
    Serial.println("OK:IMU2_RECOVERED_I2C");
    return true;
  }
  
  Serial.println("ERROR:IMU2_REINIT_FAILED");
  imu2_ready = false;
  return false;
}

// ==================== STUCK DETECTION ====================
bool checkIMU1Stuck(float gx, float gy, float gz, float qw) {
  if (!imu1_ready) return false;
  
  bool gyroStuck = (gx == imu1Health.prevGyroX) && 
                   (gy == imu1Health.prevGyroY) && 
                   (gz == imu1Health.prevGyroZ);
  bool quatStuck = (qw == imu1Health.prevQuatW);
  
  imu1Health.prevGyroX = gx;
  imu1Health.prevGyroY = gy;
  imu1Health.prevGyroZ = gz;
  imu1Health.prevQuatW = qw;
  
  if (gyroStuck && quatStuck) {
    imu1Health.stuckCount++;
    if (imu1Health.stuckCount >= STUCK_COUNT_THRESHOLD) {
      if (!imu1Health.wasStuck) {
        Serial.println("WARN:IMU1_STUCK");
        imu1Health.wasStuck = true;
      }
      return true;
    }
  } else {
    imu1Health.stuckCount = 0;
    imu1Health.lastGoodData = millis();
    if (imu1Health.wasStuck) {
      Serial.println("OK:IMU1_UNSTUCK");
      imu1Health.wasStuck = false;
    }
  }
  return false;
}

bool checkIMU2Stuck(float gx, float gy, float gz, float qw) {
  if (!imu2_ready) return false;
  
  bool gyroStuck = (gx == imu2Health.prevGyroX) && 
                   (gy == imu2Health.prevGyroY) && 
                   (gz == imu2Health.prevGyroZ);
  bool quatStuck = (qw == imu2Health.prevQuatW);
  
  imu2Health.prevGyroX = gx;
  imu2Health.prevGyroY = gy;
  imu2Health.prevGyroZ = gz;
  imu2Health.prevQuatW = qw;
  
  if (gyroStuck && quatStuck) {
    imu2Health.stuckCount++;
    if (imu2Health.stuckCount >= STUCK_COUNT_THRESHOLD) {
      if (!imu2Health.wasStuck) {
        Serial.println("WARN:IMU2_STUCK");
        imu2Health.wasStuck = true;
      }
      return true;
    }
  } else {
    imu2Health.stuckCount = 0;
    imu2Health.lastGoodData = millis();
    if (imu2Health.wasStuck) {
      Serial.println("OK:IMU2_UNSTUCK");
      imu2Health.wasStuck = false;
    }
  }
  return false;
}

// ==================== HEALTH CHECK (SILENT) ====================
void checkIMUHealth() {
  if (imu1_ready && checkIMU1Stuck(packet.g1x, packet.g1y, packet.g1z, packet.q1w)) {
    reinitIMU1();
  }
  if (imu2_ready && checkIMU2Stuck(packet.g2x, packet.g2y, packet.g2z, packet.q2w)) {
    reinitIMU2();
  }
  
  unsigned long now = millis();
  if (imu1_ready && (now - lastIMU1Data > IMU_TIMEOUT_MS)) {
    Serial.println("WARN:IMU1_TIMEOUT");
    reinitIMU1();
    lastIMU1Data = now;
  }
  if (imu2_ready && (now - lastIMU2Data > IMU_TIMEOUT_MS)) {
    Serial.println("WARN:IMU2_TIMEOUT");
    reinitIMU2();
    lastIMU2Data = now;
  }
}

// ==================== VOLTAGE MONITORING (SILENT) ====================
const uint16_t LOW_VOLTAGE_THRESHOLD_MV = 3300;
const uint16_t CRITICAL_VOLTAGE_THRESHOLD_MV = 3100;

static bool lowVoltageWarned = false;
static unsigned long lastVoltageCheck = 0;

void checkVoltage() {
  if (millis() - lastVoltageCheck < 5000) return;
  lastVoltageCheck = millis();
  
  if (battVoltage_mV > 0 && battVoltage_mV < CRITICAL_VOLTAGE_THRESHOLD_MV) {
    Serial.print("CRITICAL:LOW_VOLTAGE:");
    Serial.println(battVoltage_mV);
  } else if (battVoltage_mV > 0 && battVoltage_mV < LOW_VOLTAGE_THRESHOLD_MV) {
    if (!lowVoltageWarned) {
      Serial.print("WARN:LOW_VOLTAGE:");
      Serial.println(battVoltage_mV);
      lowVoltageWarned = true;
    }
  } else {
    lowVoltageWarned = false;
  }
}

// ==================== SD CARD FUNCTIONS ====================

static bool rtcGetBaseName(char* out, size_t outLen) {
  if (!rtcAvailable) return false;
  DateTime now = rtc.now();
  if (now.year() < 2020) return false;
  snprintf(out, outLen, "%04d%02d%02d_%02d%02d%02d",
           now.year(), now.month(), now.day(),
           now.hour(), now.minute(), now.second());
  return true;
}

void computeBaseNameOnce() {
  if (baseName[0] != '\0') return;
  if (rtcGetBaseName(baseName, sizeof(baseName))) return;
  snprintf(baseName, sizeof(baseName), "%lu", sessionId);
}

void initSD() {
  SPI1.setSCK(10);
  SPI1.setTX(11);
  SPI1.setRX(12);
  SPI1.begin();
  if (!SD.begin(SD_CS, SPI1)) {
    Serial.println("SD card init failed - running without logging");
    sdReady = false; 
    return;
  }
  if (!SD.exists(CONFIG_FILE)) {
    File f = SD.open(CONFIG_FILE, FILE_WRITE);
    if (f) { f.print("UNSET-ID-00"); f.close(); }
  }
  sdReady = true;
}

void writeDataFileHeader() {
  if (!sdReady || !dataFile) return;

  DataFileHeader hdr{};
  memcpy(hdr.magic, "NIMDATA", 7);
  hdr.magic[7] = '\0';
  hdr.version = 2;
  hdr.header_size = sizeof(DataFileHeader);
  hdr.packet_size = sizeof(DataPacket);

  memset(hdr.basename, 0, sizeof(hdr.basename));
  strncpy(hdr.basename, baseName, sizeof(hdr.basename) - 1);

  String id = readBraceID();
  memset(hdr.brace_id, 0, sizeof(hdr.brace_id));
  id.toCharArray(hdr.brace_id, sizeof(hdr.brace_id));

  hdr.start_unix = 0;
  if (rtcAvailable) {
    DateTime now = rtc.now();
    hdr.start_unix = now.unixtime();
  }

  dataFile.write((const uint8_t*)&hdr, sizeof(hdr));
  dataFile.flush();
  
  Serial.print("Header written: ");
  Serial.print(sizeof(hdr));
  Serial.println(" bytes");
}

void openDataFile() {
  if (!sdReady) return;
  if (dataFile) { dataFile.flush(); dataFile.close(); }
  
  computeBaseNameOnce();
  
  char filename[40];
  snprintf(filename, sizeof(filename), "DATA_%s.BIN", baseName);
  
  strncpy(currentFileName, filename, sizeof(currentFileName) - 1);
  
  dataFile = SD.open(filename, FILE_WRITE);
  if (dataFile) {
    Serial.print("Recording to: "); Serial.println(filename);
    
    // Only write header if file is NEW (size == 0)
    // Prevents double-header bug when file is reopened
    if (dataFile.size() == 0) {
      writeDataFileHeader();
    } else {
      Serial.print("Appending to existing file, size: ");
      Serial.println(dataFile.size());
    }
    
    // listFiles() call removed - it's in Cradle.ino now
  } else {
    Serial.println("Failed to create data file");
    sdReady = false;
  }
}

void sd_write_packet(const DataPacket &pkt) {
  if (!sdReady || !dataFile) return;
  
  dataFile.write((const uint8_t*)&pkt, sizeof(DataPacket));
  
  // Flush every 100 packets (~1 second at 100Hz)
  static uint32_t writeCount = 0;
  if ((++writeCount % 100) == 0) {
    dataFile.flush();
  }
}

void confirm_zero_buzz() {
  for (int i = 0; i < 2; i++) {
    setMotor1(200);
    delay(200);
    stopMotor1();
    delay(400);
  }
}

void performZeroReferencing() {
  if (!sdReady) { Serial.println("ZERO SKIP: SD not ready"); return; }
  Serial.println("--- ROZPOCZĘTO ZEROWANIE REFERENCYJNE (3s) ---");

  float sums[20] = {0};
  uint32_t c1 = 0, c2 = 0;
  unsigned long start = millis();
  
  while (millis() - start < 3000) {
    if (imu1_ready && imu1.dataAvailable()) {
      sums[0] += imu1.getQuatReal(); sums[1] += imu1.getQuatI(); 
      sums[2] += imu1.getQuatJ();    sums[3] += imu1.getQuatK();
      sums[4] += imu1.getLinAccelX(); sums[5] += imu1.getLinAccelY(); 
      sums[6] += imu1.getLinAccelZ();
      sums[7] += imu1.getGyroX(); sums[8] += imu1.getGyroY(); 
      sums[9] += imu1.getGyroZ();
      c1++;
    }
    if (imu2_ready && imu2.dataAvailable()) {
      sums[10] += imu2.getQuatReal(); sums[11] += imu2.getQuatI(); 
      sums[12] += imu2.getQuatJ();    sums[13] += imu2.getQuatK();
      sums[14] += imu2.getLinAccelX(); sums[15] += imu2.getLinAccelY(); 
      sums[16] += imu2.getLinAccelZ();
      sums[17] += imu2.getGyroX(); sums[18] += imu2.getGyroY(); 
      sums[19] += imu2.getGyroZ();
      c2++;
    }
    delay(1);
  }

  ZeroReferencePacket z{};
  if (c1 > 0) {
    z.q1w = sums[0]/c1; z.q1x = sums[1]/c1; z.q1y = sums[2]/c1; z.q1z = sums[3]/c1;
    z.a1x = sums[4]/c1; z.a1y = sums[5]/c1; z.a1z = sums[6]/c1;
    z.g1x = sums[7]/c1; z.g1y = sums[8]/c1; z.g1z = sums[9]/c1;
  }
  if (c2 > 0) {
    z.q2w = sums[10]/c2; z.q2x = sums[11]/c2; z.q2y = sums[12]/c2; z.q2z = sums[13]/c2;
    z.a2x = sums[14]/c2; z.a2y = sums[15]/c2; z.a2z = sums[16]/c2;
    z.g2x = sums[17]/c2; z.g2y = sums[18]/c2; z.g2z = sums[19]/c2;
  }

  computeBaseNameOnce();
  char zeroFilename[40];
  snprintf(zeroFilename, sizeof(zeroFilename), "ZERO_%s.BIN", baseName);
  
  File f = SD.open(zeroFilename, FILE_WRITE);
  if (f) {
    f.write((uint8_t*)&z, sizeof(z));
    f.close();
    Serial.print("Zapisano ZERO_REF do: "); Serial.println(zeroFilename);
  } else {
    Serial.println("BŁĄD: Nie udało się zapisać ZERO_*.BIN.");
  }
  
  Serial.print("Zero counts -> IMU1: "); Serial.print(c1);
  Serial.print(" | IMU2: "); Serial.println(c2);
  Serial.println("--- ZEROWANIE ZAKOŃCZONE ---");
  
  confirm_zero_buzz();
}
