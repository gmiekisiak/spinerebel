// ============================== Cradle.ino ==============================
// NimBrace v2.2.6 - Transfer and serial command handling
// (c) 2025-2026 Grzegorz Miekisiak / SpineRebel Technology
// Academic and non-commercial use only. See LICENSE.
// CHANGES IN v2.2.6:
// - transmitFile: removed periodic flush (was blocking at SD block boundaries)
// - moveToArchive: rename to uploaded_FILENAME instead of copying (instant)
// - listFiles: filters out uploaded_ prefix files
// - cleanupSD: added here (was removed from Logging.ino but never added here)

// --- CRC32 ---
uint32_t crc32(const uint8_t* data, size_t len) {
  uint32_t crc = 0xFFFFFFFF;
  for (size_t i = 0; i < len; i++) {
    crc ^= data[i];
    for (int j = 0; j < 8; j++) {
      crc = (crc >> 1) ^ (0xEDB88320 & (-(crc & 1)));
    }
  }
  return ~crc;
}

// --- Battery ---
void readBatteryStatus() {
  BatteryWire.beginTransmission(0x55);
  BatteryWire.write(0x04);
  BatteryWire.endTransmission(false);
  BatteryWire.requestFrom(0x55, 2);
  if (BatteryWire.available() == 2) {
    battVoltage_mV = BatteryWire.read() | (BatteryWire.read() << 8);
  }
  
  BatteryWire.beginTransmission(0x55);
  BatteryWire.write(0x10);
  BatteryWire.endTransmission(false);
  BatteryWire.requestFrom(0x55, 2);
  if (BatteryWire.available() == 2) {
    battCurrent_mA = BatteryWire.read() | (BatteryWire.read() << 8);
  }
}

bool usbPresent() {
  return (battCurrent_mA > 20) || Serial;
}

bool wantTransfer() {
  if (cradleLocked) return true;
  return usbPresent();
}

void cleanupTransferFiles() {
  if (!sdReady) return;
  
  File root = SD.open("/");
  if (!root) return;
  
  for (File f = root.openNextFile(); f; f = root.openNextFile()) {
    if (!f.isDirectory() && strcmp(f.name(), CONFIG_FILE) != 0) {
      String fname = f.name();
      if (f.size() < 1024 && (fname.startsWith("DATA_") || fname.startsWith("ZERO_"))) {
        f.close();
        SD.remove(fname);
        continue;
      }
    }
    f.close();
  }
  root.close();
}

// ==================== CLEANUP SD (delete all data files) ====================
void cleanupSD() {
  if (!sdReady) { Serial.println("ERROR:SD_NOT_READY"); return; }
  
  // Close active data file first
  if (dataFile) { dataFile.flush(); dataFile.close(); }
  
  File root = SD.open("/");
  if (!root) { Serial.println("ERROR:CANNOT_OPEN_ROOT"); return; }
  
  int deleted = 0;
  for (File f = root.openNextFile(); f; f = root.openNextFile()) {
    if (!f.isDirectory()) {
      String name = f.name();
      f.close();
      // Delete everything except CONFIG.TXT
      if (strcmp(name.c_str(), CONFIG_FILE) != 0) {
        if (SD.remove(name)) deleted++;
      }
    } else {
      f.close();
    }
  }
  root.close();
  
  Serial.print("OK:CLEANUP:"); Serial.print(deleted); Serial.println("_FILES_DELETED");
}

void switchTo(Mode newMode) {
  if (mode == newMode) return;
  if (millis() - lastModeSwitch < MODE_DEBOUNCE_MS) return;
  
  if (newMode == MODE_LOGGING && cradleLocked) {
    return;
  }
  
  lastModeSwitch = millis();

  if (newMode == MODE_TRANSFER) {
    if (dataFile) { dataFile.flush(); dataFile.close(); }
    cleanupTransferFiles();
    uploadState = UL_NEGOTIATING;
    Serial.println("MODE:TRANSFER");
  } else if (newMode == MODE_LOGGING) {
    cradleLocked = false;
    if (sdReady) openDataFile();
    uploadState = UL_IDLE;
    Serial.println("MODE:LOGGING");
  }
  mode = newMode;
}

void setRTCUnix(uint32_t unixTime) {
  if (!rtcAvailable) { Serial.println("WARN:RTC_NOT_AVAILABLE"); return; }
  rtc.adjust(DateTime(unixTime));
  baseName[0] = '\0';
  computeBaseNameOnce();
  Serial.print("OK:TIME_SYNCED:"); Serial.println((long)unixTime);
}

String readBraceID() {
  if (!sdReady) return "SD_NOT_READY";
  File f = SD.open(CONFIG_FILE, FILE_READ);
  if (!f) return "UNKNOWN";
  String id = f.readStringUntil('\n');
  id.trim();
  f.close();
  return id;
}

void writeBraceID(const String& id) {
  if (!sdReady) { Serial.println("ERROR:SD_NOT_READY"); return; }
  SD.remove(CONFIG_FILE);
  File f = SD.open(CONFIG_FILE, FILE_WRITE);
  if (f) { f.println(id); f.close(); Serial.println("OK:ID_SET"); }
  else { Serial.println("ERROR:CONFIG_WRITE_FAILED"); }
}

void resetSerialConnection() {
  Serial.flush();
  delay(50);
  Serial.end();
  delay(100);
  Serial.begin(115200);
  delay(200);
  while (Serial.available()) Serial.read();
  Serial.println("OK:SERIAL_READY");
}

void transmitFile(const String& filename) {
  if (mode != MODE_TRANSFER) { Serial.println("ERROR:BUSY_LOGGING"); return; }
  if (!sdReady) { Serial.println("ERROR:SD_NOT_READY"); return; }

  Serial.flush();
  delay(50);

  File f = SD.open(filename, FILE_READ);
  if (!f) { 
    Serial.print("ERROR:FILE_NOT_FOUND:"); 
    Serial.println(filename); 
    Serial.flush();
    return; 
  }

  size_t fileSize = f.size();
  
  Serial.print("FILE_START:"); 
  Serial.print(filename); 
  Serial.print(":"); 
  Serial.println(fileSize);
  Serial.flush();

  uint8_t buf[512];
  uint32_t file_crc = 0xFFFFFFFF;
  UploadState prev = uploadState;
  uploadState = UL_UPLOADING;
  size_t totalSent = 0;

  while (f.available()) {
    size_t n = f.read(buf, 512);
    if (!n) break;
    
    Serial.write(buf, n);
    totalSent += n;
    
    // CRC calculation
    for (size_t i = 0; i < n; i++) {
      file_crc ^= buf[i];
      for (int j = 0; j < 8; j++) {
        file_crc = (file_crc >> 1) ^ (0xEDB88320 & (-(file_crc & 1)));
      }
    }
    // NO periodic flush - it was blocking at 64KB SD block boundaries
    // causing the 99% stall on the Pi side
  }
  f.close();

  // v2.2.7: Ensure ALL binary bytes are pushed out before FILE_END
  // Without this, the last few KB sit in the USB serial buffer and
  // the Python side stalls at ~99% waiting for binary that's already
  // been replaced by the FILE_END text line
  Serial.flush();
  delay(200);  // was 10ms — not enough for 921600 baud USB buffer

  Serial.printf("\nFILE_END:CRC=%08X\n", ~file_crc);
  Serial.flush();
  
  delay(100);
  
  uploadState = prev;
}

void moveToArchive(const String& filename) {
  // v2.2.6: rename instead of copy
  // DATA_20260215_200000.BIN → uploaded_DATA_20260215_200000.BIN
  // FAT32 directory entry update - instant, no data copied
  // CMD:LIST filters out uploaded_ prefix so file is invisible next dock
  if (!sdReady) { Serial.println("ERROR:SD_NOT_READY"); return; }
  if (strcmp(filename.c_str(), CONFIG_FILE) == 0) {
    Serial.println("ERROR:CANNOT_ARCHIVE_CONFIG");
    return;
  }

  String newName = "uploaded_" + filename;

  if (SD.rename(filename, newName)) {
    Serial.print("OK:ARCHIVED:"); Serial.println(filename);
  } else {
    Serial.println("ERROR:RENAME_FAILED");
  }
}

void listFiles() {
  if (!sdReady) { Serial.println("ERROR:SD_NOT_READY"); return; }

  File root = SD.open("/");
  if (!root) { Serial.println("FILELIST:"); return; }

  Serial.print("FILELIST:");
  bool first = true;

  for (File f = root.openNextFile(); f; f = root.openNextFile()) {
    if (!f.isDirectory()) {
      String name = f.name();
      // Only show DATA_ files that have NOT been uploaded yet
      // uploaded_ prefix = already on server, invisible to Pi
      if (name.endsWith(".BIN") && !name.startsWith("uploaded_")) {
        if (!first) Serial.print(",");
        Serial.print(name);
        first = false;
      }
    }
    f.close();
  }
  root.close();
  Serial.println();
}

void listArchiveFiles() {
  // v2.2.6: archive is now uploaded_ prefix on root, not /archive/ folder
  // This lists them for diagnostic purposes
  if (!sdReady) { Serial.println("ERROR:SD_NOT_READY"); return; }

  File root = SD.open("/");
  if (!root) { Serial.println("ARCHIVELIST:"); return; }

  Serial.print("ARCHIVELIST:");
  bool first = true;

  for (File f = root.openNextFile(); f; f = root.openNextFile()) {
    if (!f.isDirectory()) {
      String name = f.name();
      if (name.startsWith("uploaded_")) {
        if (!first) Serial.print(",");
        Serial.print(name);
        first = false;
      }
    }
    f.close();
  }
  root.close();
  Serial.println();
}

// --- Serial Commands ---
void handleSerialCommands() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\r') continue;
    if (c != '\n') { commandBuffer += c; continue; }

    commandBuffer.trim();
    int p1 = commandBuffer.indexOf(':');
    int p2 = commandBuffer.indexOf(':', p1 + 1);
    String cmd = (p1 == -1) ? commandBuffer : commandBuffer.substring(0, p1);
    String arg1 = (p1 != -1) ? ((p2 != -1) ? commandBuffer.substring(p1 + 1, p2) : commandBuffer.substring(p1 + 1)) : "";
    String arg2 = (p2 != -1) ? commandBuffer.substring(p2 + 1) : "";
    cmd.toUpperCase();

    if (cmd == "CMD") {
      arg1.toUpperCase();
      
      // ==================== VERSION & INFO ====================
      if (arg1 == "VERSION" || arg1 == "GET_VERSION" || arg1 == "VER") {
        Serial.print("FIRMWARE:"); Serial.println(FIRMWARE_VERSION);
        Serial.print("BUILD:"); Serial.println(FIRMWARE_DATE);
      }
      else if (arg1 == "INFO" || arg1 == "BANNER") {
        printStartupBanner();
      }
      // ==================== DEVICE ID ====================
      else if (arg1 == "GET_ID") {
        Serial.print("BRACE_ID:"); Serial.println(readBraceID());
      }
      else if (arg1 == "SET_ID" && arg2.length() > 0) {
        writeBraceID(arg2);
      }
      // ==================== MODE & STATUS ====================
      else if (arg1 == "GET_MODE") {
        Serial.print("CURRENT_MODE:");
        Serial.println(mode == MODE_TRANSFER ? "TRANSFER" : (mode == MODE_LOGGING ? "LOGGING" : "STARTUP"));
      }
      else if (arg1 == "GET_LOCK") {
        Serial.print("CRADLE_LOCK:"); Serial.println(cradleLocked ? "LOCKED" : "UNLOCKED");
      }
      else if (arg1 == "STATUS") {
        Serial.println("----------------------------------------");
        Serial.print("FIRMWARE:    v"); Serial.println(FIRMWARE_VERSION);
        Serial.print("MODE:        "); Serial.println(mode == MODE_TRANSFER ? "TRANSFER" : (mode == MODE_LOGGING ? "LOGGING" : "STARTUP"));
        Serial.print("CRADLE_LOCK: "); Serial.println(cradleLocked ? "LOCKED" : "UNLOCKED");
        Serial.print("SD_CARD:     "); Serial.println(sdReady ? "OK" : "FAIL");
        Serial.print("IMU1:        "); Serial.println(imu1_ready ? "OK" : "FAIL");
        Serial.print("IMU2:        "); Serial.println(imu2_ready ? "OK" : "FAIL");
        Serial.print("RTC:         "); Serial.println(rtcAvailable ? "OK" : "FAIL");
        Serial.print("BATTERY:     "); Serial.print(battVoltage_mV); Serial.print("mV / "); Serial.print(battCurrent_mA); Serial.println("mA");
        Serial.print("BRACE_ID:    "); Serial.println(readBraceID());
        Serial.println("----------------------------------------");
      }
      // ==================== FILE OPERATIONS ====================
      else if (arg1 == "LIST") {
        listFiles();
      }
      else if (arg1 == "LIST_ARCHIVE") {
        listArchiveFiles();
      }
      else if (arg1 == "FILESIZE" && arg2.length() > 0) {
        File f = SD.open(arg2, FILE_READ);
        if (f) { Serial.print("FILESIZE:"); Serial.println(f.size()); f.close(); }
        else { Serial.print("ERROR:FILE_NOT_FOUND:"); Serial.println(arg2); }
      }
      else if (arg1 == "DELETE" && arg2.length() > 0) {
        if (strcmp(arg2.c_str(), CONFIG_FILE) == 0) {
          Serial.println("ERROR:CANNOT_DELETE_CONFIG");
        } else {
          if (dataFile && String(dataFile.name()).equals(arg2)) dataFile.close();
          if (SD.remove(arg2)) { Serial.print("OK:DELETED:"); Serial.println(arg2); }
          else { Serial.print("ERROR:DELETE_FAILED:"); Serial.println(arg2); }
        }
      }
      else if (arg1 == "READ_FILE" && arg2.length() > 0) {
        transmitFile(arg2);
      }
      else if (arg1 == "ARCHIVE" && arg2.length() > 0) {
        moveToArchive(arg2);
      }
      else if (arg1 == "RENAME" && arg2.length() > 0) {
        // CMD:RENAME:oldname:newname — used to flag bad files as x_FILENAME
        int sep = arg2.indexOf(':');
        if (sep > 0) {
          String oldName = arg2.substring(0, sep);
          String newName = arg2.substring(sep + 1);
          if (SD.rename(oldName, newName)) {
            Serial.print("OK:RENAMED:"); Serial.print(oldName);
            Serial.print("->"); Serial.println(newName);
          } else {
            Serial.println("ERROR:RENAME_FAILED");
          }
        } else {
          Serial.println("ERROR:RENAME_NEEDS_TWO_ARGS");
        }
      }
      else if (arg1 == "CLEANUP") {
        cleanupSD();
      }
      // ==================== TIME ====================
      else if (arg1 == "SET_TIME" && arg2.length() > 0) {
        setRTCUnix((uint32_t)strtoul(arg2.c_str(), nullptr, 10));
      }
      else if (arg1 == "GET_TIME") {
        if (rtcAvailable) {
          DateTime now = rtc.now();
          Serial.print("RTC_TIME:"); Serial.println(now.unixtime());
          Serial.printf("RTC_DATE:%04d-%02d-%02d %02d:%02d:%02d\n", 
            now.year(), now.month(), now.day(),
            now.hour(), now.minute(), now.second());
        } else {
          Serial.println("ERROR:RTC_NOT_AVAILABLE");
        }
      }
      // ==================== UPLOAD STATE ====================
      else if (arg1 == "UL_STATE" && arg2.length() > 0) {
        arg2.toUpperCase();
        if (arg2 == "NEGOTIATING") uploadState = UL_NEGOTIATING;
        else if (arg2 == "UPLOADING") uploadState = UL_UPLOADING;
        else if (arg2 == "COMPLETE") uploadState = UL_COMPLETE;
        else uploadState = UL_IDLE;
        Serial.print("OK:UL_STATE:"); Serial.println(arg2);
      }
      // ==================== CRADLE LOCK ====================
      else if (arg1 == "STAY_IN_CRADLE" || arg1 == "LOCK_CRADLE") {
        cradleLocked = true;
        lastModeSwitch = millis();
        Serial.println("OK:CRADLE_LOCKED");
      }
      else if (arg1 == "UNLOCK_CRADLE" || arg1 == "RELEASE_CRADLE") {
        cradleLocked = false;
        Serial.println("OK:CRADLE_UNLOCKED");
      }
      // ==================== UTILITY ====================
      else if (arg1 == "PING") {
        Serial.println("OK:PONG");
      }
      else if (arg1 == "DEBUG_EULER") {
        debugEuler = !debugEuler;
        Serial.print("DEBUG_EULER:"); Serial.println(debugEuler ? "ON" : "OFF");
      }
      else if (arg1 == "RESET_SERIAL") {
        resetSerialConnection();
      }
      else if (arg1 == "HELP") {
        Serial.println("========== NIMBRACE v2.2.6 COMMANDS ==========");
        Serial.println("CMD:VERSION     - Show firmware version");
        Serial.println("CMD:STATUS      - Full device status");
        Serial.println("CMD:INFO        - Show startup banner");
        Serial.println("CMD:GET_ID      - Get brace ID");
        Serial.println("CMD:SET_ID:xxx  - Set brace ID");
        Serial.println("CMD:GET_MODE    - Get current mode");
        Serial.println("CMD:GET_LOCK    - Get cradle lock state");
        Serial.println("CMD:LIST        - List data files (excludes uploaded_)");
        Serial.println("CMD:LIST_ARCHIVE- List uploaded_ files");
        Serial.println("CMD:READ_FILE:x - Download file x");
        Serial.println("CMD:ARCHIVE:x   - Rename x to uploaded_x (instant)");
        Serial.println("CMD:DELETE:x    - Delete file x");
        Serial.println("CMD:CLEANUP     - Delete all data files");
        Serial.println("CMD:SET_TIME:x  - Set RTC (unix time)");
        Serial.println("CMD:GET_TIME    - Get RTC time");
        Serial.println("CMD:STAY_IN_CRADLE - Lock transfer mode");
        Serial.println("CMD:UNLOCK_CRADLE  - Allow logging");
        Serial.println("CMD:RESET_SERIAL   - Reset serial connection");
        Serial.println("CMD:PING        - Health check");
        Serial.println("CMD:HELP        - This help");
        Serial.println("===============================================");
      }
      else {
        Serial.print("ERROR:UNKNOWN_COMMAND:"); Serial.println(arg1);
      }
    }
    commandBuffer = "";
  }
}
