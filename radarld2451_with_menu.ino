#include <Arduino.h>

// HLK-LD2451 radar on Raspberry Pi Pico (Earle Philhower core)
// UART0 used as Serial1:
//   TX0 = GP16 -> radar RX
//   RX0 = GP17 <- radar TX
// OT1 radar output on GP18
static const uint8_t RADAR_UART_TX_PIN = 16;
static const uint8_t RADAR_UART_RX_PIN = 17;
static const uint8_t RADAR_OT1_PIN     = 18;

// ------------- Protocol constants (from HLK-LD2451 manual) -------------
static const uint8_t CMD_HEADER[4]  = { 0xFD, 0xFC, 0xFB, 0xFA };
static const uint8_t CMD_TAIL[4]    = { 0x04, 0x03, 0x02, 0x01 };
static const uint8_t DATA_HEADER[4] = { 0xF4, 0xF3, 0xF2, 0xF1 };
static const uint8_t DATA_TAIL[4]   = { 0xF8, 0xF7, 0xF6, 0xF5 };

enum class Ld2451Cmd : uint16_t {
  EnableConfig        = 0x00FF,
  EndConfig           = 0x00FE,
  SetDetectionParams  = 0x0002,
  ReadDetectionParams = 0x0012,
  SetSensitivity      = 0x0003,
  ReadSensitivity     = 0x0013,
  ReadFirmwareVersion = 0x00A0,
  RestoreFactory      = 0x00A2,
  RestartModule       = 0x00A3,
};

struct Ld2451Target {
  int8_t   angleDeg;   // -128..+127, 0° is straight ahead
  uint8_t  distanceM;  // meters (coarse, close range is not very accurate)
  bool     approaching;// true: moving towards radar, false: moving away
  uint16_t speedKmh;   // km/h, 1 LSB = 1 km/h
  uint8_t  snr;        // signal-to-noise ratio 0..255
};

struct Ld2451DetectionParams {
  uint8_t maxDistanceM;     // 0x0A..0xFF, meters
  uint8_t directionMode;    // 0=only away, 1=only approach, 2=both
  uint8_t minSpeedKmh;      // 0..120, minimum speed for detection
  uint8_t noTargetDelaySec; // 0..255, delay before "no target" state
};

struct Ld2451Sensitivity {
  uint8_t triggerTimes; // number of consecutive detections (1..10, default 4)
  uint8_t snrLevel;     // SNR threshold, higher = less sensitive
  uint8_t ext1;         // reserved
  uint8_t ext2;         // reserved
};

struct Ld2451FirmwareVersion {
  uint16_t type;   // 0x2451
  uint16_t major;  // e.g. 0x0101
  uint32_t minor;  // e.g. 0x24150510
};

// --------------------- Ld2451Radar class ---------------------
class Ld2451Radar {
public:
  explicit Ld2451Radar(HardwareSerial &port)
    : _serial(port), _dataLen(0) {}

  // Serial1 pins and baud are configured in setup().
  void begin(uint32_t baud = 115200) {
    _serial.begin(baud);
  }

  // ---- Configuration session handling (enable / end) ----
  bool enterConfig(uint32_t timeoutMs = 200) {
    // Enable configuration command:
    //   Command word: 0x00FF
    //   Command value: 0x0001
    uint8_t value[2] = { 0x01, 0x00 }; // 0x0001 little endian
    if (!sendCommand(Ld2451Cmd::EnableConfig, value, sizeof(value)))
      return false;

    uint8_t ack[16];
    size_t n = readAck(ack, sizeof(ack), timeoutMs);
    if (n < 6) return false;

    // ACK command word = original | 0x0100
    uint16_t ackCmd = wordLE(ack[0], ack[1]);
    if (ackCmd != ((uint16_t)Ld2451Cmd::EnableConfig | 0x0100)) return false;

    uint16_t status = wordLE(ack[2], ack[3]); // 0 = success
    return status == 0;
  }

  bool endConfig(uint32_t timeoutMs = 200) {
    // End configuration command: 0x00FE, no value
    if (!sendCommand(Ld2451Cmd::EndConfig, nullptr, 0))
      return false;

    uint8_t ack[8];
    size_t n = readAck(ack, sizeof(ack), timeoutMs);
    if (n < 4) return false;

    uint16_t ackCmd = wordLE(ack[0], ack[1]);
    if (ackCmd != ((uint16_t)Ld2451Cmd::EndConfig | 0x0100)) return false;

    uint16_t status = wordLE(ack[2], ack[3]);
    return status == 0;
  }

  // ---- Detection parameters (distance, direction, min speed, delay) ----
  bool readDetectionParams(Ld2451DetectionParams &out, uint32_t timeoutMs = 200) {
    if (!sendCommand(Ld2451Cmd::ReadDetectionParams, nullptr, 0))
      return false;

    uint8_t ack[8];
    size_t n = readAck(ack, sizeof(ack), timeoutMs);
    if (n < 8) return false;

    uint16_t ackCmd = wordLE(ack[0], ack[1]);
    if (ackCmd != ((uint16_t)Ld2451Cmd::ReadDetectionParams | 0x0100)) return false;

    uint16_t status = wordLE(ack[2], ack[3]);
    if (status != 0) return false;

    out.maxDistanceM     = ack[4];
    out.directionMode    = ack[5];
    out.minSpeedKmh      = ack[6];
    out.noTargetDelaySec = ack[7];
    return true;
  }

  bool setDetectionParams(const Ld2451DetectionParams &p, uint32_t timeoutMs = 200) {
    uint8_t value[4] = {
      p.maxDistanceM,
      p.directionMode,
      p.minSpeedKmh,
      p.noTargetDelaySec
    };
    if (!sendCommand(Ld2451Cmd::SetDetectionParams, value, sizeof(value)))
      return false;

    uint8_t ack[8];
    size_t n = readAck(ack, sizeof(ack), timeoutMs);
    if (n < 4) return false;

    uint16_t ackCmd = wordLE(ack[0], ack[1]);
    if (ackCmd != ((uint16_t)Ld2451Cmd::SetDetectionParams | 0x0100)) return false;

    uint16_t status = wordLE(ack[2], ack[3]);
    return status == 0;
  }

  // ---- Sensitivity handling ----
  bool readSensitivity(Ld2451Sensitivity &out, uint32_t timeoutMs = 200) {
    if (!sendCommand(Ld2451Cmd::ReadSensitivity, nullptr, 0))
      return false;

    uint8_t ack[8];
    size_t n = readAck(ack, sizeof(ack), timeoutMs);
    if (n < 8) return false;

    uint16_t ackCmd = wordLE(ack[0], ack[1]);
    if (ackCmd != ((uint16_t)Ld2451Cmd::ReadSensitivity | 0x0100)) return false;

    uint16_t status = wordLE(ack[2], ack[3]);
    if (status != 0) return false;

    out.triggerTimes = ack[4];
    out.snrLevel     = ack[5];
    out.ext1         = ack[6];
    out.ext2         = ack[7];
    return true;
  }

  bool setSensitivity(const Ld2451Sensitivity &s, uint32_t timeoutMs = 200) {
    uint8_t value[4] = { s.triggerTimes, s.snrLevel, s.ext1, s.ext2 };
    if (!sendCommand(Ld2451Cmd::SetSensitivity, value, sizeof(value)))
      return false;

    uint8_t ack[8];
    size_t n = readAck(ack, sizeof(ack), timeoutMs);
    if (n < 4) return false;

    uint16_t ackCmd = wordLE(ack[0], ack[1]);
    if (ackCmd != ((uint16_t)Ld2451Cmd::SetSensitivity | 0x0100)) return false;

    uint16_t status = wordLE(ack[2], ack[3]);
    return status == 0;
  }

  // ---- Firmware / factory reset / restart ----
  bool readFirmwareVersion(Ld2451FirmwareVersion &out, uint32_t timeoutMs = 200) {
    if (!sendCommand(Ld2451Cmd::ReadFirmwareVersion, nullptr, 0))
      return false;

    uint8_t ack[16];
    size_t n = readAck(ack, sizeof(ack), timeoutMs);
    if (n < 12) return false;

    uint16_t ackCmd = wordLE(ack[0], ack[1]);
    if (ackCmd != ((uint16_t)Ld2451Cmd::ReadFirmwareVersion | 0x0100)) return false;

    uint16_t status = wordLE(ack[2], ack[3]);
    if (status != 0) return false;

    out.type  = wordLE(ack[4], ack[5]);
    out.major = wordLE(ack[6], ack[7]);
    out.minor = (uint32_t)ack[8] |
                ((uint32_t)ack[9]  << 8) |
                ((uint32_t)ack[10] << 16) |
                ((uint32_t)ack[11] << 24);
    return true;
  }

  bool restoreFactory(uint32_t timeoutMs = 200) {
    if (!sendCommand(Ld2451Cmd::RestoreFactory, nullptr, 0))
      return false;

    uint8_t ack[8];
    size_t n = readAck(ack, sizeof(ack), timeoutMs);
    if (n < 4) return false;

    uint16_t ackCmd = wordLE(ack[0], ack[1]);
    if (ackCmd != ((uint16_t)Ld2451Cmd::RestoreFactory | 0x0100)) return false;

    uint16_t status = wordLE(ack[2], ack[3]);
    return status == 0;
  }

  bool restartModule(uint32_t timeoutMs = 200) {
    if (!sendCommand(Ld2451Cmd::RestartModule, nullptr, 0))
      return false;

    uint8_t ack[8];
    size_t n = readAck(ack, sizeof(ack), timeoutMs);
    if (n < 4) return false;

    uint16_t ackCmd = wordLE(ack[0], ack[1]);
    if (ackCmd != ((uint16_t)Ld2451Cmd::RestartModule | 0x0100)) return false;

    uint16_t status = wordLE(ack[2], ack[3]);
    return status == 0;
  }

  // ---- Data frames: parse radar target output ----
  // Returns number of targets parsed (0 if no complete frame).
  int readTargets(Ld2451Target *targets, int maxTargets) {
    // Fill RX buffer from UART
    while (_serial.available() > 0 && _dataLen < sizeof(_dataBuf)) {
      _dataBuf[_dataLen++] = (uint8_t)_serial.read();
    }

    while (true) {
      size_t start = findHeader(DATA_HEADER, 4);
      if (start == SIZE_MAX) {
        // Keep only last few bytes in case header was split
        shrinkBuffer(3);
        return 0;
      }
      if (start + 6 > _dataLen) return 0; // need more for length

      uint16_t payloadLen = wordLE(_dataBuf[start + 4], _dataBuf[start + 5]);
      uint16_t totalLen   = 4 + 2 + payloadLen + 4; // header + len + payload + tail
      if (start + totalLen > _dataLen) return 0;    // incomplete

      size_t tailPos = start + totalLen - 4;
      if (!matchAt(DATA_TAIL, 4, tailPos)) {
        // Bad header, drop one byte and resync
        dropBytes(start + 1);
        continue;
      }

      const uint8_t *payload = _dataBuf + start + 6;
      dropBytes(start + totalLen);

      if (payloadLen < 2) return 0;

      uint8_t alarm       = payload[0]; // not used here
      uint8_t targetCount = payload[1];
      if (targetCount == 0 || maxTargets <= 0 || !targets) return 0;

      size_t offset = 2;
      int written = 0;
      for (uint8_t i = 0; i < targetCount && written < maxTargets; ++i) {
        if (offset + 5 > payloadLen) break;

        uint8_t  angleRaw = payload[offset + 0];
        uint8_t  dist     = payload[offset + 1];
        uint16_t speedRaw = wordLE(payload[offset + 2], payload[offset + 3]);
        uint8_t  snr      = payload[offset + 4];
        offset += 5;

        Ld2451Target &t = targets[written++];
        t.angleDeg  = (int8_t)((int)angleRaw - 0x80);
        t.distanceM = dist;

        // According to the manual:
        //   0x003C → 60 km/h approaching
        //   0x013C → 60 km/h away
        uint8_t speedVal    = speedRaw & 0x00FF;          // 0..120 km/h
        bool    approaching = ((speedRaw & 0x0100) == 0); // 0x00xx=approach, 0x01xx=away

        t.approaching = approaching;
        t.speedKmh    = speedVal;
        t.snr         = snr;
      }
      (void)alarm;
      return written;
    }
  }

private:
  HardwareSerial &_serial;
  uint8_t _dataBuf[256];
  size_t  _dataLen;

  static uint16_t wordLE(uint8_t lo, uint8_t hi) {
    return (uint16_t)lo | ((uint16_t)hi << 8);
  }

  bool sendCommand(Ld2451Cmd cmd, const uint8_t *value, uint16_t valueLen) {
    uint8_t buf[32];
    uint16_t idx = 0;

    // Frame header
    memcpy(&buf[idx], CMD_HEADER, 4); idx += 4;

    // Data length = 2 bytes command word + N bytes command value
    uint16_t dataLen = 2 + valueLen;
    buf[idx++] = (uint8_t)(dataLen & 0xFF);
    buf[idx++] = (uint8_t)(dataLen >> 8);

    // Command word (little endian)
    uint16_t cmdWord = (uint16_t)cmd;
    buf[idx++] = (uint8_t)(cmdWord & 0xFF);
    buf[idx++] = (uint8_t)(cmdWord >> 8);

    // Command value (if any)
    for (uint16_t i = 0; i < valueLen; ++i) buf[idx++] = value[i];

    // Frame tail
    memcpy(&buf[idx], CMD_TAIL, 4); idx += 4;

    _serial.write(buf, idx);
    _serial.flush();
    return true;
  }

  // Read ACK frame and return number of bytes in the ACK payload
  // (command word & 0x0100 + return value).
  size_t readAck(uint8_t *ackBuf, size_t ackBufSize, uint32_t timeoutMs) {
    uint32_t startMs = millis();
    enum State { FIND_HEADER, READ_LEN, READ_DATA, READ_TAIL } state = FIND_HEADER;
    uint8_t headerBuf[4] = {0};
    uint8_t tailBuf[4]   = {0};
    uint16_t neededDataLen = 0;
    uint16_t idx = 0;

    while (millis() - startMs < timeoutMs) {
      if (_serial.available() <= 0) {
        delay(1);
        continue;
      }
      uint8_t b = (uint8_t)_serial.read();
      switch (state) {
        case FIND_HEADER:
          headerBuf[0] = headerBuf[1];
          headerBuf[1] = headerBuf[2];
          headerBuf[2] = headerBuf[3];
          headerBuf[3] = b;
          if (memcmp(headerBuf, CMD_HEADER, 4) == 0) {
            state = READ_LEN;
            idx = 0;
          }
          break;

        case READ_LEN:
          ((uint8_t*)&neededDataLen)[idx++] = b;
          if (idx >= 2) {
            idx = 0;
            if (neededDataLen > ackBufSize) return 0;
            state = READ_DATA;
          }
          break;

        case READ_DATA:
          ackBuf[idx++] = b;
          if (idx >= neededDataLen) {
            idx = 0;
            state = READ_TAIL;
          }
          break;

        case READ_TAIL:
          tailBuf[idx++] = b;
          if (idx >= 4) {
            if (memcmp(tailBuf, CMD_TAIL, 4) == 0) return neededDataLen;
            return 0;
          }
          break;
      }
    }
    return 0; // timeout
  }

  size_t findHeader(const uint8_t *header, size_t len) const {
    if (_dataLen < len) return SIZE_MAX;
    for (size_t i = 0; i + len <= _dataLen; ++i) {
      bool ok = true;
      for (size_t j = 0; j < len; ++j) {
        if (_dataBuf[i + j] != header[j]) { ok = false; break; }
      }
      if (ok) return i;
    }
    return SIZE_MAX;
  }

  bool matchAt(const uint8_t *pattern, size_t len, size_t pos) const {
    if (pos + len > _dataLen) return false;
    for (size_t i = 0; i < len; ++i) {
      if (_dataBuf[pos + i] != pattern[i]) return false;
    }
    return true;
  }

  void dropBytes(size_t count) {
    if (count >= _dataLen) { _dataLen = 0; return; }
    memmove(_dataBuf, _dataBuf + count, _dataLen - count);
    _dataLen -= count;
  }

  void shrinkBuffer(size_t keep) {
    if (_dataLen <= keep) return;
    memmove(_dataBuf, _dataBuf + (_dataLen - keep), keep);
    _dataLen = keep;
  }
};

// --------------------- Simple serial setup menu ---------------------
bool g_inMenu = false;

int readIntBlocking() {
  String s;
  while (true) {
    while (Serial.available()) {
      char c = Serial.read();
      if (c == '\r' || c == '\n') {
        if (s.length() > 0) return s.toInt();
      } else {
        s += c;
      }
    }
  }
}

void printMenu() {
  Serial.println();
  Serial.println(F("=== HLK-LD2451 setup menu ==="));
  Serial.println(F("1: Read detection params"));
  Serial.println(F("2: Set detection params"));
  Serial.println(F("3: Read sensitivity"));
  Serial.println(F("4: Set sensitivity"));
  Serial.println(F("5: Read firmware version"));
  Serial.println(F("6: Factory reset + restart"));
  Serial.println(F("x: Exit menu"));
  Serial.print(F("Choice: "));
}

Ld2451Radar radar(Serial1);

void handleMenuChar(char c) {
  if (c == '1') {
    Serial.println(F("\n[Read detection params]"));
    if (radar.enterConfig()) {
      Ld2451DetectionParams p{};
      if (radar.readDetectionParams(p)) {
        Serial.print(F("  maxDist(m): ")); Serial.println(p.maxDistanceM);
        Serial.print(F("  dirMode  : ")); Serial.println(p.directionMode);
        Serial.print(F("  minSpeed : ")); Serial.println(p.minSpeedKmh);
        Serial.print(F("  noTgtDly : ")); Serial.println(p.noTargetDelaySec);
      } else {
        Serial.println(F("  ERROR reading params"));
      }
      radar.endConfig();
    } else {
      Serial.println(F("  enterConfig() failed"));
    }

  } else if (c == '2') {
    Serial.println(F("\n[Set detection params]"));
    Ld2451DetectionParams p{};
    Serial.print(F("Max distance (m, 10-255): "));
    p.maxDistanceM = (uint8_t)readIntBlocking();
    Serial.print(F("\nDirection (0=away,1=approach,2=both): "));
    p.directionMode = (uint8_t)readIntBlocking();
    Serial.print(F("\nMin speed (km/h 0-120): "));
    p.minSpeedKmh = (uint8_t)readIntBlocking();
    Serial.print(F("\nNo-target delay (s 0-255): "));
    p.noTargetDelaySec = (uint8_t)readIntBlocking();
    Serial.println();

    if (radar.enterConfig()) {
      if (radar.setDetectionParams(p)) {
        Serial.println(F("  OK: params written"));
      } else {
        Serial.println(F("  ERROR writing params"));
      }
      radar.endConfig();
    } else {
      Serial.println(F("  enterConfig() failed"));
    }

  } else if (c == '3') {
    Serial.println(F("\n[Read sensitivity]"));
    if (radar.enterConfig()) {
      Ld2451Sensitivity s{};
      if (radar.readSensitivity(s)) {
        Serial.print(F("  triggerTimes: ")); Serial.println(s.triggerTimes);
        Serial.print(F("  snrLevel    : ")); Serial.println(s.snrLevel);
      } else {
        Serial.println(F("  ERROR reading sensitivity"));
      }
      radar.endConfig();
    } else {
      Serial.println(F("  enterConfig() failed"));
    }

  } else if (c == '4') {
    Serial.println(F("\n[Set sensitivity]"));
    Ld2451Sensitivity s{};
    Serial.print(F("Trigger times (1-10, default 4): "));
    s.triggerTimes = (uint8_t)readIntBlocking();
    Serial.print(F("\nSNR level (0-8, higher=less sensitive): "));
    s.snrLevel = (uint8_t)readIntBlocking();
    s.ext1 = 0;
    s.ext2 = 0;
    Serial.println();

    if (radar.enterConfig()) {
      if (radar.setSensitivity(s)) {
        Serial.println(F("  OK: sensitivity written"));
      } else {
        Serial.println(F("  ERROR writing sensitivity"));
      }
      radar.endConfig();
    } else {
      Serial.println(F("  enterConfig() failed"));
    }

  } else if (c == '5') {
    Serial.println(F("\n[Read firmware version]"));
    if (radar.enterConfig()) {
      Ld2451FirmwareVersion v{};
      if (radar.readFirmwareVersion(v)) {
        Serial.print(F("  type : 0x")); Serial.println(v.type, HEX);
        Serial.print(F("  major: 0x")); Serial.println(v.major, HEX);
        Serial.print(F("  minor: 0x")); Serial.println(v.minor, HEX);
      } else {
        Serial.println(F("  ERROR reading version"));
      }
      radar.endConfig();
    } else {
      Serial.println(F("  enterConfig() failed"));
    }

  } else if (c == '6') {
    Serial.println(F("\n[Factory reset + restart]"));
    if (radar.enterConfig()) {
      bool ok = radar.restoreFactory();
      Serial.println(ok ? F("  Factory reset OK") : F("  Factory reset FAILED"));
      radar.endConfig();
    } else {
      Serial.println(F("  enterConfig() failed"));
    }

    // After factory reset, restart module to apply changes
    if (radar.enterConfig()) {
      bool ok2 = radar.restartModule();
      Serial.println(ok2 ? F("  Restart command sent") : F("  Restart FAILED"));
      radar.endConfig();
    }

  } else if (c == 'x' || c == 'X') {
    Serial.println(F("\n[Exit menu]"));
    g_inMenu = false;
  }
}

// --------------------- Arduino setup / loop ---------------------
void setup() {
  Serial.begin(115200);
  while (!Serial) {}

  pinMode(RADAR_OT1_PIN, INPUT);

  // Assign UART0 pins (Earle Philhower core)
  Serial1.setTX(RADAR_UART_TX_PIN);
  Serial1.setRX(RADAR_UART_RX_PIN);

  radar.begin(115200);

  Serial.println(F("HLK-LD2451 + Pico (Philhower)"));
  Serial.println(F("Normal run. Press 'm' + Enter for setup menu."));
}

void loop() {
  // Enter setup menu on 'm'
  if (!g_inMenu && Serial.available()) {
    char c = Serial.read();
    if (c == 'm' || c == 'M') {
      g_inMenu = true;
      printMenu();
    }
  }

  if (g_inMenu) {
    // Menu handling, no radar data processing
    if (Serial.available()) {
      char c = Serial.read();
      if (c != '\r' && c != '\n') {
        handleMenuChar(c);
        if (g_inMenu) printMenu();
      }
    }
    // Drain radar serial while in menu
    while (Serial1.available()) (void)Serial1.read();
    delay(10);
    return;
  }

  // Normal runtime: monitor OT1 and print targets
  static int lastOt1 = -1;
  int ot1 = digitalRead(RADAR_OT1_PIN);
  if (ot1 != lastOt1) {
    Serial.print(F("OT1 = ")); Serial.println(ot1);
    lastOt1 = ot1;
  }

  Ld2451Target ts[4];
  int n = radar.readTargets(ts, 4);
  if (n > 0) {
    Serial.print(F("Targets: ")); Serial.println(n);
    for (int i = 0; i < n; ++i) {
      Serial.print(F("  T")); Serial.print(i + 1);
      Serial.print(F(" dist=")); Serial.print(ts[i].distanceM);  // coarse, ~2–3m minimum
      Serial.print(F("m angle=")); Serial.print(ts[i].angleDeg);
      Serial.print(F("deg v=")); Serial.print(ts[i].speedKmh);
      Serial.print(F("km/h "));
      Serial.print(ts[i].approaching ? F("->") : F("<-"));
      Serial.print(F(" SNR=")); Serial.println(ts[i].snr);
    }
  }

  delay(10);
}