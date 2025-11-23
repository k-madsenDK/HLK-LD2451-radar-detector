# HLK-LD2451-radar-detector
small c++ code snip for handling the HLK-LD2451 radar detector

HLK-LD2451 radar on Raspberry Pi Pico (Earle Philhower core)

This sketch shows how to connect and use the Hi-Link HLK-LD2451 radar module with a Raspberry Pi Pico using the Earle Philhower RP2040 core.

It is designed for setups like a Tachyon/Hailo camera system where you only need a binary “object detected” signal and a rough distance, not centimeter‑accurate ranging.
Hardware connections

    Radar UART:
        Pico GP16 / TX0 → HLK-LD2451 RX
        Pico GP17 / RX0 ← HLK-LD2451 TX
    Radar alarm output:
        HLK-LD2451 OT1 → Pico GP18 (configured as digital input)

The sketch uses:

static const uint8_t RADAR_UART_TX_PIN = 16;
static const uint8_t RADAR_UART_RX_PIN = 17;
static const uint8_t RADAR_OT1_PIN     = 18;

On the Philhower core, UART0 is exposed as Serial1. Pins are assigned in setup():

Serial1.setTX(RADAR_UART_TX_PIN);
Serial1.setRX(RADAR_UART_RX_PIN);
radar.begin(115200);

Protocol

The implementation follows the official HLK-LD2451 serial protocol:

    Command / ACK frames (configuration):
        Header: FD FC FB FA
        Tail: 04 03 02 01
        Length: 2 bytes (little endian), number of bytes in the payload
        Payload (command frame):
        command word (2 bytes LE) + command value (N bytes)
        Payload (ACK frame):
        (command word | 0x0100) (2 bytes LE) + return value (N bytes)

    Data frames (targets):
        Header: F4 F3 F2 F1
        Tail: F8 F7 F6 F5
        Payload format:
        alarm (1 byte) + targetCount (1 byte) + targets...
        Each target is 5 bytes:
            angle (1 byte) → actual angle = value − 0x80, in degrees
            distance (1 byte) → meters (very coarse, ~2–3 m minimum range)
            speed (2 bytes LE)
            SNR (1 byte)

Speed decoding (from the datasheet example):

    0x003C → 60 km/h approaching
    0x013C → 60 km/h away

So the code interprets:

uint16_t speedRaw = wordLE(lo, hi);
uint8_t  speedVal = speedRaw & 0x00FF;          // 0..120 km/h
bool     approaching = ((speedRaw & 0x0100) == 0); // 0x00xx = towards, 0x01xx = away

Ld2451Radar class

The Ld2451Radar class wraps:

    Configuration session handling:
        enterConfig() → sends Enable Configuration (0x00FF)
        endConfig() → sends End Configuration (0x00FE)
    Detection parameters:
        readDetectionParams()
        setDetectionParams()
    Sensitivity:
        readSensitivity()
        setSensitivity()
    Firmware and maintenance:
        readFirmwareVersion()
        restoreFactory()
        restartModule()
    Runtime data:
        readTargets() → parses data frames into an array of Ld2451Target

Targets carry:

struct Ld2451Target {
  int8_t   angleDeg;   // -128..+127, 0° ≈ straight ahead
  uint8_t  distanceM;  // meters, coarse resolution (close range ~2–3 m)
  bool     approaching;// true = moving towards the radar
  uint16_t speedKmh;   // km/h, 1 LSB = 1 km/h
  uint8_t  snr;        // 0..255
};

For a Hailo/camera use case you will typically only care about:

    targetCount > 0
    OT1 == 1
    maybe distanceM <= some_threshold (e.g. 3–5 m)

and then trigger your camera re‑positioning / classification pipeline.
Serial setup menu

Press m + Enter in the serial monitor (115200 baud) to enter a simple setup menu:

    1 – Read detection parameters (maxDist, direction, min speed, delay)
    2 – Set detection parameters
    3 – Read sensitivity (trigger count, SNR level)
    4 – Set sensitivity
    5 – Read firmware version
    6 – Factory reset + restart module
    x – Exit menu

Configuration is always wrapped in the correct sequence:

    Send Enable Configuration (enterConfig()).
    Send one or more configuration commands (set/read).
    Send End Configuration (endConfig()).

If ACK is missing or indicates failure, the helper functions return false.
Notes for close-range use

The HLK-LD2451 is a long-range radar (up to ~100 m). At very short distances:

    Reported distance is quite coarse and often sticks at 2–3 m.
    Speed may stay at 0 km/h for slow motion.
    OT1 and SNR are still very useful as “presence/motion detected” indicators.

For a Hailo camera system, a practical approach is:

    Treat “at least 1 target + high SNR + OT1 = 1” as object detected.
    Use distanceM only as a rough near/far hint (e.g. < 3 m = very close).
