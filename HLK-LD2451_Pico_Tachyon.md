# HLK-LD2451 radar + Raspberry Pi Pico (Earle Philhower)  
for Tachyon / Hailo camera positioning

This document describes how to connect and use the **Hi-Link HLK‑LD2451** 24 GHz vehicle status radar with a **Raspberry Pi Pico** (Earle Philhower Arduino core), targeting a Tachyon / Hailo‑based device where the radar is used to:

- detect **presence/motion of vehicles** (or large objects),
- give a **coarse distance / angle**,
- trigger **camera movement** and then let Hailo + camera do object classification.

Precise ranging below ~2–3 m is *not* required in this use‑case; the radar mainly acts as a robust motion/presence sensor.

---

## 1. Hardware overview

From `hlk_ld2451_user_manual2.md` (user manual, V1.00, 2024‑05‑07):

- Product: **HLK‑LD2451 Vehicle Status Detection Module**
- Frequency: **24 GHz FMCW**
- Max detection distance: **up to 100 m**
- Typical use: outdoor vehicle detection (approach / depart, speed, distance, angle)
- Interfaces:
  - **UART** (TTL, 3.3 V logic)
  - **2× GPIO** (OT1, OT2)
- Power:
  - **5 V supply**, > 300 mA capacity
  - Average current ≈ 107 mA

Relevant pinout (section 4.2):

| Pin | Name | Function                                |
| --- | ---- | --------------------------------------- |
| 1   | VIN  | 5 V power input                         |
| 2   | GND  | Ground                                  |
| 3   | OT1  | GPIO1: approach alarm output            |
| 4   | TX   | UART TX (module → Pico RX)              |
| 5   | RX   | UART RX (module ← Pico TX)              |
| 6   | OT2  | GPIO2 (currently unused in this sketch) |

Electrical notes:

- Module **IO level is 3.3 V**, compatible with RP2040.
- **VIN must be 5 V**, not 3.3 V.

Field of view (from section 6):

- Detection angle: **±20°** (horizontal), so ~40° total.

---

## 2. Connection to Raspberry Pi Pico (Earle Philhower core)

The example uses **UART0 as Serial1**:

- Pico **GP16 / TX0** → HLK‑LD2451 **RX**
- Pico **GP17 / RX0** ← HLK‑LD2451 **TX**
- HLK‑LD2451 **OT1** → Pico **GP18** (digital input)

Power:

- HLK‑LD2451 **VIN** → 5 V (from external 5 V supply or regulated 5 V rail)
- HLK‑LD2451 **GND** → Pico GND

On Philhower core, assign UART pins:

```cpp
Serial1.setTX(16);  // GP16
Serial1.setRX(17);  // GP17
Serial1.begin(115200);
```

---

## 3. Serial protocol (summary)

From the HLK‑LD2451 serial protocol document:

### 3.1 Configuration commands / ACK

- **Command and ACK frames**:

  - Header: `FD FC FB FA`
  - Tail:   `04 03 02 01`
  - Data length: 2 bytes, little‑endian, number of bytes in the data payload
  - Command frame payload:
    - `command word (2 bytes LE) + command value (N bytes)`
  - ACK frame payload:
    - `(command word | 0x0100) (2 bytes LE) + return value (N bytes)`

- Key commands:

  - **Enable configuration**  
    - Word: `0x00FF`  
    - Value: `0x0001` (2 bytes LE)  
    - Must be sent **before any other config command**.
  - **End configuration**  
    - Word: `0x00FE`  
    - No value  
    - Must be sent after configuration is done.
  - **Detection parameters** (`0x0002` / `0x0012`)
  - **Sensitivity** (`0x0003` / `0x0013`)
  - **Firmware version** (`0x00A0`)
  - **Factory reset** (`0x00A2`)
  - **Restart module** (`0x00A3`)

Configuration sequence (from the protocol figure):

1. Send **Enable config** (`0x00FF 0x0001`), wait for ACK.
2. Send one or more config commands, wait for ACK after each.
3. Send **End config** (`0x00FE`), wait for ACK.

If any ACK is missing or indicates failure, the command did not take effect.

### 3.2 Data frames (target output)

- **Data frames** (runtime target output):

  - Header: `F4 F3 F2 F1`
  - Tail:   `F8 F7 F6 F5`
  - Payload format:

    ```text
    alarm (1 byte)
    targetCount (1 byte)
    for each target (5 bytes):
      angle (1 byte)
      distance (1 byte)
      speed (2 bytes LE)
      snr (1 byte)
    ```

- Fields:

  - **angle**: 1 byte, actual angle = value − `0x80` (degrees).
  - **distance**: 1 byte, meters. Close range (<2–3 m) is coarse and often reported as 2–3 m continuously.
  - **speed**: 2 bytes little‑endian:
    - Example from manual:
      - `0x003C` → 60 km/h **approaching**
      - `0x013C` → 60 km/h **away**
    - Interpretation in code:
      - `speedKmh = speedRaw & 0x00FF` (LSB is km/h)
      - `approaching = ((speedRaw & 0x0100) == 0)` (bit 8 = direction)
  - **snr**: 0–255.

---

## 4. Ld2451Radar C++ class

The sketch defines a small, self‑contained C++ class `Ld2451Radar` to:

- Wrap the configuration protocol (enter/exit config, set/read parameters).
- Parse data frames into a C‑struct for easy use in higher‑level logic.
- Provide a simple **serial setup menu** to adjust:
  - max detection distance
  - direction mode (near / far / both)
  - minimum speed
  - no‑target delay
  - sensitivity (trigger count, SNR threshold)
  - firmware version
  - factory reset + restart

For Tachyon/Hailo, you typically care about:

- “Is there *any* target?” (`targetCount > 0`)
- “Is OT1 high?” (radar indicates approaching motion)
- Rough distance (e.g. `< 3 m` = close, trigger camera re‑position)

Accuracy below 2–3 m is limited; that’s acceptable in this design.

---

## 5. Sketch: HLK‑LD2451 + Pico with setup menu

See `radarld2451_with_menu.ino` in this Space for the full example.

Key points:

- **begin()**:

  ```cpp
  Serial1.setTX(RADAR_UART_TX_PIN);
  Serial1.setRX(RADAR_UART_RX_PIN);
  radar.begin(115200);
  ```

- **Normal operation**:

  - Read **OT1** on GP18:

    ```cpp
    int ot1 = digitalRead(RADAR_OT1_PIN); // 1 = approaching alarm
    ```

  - Parse radar data:

    ```cpp
    Ld2451Target targets[4];
    int n = radar.readTargets(targets, 4);
    if (n > 0) {
      // use targets[0..n-1]
    }
    ```

- **Setup menu** (USB serial @115200):

  - Press `m` + Enter to open.
  - Commands:
    - `1`: Read detection params.
    - `2`: Set detection params.
    - `3`: Read sensitivity.
    - `4`: Set sensitivity.
    - `5`: Read firmware version.
    - `6`: Factory reset + restart.
    - `x`: Exit menu.

Configuration commands always follow:

```cpp
if (radar.enterConfig()) {
  // send set/read command(s)
  radar.endConfig();
}
```

---

## 6. Using HLK‑LD2451 as a trigger for Tachyon / Hailo

For a combined Tachyon + Hailo camera system, a practical pattern is:

1. **Radar monitors continuously**:
   - Check `OT1` and/or `targetCount > 0` in the Pico.
2. On detection:
   - If `OT1 == 1` (approach) or `targetCount > 0` with high SNR, and
   - Distance is within your interest zone, e.g. `distanceM <= 3`:
3. **Move camera / gimbal** to center the radar target region.
4. Let **Hailo + camera**:
   - Run object detection (car, truck, human, etc.).
   - Make high‑level decisions (open gate, alarm, log event…).

Radar gives a fast, robust trigger (typically 60–200 ms latency), while the camera takes care of recognition.

---

## 7. Performance and installation notes

From `hlk_ld2451_user_manual2.md`:

- **Detection distance**: up to 100 m.
- **Detection angle**: ±20° (≈40° FOV).
- **Sweep bandwidth**: <200 MHz.
- **Operating temperature**: −40 … 85 °C.
- **Dimensions**: 70 mm × 35 mm.
- **Power**: 5 V, > 300 mA.

Installation hints:

- Keep the **antenna area unobstructed** and facing the region of interest.
- The module should be mechanically stable (no shaking of the radar itself).
- Avoid motion behind the radar (back‑lobe) or shield its rear with metal if necessary.
- If mounted in an enclosure:
  - Use RF‑transparent material at 24 GHz.
  - Avoid metal or EM‑shielding material in front of the antenna.

---

## 8. Summary

- HLK‑LD2451 is a 24 GHz FMCW radar optimized for vehicle detection (up to 100 m, ±20°).
- The provided `Ld2451Radar` C++ class for **Raspberry Pi Pico (Earle Philhower)**:
  - Implements the official HLK protocol for configuration and data.
  - Parses targets (angle, distance, speed, SNR).
  - Provides a serial setup menu and factory reset.
- For Tachyon / Hailo use:
  - Treat radar as a **presence / motion sensor** with rough distance.
  - Trigger camera repositioning when OT1 or targets indicate something in front of the radar.
  - Let Hailo handle object classification and decision logic.

This combination gives a robust, low‑latency “radar‑guided” camera system where precise distance measurement is less important than reliable detection and direction of motion.