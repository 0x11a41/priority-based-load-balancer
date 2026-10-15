Build a complete ESP32-based multi-node power management system with Master-Slave architecture using ESP-NOW for communication.

### Hardware Overview
**Master Node** (MAC: B4:BF:E9:6A:11:5C)
- ESP32 DevKit
- 16x2 I2C LCD (SDA = GPIO21, SCL = GPIO19, I2C address 0x27)
- 3 push buttons (active LOW, INPUT_PULLUP):
  - BTN1 (GPIO4) → Change Mode
  - BTN2 (GPIO5) → Selection / Cursor
  - BTN3 (GPIO18) → Toggle / Increment (mode-specific)
- Buzzer on GPIO13 (use ESP32 LEDC PWM at ~2700 Hz)

**Slave Nodes** (3 units: A, B, C)
- Slave A MAC: B4:BF:E9:E1:47:1C
- Slave B MAC: B0:3F:D3:51:CA:F8
- Slave C MAC: 20:50:0D:4E:D5:54
- Each slave has:
  - PZEM-004T v4.0 on Serial2 (RX=GPIO16, TX=GPIO17) for real-time power measurement
  - 5V inverted relay on GPIO4 (HIGH = OFF / load disconnected, LOW = ON / load connected)
  - Local switch on GPIO19 (for manual override or status)

### Functional Requirements

#### Communication
- Use ESP-NOW for reliable, low-latency Master ↔ Slave communication.
- Master polls or receives continuous power data (Watts) from all three slaves.
- Master can send commands to individual slaves: turn relay ON/OFF, change priority, etc.
- Handle MAC addresses hardcoded for the three known slaves.
- Implement robust packet structure for: power readings, relay status, priority, and commands.

#### Master UI – Infinite State Machine (Casio F91 style)
Three buttons control a clean state machine with the following modes:

1. **Dashboard** (default)
   ```
   PWR: 185 W / 196 W
   26C SHED: 1/3
   ```
   - Line 1: Current total power / Maximum allowed power (MDL)
   - Line 2: Temperature (if available) + Shed status (how many loads are currently shed)
   - BTN3 toggles LCD backlight

2. **Set MDL** (Maximum Demand Limit)
   ```
   MDL: 0[1]96 W
   ```
   - Cursor (brackets) can be moved left/right with BTN2
   - BTN3 increments the selected digit
   - Save on mode exit or long-press

3. **Change Priority**
   ```
   PRIORITIES
   A:[H] B:[M] C[L]
   ```
   - Cycle priority of each load (High / Medium / Low) using BTN2 + BTN3
   - High priority loads are shed last

4. **Individual Load Information**
   ```
   A: 040 W B: 080 W
   C: 065 W
   ```
   - Live power reading from each slave

#### Core Logic (Master)
- Continuously receive power readings from all three slaves.
- Calculate total power.
- If total power exceeds MDL → automatically shed lowest-priority loads one by one until under limit.
- When power drops sufficiently, restore loads according to priority (High first).
- Support manual override via the UI.
- Buzzer should beep on critical events (shedding started, MDL reached, button long-press confirmation, etc.).

#### Slave Logic
- Continuously read power from PZEM-004T (active power in Watts).
- Send power reading + relay status to Master periodically (e.g. every 500–1000 ms).
- Listen for commands from Master and control the inverted relay accordingly.
- Local switch on GPIO19 can optionally force relay state or report status.

### Technical Requirements
- Clean, modular Arduino code (PlatformIO or Arduino IDE compatible).
- Separate folders/files for Master and Slave.
- Use non-blocking code where possible (millis-based timing, no long delays).
- Debounce all buttons properly.
- LCD updates should be efficient (avoid full redraw every loop).
- Handle ESP-NOW peer management cleanly.
- Include basic error handling (lost slave, PZEM read failure, etc.).
- Comments explaining the state machine and priority shedding logic.

### Deliverables
1. Complete `master.ino` (or PlatformIO project) with full UI state machine + ESP-NOW + power management logic.
2. Complete `slave.ino` that works for any of the three slaves (identify itself by MAC or compile-time define).
3. Shared header(s) for packet structures and constants if useful.
4. Clear pin definitions matching the hardware described above.
5. Brief README explaining how to flash Master and the three Slaves.

Start from the existing skeleton code if helpful, but implement the full system described above.
