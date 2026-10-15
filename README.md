# ESP32 Multi-Node Power Management System

Master-Slave architecture using **ESP-NOW** for load shedding / demand control.

## Hardware

### Master
| Component          | Pin / Notes                  |
|--------------------|------------------------------|
| ESP32 DevKit       | MAC `B4:BF:E9:6A:11:5C`     |
| 16×2 I2C LCD       | SDA=21, SCL=19, addr `0x27` |
| Button 1 (Mode)    | GPIO 4 (INPUT_PULLUP)       |
| Button 2 (Select)  | GPIO 5                      |
| Button 3 (Action)  | GPIO 18                     |
| Buzzer             | GPIO 13 (LEDC ~2700 Hz)     |

### Slaves (A / B / C)
| Component          | Pin / Notes                          |
|--------------------|--------------------------------------|
| ESP32 DevKit       | See MACs below                       |
| PZEM-004T v4.0     | Serial2: RX=16, TX=17                |
| Inverted 5 V Relay | GPIO 4 (HIGH = OFF, LOW = ON)        |
| Local Switch       | GPIO 19 (INPUT_PULLUP)               |

**Slave MAC addresses**
- A → `B4:BF:E9:E1:47:1C`
- B → `B0:3F:D3:51:CA:F8`
- C → `20:50:0D:4E:D5:54`

## Folder Structure

```
artifacts/
├── common/
│   └── espnow_packets.h      # Shared packet definitions
├── master/
│   └── master.ino            # Full master firmware
├── slave/
│   └── slave.ino             # Slave firmware (change THIS_SLAVE)
├── CONTEXT.md                # Original design notes
└── README.md                 # This file
```

## How to Flash

### 1. Libraries required (Arduino IDE / PlatformIO)

- `LiquidCrystal_I2C` (for master)
- Built-in: `WiFi`, `esp_now`, `HardwareSerial`

No external PZEM library is needed – a minimal Modbus RTU reader is included.

### 2. Master

1. Open `master/master.ino`
2. Select the correct ESP32 board and the Master’s serial port
3. Upload  
   (Packet definitions are already included in the sketch; the `common/` header is kept for reference.)

### 3. Slaves

1. Open `slave/slave.ino`
2. Change the line near the top:
   ```cpp
   #define THIS_SLAVE  0   // 0 = A, 1 = B, 2 = C
   ```
3. Upload to the corresponding physical board
4. Repeat for the other two slaves (changing `THIS_SLAVE` each time)

## UI Modes (Master)

| Mode              | BTN1 (Mode) | BTN2 (Select)     | BTN3 (Action)          |
|-------------------|-------------|-------------------|------------------------|
| Dashboard         | next mode   | –                 | Toggle backlight       |
| Set MDL           | next mode   | Move cursor       | Increment digit        |
| Change Priority   | next mode   | Select slave      | Cycle H → M → L        |
| Load Info         | next mode   | –                 | (reserved)             |

## Behaviour Summary

- Each slave reports active power + relay state ~every 700 ms.
- Master sums the power of all online loads.
- If total > MDL → shed the **lowest-priority** load that is currently ON.
- If total < 92 % of MDL → restore the **highest-priority** load that is currently OFF (with a simple projected-power check).
- Priorities and MDL are stored only on the Master.
- Local switch on a slave forces its relay ON (simple manual override).

## Notes / Future Improvements

- Temperature shown in the original design mock-up is not available from the current hardware (no temp sensor). The Dashboard shows only shed status.
- Hysteresis (92 %) prevents rapid on/off oscillation.
- Lost slaves are marked offline after 3 s of silence.
- For production you may want to add CRC checking on ESP-NOW packets, persistent storage of MDL/priorities (Preferences / NVS), and a more robust PZEM library if higher accuracy is required.
