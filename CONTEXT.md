## LCD Display design
It will be an infinite state machine. Using the 3 button layout found in casio-f91 type watches could be the most simple and straight forward approach I can think of.
- button-1: for changing current [[#Mode]]
- button-2: selection or cursor
- button-3: toggling or incrementing (Mode specific)

### Mode
#### 1. Dashboard
```text
PWR: 185 W / 196 W
26C SHED: 1/3
```
#### 2. Set MDL: 
```text
MDL: 0[1]96 W

```
#### 3. Change priority
```text
PRIORITIES
A:[H] B:[M] C[L]
```
#### 4. Individual load information
```text
A: 040 W B: 080 W
C: 065 W
```

In dashboard mode, button-3 can be used to toggle the backlight of LCD.

---

## Physical setup details
- Microcontroller: ESP32 Devkit
- voltage measurement module: PZEM 004t v4.0
- Relay: 5v 1 Channel Inverted relay.
- LCD display: 16x2 I2C LCD
- buttons: push buttons (x3)
- switch count: 3

### Master node
```
MAC Address: B4:BF:E9:6A:11:5C

16x2 I2C LCD:
  SDA: D21
  SCL: D19

PUSH BUTTONS:
  BTN1: D4
  BTN2: D5
  BTN3: D18

BUZZER: D13
````

### Slave Node
```
Total number of slave nodes: 3 [A, B, C]

#### Mac addresses
  A: B4:BF:E9:E1:47:1C
  B: B0:3F:D3:51:CA:F8
  C: 20:50:0D:4E:D5:54

SWITCH: D19
RELAY: D4
PZEM: (16, 17)[RX2, TX2]
```
