/*
 * ============================================================
 *  Master Node - ESP32 Power Management System
 *  MAC: B4:BF:E9:6A:11:5C
 *
 *  Features:
 *   - 16x2 I2C LCD + 3-button Casio-style UI (state machine)
 *   - ESP-NOW communication with 3 slaves
 *   - Automatic load shedding based on priority + MDL
 *   - Buzzer feedback
 * ============================================================
 */

#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <Preferences.h>
#include "driver/gpio.h"

Preferences prefs;   // NVS storage for MDL + priorities

// ============================================================
// Shared packet definitions (also in common/espnow_packets.h)
// ============================================================
enum PacketType : uint8_t {
  PKT_POWER_REPORT = 1,
  PKT_COMMAND      = 2,
  PKT_ACK          = 3
};

enum CommandType : uint8_t {
  CMD_RELAY_OFF = 0,
  CMD_RELAY_ON  = 1,
  CMD_SET_PRIORITY = 2
};

enum PriorityLevel : uint8_t {
  PRIO_HIGH   = 0,
  PRIO_MEDIUM = 1,
  PRIO_LOW    = 2
};

struct PowerReport {
  uint8_t  type;
  uint8_t  slaveId;
  float    power;
  bool     relayOn;
  bool     switchOn;     // physical switch state (must be ON for load to run)
  uint8_t  priority;
  uint32_t uptimeSec;
} __attribute__((packed));

struct CommandPacket {
  uint8_t type;
  uint8_t slaveId;
  uint8_t cmd;
  uint8_t value;
} __attribute__((packed));

#define NUM_SLAVES 3
static const char* const PRIO_NAMES[3] = { "H", "M", "L" };

// ------------------------------------------------------------
// Pin definitions (Master)
// ------------------------------------------------------------
#define SDA_PIN     21
#define SCL_PIN     19
#define I2C_ADDR    0x27

#define BTN1_PIN    4     // Mode change
#define BTN2_PIN    5     // Selection / Cursor
#define BTN3_PIN    18    // Toggle / Increment

#define BUZZER_PIN  13

// LEDC for buzzer
#define PWM_CHANNEL 0
#define PWM_FREQ    2700
#define PWM_RES     8

// ------------------------------------------------------------
// LCD
// ------------------------------------------------------------
LiquidCrystal_I2C lcd(I2C_ADDR, 16, 2);

// ------------------------------------------------------------
// Slave MAC addresses (hardcoded)
// ------------------------------------------------------------
uint8_t slaveMac[NUM_SLAVES][6] = {
  {0xB4, 0xBF, 0xE9, 0xE1, 0x47, 0x1C}, // A
  {0xB0, 0x3F, 0xD3, 0x51, 0xCA, 0xF8}, // B
  {0x20, 0x50, 0x0D, 0x4E, 0xD5, 0x54}  // C
};

// ------------------------------------------------------------
// Runtime state for each slave
// ------------------------------------------------------------
struct SlaveState {
  float    power;          // last raw reported Watts
  float    smoothPower;    // exponential moving average (stable value)
  bool     relayOn;        // true = load connected (from slave report)
  bool     switchOn;       // physical switch on the slave
  uint8_t  priority;       // PRIO_HIGH / MEDIUM / LOW
  uint32_t lastSeenMs;     // millis() of last report
  bool     online;
  bool     fresh;          // true if last report is recent enough for decisions
  bool     manualOverride; // if true, auto-shed ignores this slave
};

SlaveState slaves[NUM_SLAVES];

// EMA factor: higher = faster reaction, lower = smoother
// 0.30 → good balance for 2 s reports
const float POWER_EMA_ALPHA = 0.30f;

// A reading is "fresh" if received within this window
const uint32_t FRESH_MS = 4500;   // ~2 report intervals + margin

// Minimum time between any shed or restore action (stops oscillation)
const uint32_t ACTION_COOLDOWN_MS = 8000;
uint32_t lastActionMs = 0;

// ------------------------------------------------------------
// Global system state
// ------------------------------------------------------------
uint16_t mdl = 196;                // Maximum Demand Limit (Watts)
bool     backlightOn = true;
uint8_t  shedCount = 0;            // how many loads currently shed

// UI State Machine
enum UiMode : uint8_t {
  MODE_DASHBOARD = 0,
  MODE_SET_MDL,
  MODE_PRIORITY,
  MODE_LOAD_INFO,
  NUM_MODES
};

UiMode currentMode = MODE_DASHBOARD;

// Set MDL editor state
uint8_t mdlDigits[4] = {0, 1, 9, 6}; // 0196
int8_t  mdlCursor = 1;               // which digit is selected (0-3)

// Priority editor
int8_t  prioCursor = 0;              // which slave is selected (0-2)

// Button handling
struct Button {
  uint8_t  pin;
  bool     lastState;
  bool     pressed;
  bool     longPressed;
  uint32_t pressStart;
  uint32_t lastDebounce;
};

Button btn1 = {BTN1_PIN, HIGH, false, false, 0, 0};
Button btn2 = {BTN2_PIN, HIGH, false, false, 0, 0};
Button btn3 = {BTN3_PIN, HIGH, false, false, 0, 0};

const uint32_t DEBOUNCE_MS   = 40;
const uint32_t LONG_PRESS_MS = 800;

// Timing
uint32_t lastLcdUpdate = 0;
uint32_t lastShedCheck = 0;
uint32_t lastBuzzerOff = 0;
bool     buzzerActive  = false;

const uint32_t LCD_UPDATE_MS    = 200;  // faster so cursor blink looks smooth
const uint32_t SHED_CHECK_MS    = 1500;
const uint32_t SLAVE_TIMEOUT_MS = 8000;  // reports every 2 s → allow a few misses

// ------------------------------------------------------------
// Forward declarations
// ------------------------------------------------------------
void updateButtons();
void handleButtonEvents();
void updateLcd();
void checkShedding();
float getStableTotal(bool onlyRelayOn);
bool  dataIsCoherent();
void sendCommand(uint8_t slaveId, uint8_t cmd, uint8_t value = 0);
void beep(uint16_t durationMs = 80, uint8_t duty = 128);
void onDataRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len);
void onDataSent(const wifi_tx_info_t *info, esp_now_send_status_t status);
void loadSettings();
void saveSettings();
void syncMdlDigits();   // keep mdlDigits[] in sync with mdl
void playStartupAnimation();
bool cursorBlinkOn();   // Casio-style blink (~2.5 Hz)

// ============================================================
// Setup
// ============================================================
void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println("\n=== Master Power Manager starting ===");

  // ----- GPIO -----
  pinMode(BTN1_PIN, INPUT_PULLUP);
  pinMode(BTN2_PIN, INPUT_PULLUP);
  pinMode(BTN3_PIN, INPUT_PULLUP);

  pinMode(BUZZER_PIN, OUTPUT);
  gpio_set_drive_capability((gpio_num_t)BUZZER_PIN, GPIO_DRIVE_CAP_3);
  // Arduino-ESP32 3.x API
  ledcAttach(BUZZER_PIN, PWM_FREQ, PWM_RES);
  ledcWrite(BUZZER_PIN, 0);

  // ----- LCD -----
  Wire.begin(SDA_PIN, SCL_PIN);
  lcd.init();
  lcd.backlight();
  lcd.clear();

  // ----- Initialise slave state (defaults, then overwrite from NVS) -----
  for (int i = 0; i < NUM_SLAVES; i++) {
    slaves[i].power = 0;
    slaves[i].smoothPower = 0;
    slaves[i].relayOn = false;         // unknown until first report
    slaves[i].switchOn = false;
    slaves[i].priority = (i == 0) ? PRIO_HIGH : (i == 1) ? PRIO_MEDIUM : PRIO_LOW;
    slaves[i].lastSeenMs = 0;
    slaves[i].online = false;
    slaves[i].fresh = false;
    slaves[i].manualOverride = false;
  }

  // Load MDL + priorities from flash (survives power cycles)
  loadSettings();
  syncMdlDigits();

  // ----- WiFi + ESP-NOW -----
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  delay(100);

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed");
    lcd.setCursor(0, 1);
    lcd.print("ESP-NOW FAIL");
    while (true) delay(1000);
  }

  esp_now_register_recv_cb(onDataRecv);
  esp_now_register_send_cb(onDataSent);

  // Add all slaves as peers
  for (int i = 0; i < NUM_SLAVES; i++) {
    esp_now_peer_info_t peer = {};
    memcpy(peer.peer_addr, slaveMac[i], 6);
    peer.channel = 0;
    peer.encrypt = false;
    if (esp_now_add_peer(&peer) != ESP_OK) {
      Serial.printf("Failed to add peer %d\n", i);
    } else {
      Serial.printf("Peer %c added\n", 'A' + i);
    }
  }

  // Show MAC for verification
  Serial.print("Master MAC: ");
  Serial.println(WiFi.macAddress());

  // Startup animation (Casio-style)
  playStartupAnimation();

  lcd.clear();
  updateLcd();
}

// ============================================================
// Main loop
// ============================================================
void loop() {
  updateButtons();
  handleButtonEvents();

  uint32_t now = millis();

  // Periodic LCD refresh
  if (now - lastLcdUpdate >= LCD_UPDATE_MS) {
    lastLcdUpdate = now;
    updateLcd();
  }

  // Periodic shedding logic
  if (now - lastShedCheck >= SHED_CHECK_MS) {
    lastShedCheck = now;
    checkShedding();
  }

  // Turn buzzer off after duration
  if (buzzerActive && now >= lastBuzzerOff) {
    ledcWrite(BUZZER_PIN, 0);
    buzzerActive = false;
  }

  // Update online / fresh flags for every slave
  for (int i = 0; i < NUM_SLAVES; i++) {
    if (slaves[i].online) {
      uint32_t age = now - slaves[i].lastSeenMs;
      slaves[i].fresh = (age <= FRESH_MS);
      if (age > SLAVE_TIMEOUT_MS) {
        slaves[i].online = false;
        slaves[i].fresh  = false;
        Serial.printf("Slave %c timed out\n", 'A' + i);
      }
    } else {
      slaves[i].fresh = false;
    }
  }

  delay(5); // small yield
}

// ============================================================
// Button handling (debounce + short/long press)
// ============================================================
void updateButton(Button &b) {
  bool reading = digitalRead(b.pin);
  uint32_t now = millis();

  if (reading != b.lastState) {
    b.lastDebounce = now;
  }

  if ((now - b.lastDebounce) > DEBOUNCE_MS) {
    if (reading == LOW && !b.pressed) {
      // just pressed
      b.pressed = true;
      b.longPressed = false;
      b.pressStart = now;
    }
    else if (reading == HIGH && b.pressed) {
      // just released
      b.pressed = false;
    }
  }
  b.lastState = reading;
}

void updateButtons() {
  updateButton(btn1);
  updateButton(btn2);
  updateButton(btn3);
}

// Detect short press on release and long press while held
bool shortPress(Button &b) {
  static bool wasPressed[3] = {false, false, false};
  int idx = (&b == &btn1) ? 0 : (&b == &btn2) ? 1 : 2;

  bool result = false;
  if (wasPressed[idx] && !b.pressed && !b.longPressed) {
    result = true; // released after short press
  }
  wasPressed[idx] = b.pressed;
  return result;
}

bool longPress(Button &b) {
  if (b.pressed && !b.longPressed && (millis() - b.pressStart >= LONG_PRESS_MS)) {
    b.longPressed = true;
    return true;
  }
  return false;
}

void handleButtonEvents() {
  // BTN1 – change mode
  if (shortPress(btn1)) {
    currentMode = (UiMode)((currentMode + 1) % NUM_MODES);
    beep(40);
  }

  // Mode-specific actions
  switch (currentMode) {
    case MODE_DASHBOARD:
      if (shortPress(btn3)) {
        backlightOn = !backlightOn;
        if (backlightOn) lcd.backlight();
        else lcd.noBacklight();
        beep(30);
      }
      break;

    case MODE_SET_MDL:
      if (shortPress(btn2)) {
        mdlCursor = (mdlCursor + 1) % 4;
        beep(30);
      }
      if (shortPress(btn3)) {
        mdlDigits[mdlCursor] = (mdlDigits[mdlCursor] + 1) % 10;
        // Rebuild mdl value
        mdl = mdlDigits[0] * 1000 + mdlDigits[1] * 100 + mdlDigits[2] * 10 + mdlDigits[3];
        saveSettings();          // persist immediately
        beep(30);
      }
      if (longPress(btn3)) {
        saveSettings();
        beep(120);
      }
      break;

    case MODE_PRIORITY:
      if (shortPress(btn2)) {
        prioCursor = (prioCursor + 1) % NUM_SLAVES;
        beep(30);
      }
      if (shortPress(btn3)) {
        // cycle priority H -> M -> L -> H
        slaves[prioCursor].priority = (slaves[prioCursor].priority + 1) % 3;
        saveSettings();          // persist immediately
        beep(30);
      }
      break;

    case MODE_LOAD_INFO:
      // currently read-only
      if (shortPress(btn3)) {
        beep(30);
      }
      break;
  }
}

// ============================================================
// LCD rendering
// ============================================================
void updateLcd() {
  char line1[17];
  char line2[17];

  switch (currentMode) {
    case MODE_DASHBOARD: {
      // Use stable (smoothed) total of currently-ON loads
      float total = getStableTotal(true);
      shedCount = 0;
      for (int i = 0; i < NUM_SLAVES; i++) {
        if (slaves[i].online && !slaves[i].relayOn) shedCount++;
      }
      // Show total; append '?' when some readings are still catching up
      if (dataIsCoherent()) {
        snprintf(line1, sizeof(line1), "PWR:%3.0fW/%3dW", total, mdl);
      } else {
        snprintf(line1, sizeof(line1), "PWR:%3.0f?/%3dW", total, mdl);
      }
      snprintf(line2, sizeof(line2), "SHED:%d/3", shedCount);
      break;
    }

    case MODE_SET_MDL: {
      // MDL: 0196 W   — selected digit blinks (Casio style, no brackets)
      bool blink = cursorBlinkOn();
      int pos = 0;
      pos += snprintf(line1 + pos, sizeof(line1) - pos, "MDL: ");
      for (int i = 0; i < 4; i++) {
        char ch = '0' + mdlDigits[i];
        if (i == mdlCursor && !blink) ch = ' ';   // blank while blink off
        line1[pos++] = ch;
      }
      snprintf(line1 + pos, sizeof(line1) - pos, " W");
      line2[0] = 0;   // second line empty
      break;
    }

    case MODE_PRIORITY: {
      snprintf(line1, sizeof(line1), "PRIORITIES");
      // A:H B:M C:L  — only the priority letter (H/M/L) of the selection blinks
      bool blink = cursorBlinkOn();
      char buf[20];
      int pos = 0;
      for (int i = 0; i < NUM_SLAVES; i++) {
        char letter = PRIO_NAMES[slaves[i].priority][0];
        if (i == prioCursor && !blink) letter = ' ';  // blank only H/M/L
        pos += sprintf(buf + pos, "%c:%c", 'A' + i, letter);
        if (i < NUM_SLAVES - 1) buf[pos++] = ' ';
      }
      buf[pos] = 0;
      strncpy(line2, buf, 16);
      line2[16] = 0;
      break;
    }

    case MODE_LOAD_INFO: {
      // Display smoothed power for stability; "---" when offline
      char aBuf[8], bBuf[8], cBuf[8];
      if (slaves[0].online)
        snprintf(aBuf, sizeof(aBuf), "%3.0fW", slaves[0].smoothPower);
      else
        snprintf(aBuf, sizeof(aBuf), "--- ");
      if (slaves[1].online)
        snprintf(bBuf, sizeof(bBuf), "%3.0fW", slaves[1].smoothPower);
      else
        snprintf(bBuf, sizeof(bBuf), "--- ");
      if (slaves[2].online)
        snprintf(cBuf, sizeof(cBuf), "%3.0fW", slaves[2].smoothPower);
      else
        snprintf(cBuf, sizeof(cBuf), "--- ");

      snprintf(line1, sizeof(line1), "A:%s B:%s", aBuf, bBuf);
      snprintf(line2, sizeof(line2), "C:%s %s",
               cBuf,
               slaves[2].online ? (slaves[2].relayOn ? "ON " : "OFF") : "   ");
      break;
    }
  }

  // Write to LCD
  lcd.setCursor(0, 0);
  lcd.print(line1);
  for (int i = strlen(line1); i < 16; i++) lcd.print(' ');

  lcd.setCursor(0, 1);
  lcd.print(line2);
  for (int i = strlen(line2); i < 16; i++) lcd.print(' ');
}

// ============================================================
// Helper: stable total demand from smoothed, fresh readings
// ============================================================
float getStableTotal(bool onlyRelayOn) {
  float total = 0;
  for (int i = 0; i < NUM_SLAVES; i++) {
    // Only count slaves that are online AND have a recent reading
    if (slaves[i].online && slaves[i].fresh) {
      if (!onlyRelayOn || slaves[i].relayOn) {
        total += slaves[i].smoothPower;
      }
    }
  }
  return total;
}

// Returns true only when every currently-online slave has fresh data.
// Prevents decisions based on a partial / staggered snapshot.
bool dataIsCoherent() {
  int onlineCount = 0;
  int freshCount  = 0;
  for (int i = 0; i < NUM_SLAVES; i++) {
    if (slaves[i].online) {
      onlineCount++;
      if (slaves[i].fresh) freshCount++;
    }
  }
  // Need at least one online slave, and all of them must be fresh
  return (onlineCount > 0) && (freshCount == onlineCount);
}

// ============================================================
// Automatic load shedding / restoration
// ============================================================
void checkShedding() {
  // Do not act on incomplete or stale data
  if (!dataIsCoherent()) return;

  // Cooldown prevents rapid oscillate / beep spam
  uint32_t now = millis();
  if (now - lastActionMs < ACTION_COOLDOWN_MS) return;

  float total = getStableTotal(true);   // only currently-ON loads

  // ----- Shed if over MDL -----
  if (total > mdl) {
    // Find lowest-priority online+fresh load that is currently ON
    int candidate = -1;
    uint8_t worstPrio = 0;

    for (int i = 0; i < NUM_SLAVES; i++) {
      if (slaves[i].online && slaves[i].fresh &&
          slaves[i].relayOn && !slaves[i].manualOverride) {
        if (slaves[i].priority >= worstPrio) {
          worstPrio = slaves[i].priority;
          candidate = i;
        }
      }
    }

    if (candidate >= 0) {
      Serial.printf("Shedding slave %c (prio %d)  total=%.0f\n",
                    'A' + candidate, worstPrio, total);
      sendCommand(candidate, CMD_RELAY_OFF);
      // Do NOT optimistically change relayOn — wait for slave report
      lastActionMs = now;
      beep(150, 180);
    }
  }
  // ----- Restore if under MDL (hysteresis) -----
  else if (total < mdl * 0.92f) {
    int candidate = -1;
    uint8_t bestPrio = 3;

    for (int i = 0; i < NUM_SLAVES; i++) {
      // Only restore if:
      //  - slave is online & fresh
      //  - relay is currently OFF
      //  - physical switch is ON (otherwise restore is pointless and causes flaps)
      if (slaves[i].online && slaves[i].fresh &&
          !slaves[i].relayOn &&
          slaves[i].switchOn &&
          !slaves[i].manualOverride) {
        if (slaves[i].priority < bestPrio) {
          bestPrio = slaves[i].priority;
          candidate = i;
        }
      }
    }

    if (candidate >= 0) {
      // Project using last known smooth power (0 if load was off and cold)
      float projected = total + slaves[candidate].smoothPower;
      if (projected <= mdl) {
        Serial.printf("Restoring slave %c (prio %d)  total=%.0f proj=%.0f\n",
                      'A' + candidate, bestPrio, total, projected);
        sendCommand(candidate, CMD_RELAY_ON);
        // Wait for slave report before updating relayOn
        lastActionMs = now;
        beep(60);
      }
    }
  }
}

// ============================================================
// ESP-NOW helpers
// ============================================================
void sendCommand(uint8_t slaveId, uint8_t cmd, uint8_t value) {
  if (slaveId >= NUM_SLAVES) return;

  CommandPacket pkt;
  pkt.type    = PKT_COMMAND;
  pkt.slaveId = slaveId;
  pkt.cmd     = cmd;
  pkt.value   = value;

  esp_err_t result = esp_now_send(slaveMac[slaveId], (uint8_t*)&pkt, sizeof(pkt));
  if (result != ESP_OK) {
    Serial.printf("Send to %c failed: %d\n", 'A' + slaveId, result);
  }
}

void onDataRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
  if (len < 1) return;

  uint8_t type = data[0];

  if (type == PKT_POWER_REPORT && len >= (int)sizeof(PowerReport)) {
    PowerReport *rpt = (PowerReport*)data;
    if (rpt->slaveId >= NUM_SLAVES) return;

    int id = rpt->slaveId;
    float newPower = rpt->power;
    if (newPower < 0) newPower = 0;   // guard

    // First report after coming online → seed the EMA
    if (!slaves[id].online) {
      slaves[id].smoothPower = newPower;
    } else {
      // Exponential moving average for stability
      slaves[id].smoothPower =
          POWER_EMA_ALPHA * newPower +
          (1.0f - POWER_EMA_ALPHA) * slaves[id].smoothPower;
    }

    slaves[id].power      = newPower;
    slaves[id].relayOn    = rpt->relayOn;
    slaves[id].switchOn   = rpt->switchOn;
    slaves[id].lastSeenMs = millis();
    slaves[id].online     = true;
    slaves[id].fresh      = true;
  }
}

void onDataSent(const wifi_tx_info_t *info, esp_now_send_status_t status) {
  // Optional delivery tracking
}

// ============================================================
// Buzzer
// ============================================================
void beep(uint16_t durationMs, uint8_t duty) {
  ledcWrite(BUZZER_PIN, duty);
  buzzerActive = true;
  lastBuzzerOff = millis() + durationMs;
}

// Casio-style cursor blink (~2.5 Hz)
bool cursorBlinkOn() {
  return ((millis() / 400) % 2) == 0;
}

// Startup splash animation
void playStartupAnimation() {
  lcd.clear();

  // Line 1: title appears left → right
  const char *title = " HELLO POWER! ";
  lcd.setCursor(0, 0);
  for (int i = 0; i < 16; i++) {
    lcd.print(title[i]);
    delay(35);
  }

  // Line 2: progress bar fills
  lcd.setCursor(0, 1);
  for (int i = 0; i < 16; i++) {
    lcd.print((char)0xFF);   // solid block character
    delay(40);
  }

  beep(60);
  delay(200);

  // Brief "Ready"
  lcd.clear();
  lcd.setCursor(5, 0);
  lcd.print("Ready");
  beep(40);
  delay(450);

  // Soft blink of backlight once
  lcd.noBacklight();
  delay(120);
  lcd.backlight();
  delay(150);
}

// ============================================================
// Persistent settings (NVS / Preferences)
// Survives power cycles and resets
// ============================================================
void loadSettings() {
  prefs.begin("pwrMgr", true);   // read-only

  mdl = prefs.getUShort("mdl", 196);   // default 196 W

  // Clamp to a sane range
  if (mdl < 10)   mdl = 10;
  if (mdl > 9999) mdl = 9999;

  for (int i = 0; i < NUM_SLAVES; i++) {
    char key[8];
    snprintf(key, sizeof(key), "prio%d", i);
    uint8_t p = prefs.getUChar(key, slaves[i].priority);
    if (p > PRIO_LOW) p = PRIO_LOW;
    slaves[i].priority = p;
  }

  prefs.end();

  Serial.printf("Loaded settings: MDL=%d  priorities=%d,%d,%d\n",
                mdl,
                slaves[0].priority,
                slaves[1].priority,
                slaves[2].priority);
}

void saveSettings() {
  prefs.begin("pwrMgr", false);  // read-write

  prefs.putUShort("mdl", mdl);

  for (int i = 0; i < NUM_SLAVES; i++) {
    char key[8];
    snprintf(key, sizeof(key), "prio%d", i);
    prefs.putUChar(key, slaves[i].priority);
  }

  prefs.end();
  Serial.printf("Saved settings: MDL=%d  priorities=%d,%d,%d\n",
                mdl,
                slaves[0].priority,
                slaves[1].priority,
                slaves[2].priority);
}

void syncMdlDigits() {
  uint16_t v = mdl;
  mdlDigits[3] = v % 10; v /= 10;
  mdlDigits[2] = v % 10; v /= 10;
  mdlDigits[1] = v % 10; v /= 10;
  mdlDigits[0] = v % 10;
}