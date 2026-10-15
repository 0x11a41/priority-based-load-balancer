/*
 * ============================================================
 *  Slave Node - ESP32 Power Management System
 *
 *  Identity is detected automatically from the board's MAC.
 *  No need to change any #define when flashing different units.
 *
 *  Hardware:
 *    - PZEM-004T v4.0 on Serial2 (RX=16, TX=17)
 *    - Inverted 5V relay on GPIO4  (HIGH = OFF, LOW = ON)
 *    - Local switch on GPIO19 (INPUT_PULLUP)
 *        LOW  = switch ON  → appliance may receive power
 *        HIGH = switch OFF → socket forced OFF
 *
 *  Behaviour:
 *    - Reads active power from PZEM every ~1.5 s
 *    - Reports to Master every 2 seconds
 *    - Physical switch is the primary enable:
 *        switch OFF → relay forced OFF (master cannot turn it on)
 *        switch ON  → master can command relay ON/OFF
 * ============================================================
 */

#include <WiFi.h>
#include <esp_now.h>
#include <HardwareSerial.h>

// ============================================================
// Shared packet definitions
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

struct PowerReport {
  uint8_t  type;
  uint8_t  slaveId;
  float    power;
  bool     relayOn;
  bool     switchOn;     // physical switch state
  uint8_t  priority;
  uint32_t uptimeSec;
} __attribute__((packed));

struct CommandPacket {
  uint8_t type;
  uint8_t slaveId;
  uint8_t cmd;
  uint8_t value;
} __attribute__((packed));

// ------------------------------------------------------------
// Pin definitions
// ------------------------------------------------------------
#define RELAY_PIN   4     // inverted: HIGH = load OFF, LOW = load ON
#define SWITCH_PIN  19    // INPUT_PULLUP, LOW = switch closed/ON

#define PZEM_RX     16
#define PZEM_TX     17

// ------------------------------------------------------------
// Known MACs (must match CONTEXT.md / master)
// ------------------------------------------------------------
uint8_t masterMac[6] = {0xB4, 0xBF, 0xE9, 0x6A, 0x11, 0x5C};

const uint8_t knownSlaveMac[3][6] = {
  {0xB4, 0xBF, 0xE9, 0xE1, 0x47, 0x1C}, // A
  {0xB0, 0x3F, 0xD3, 0x51, 0xCA, 0xF8}, // B
  {0x20, 0x50, 0x0D, 0x4E, 0xD5, 0x54}  // C
};

// ------------------------------------------------------------
// PZEM-004T minimal Modbus RTU reader (Active Power only)
// ------------------------------------------------------------
HardwareSerial pzemSerial(2);

float readPzemPower() {
  uint8_t req[] = {0x01, 0x04, 0x00, 0x03, 0x00, 0x01, 0x00, 0x00};

  uint16_t crc = 0xFFFF;
  for (int i = 0; i < 6; i++) {
    crc ^= req[i];
    for (int j = 0; j < 8; j++) {
      if (crc & 1) crc = (crc >> 1) ^ 0xA001;
      else         crc >>= 1;
    }
  }
  req[6] = crc & 0xFF;
  req[7] = crc >> 8;

  while (pzemSerial.available()) pzemSerial.read();
  pzemSerial.write(req, 8);
  pzemSerial.flush();

  uint32_t start = millis();
  uint8_t resp[7];
  int idx = 0;
  while (millis() - start < 150) {
    if (pzemSerial.available()) {
      resp[idx++] = pzemSerial.read();
      if (idx >= 7) break;
    }
  }

  if (idx < 7) return -1.0f;
  if (resp[0] != 0x01 || resp[1] != 0x04 || resp[2] != 0x02) return -1.0f;

  uint16_t raw = (resp[3] << 8) | resp[4];
  return raw * 0.1f;   // unit = 0.1 W
}

// ------------------------------------------------------------
// State
// ------------------------------------------------------------
int8_t  mySlaveId = -1;       // 0=A, 1=B, 2=C, -1=unknown
bool    switchOn  = false;    // physical switch state
bool    masterWantsOn = true; // last command from master
bool    relayOn   = false;    // actual relay state
float   lastPower = 0.0f;

uint32_t lastReportMs = 0;
uint32_t lastPzemMs   = 0;

const uint32_t REPORT_INTERVAL_MS = 2000;  // 2 seconds as requested
const uint32_t PZEM_INTERVAL_MS   = 1500;

// ------------------------------------------------------------
// Helpers
// ------------------------------------------------------------
bool macEqual(const uint8_t *a, const uint8_t *b) {
  for (int i = 0; i < 6; i++) if (a[i] != b[i]) return false;
  return true;
}

int8_t identifySlave() {
  uint8_t mac[6];
  WiFi.macAddress(mac);
  for (int i = 0; i < 3; i++) {
    if (macEqual(mac, knownSlaveMac[i])) return i;
  }
  return -1;
}

void applyRelayLogic() {
  // Physical switch is the primary gate.
  // Relay may only be ON when switch is ON AND master wants it ON.
  bool desired = switchOn && masterWantsOn;
  if (desired != relayOn) {
    relayOn = desired;
    digitalWrite(RELAY_PIN, relayOn ? LOW : HIGH);  // inverted
    Serial.printf("Relay → %s (switch=%s masterWants=%s)\n",
                  relayOn ? "ON" : "OFF",
                  switchOn ? "ON" : "OFF",
                  masterWantsOn ? "ON" : "OFF");
  }
}

void sendPowerReport();
void onDataRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len);

// ============================================================
// Setup
// ============================================================
void setup() {
  Serial.begin(115200);
  delay(300);

  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, HIGH);   // start OFF (safe)
  relayOn = false;
  masterWantsOn = true;            // default: allow ON when switch is closed

  pinMode(SWITCH_PIN, INPUT_PULLUP);

  pzemSerial.begin(9600, SERIAL_8N1, PZEM_RX, PZEM_TX);

  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  delay(100);

  mySlaveId = identifySlave();

  Serial.println("\n=== Slave starting ===");
  Serial.print("MAC: ");
  Serial.println(WiFi.macAddress());

  if (mySlaveId < 0) {
    Serial.println("ERROR: This MAC is not in the known slave list!");
    Serial.println("Update knownSlaveMac[] in the sketch.");
  } else {
    Serial.printf("Identified as Slave %c (ID %d)\n", 'A' + mySlaveId, mySlaveId);
  }

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed");
    while (true) delay(1000);
  }

  esp_now_register_recv_cb(onDataRecv);

  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, masterMac, 6);
  peer.channel = 0;
  peer.encrypt = false;
  if (esp_now_add_peer(&peer) != ESP_OK) {
    Serial.println("Failed to add master peer");
  } else {
    Serial.println("Master peer added");
  }

  // Initial switch read
  switchOn = (digitalRead(SWITCH_PIN) == LOW);
  applyRelayLogic();
}

// ============================================================
// Main loop
// ============================================================
void loop() {
  uint32_t now = millis();

  // --- Physical switch (read every loop) ---
  bool sw = (digitalRead(SWITCH_PIN) == LOW);  // LOW = ON
  if (sw != switchOn) {
    switchOn = sw;
    Serial.printf("Switch → %s\n", switchOn ? "ON" : "OFF");
    applyRelayLogic();   // immediately enforce
  }

  // --- PZEM read ---
  if (now - lastPzemMs >= PZEM_INTERVAL_MS) {
    lastPzemMs = now;
    float p = readPzemPower();
    if (p >= 0.0f) {
      lastPower = p;
    }
  }

  // --- Report to master every 2 s ---
  if (mySlaveId >= 0 && (now - lastReportMs >= REPORT_INTERVAL_MS)) {
    lastReportMs = now;
    sendPowerReport();
  }

  delay(20);
}

// ============================================================
// ESP-NOW
// ============================================================
void sendPowerReport() {
  PowerReport rpt;
  rpt.type      = PKT_POWER_REPORT;
  rpt.slaveId   = (uint8_t)mySlaveId;
  rpt.power     = lastPower;
  rpt.relayOn   = relayOn;
  rpt.switchOn  = switchOn;
  rpt.priority  = 0;
  rpt.uptimeSec = millis() / 1000;

  esp_err_t result = esp_now_send(masterMac, (uint8_t*)&rpt, sizeof(rpt));
  if (result != ESP_OK) {
    Serial.printf("Report send failed: %d\n", result);
  }
}

void onDataRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
  if (len < 1 || mySlaveId < 0) return;

  uint8_t type = data[0];

  if (type == PKT_COMMAND && len >= (int)sizeof(CommandPacket)) {
    CommandPacket *cmd = (CommandPacket*)data;
    if (cmd->slaveId != mySlaveId) return;

    switch (cmd->cmd) {
      case CMD_RELAY_OFF:
        masterWantsOn = false;
        Serial.println("CMD: Master wants OFF");
        applyRelayLogic();
        break;
      case CMD_RELAY_ON:
        masterWantsOn = true;
        Serial.println("CMD: Master wants ON");
        applyRelayLogic();   // still gated by physical switch
        break;
      default:
        break;
    }
  }
}