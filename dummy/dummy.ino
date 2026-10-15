/*
 * ============================================================
 *  DUMMY Slave – Socket A (PZEM removed / damaged)
 *
 *  Same behaviour as a normal slave, except power is simulated:
 *    - Switch ON  + relay ON  → random 14.0 … 16.0 W
 *    - Switch OFF or relay OFF → 0 W
 *
 *  Hardware used:
 *    - Inverted 5V relay on GPIO4  (HIGH = OFF, LOW = ON)
 *    - Local switch on GPIO19 (INPUT_PULLUP)
 *        LOW  = switch ON
 *        HIGH = switch OFF
 *
 *  Flash this sketch ONLY to the Socket A board
 *  (MAC B4:BF:E9:E1:47:1C). Identity is still auto-detected.
 * ============================================================
 */

#include <WiFi.h>
#include <esp_now.h>

// ============================================================
// Shared packet definitions (must match master)
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
  bool     switchOn;
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
// Pins
// ------------------------------------------------------------
#define RELAY_PIN   4
#define SWITCH_PIN  19

// ------------------------------------------------------------
// MACs
// ------------------------------------------------------------
uint8_t masterMac[6] = {0xB4, 0xBF, 0xE9, 0x6A, 0x11, 0x5C};

const uint8_t knownSlaveMac[3][6] = {
  {0xB4, 0xBF, 0xE9, 0xE1, 0x47, 0x1C}, // A
  {0xB0, 0x3F, 0xD3, 0x51, 0xCA, 0xF8}, // B
  {0x20, 0x50, 0x0D, 0x4E, 0xD5, 0x54}  // C
};

// ------------------------------------------------------------
// State
// ------------------------------------------------------------
int8_t  mySlaveId     = -1;
bool    switchOn      = false;
bool    masterWantsOn = true;
bool    relayOn       = false;
float   lastPower     = 0.0f;

uint32_t lastReportMs = 0;
uint32_t lastSimMs    = 0;

const uint32_t REPORT_INTERVAL_MS = 2000;
const uint32_t SIM_INTERVAL_MS    = 1500;  // how often to pick a new random wattage

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

// Simulated power: 14.0 – 16.0 W when load is active, else 0
float simulatePower() {
  if (!switchOn || !relayOn) return 0.0f;

  // Random float in [14.0, 16.0]
  float r = 14.0f + (float)(random(0, 201)) / 100.0f;  // 14.00 … 16.00
  return r;
}

void applyRelayLogic() {
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

  // Seed random from an analog pin (or millis if floating)
  randomSeed(analogRead(0) ^ millis());

  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, HIGH);   // start OFF
  relayOn = false;
  masterWantsOn = true;

  pinMode(SWITCH_PIN, INPUT_PULLUP);

  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  delay(100);

  mySlaveId = identifySlave();

  Serial.println("\n=== DUMMY Slave (no PZEM) starting ===");
  Serial.print("MAC: ");
  Serial.println(WiFi.macAddress());

  if (mySlaveId < 0) {
    Serial.println("ERROR: MAC not in known slave list!");
  } else {
    Serial.printf("Identified as Slave %c (ID %d)  [DUMMY power 14-16 W]\n",
                  'A' + mySlaveId, mySlaveId);
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

  switchOn = (digitalRead(SWITCH_PIN) == LOW);
  applyRelayLogic();
  lastPower = simulatePower();
}

// ============================================================
// Main loop
// ============================================================
void loop() {
  uint32_t now = millis();

  // Physical switch
  bool sw = (digitalRead(SWITCH_PIN) == LOW);
  if (sw != switchOn) {
    switchOn = sw;
    Serial.printf("Switch → %s\n", switchOn ? "ON" : "OFF");
    applyRelayLogic();
    lastPower = simulatePower();   // update immediately
  }

  // Refresh simulated wattage periodically (small variation)
  if (now - lastSimMs >= SIM_INTERVAL_MS) {
    lastSimMs = now;
    lastPower = simulatePower();
  }

  // Report every 2 s
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
  } else {
    Serial.printf("Report: %.1f W  relay=%s switch=%s\n",
                  lastPower,
                  relayOn ? "ON" : "OFF",
                  switchOn ? "ON" : "OFF");
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
        lastPower = simulatePower();
        break;
      case CMD_RELAY_ON:
        masterWantsOn = true;
        Serial.println("CMD: Master wants ON");
        applyRelayLogic();
        lastPower = simulatePower();
        break;
      default:
        break;
    }
  }
}
