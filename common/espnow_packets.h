#pragma once
#include <stdint.h>

// ============================================================
// Shared ESP-NOW packet definitions for Master <-> Slave
// ============================================================

enum PacketType : uint8_t {
  PKT_POWER_REPORT = 1,   // Slave -> Master
  PKT_COMMAND      = 2,   // Master -> Slave
  PKT_ACK          = 3    // optional
};

enum CommandType : uint8_t {
  CMD_RELAY_OFF = 0,
  CMD_RELAY_ON  = 1,
  CMD_SET_PRIORITY = 2    // optional future use
};

enum PriorityLevel : uint8_t {
  PRIO_HIGH   = 0,   // shed last
  PRIO_MEDIUM = 1,
  PRIO_LOW    = 2    // shed first
};

// Slave -> Master (sent every ~500-1000 ms)
struct PowerReport {
  uint8_t type;          // PKT_POWER_REPORT
  uint8_t slaveId;       // 0 = A, 1 = B, 2 = C
  float   power;         // Active power in Watts
  bool    relayOn;       // true = load connected (relay LOW)
  uint8_t priority;      // current priority (for reference)
  uint32_t uptimeSec;    // optional health
} __attribute__((packed));

// Master -> Slave
struct CommandPacket {
  uint8_t type;          // PKT_COMMAND
  uint8_t slaveId;       // target slave
  uint8_t cmd;           // CommandType
  uint8_t value;         // e.g. new priority
} __attribute__((packed));

// Slave IDs
#define SLAVE_A 0
#define SLAVE_B 1
#define SLAVE_C 2
#define NUM_SLAVES 3

// Human-readable names
static const char* const SLAVE_NAMES[NUM_SLAVES] = { "A", "B", "C" };
static const char* const PRIO_NAMES[3] = { "H", "M", "L" };
 
