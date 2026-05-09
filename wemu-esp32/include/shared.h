#pragma once
#include <stdint.h>

static uint8_t MASTER_MAC[6] = {0xA0, 0xDD, 0x6C, 0x74, 0xB8, 0xC4};

// ─── Message sent relay → master via ESP-NOW ──────────────────────────────────
struct DistMessage {
    uint8_t pair_id;  // 1 or 2
    float   dist_cm;
};

// ─── UART2 wiring (controller CDK only — the one that outputs distance) ────────
//   CDK Pi header Pin 8  (TX) → ESP32 GPIO16 (RX2)
//   CDK Pi header Pin 10 (RX) → ESP32 GPIO17 (TX2)
//   CDK Pi header Pin 4  (5V) → ESP32 VIN  ─┐
//   CDK Pi header Pin 6  (GND)→ ESP32 GND  ─┘ (Y-split to both CDKs)
//
// Controlee CDK: power only (Pi header Pin 4+6), no data lines needed.
#define CTRL_RX  16
#define CTRL_TX  17

// ─── Pair ID strap pin ────────────────────────────────────────────────────────
//   GPIO34 floating → Pair 1 (channel 9)
//   GPIO34 to GND   → Pair 2 (channel 5)
#define PAIR_ID_PIN  34

#define DIST_MIN  10.0f
#define DIST_MAX  400.0f