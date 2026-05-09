#pragma once
#include <stdint.h>

// Keep MASTER_MAC in sync with wemu_esp32/include/shared.h
// After flashing master and reading its MAC, update both files
static uint8_t MASTER_MAC[6] = {0xA0, 0xDD, 0x6C, 0x74, 0xB8, 0xC4};

struct DistMessage {
    uint8_t pair_id;
    float   dist_cm;
};

#define DIST_MIN  10.0f
#define DIST_MAX  400.0f