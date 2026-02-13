#pragma once
#include <Arduino.h>

// ============================================================================
// Seashells Master configuration
// ============================================================================

// Default ESP-NOW/WiFi channel to use on first boot (can be changed at runtime
// and persisted to NVS using the "CHAN <1-13>" serial command).
#define NOW_DEFAULT_CHANNEL 6

// OTA URL for the Side binary (Master sends this to Sides).
#define OTA_URL_SIDE_BIN  "http://172.20.10.3:8000/Seashells/Seashells_Side/build/esp32.esp32.um_feathers3/Seashells_Side.ino.bin"
