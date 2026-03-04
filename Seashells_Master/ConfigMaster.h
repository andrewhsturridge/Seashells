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

// ============================================================================
// MixFix (default)
// ============================================================================
// If your RIGHT speakers use mono I2S amps that are effectively outputting an
// L+R mix (common on MAX98357A boards when SD/MODE is in "mix" mode), MixFix can
// cancel LEFT leakage by pre-distorting the RIGHT samples.
//
// These defaults make the system "just work" after any reboot (even if no one
// types the serial command "MIXFIX ON"). You can still override at runtime:
//   MIXFIX OFF
//   MIXFIX ON
//   MIXFIX [A|B|BOTH] <mask 0-3> <k_milli> <m_milli>
//
// mask bits:
//   bit0 = I2S0 RIGHT (slot1 / Speaker2)
//   bit1 = I2S1 RIGHT (slot3 / Speaker4)
#define AUTO_ENABLE_MIXFIX           1
#define AUTO_ENABLE_MIXFIX_MASK      3
// For MIX ~= (L+R)/2 (Adafruit MAX98357A default mix behavior):
//   feed R' = 2R - L  -> k=2.0, m=1.0
#define AUTO_ENABLE_MIXFIX_K_MILLI   2000
#define AUTO_ENABLE_MIXFIX_M_MILLI   1000
