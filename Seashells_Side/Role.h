#pragma once
#include <Arduino.h>
#include <Preferences.h>

namespace Role {
  static Preferences prefs;
  static uint8_t sideId = 0xFF;   // 0xFF = UNASSIGNED

  inline void begin() {
    prefs.begin("seashells", false);
    sideId = prefs.getUChar("sideId", 0xFF);
  }
  inline uint8_t get() { return sideId; }

  inline void set(uint8_t id, bool persist=true) {
    sideId = id;
    if (persist) prefs.putUChar("sideId", sideId);
  }
}
