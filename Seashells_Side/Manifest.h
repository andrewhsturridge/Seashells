#pragma once
#include <Arduino.h>

enum Pool : uint8_t { POOL_A = 0, POOL_B = 1 };

// Extended metadata for each clip from manifest.csv
struct ClipMeta {
  uint16_t id;         // numeric ID used in protocol
  Pool     pool;       // A / B (legacy; still counted for Hello)
  String   path;       // SD path, e.g. "/animals/farm/cow.wav"
  bool     precache;   // true = load into PSRAM at boot
  int8_t   volume_db;  // per-clip trim

  // Structured category fields
  String   base;       // e.g. "animals", "tones"
  String   sub;        // e.g. "farm", "jungle", "simple", "sweep"
  String   sub2;       // e.g. "cow", "dogs", "low_beep"
  String   tags;       // optional extra tags (may be empty)
};

// Load /manifest.csv from SD into catalog[]
bool Manifest_load();

// Catalog lookup by ID (returns nullptr if not found)
const ClipMeta* Manifest_find(uint16_t id);

// LEGACY pool-based random picker (still available if needed)
uint8_t Manifest_pickRandom(Pool pool, uint8_t need, uint16_t* out, uint8_t maxOut);

// NEW: category-based pickers (by base)
uint8_t Manifest_pickRandomByBase(const String& base, uint8_t need, uint16_t* out, uint8_t maxOut);
uint8_t Manifest_pickRandomByBaseNot(const String& forbiddenBase, uint8_t need, uint16_t* out, uint8_t maxOut);

// Precache all clips with precache=1 into PSRAM cache (best-effort)
void Manifest_precacheAll();

// Check if the given id is precached and (if so) return pointer + sample count
bool Manifest_getCached(uint16_t id, int16_t** data, size_t* samples);
