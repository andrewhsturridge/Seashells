#pragma once
#include <Arduino.h>

enum Pool : uint8_t { POOL_A=0, POOL_B=1 };

struct ClipMeta {
  uint16_t id;
  Pool     pool;
  String   path;
  bool     precache;
  int8_t   volume_db;
};

// Load /manifest.csv from SD into catalog[]
bool Manifest_load();

// Catalog lookup by ID
const ClipMeta* Manifest_find(uint16_t id);

// Random unique picks from a pool
// Returns number written to out[], up to maxOut
uint8_t Manifest_pickRandom(Pool pool, uint8_t need, uint16_t* out, uint8_t maxOut);

// Precache all clips with precache=1 into PSRAM cache (best-effort)
void Manifest_precacheAll();

// Check if the given id is precached and (if so) return pointer + sample count
bool Manifest_getCached(uint16_t id, int16_t** data, size_t* samples);
