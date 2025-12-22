#include "Manifest.h"
#include <SD.h>

// External audio helper used for precache
extern bool loadWavIntoRam(const char* path, const char* tag, int16_t** outBuf, size_t* outSamples);

// Catalog of all clips
static const size_t MAX_CLIPS = 512;
static ClipMeta catalog[MAX_CLIPS];
static size_t   catalogCount = 0;

// Simple precache cache (best-effort)
struct CacheEntry {
  uint16_t id;
  int16_t* data;
  size_t   samples;
};

static CacheEntry cache[64];
static size_t     cacheCount = 0;

// ───────────────── Manifest loading ─────────────────

bool Manifest_load() {
  File f = SD.open("/manifest.csv", FILE_READ);
  if (!f) {
    Serial.println("[MANIFEST] missing /manifest.csv");
    catalogCount = 0;
    return false;
  }

  catalogCount = 0;
  String line;

  while (f.available()) {
    line = f.readStringUntil('\n');
    line.trim();
    if (!line.length()) continue;

    // Skip comments
    if (line[0] == '#') continue;

    // Skip header line if present
    if (line.startsWith("id,")) continue;

    // Skip separator/blank rows like ',,,,,,,,' (ID must start with a digit)
    if (!isDigit(line[0])) continue;

    // Expected format:
    // id,pool,path,precache,volume_db,base,sub,sub2,tags
    int c1 = line.indexOf(',');              if (c1 < 0) continue;
    int c2 = line.indexOf(',', c1 + 1);      if (c2 < 0) continue;
    int c3 = line.indexOf(',', c2 + 1);      if (c3 < 0) continue;
    int c4 = line.indexOf(',', c3 + 1);      if (c4 < 0) continue;
    int c5 = line.indexOf(',', c4 + 1);      if (c5 < 0) continue;
    int c6 = line.indexOf(',', c5 + 1);      if (c6 < 0) continue;
    int c7 = line.indexOf(',', c6 + 1);      if (c7 < 0) continue;
    int c8 = line.indexOf(',', c7 + 1);      if (c8 < 0) continue;

    ClipMeta m{};

    // Field 0: id
    m.id = (uint16_t)line.substring(0, c1).toInt();

    // Field 1: pool (A/B)
    char poolCh = line.substring(c1 + 1, c2).charAt(0);
    m.pool = (poolCh == 'B' || poolCh == 'b') ? POOL_B : POOL_A;

    // Field 2: path
    m.path = line.substring(c2 + 1, c3);
    m.path.trim();

    // Field 3: precache (0/1)
    m.precache = (line.substring(c3 + 1, c4).toInt() != 0);

    // Field 4: volume_db
    m.volume_db = (int8_t)line.substring(c4 + 1, c5).toInt();

    // Field 5: base
    m.base = line.substring(c5 + 1, c6);
    m.base.trim();

    // Field 6: sub
    m.sub = line.substring(c6 + 1, c7);
    m.sub.trim();

    // Field 7: sub2
    m.sub2 = line.substring(c7 + 1, c8);
    m.sub2.trim();

    // Field 8: tags
    m.tags = line.substring(c8 + 1);
    m.tags.trim();

    // Reserve ID=0 as "no clip" (used to clear slots)
    if (m.id == 0) continue;

    if (catalogCount < MAX_CLIPS) {
      catalog[catalogCount++] = m;
    }
  }

  f.close();
  Serial.printf("[MANIFEST] loaded %u clips\n", (unsigned)catalogCount);
  return (catalogCount > 0);
}

// ───────────────── Lookup helpers ─────────────────

const ClipMeta* Manifest_find(uint16_t id) {
  for (size_t i = 0; i < catalogCount; i++) {
    if (catalog[i].id == id) return &catalog[i];
  }
  return nullptr;
}

// Legacy pool-based picker (still here if we ever need it)
uint8_t Manifest_pickRandom(Pool pool, uint8_t need, uint16_t* out, uint8_t maxOut) {
  if (!need || maxOut == 0 || catalogCount == 0) return 0;

  uint8_t n = 0;
  int guard = 0;

  while (n < need && guard < 2000) {
    guard++;
    size_t idx = random(catalogCount);
    const ClipMeta& m = catalog[idx];

    if (m.pool != pool) continue;

    uint16_t id = m.id;
    bool dup = false;
    for (uint8_t j = 0; j < n; j++) {
      if (out[j] == id) { dup = true; break; }
    }
    if (dup) continue;

    out[n++] = id;
    if (n >= maxOut) break;
  }
  return n;
}

// NEW: pick by base == given base
uint8_t Manifest_pickRandomByBase(const String& base, uint8_t need, uint16_t* out, uint8_t maxOut) {
  if (!need || maxOut == 0 || catalogCount == 0) return 0;

  uint8_t n = 0;
  int guard = 0;

  while (n < need && guard < 3000) {
    guard++;
    size_t idx = random(catalogCount);
    const ClipMeta& m = catalog[idx];

    if (!m.base.equalsIgnoreCase(base)) continue;

    uint16_t id = m.id;
    bool dup = false;
    for (uint8_t j = 0; j < n; j++) {
      if (out[j] == id) { dup = true; break; }
    }
    if (dup) continue;

    out[n++] = id;
    if (n >= maxOut) break;
  }
  return n;
}

// NEW: pick by base != forbiddenBase
uint8_t Manifest_pickRandomByBaseNot(const String& forbiddenBase, uint8_t need, uint16_t* out, uint8_t maxOut) {
  if (!need || maxOut == 0 || catalogCount == 0) return 0;

  uint8_t n = 0;
  int guard = 0;

  while (n < need && guard < 3000) {
    guard++;
    size_t idx = random(catalogCount);
    const ClipMeta& m = catalog[idx];

    if (m.base.equalsIgnoreCase(forbiddenBase)) continue;

    uint16_t id = m.id;
    bool dup = false;
    for (uint8_t j = 0; j < n; j++) {
      if (out[j] == id) { dup = true; break; }
    }
    if (dup) continue;

    out[n++] = id;
    if (n >= maxOut) break;
  }
  return n;
}

// ───────────────── Precache support ─────────────────

void Manifest_precacheAll() {
  cacheCount = 0;
  for (size_t i = 0; i < catalogCount && cacheCount < (sizeof(cache)/sizeof(cache[0])); i++) {
    const ClipMeta& m = catalog[i];
    if (!m.precache) continue;
    if (!m.path.length()) continue;

    int16_t* buf = nullptr;
    size_t   samples = 0;

    String tag = "ID" + String(m.id);

    if (loadWavIntoRam(m.path.c_str(), tag.c_str(), &buf, &samples)) {
      cache[cacheCount].id      = m.id;
      cache[cacheCount].data    = buf;
      cache[cacheCount].samples = samples;
      cacheCount++;
    }
    yield();
  }
  Serial.printf("[MANIFEST] precached %u clips\n", (unsigned)cacheCount);
}

bool Manifest_getCached(uint16_t id, int16_t** data, size_t* samples) {
  for (size_t i = 0; i < cacheCount; i++) {
    if (cache[i].id == id) {
      if (data)    *data    = cache[i].data;
      if (samples) *samples = cache[i].samples;
      return true;
    }
  }
  return false;
}
