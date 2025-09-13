#include "Manifest.h"
#include <SD.h>

static const size_t MAX_CLIPS = 512;
static ClipMeta catalog[MAX_CLIPS];
static size_t catalogCount = 0;

struct CacheEntry {
  uint16_t id;
  int16_t* data;
  size_t   samples;
};
static CacheEntry cache[64];
static size_t cacheCount = 0;

extern bool loadWavIntoRam(const char* path, const char* tag, int16_t** outBuf, size_t* outSamples);

bool Manifest_load() {
  File f = SD.open("/manifest.csv", FILE_READ);
  if (!f) { Serial.println("[MANIFEST] missing /manifest.csv"); return false; }
  catalogCount = 0;
  String line;
  while (f.available()) {
    line = f.readStringUntil('\n');
    line.trim();
    if (!line.length() || line[0]=='#') continue;

    // id,pool,path,precache,volume_db,tags
    int c1=line.indexOf(','); if (c1<0) continue;
    int c2=line.indexOf(',',c1+1); if (c2<0) continue;
    int c3=line.indexOf(',',c2+1); if (c3<0) continue;
    int c4=line.indexOf(',',c3+1); if (c4<0) continue;

    ClipMeta m{};
    m.id = (uint16_t)line.substring(0,c1).toInt();
    char poolCh = line.substring(c1+1,c2).charAt(0);
    m.pool = (poolCh=='B'||poolCh=='b') ? POOL_B : POOL_A;
    m.path = line.substring(c2+1,c3);
    m.precache = line.substring(c3+1,c4).toInt() != 0;
    m.volume_db = (int8_t)line.substring(c4+1).toInt();

    if (catalogCount < MAX_CLIPS) catalog[catalogCount++] = m;
  }
  f.close();
  Serial.printf("[MANIFEST] loaded %u clips\n", (unsigned)catalogCount);
  return catalogCount>0;
}

const ClipMeta* Manifest_find(uint16_t id) {
  for (size_t i=0;i<catalogCount;i++) if (catalog[i].id==id) return &catalog[i];
  return nullptr;
}

uint8_t Manifest_pickRandom(Pool pool, uint8_t need, uint16_t* out, uint8_t maxOut) {
  if (!need || maxOut==0) return 0;
  uint8_t n=0; int guard=0;
  while (n<need && guard<2000) {
    guard++;
    if (catalogCount==0) break;
    size_t idx = random(catalogCount);
    if (catalog[idx].pool != pool) continue;
    uint16_t id = catalog[idx].id;
    bool dup=false; for (uint8_t j=0;j<n;j++) if (out[j]==id) { dup=true; break; }
    if (dup) continue;
    out[n++] = id;
  }
  return n;
}

void Manifest_precacheAll() {
  // Best-effort: load clips with precache=1
  for (size_t i=0;i<catalogCount && cacheCount<64; i++) {
    const ClipMeta& m = catalog[i];
    if (!m.precache) continue;
    int16_t* buf=nullptr; size_t samples=0;
    if (loadWavIntoRam(m.path.c_str(), String("ID"+String(m.id)).c_str(), &buf, &samples)) {
      cache[cacheCount++] = { m.id, buf, samples };
    }
    yield();
  }
  Serial.printf("[MANIFEST] precached %u clips\n", (unsigned)cacheCount);
}

bool Manifest_getCached(uint16_t id, int16_t** data, size_t* samples) {
  for (size_t i=0;i<cacheCount;i++) if (cache[i].id==id) {
    *data = cache[i].data; *samples = cache[i].samples; return true;
  }
  return false;
}
