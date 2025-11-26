#pragma once
#include <Arduino.h>

struct MasterClipMeta {
  uint16_t id;
  const char* base;
  const char* sub;
  const char* sub2;
};

extern const MasterClipMeta MASTER_CLIPS[];
extern const size_t MASTER_CLIP_COUNT;

// Lookup by ID; returns nullptr if not found
const MasterClipMeta* MasterManifest_find(uint16_t id);
