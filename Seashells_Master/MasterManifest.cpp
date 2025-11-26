#include "MasterManifest.h"

// This table should mirror the IDs and categories in your SD manifest.csv on the Sides.
// For now: animals + tones only.

const MasterClipMeta MASTER_CLIPS[] = {
  // BASE: animals
  {1001, "animals", "farm",   "chicken"},
  {1002, "animals", "farm",   "cow"},
  {1003, "animals", "farm",   "duck"},
  {1004, "animals", "farm",   "horse"},
  {1005, "animals", "farm",   "pigs"},
  {1006, "animals", "farm",   "rooster"},
  {1007, "animals", "farm",   "sheep"},

  {1101, "animals", "jungle", "birds"},
  {1102, "animals", "jungle", "monkeys"},
  {1103, "animals", "jungle", "elephant"},
  {1104, "animals", "jungle", "frogs"},
  {1105, "animals", "jungle", "gibbon"},
  {1106, "animals", "jungle", "tigers"},

  {1201, "animals", "pets",   "dogs"},
  {1202, "animals", "pets",   "cats"},

  // BASE: tones (synthetic on Sides)
  {5001, "tones", "simple",  "low_beep"},
  {5002, "tones", "simple",  "mid_beep"},
  {5003, "tones", "simple",  "high_beep"},

  {5101, "tones", "sweep",   "up_short"},
  {5102, "tones", "sweep",   "down_short"},
  {5103, "tones", "sweep",   "siren_slow"},

  {5201, "tones", "noise",   "burst_short"},
  {5202, "tones", "noise",   "burst_long"},

  {5301, "tones", "rhythm",  "double_click"},
  {5302, "tones", "rhythm",  "triple_beep"},
};

const size_t MASTER_CLIP_COUNT = sizeof(MASTER_CLIPS) / sizeof(MASTER_CLIPS[0]);

const MasterClipMeta* MasterManifest_find(uint16_t id) {
  for (size_t i = 0; i < MASTER_CLIP_COUNT; i++) {
    if (MASTER_CLIPS[i].id == id) return &MASTER_CLIPS[i];
  }
  return nullptr;
}
