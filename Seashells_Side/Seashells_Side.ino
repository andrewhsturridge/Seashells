/*
  Seashells Side – 4 buttons + 4 RGBs + loop/one-shot WAVs (ESP32-S3 Feather)
  - Keeps the known-good audio/LED pipeline
  - Adds: Manifest CSV loader, ESP-NOW GameBus, GameMode gating, Loop-all for "announce"
  - Now with synthetic tones for base=tones (no WAV needed)
*/

#include <Arduino.h>
#include <SPI.h>
#include <SD.h>
#include <Adafruit_NeoPixel.h>
#include <cstring>
#include <math.h>
#include <esp_wifi.h>
#include <HTTPClient.h>
#include <Update.h>
#include "driver/i2s.h"

#include "ConfigSide.h"
#include "Messages.h"
#include "Manifest.h"
#include "GameBusSide.h"
#include "Role.h"
#include "AudioEngine.h"
#include "OtaUpdate.h"

// Master trim for this side (in dB). Use 0 for unity, negatives to reduce.
#define MASTER_GAIN_DB -10

// ======= RGB setup =======
#define NUM_LEDS_PER  1
Adafruit_NeoPixel rgb1(NUM_LEDS_PER, RGB1_PIN, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel rgb2(NUM_LEDS_PER, RGB2_PIN, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel rgb3(NUM_LEDS_PER, RGB3_PIN, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel rgb4(NUM_LEDS_PER, RGB4_PIN, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel* const RGBT[4] = { &rgb1, &rgb2, &rgb3, &rgb4 };

// Buttons
const uint8_t BTN_PINS[4] = { BTN1_PIN, BTN2_PIN, BTN3_PIN, BTN4_PIN };
bool     lastRaw[4]      = { false,false,false,false };
bool     pressed[4]      = { false,false,false,false };
uint32_t lastChangeMs[4] = { 0,0,0,0 };

// ======= Audio framing =======
constexpr size_t FRAME_SAMPLES = 1024;           // per-channel per-frame
constexpr size_t BYTES_PER_CH  = FRAME_SAMPLES * 2;  // 16-bit mono
constexpr size_t OUT_BYTES     = FRAME_SAMPLES * 4;  // interleaved stereo

Channel ch[4];  // 0->L I2S0, 1->R I2S0, 2->L I2S1, 3->R I2S1

// Buffers
int16_t tmpMono[4][FRAME_SAMPLES]; // per-channel mono working buffers
uint8_t outLR0[OUT_BYTES];         // I2S0 interleaved L/R frame
uint8_t outLR1[OUT_BYTES];         // I2S1 interleaved L/R frame

// Game mode gating
static bool gameMode = false;      // when true, we don't auto-play on press; we only send BTN_EVENT
static uint16_t curSlotIds[4] = {0,0,0,0}; // current clip ID per slot

// ======= Audio diagnostics =======
// When enabled, the Side ignores normal game audio commands and outputs
// synthetic tones directly on the 4 channels. This makes it easy to detect:
//   - left/right channel swap
//   - L+R summing on an amp
//   - crosstalk between channels
static bool    diagAudioActive  = false;
static uint8_t diagAudioPattern = 0;

// ======= MixFix (runtime) =======
// Some mono I2S amp boards can be strapped for LEFT, RIGHT, or MIX (L+R).
// If a RIGHT speaker amp is effectively outputting an L+R mix, you can
// mathematically cancel LEFT leakage by pre-distorting the RIGHT samples.
//
// This is controlled at runtime via the Master command:
//   MIXFIX <side> <mask> <k_milli> <m_milli>
// and also used by DIAG patterns 14/15 (auto toggle).
//
// mask bits: bit0 -> I2S0 RIGHT (slot1), bit1 -> I2S1 RIGHT (slot3)
static volatile uint8_t mixFixMask =
  (uint8_t)((FIX_RIGHT_MIX_I2S0 ? 0x01 : 0x00) | (FIX_RIGHT_MIX_I2S1 ? 0x02 : 0x00));

// Q12 fixed-point multipliers (4096=1.0, 8192=2.0)
static volatile int16_t mixFixKQ12 = (int16_t)(RIGHT_MIX_MODEL_AVG ? 8192 : 4096);
static volatile int16_t mixFixMQ12 = (int16_t)4096;

// DIAG helper state for patterns 14/15 (auto toggles MixFix on a bus)
static bool     diagMixFixToggleActive = false;
static uint8_t  diagMixFixToggleBit    = 0;
static uint32_t diagMixFixNextMs       = 0;
static bool     diagMixFixOn           = false;

static bool     diagMixFixSaved        = false;
static uint8_t  diagMixFixSavedMask    = 0;
static int16_t  diagMixFixSavedKQ12    = 0;
static int16_t  diagMixFixSavedMQ12    = 0;

// Forward decls (defined near the main loop)
static inline void diagMixFixStopRestore();
static inline void diagMixFixStartToggle(uint8_t bit);

// Optional per-slot trim (set by the Master). This is applied on top of:
//   MASTER_GAIN_DB * manifest volume_db
//
// Used for diagnostics / mitigation if a speaker/amp is effectively mixing L+R.
static int8_t  slotTrimDb[4]  = {0,0,0,0};     // signed dB per slot
static int32_t slotTrimQ15[4] = {32768,32768,32768,32768};
static int32_t baseGainQ15[4] = {32768,32768,32768,32768}; // master*clip (pre-trim)

// ---- Blink controller (non-blocking) ----
struct BlinkCtrl {
  bool     active   = false;
  bool     phaseOn  = false;
  uint8_t  mode     = 0;     // 0=single color (all LEDs), 1=per-slot colors
  uint8_t  color    = 0;     // mode=0: 0=red, 1=green, 2=white
  uint8_t  slotColors[4] = {0,0,0,0}; // mode=1: per-slot codes (0=red,1=green,2=white,3=off)
  uint8_t  remaining= 0;     // on->off transitions left
  uint16_t on_ms    = 0;
  uint16_t off_ms   = 0;
  uint32_t nextMs   = 0;
} blink;

// Number of blink pulses shown between rounds/results (kept in sync with the Master).
static constexpr uint8_t BLINK_REPS = 5;


static inline void ledsAllColor(uint8_t r,uint8_t g,uint8_t b){
  for (int i=0;i<4;i++) RGBT[i]->setPixelColor(0, RGBT[i]->Color(r,g,b));
  for (int i=0;i<4;i++) RGBT[i]->show();
}

static inline void ledsAllOff(){
  for (int i=0;i<4;i++) RGBT[i]->setPixelColor(0, 0);
  for (int i=0;i<4;i++) RGBT[i]->show();
}

static inline void colorCodeToRGB(uint8_t code, uint8_t& r, uint8_t& g, uint8_t& b) {
  // Keep these codes aligned with Messages.h comments:
  // 0=red, 1=green, 2=white, 3=off
  switch (code) {
    case 0: r = 255; g = 0;   b = 0;   break;
    case 1: r = 0;   g = 255; b = 0;   break;
    case 2: r = 255; g = 255; b = 255; break;
    default: r = 0;  g = 0;   b = 0;   break;
  }
}

static inline void ledsSlotsByCode(const uint8_t codes[4]) {
  for (int i = 0; i < 4; i++) {
    uint8_t r, g, b;
    colorCodeToRGB(codes[i], r, g, b);
    RGBT[i]->setPixelColor(0, RGBT[i]->Color(r, g, b));
  }
  for (int i=0;i<4;i++) RGBT[i]->show();
}

static void blinkStart(uint8_t color, uint16_t on_ms, uint16_t off_ms, uint8_t reps=BLINK_REPS){
  blink.active = true;
  blink.phaseOn = true;
  blink.mode = 0;
  blink.color = color;
  blink.on_ms = on_ms;
  blink.off_ms= off_ms;
  blink.remaining = reps;

  uint8_t r=(color==0||color==2)?255:0;
  uint8_t g=(color==1||color==2)?255:0;
  uint8_t b=(color==2)?255:0;
  ledsAllColor(r,g,b);
  blink.nextMs = millis() + blink.on_ms;
}

static void blinkStartSlots(const uint8_t slotColors[4], uint16_t on_ms, uint16_t off_ms, uint8_t reps=BLINK_REPS){
  blink.active = true;
  blink.phaseOn = true;
  blink.mode = 1;
  blink.color = 0;
  memcpy(blink.slotColors, slotColors, 4);
  blink.on_ms = on_ms;
  blink.off_ms= off_ms;
  blink.remaining = reps;

  ledsSlotsByCode(blink.slotColors);
  blink.nextMs = millis() + blink.on_ms;
}

static void blinkStop(){
  blink.active = false;
  blink.phaseOn = false;
  blink.remaining = 0;
  ledsAllOff();
}

static void blinkUpdate(){
  if (!blink.active) return;
  if (millis() < blink.nextMs) return;

  if (blink.phaseOn) {
    ledsAllOff();
    blink.phaseOn = false;
    blink.nextMs = millis() + blink.off_ms;
    if (blink.remaining) {
      if (--blink.remaining == 0) {
        blink.active = false;
      }
    }
  } else {
    if (blink.mode == 0) {
      uint8_t r=(blink.color==0||blink.color==2)?255:0;
      uint8_t g=(blink.color==1||blink.color==2)?255:0;
      uint8_t b=(blink.color==2)?255:0;
      ledsAllColor(r,g,b);
    } else {
      ledsSlotsByCode(blink.slotColors);
    }
    blink.phaseOn = true;
    blink.nextMs = millis() + blink.on_ms;
  }
}

// Show OTA progress across 4 pixels in CYAN (0..100%)
void otaShowProgress(uint8_t pct) {
  for (int i=0;i<4;i++) { RGBT[i]->setPixelColor(0, 0); }
  uint8_t lit = (pct >= 100) ? 4 : (pct / 25);
  for (uint8_t i=0; i<lit; i++) {
    RGBT[i]->setPixelColor(0, RGBT[i]->Color(0, 255, 255)); // CYAN
  }
  for (int i=0;i<4;i++) RGBT[i]->show();
}

// ======= Helpers: LEDs =======
static inline void ledOff(uint8_t i){ RGBT[i]->setPixelColor(0, 0); RGBT[i]->show(); }
static inline void ledWhite(uint8_t i){ RGBT[i]->setPixelColor(0, RGBT[i]->Color(255,255,255)); RGBT[i]->show(); }
static inline void ledColorAll(uint8_t r,uint8_t g,uint8_t b){ for(int i=0;i<4;i++){ RGBT[i]->setPixelColor(0, RGBT[i]->Color(r,g,b)); } for(int i=0;i<4;i++) RGBT[i]->show(); }

// Configure a tone channel based on ClipMeta base/sub/sub2
static void configureToneChannel(Channel& C, const ClipMeta* cm, int slotIdx) {
  C.isTone = true;
  C.toneMode = TONE_SIMPLE;
  C.toneFreq1 = 880.0f;
  C.toneFreq2 = 1200.0f;
  C.tonePhase = 0.0f;
  C.toneSweepPos = 0.0f;
  C.toneSweepRate = 0.0f;
  C.tonePatternSamples = 0;
  C.path = "";
  C.useRAM = false;
  if (C.sd.f) C.sd.f.close();

  // Default mapping for base=tones
  String sub  = cm->sub;
  String sub2 = cm->sub2;

  sub.toLowerCase();
  sub2.toLowerCase();

  if (sub == "simple") {
    C.toneMode = TONE_SIMPLE;
    if (sub2 == "low_beep") {
      C.toneFreq1 = 600.0f;
    } else if (sub2 == "mid_beep") {
      C.toneFreq1 = 1000.0f;
    } else if (sub2 == "high_beep") {
      C.toneFreq1 = 1600.0f;
    } else {
      C.toneFreq1 = 1000.0f;
    }
  } else if (sub == "sweep") {
    if (sub2 == "up_short") {
      C.toneMode = TONE_SWEEP_UP;
      C.toneFreq1 = 400.0f;
      C.toneFreq2 = 1400.0f;
    } else if (sub2 == "down_short") {
      C.toneMode = TONE_SWEEP_DOWN;
      C.toneFreq1 = 1400.0f;
      C.toneFreq2 = 400.0f;
    } else if (sub2 == "siren_slow") {
      C.toneMode = TONE_SIREN;
      C.toneFreq1 = 500.0f;
      C.toneFreq2 = 1200.0f;
    } else {
      C.toneMode = TONE_SWEEP_UP;
      C.toneFreq1 = 500.0f;
      C.toneFreq2 = 1500.0f;
    }
  } else if (sub == "noise") {
    C.toneMode = TONE_NOISE;
  } else if (sub == "rhythm") {
    if (sub2 == "double_click") {
      C.toneMode = TONE_DOUBLE_CLICK;
      C.toneFreq1 = 1200.0f;
    } else if (sub2 == "triple_beep") {
      C.toneMode = TONE_TRIPLE_BEEP;
      C.toneFreq1 = 1000.0f;
    } else {
      C.toneMode = TONE_DOUBLE_CLICK;
      C.toneFreq1 = 1000.0f;
    }
  } else {
    // Fallback: simple mid beep
    C.toneMode = TONE_SIMPLE;
    C.toneFreq1 = 1000.0f;
  }

  Serial.printf("[SCENE] slot %d: id=%u TONE base=%s sub=%s sub2=%s f1=%.1f f2=%.1f mode=%d\n",
                slotIdx,
                (unsigned)cm->id,
                cm->base.c_str(),
                cm->sub.c_str(),
                cm->sub2.c_str(),
                C.toneFreq1,
                C.toneFreq2,
                (int)C.toneMode);
}

void side_setScene(uint16_t ids[4]) {
  if (diagAudioActive) {
    Serial.println("[DIAG] Ignoring SET_SCENE (diag active)");
    return;
  }
  for (int i = 0; i < 4; ++i) {
    curSlotIds[i] = ids[i];

    // Reset base state
    ch[i].idx   = 0;
    ch[i].state = IDLE;

    // No assignment → silence this slot cleanly
    if (ids[i] == 0) {
      if (ch[i].sd.f) ch[i].sd.f.close();
      ch[i].path   = "";
      ch[i].useRAM = false;
      ch[i].isTone = false;
      ch[i].toneMode = TONE_NONE;
      baseGainQ15[i] = masterGainQ15; // doesn't matter (silence), but keep sane
      ch[i].gainQ15  = q15_mul(baseGainQ15[i], slotTrimQ15[i]);
      Serial.printf("[SCENE] slot %d: id=0 (cleared)\n", i);
      continue;
    }

    const ClipMeta* cm = Manifest_find(ids[i]);
    if (!cm) {
      if (ch[i].sd.f) ch[i].sd.f.close();
      ch[i].path   = "";
      ch[i].useRAM = false;
      ch[i].isTone = false;
      ch[i].toneMode = TONE_NONE;
      baseGainQ15[i] = masterGainQ15;
      ch[i].gainQ15  = q15_mul(baseGainQ15[i], slotTrimQ15[i]);
      Serial.printf("[SCENE] slot %d: id=%u NOT FOUND\n", i, (unsigned)ids[i]);
      continue;
    }

    // Compute per-clip gain: master * per-clip (dB -> Q15)
    int32_t clipQ = q15_from_db(cm->volume_db);
    baseGainQ15[i] = q15_mul(masterGainQ15, clipQ);
    ch[i].gainQ15  = q15_mul(baseGainQ15[i], slotTrimQ15[i]);

    // If this is a synthetic tone, configure tone channel and skip SD
    if (cm->base.equalsIgnoreCase("tones")) {
      configureToneChannel(ch[i], cm, i);
      continue;
    }

    // Otherwise: file-backed audio (animals, etc.)
    ch[i].isTone = false;
    ch[i].toneMode = TONE_NONE;
    ch[i].tonePhase = 0.0f;
    ch[i].toneSweepPos = 0.0f;
    ch[i].toneSweepRate = 0.0f;
    ch[i].tonePatternSamples = 0;

    ch[i].path = cm->path;

    // Prefer PSRAM cache, else SD
    int16_t* buf = nullptr;
    size_t   samples = 0;
    if (Manifest_getCached(ids[i], &buf, &samples) && buf && samples > 0) {
      if (ch[i].sd.f) ch[i].sd.f.close();
      ch[i].useRAM       = true;
      ch[i].ram.data     = buf;
      ch[i].ram.samples  = samples;
      Serial.printf("[SCENE] slot %d: id=%u RAM OK (%s)\n", i, (unsigned)ids[i], ch[i].path.c_str());
    } else {
      ch[i].useRAM = false;
      if (!openForSD(ch[i], i)) {
        ch[i].path = "";
        if (ch[i].sd.f) ch[i].sd.f.close();
        Serial.printf("[SCENE] slot %d: id=%u SD OPEN FAIL\n", i, (unsigned)ids[i]);
      } else {
        ch[i].sd.cur = 0;
        if (ch[i].sd.f) ch[i].sd.f.seek(ch[i].sd.dataStart);
        Serial.printf("[SCENE] slot %d: id=%u SD OK (%s)\n", i, (unsigned)ids[i], ch[i].path.c_str());
      }
    }
  }
}

// Set per-slot trim (dB) sent by the Master.
// This is applied on top of the scene's base gain (master*clip).
void side_setSlotTrimDb(const int8_t db[4]) {
  if (!db) return;
  for (int i = 0; i < 4; i++) {
    slotTrimDb[i]  = db[i];
    slotTrimQ15[i] = q15_from_db(db[i]);
    ch[i].gainQ15  = q15_mul(baseGainQ15[i], slotTrimQ15[i]);
  }
  Serial.printf("[AUDIO] slot trims dB: [%d %d %d %d]\n",
                (int)slotTrimDb[0], (int)slotTrimDb[1], (int)slotTrimDb[2], (int)slotTrimDb[3]);
}

// Runtime mix-fix configuration (sent by the Master via MIXFIX_SET).
// mask bits: bit0 -> apply to I2S0 RIGHT (slot1), bit1 -> apply to I2S1 RIGHT (slot3)
void side_setMixFix(uint8_t mask, int16_t kQ12, int16_t mQ12) {
  // Clamp mask to known bits
  mask &= 0x03;

  // Clamp multipliers to a reasonable range to avoid wild overflow.
  // (Q12: 4096=1.0, 8192=2.0)
  if (kQ12 < 0) kQ12 = 0;
  if (kQ12 > 16384) kQ12 = 16384; // 4.0
  if (mQ12 < 0) mQ12 = 0;
  if (mQ12 > 16384) mQ12 = 16384; // 4.0

  mixFixMask = mask;
  mixFixKQ12 = kQ12;
  mixFixMQ12 = mQ12;

  Serial.printf("[MIXFIX] set mask=0x%02X kQ12=%d (%.3f) mQ12=%d (%.3f)\n",
                (unsigned)mask,
                (int)kQ12, (double)kQ12 / 4096.0,
                (int)mQ12, (double)mQ12 / 4096.0);
}

// ======= Audio diagnostics (tone generator) =======
static void diagClearAll() {
  // Stop blinking / set LEDs off
  blinkStop();

  // Reset trims to 0dB so diag results are not affected by mitigation.
  int8_t z[4] = {0,0,0,0};
  for (int i=0;i<4;i++) {
    slotTrimDb[i]  = 0;
    slotTrimQ15[i] = 32768;
    baseGainQ15[i] = masterGainQ15;
  }

  // Force all channels silent and detach any file playback.
  for (int i=0;i<4;i++) {
    if (ch[i].sd.f) ch[i].sd.f.close();
    ch[i].path = "";
    ch[i].useRAM = false;
    ch[i].ram.data = nullptr; // cached buffers live in Manifest cache; do not free here.
    ch[i].ram.samples = 0;
    ch[i].idx = 0;
    ch[i].state = IDLE;

    ch[i].isTone = false;
    ch[i].toneMode = TONE_NONE;
    ch[i].toneFreq1 = 440.0f;
    ch[i].toneFreq2 = 880.0f;
    ch[i].tonePhase = 0.0f;
    ch[i].toneSweepPos = 0.0f;
    ch[i].toneSweepRate = 0.0f;
    ch[i].tonePatternSamples = 0;

    // Gain: keep MASTER_GAIN_DB in effect so you can reproduce the exact
    // volume where the problem happens.
    ch[i].gainQ15 = baseGainQ15[i];
  }

  // LEDs fully off by default
  ledsAllOff();
}

static void diagToneLoopSlot(uint8_t slot, float freqHz) {
  slot &= 3;
  Channel& C = ch[slot];
  C.isTone = true;
  C.useRAM = false;
  C.path = "";
  if (C.sd.f) C.sd.f.close();

  C.toneMode = TONE_SIMPLE;
  C.toneFreq1 = freqHz;
  C.toneFreq2 = freqHz;
  C.tonePhase = 0.0f;
  C.toneSweepPos = 0.0f;
  C.toneSweepRate = 0.0f;
  C.tonePatternSamples = 0;

  C.idx = 0;
  C.state = LOOPING;

  // Ensure gain is consistent (master gain only, no per-clip dB).
  baseGainQ15[slot] = masterGainQ15;
  C.gainQ15 = baseGainQ15[slot];
}

// Public entry point called by GameBus: enable/disable + select a pattern.
// Patterns are documented in Messages.h (DIAG_AUDIO).
void side_setDiagAudio(uint8_t pattern_id) {
  // If we were previously in a MixFix toggle pattern (14/15), restore MixFix settings
  // before switching to a new pattern.
  diagMixFixStopRestore();

  diagAudioPattern = pattern_id;
  diagAudioActive  = (pattern_id != 0);

  if (!diagAudioActive) {
    // Exit diag: silence and let the Master send a fresh SET_SCENE.
    diagClearAll();
    Serial.println("[DIAG] Audio diag OFF");
    return;
  }

  // In diag, prevent local auto-play on press (presses send BTN_EVENT instead).
  gameMode = true;

  // Reset everything to a known baseline, then apply the requested pattern.
  diagClearAll();

  // LED codes: 0=red, 1=green, 2=white, 3=off
  uint8_t leds[4] = {3,3,3,3};

  // Tones chosen to be clearly distinct.
  // Pair 0 (I2S0): 440Hz (L), 880Hz (R)
  // Pair 1 (I2S1): 550Hz (L), 1100Hz (R)
  const float f0L = 440.0f;
  const float f0R = 880.0f;
  const float f1L = 550.0f;
  const float f1R = 1100.0f;

  // Odd/common simulation tones
  const float fCommon = 800.0f;
  const float fOdd    = 1600.0f;

  switch (pattern_id) {
    default:
    case 1: // I2S0 Left only (slot0)
      diagToneLoopSlot(0, f0L);
      leds[0] = 2;
      break;
    case 2: // I2S0 Right only (slot1)
      diagToneLoopSlot(1, f0R);
      leds[1] = 2;
      break;
    case 3: // I2S0 L+R simultaneously (different tones)
      diagToneLoopSlot(0, f0L);
      diagToneLoopSlot(1, f0R);
      leds[0] = 2; leds[1] = 2;
      break;
    case 4: // I2S1 Left only (slot2)
      diagToneLoopSlot(2, f1L);
      leds[2] = 2;
      break;
    case 5: // I2S1 Right only (slot3)
      diagToneLoopSlot(3, f1R);
      leds[3] = 2;
      break;
    case 6: // I2S1 L+R simultaneously (different tones)
      diagToneLoopSlot(2, f1L);
      diagToneLoopSlot(3, f1R);
      leds[2] = 2; leds[3] = 2;
      break;
    case 7: // All four unique tones
      diagToneLoopSlot(0, 330.0f);
      diagToneLoopSlot(1, 660.0f);
      diagToneLoopSlot(2, 990.0f);
      diagToneLoopSlot(3, 1320.0f);
      leds[0] = 2; leds[1] = 2; leds[2] = 2; leds[3] = 2;
      break;

    // Odd/common simulation (helps reproduce the exact symptom)
    case 8: // odd on slot0
      diagToneLoopSlot(0, fOdd);
      diagToneLoopSlot(1, fCommon);
      diagToneLoopSlot(2, fCommon);
      diagToneLoopSlot(3, fCommon);
      leds[0] = 1; leds[1] = 0; leds[2] = 0; leds[3] = 0;
      break;
    case 9: // odd on slot1
      diagToneLoopSlot(0, fCommon);
      diagToneLoopSlot(1, fOdd);
      diagToneLoopSlot(2, fCommon);
      diagToneLoopSlot(3, fCommon);
      leds[0] = 0; leds[1] = 1; leds[2] = 0; leds[3] = 0;
      break;
    case 10: // odd on slot2
      diagToneLoopSlot(0, fCommon);
      diagToneLoopSlot(1, fCommon);
      diagToneLoopSlot(2, fOdd);
      diagToneLoopSlot(3, fCommon);
      leds[0] = 0; leds[1] = 0; leds[2] = 1; leds[3] = 0;
      break;
    case 11: // odd on slot3
      diagToneLoopSlot(0, fCommon);
      diagToneLoopSlot(1, fCommon);
      diagToneLoopSlot(2, fCommon);
      diagToneLoopSlot(3, fOdd);
      leds[0] = 0; leds[1] = 0; leds[2] = 0; leds[3] = 1;
      break;

    case 12: // PHASE-CANCEL I2S0: slot0=+440Hz, slot1=-440Hz
      diagToneLoopSlot(0, f0L);
      diagToneLoopSlot(1, f0L);
      ch[1].gainQ15 = -ch[1].gainQ15;
      leds[0] = 2; leds[1] = 2;
      break;

    case 13: // PHASE-CANCEL I2S1: slot2=+550Hz, slot3=-550Hz
      diagToneLoopSlot(2, f1L);
      diagToneLoopSlot(3, f1L);
      ch[3].gainQ15 = -ch[3].gainQ15;
      leds[2] = 2; leds[3] = 2;
      break;

    case 14: // MIXFIX TOGGLE (I2S0): slot0=+440Hz, slot1 silent; toggles mixfix for I2S0
      diagToneLoopSlot(0, f0L);
      diagMixFixStartToggle(0x01);
      leds[0] = 2; // slot0 white
      leds[1] = 0; // slot1 red initially (OFF); will be updated in diagMixFixTick()
      break;

    case 15: // MIXFIX TOGGLE (I2S1): slot2=+550Hz, slot3 silent; toggles mixfix for I2S1
      diagToneLoopSlot(2, f1L);
      diagMixFixStartToggle(0x02);
      leds[2] = 2; // slot2 white
      leds[3] = 0; // slot3 red initially (OFF)
      break;
  }

  ledsSlotsByCode(leds);
  Serial.printf("[DIAG] Audio pattern=%u\n", (unsigned)pattern_id);
}

void side_playSlot(uint8_t slot) {
  if (diagAudioActive) return;
  slot &= 3;
  Channel& C = ch[slot];
  C.state = PLAYING;
  C.idx   = 0;

  if (C.isTone && C.toneMode != TONE_NONE) {
    // Reset tone phase/pattern for a clean one-shot
    C.tonePhase = 0.0f;
    C.toneSweepPos = 0.0f;
    C.toneSweepRate = 0.0f;
    C.tonePatternSamples = 0;
  } else {
    if (!C.useRAM) {
      C.sd.cur = 0;
      if (C.sd.f) C.sd.f.seek(C.sd.dataStart);
    }
  }
}

void side_ledAllWhite() {
  if (diagAudioActive) return;
  blinkStop();
  ledsAllColor(255, 255, 255);
}

// Soft white refresh: stop blinking but do NOT force a full "all off" frame first.
// This helps when the Master retries white during WAIT (avoids visible flicker).
void side_ledAllWhiteSoft() {
  if (diagAudioActive) return;
  blink.active = false;
  blink.phaseOn = false;
  blink.remaining = 0;
  ledsAllColor(255, 255, 255);
}

// Set SOLID per-slot LED colors (no blinking).
// codes: 0=red, 1=green, 2=white, 3=off
void side_ledSolidSlots(const uint8_t slotColors[4]) {
  if (diagAudioActive) return;
  // Cancel any blinking without forcing an all-off frame.
  blink.active = false;
  blink.phaseOn = false;
  blink.remaining = 0;
  ledsSlotsByCode(slotColors);
}


void side_blinkAll(uint8_t color, uint16_t on_ms, uint16_t off_ms) {
  if (diagAudioActive) return;
  blinkStart(color, on_ms, off_ms, /*reps*/BLINK_REPS);
}

// Per-slot blink pattern (used by the Master to reveal the correct answer).
// slotColors: 0=red, 1=green, 2=white, 3=off
void side_blinkSlots(const uint8_t slotColors[4], uint16_t on_ms, uint16_t off_ms) {
  if (diagAudioActive) return;
  blinkStartSlots(slotColors, on_ms, off_ms, /*reps*/BLINK_REPS);
}

void side_setGameMode(bool en){ gameMode=en; }

void side_startLoopAll(){
  if (diagAudioActive) return;
  for (int i=0;i<4;i++){
    // IMPORTANT: make this command idempotent.
    // We often resend START_LOOP_ALL from the Master for reliability (ESP-NOW can drop/reorder).
    // If a channel is *already looping*, do NOT restart it (avoid audible restart/glitch).
    if (ch[i].state == LOOPING) continue;

    ch[i].state = LOOPING;
    ch[i].idx = 0;
    if (ch[i].isTone && ch[i].toneMode != TONE_NONE) {
      ch[i].tonePhase = 0.0f;
      ch[i].toneSweepPos = 0.0f;
      ch[i].toneSweepRate = 0.0f;
      ch[i].tonePatternSamples = 0;
    } else if (!ch[i].useRAM) {
      ch[i].sd.cur = 0;
      if (ch[i].sd.f) ch[i].sd.f.seek(ch[i].sd.dataStart);
    }
  }
}

void side_stopAll(){
  for (int i=0;i<4;i++){
    ch[i].state = IDLE;
  }
}

void printSideMacs() {
  uint8_t sta[6], ap[6];
  esp_wifi_get_mac(WIFI_IF_STA, sta);
  esp_wifi_get_mac(WIFI_IF_AP,  ap);

  uint8_t sid = Role::get();
  if (sid==0xFF) {
    Serial.printf("Side UNASSIGNED STA MAC: %02X:%02X:%02X:%02X:%02X:%02X\n",
                  sta[0],sta[1],sta[2],sta[3],sta[4],sta[5]);
    Serial.printf("Side UNASSIGNED AP  MAC: %02X:%02X:%02X:%02X:%02X:%02X\n",
                  ap[0],ap[1],ap[2],ap[3],ap[4],ap[5]);
  } else {
    Serial.printf("Side %u STA MAC: %02X:%02X:%02X:%02X:%02X:%02X\n", (unsigned)sid,
                  sta[0],sta[1],sta[2],sta[3],sta[4],sta[5]);
    Serial.printf("Side %u AP  MAC: %02X:%02X:%02X:%02X:%02X:%02X\n", (unsigned)sid,
                  ap[0],ap[1],ap[2],ap[3],ap[4],ap[5]);
  }
}

// ======= Setup =======
void setup() {
  Serial.begin(115200);
  delay(80);

  Role::begin();

  uint8_t sid = Role::get();
  Serial.printf("\n[Seashells Side %s]\n",
                sid==0xFF ? "UNASSIGNED" : (sid==0 ? "A" : "B"));

  masterGainQ15 = q15_from_db(MASTER_GAIN_DB);

  // Default trims to 0 dB so effective gain starts at baseGain.
  for (int i=0;i<4;i++) {
    slotTrimDb[i]  = 0;
    slotTrimQ15[i] = 32768;
    baseGainQ15[i] = masterGainQ15;
    ch[i].gainQ15  = baseGainQ15[i];
  }

  for (int i=0;i<4;++i) pinMode(BTN_PINS[i], INPUT);

  RGBT[0]->begin(); RGBT[0]->setBrightness(BRIGHTNESS); ledOff(0);
  RGBT[1]->begin(); RGBT[1]->setBrightness(BRIGHTNESS); ledOff(1);
  RGBT[2]->begin(); RGBT[2]->setBrightness(BRIGHTNESS); ledOff(2);
  RGBT[3]->begin(); RGBT[3]->setBrightness(BRIGHTNESS); ledOff(3);

  SPI.begin(SD_SCK, SD_MISO, SD_MOSI);
  pinMode(SD_CS, OUTPUT); digitalWrite(SD_CS, HIGH);
  if (!SD.begin(SD_CS, SPI, 12000000)) {
    Serial.println("SD mount failed (check wiring/FAT32)"); while(1) delay(1000);
  }
  Serial.println("SD OK");
  listRootOnce();

  if (!Manifest_load()) Serial.println("[WARN] No manifest loaded");
  Manifest_precacheAll();

  for (int i=0;i<4;i++){
    ch[i].path="";
    ch[i].state=IDLE;
    ch[i].useRAM=false;
    ch[i].idx=0;
    ch[i].isTone=false;
    ch[i].toneMode=TONE_NONE;
  }

  i2s_init_common(I2S_NUM_0, I2S0_DOUT, I2S0_BCLK, I2S0_LRCK);
  i2s_init_common(I2S_NUM_1, I2S1_DOUT, I2S1_BCLK, I2S1_LRCK);

  GameBus_init();

  printSideMacs();
  Serial.printf("[SIDE] role=%s\n", Role::get()==0xFF?"UNASSIGNED":(Role::get()==0?"A":"B"));

  uint16_t a=0,b=0;
  for (uint16_t id=0; id<65535; id++){
    const ClipMeta* cm = Manifest_find(id);
    if (!cm) continue;
    (cm->pool==POOL_A)?a++:b++;
  }
  GameBus_sendHello(a,b);
}

// ======= Main loop =======
// --- Helper for MixFix (runtime) ---
static inline int16_t sat16(int32_t x) {
  if (x > 32767) return 32767;
  if (x < -32768) return -32768;
  return (int16_t)x;
}

// MixFix: R_out = (k*R - m*L) >> 12
static inline int16_t mixFixRightSample(int16_t L, int16_t R, int16_t kQ12, int16_t mQ12) {
  int32_t v = ((int32_t)kQ12 * (int32_t)R - (int32_t)mQ12 * (int32_t)L) >> 12;
  return sat16(v);
}

// DIAG helper: patterns 14/15 toggle MixFix on/off periodically to prove whether
// the RIGHT speaker is electrically mixing L+R (vs just acoustic bleed).
static inline void diagMixFixStopRestore() {
  diagMixFixToggleActive = false;
  diagMixFixToggleBit    = 0;
  diagMixFixOn           = false;
  diagMixFixNextMs       = 0;

  if (diagMixFixSaved) {
    mixFixMask = diagMixFixSavedMask;
    mixFixKQ12 = diagMixFixSavedKQ12;
    mixFixMQ12 = diagMixFixSavedMQ12;
    diagMixFixSaved = false;
  }
}

static inline void diagMixFixStartToggle(uint8_t bit /*0x01 or 0x02*/) {
  if (!diagMixFixSaved) {
    diagMixFixSavedMask = mixFixMask;
    diagMixFixSavedKQ12 = mixFixKQ12;
    diagMixFixSavedMQ12 = mixFixMQ12;
    diagMixFixSaved = true;
  }

  // Use a known-good setting for the test (works for the common MIX ~= (L+R)/2 case).
  mixFixKQ12 = 8192;  // 2.0
  mixFixMQ12 = 4096;  // 1.0

  diagMixFixToggleActive = true;
  diagMixFixToggleBit    = (bit & 0x03);
  diagMixFixOn           = false;
  diagMixFixNextMs       = millis() + 900;

  // Ensure starting state is OFF for the toggled bit.
  mixFixMask = (uint8_t)(mixFixMask & (uint8_t)~diagMixFixToggleBit);
}

static inline void diagMixFixTick() {
  if (!diagAudioActive) return;
  if (!diagMixFixToggleActive) return;
  if (!(diagAudioPattern == 14 || diagAudioPattern == 15)) return;

  uint32_t now = millis();
  if ((int32_t)(now - diagMixFixNextMs) < 0) return;
  diagMixFixNextMs = now + 900;
  diagMixFixOn = !diagMixFixOn;

  uint8_t m = mixFixMask;
  if (diagMixFixOn) m |= diagMixFixToggleBit;
  else              m &= (uint8_t)~diagMixFixToggleBit;
  mixFixMask = m;

  // Update LEDs so you can see the current state:
  //   RED = MixFix OFF, GREEN = MixFix ON
  uint8_t codes[4] = {3,3,3,3};
  if (diagAudioPattern == 14) {
    codes[0] = 2; // slot0 white (tone)
    codes[1] = diagMixFixOn ? 1 : 0;
  } else {
    codes[2] = 2; // slot2 white (tone)
    codes[3] = diagMixFixOn ? 1 : 0;
  }
  ledsSlotsByCode(codes);
}

void loop() {
  Ota_loopTick();
  GameBus_pump();  // process queued ESP-NOW commands in the main loop (avoids LED glitches)

  uint32_t now = millis();
  // DIAG patterns 14/15: periodically toggle MixFix on/off.
  diagMixFixTick();

  // Snapshot MixFix params once per audio frame (avoid reading volatile per sample)
  uint8_t  mfMask = mixFixMask;
  int16_t  mfKQ12 = mixFixKQ12;
  int16_t  mfMQ12 = mixFixMQ12;
  for (int i=0;i<4;++i) {
    bool raw = (digitalRead(BTN_PINS[i]) == LOW);
    if (raw != lastRaw[i]) { lastRaw[i] = raw; lastChangeMs[i] = now; }
    if ((now - lastChangeMs[i]) > DEBOUNCE_MS) {
      if (pressed[i] != raw) {
        pressed[i] = raw;
        if (pressed[i]) {
          if (!gameMode) {
            ledWhite(i);
            side_playSlot(i);
          } else {
            Serial.printf("[SIDE] BTN press slot=%d, sending BTN_EVENT (role=%u)\n",
                          i, (unsigned)Role::get());
            GameBus_sendBtnEvent(i);
          }
        } else {
          if (!gameMode) ledOff(i);
        }
      }
    }
  }

  for (int i=0;i<4;++i) fillChannelFrame(i, tmpMono[i]);
  for (int i=0;i<4;++i) applyGain(tmpMono[i], FRAME_SAMPLES, ch[i].gainQ15);

  {
    int16_t* o = (int16_t*)outLR0;
    int16_t* L = tmpMono[0];
    int16_t* R = tmpMono[1];
    for (size_t n=0;n<FRAME_SAMPLES;++n) {
      int16_t l = L[n];
      int16_t r = R[n];
      if (mfMask & 0x01) {
        r = mixFixRightSample(l, r, mfKQ12, mfMQ12);
      }
      *o++ = l;
      *o++ = r;
    }
  }
  {
    int16_t* o = (int16_t*)outLR1;
    int16_t* L = tmpMono[2];
    int16_t* R = tmpMono[3];
    for (size_t n=0;n<FRAME_SAMPLES;++n) {
      int16_t l = L[n];
      int16_t r = R[n];
      if (mfMask & 0x02) {
        r = mixFixRightSample(l, r, mfKQ12, mfMQ12);
      }
      *o++ = l;
      *o++ = r;
    }
  }

  size_t w0=0, w1=0;
  i2s_write(I2S_NUM_0, outLR0, OUT_BYTES, &w0, portMAX_DELAY);
  i2s_write(I2S_NUM_1, outLR1, OUT_BYTES, &w1, portMAX_DELAY);

  blinkUpdate();
}
