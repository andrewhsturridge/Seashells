#pragma once
#include <Arduino.h>
#include <SD.h>
#include "driver/i2s.h"
#include "ConfigSide.h"   // pins, SAMPLE_RATE, SD_* defines

// ---- Playback state & channel types ----
enum PlayState : uint8_t { IDLE=0, PLAYING=1, LOOPING=2 };

// Source for a channel: file-backed (WAV) or synthetic tone
enum ToneMode : uint8_t {
  TONE_NONE = 0,
  TONE_SIMPLE = 1,
  TONE_SWEEP_UP = 2,
  TONE_SWEEP_DOWN = 3,
  TONE_SIREN = 4,
  TONE_NOISE = 5,
  TONE_DOUBLE_CLICK = 6,
  TONE_TRIPLE_BEEP = 7
};

struct TrackRAM { int16_t* data = nullptr; size_t samples = 0; };
struct TrackSD  { File f; uint32_t dataStart = 44, dataEnd = 44, cur = 0; };

struct Channel {
  // File-backed audio fields
  String   path;
  PlayState state = IDLE;
  uint8_t   vol   = 255;
  size_t    idx   = 0;
  bool      useRAM = false;
  TrackRAM  ram;
  TrackSD   sd;
  int32_t   gainQ15 = 32768;   // Q15 (1.0 == 32768)

  // Tone synthesis fields (used when isTone = true)
  bool      isTone = false;
  ToneMode  toneMode = TONE_NONE;
  float     toneFreq1 = 440.0f;     // base frequency (Hz)
  float     toneFreq2 = 880.0f;     // secondary freq for sweeps/siren (Hz)
  float     tonePhase = 0.0f;       // 0..2π
  float     toneSweepPos = 0.0f;    // 0..1 for sweeps/LFO
  float     toneSweepRate = 0.0f;   // step per sample for sweepPos
  uint32_t  tonePatternSamples = 0; // for rhythmic patterns
};

// Global channels live in your .ino; this gives us access here
extern Channel ch[4];

// ---- Prototypes (same names you already use) ----
void     listRootOnce();
bool     remountSD(uint32_t hz);
bool     remountAndReopenAll(uint32_t hz1 = 12000000, uint32_t hz2 = 8000000);
bool     openForSD(Channel& C, int idx);
bool     loadWavIntoRam(const char* path, const char* tag, int16_t** outBuf, size_t* outSamples);
size_t   sdReadReliable(Channel& C, uint8_t* dst, size_t want);
void     fillChannelFrame(int idx, int16_t* dst);  // uses internal frame constants
void     i2s_init_common(i2s_port_t port, int dout, int bclk, int lrck);

// Volume helpers & master gain (moved out of .ino)
int32_t  q15_from_db(int8_t db);
int32_t  q15_mul(int32_t a, int32_t b);
void     applyGain(int16_t* buf, size_t n, int32_t g);

// Exposed so you can set it once from the .ino, e.g.:
//   masterGainQ15 = q15_from_db(MASTER_GAIN_DB);
extern int32_t masterGainQ15;
