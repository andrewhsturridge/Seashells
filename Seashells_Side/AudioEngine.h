#pragma once
#include <Arduino.h>
#include <SD.h>
#include "driver/i2s.h"
#include "ConfigSide.h"   // pins, SAMPLE_RATE, SD_* defines

// ---- Playback state & channel types (moved out of the .ino) ----
enum PlayState : uint8_t { IDLE=0, PLAYING=1, LOOPING=2 };

struct TrackRAM { int16_t* data = nullptr; size_t samples = 0; };
struct TrackSD  { File f; uint32_t dataStart = 44, dataEnd = 44, cur = 0; };

struct Channel {
  String   path;
  PlayState state = IDLE;
  uint8_t   vol   = 255;
  size_t    idx   = 0;
  bool      useRAM = false;
  TrackRAM  ram;
  TrackSD   sd;
  int16_t   gainQ15 = 32767;   // Q1.15
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
int16_t  q15_from_db(int8_t db);
int16_t  q15_mul(int16_t a, int16_t b);
void     applyGain(int16_t* buf, size_t n, int16_t g);

// Exposed so you can set it once from the .ino, e.g.:
//   masterGainQ15 = q15_from_db(MASTER_GAIN_DB);
extern int16_t masterGainQ15;
