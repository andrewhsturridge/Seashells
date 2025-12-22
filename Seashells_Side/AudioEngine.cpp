#include "AudioEngine.h"
#include <SPI.h>
#include <math.h>

// Local constants used by helpers (keep in sync with your .ino)
static constexpr size_t kFrameSamples = 1024;                // per-channel samples/frame
static constexpr size_t kBytesPerCh   = kFrameSamples * 2;   // 16-bit mono
static constexpr float  kTwoPi        = 6.28318530718f;

// Set to 1 if you want verbose WAV header prints.
static constexpr bool kWavDebug = false;

// ───────────────── SD & WAV helpers ─────────────────

static inline uint16_t rd16le(const uint8_t* p) {
  return (uint16_t)p[0] | ((uint16_t)p[1] << 8);
}
static inline uint32_t rd32le(const uint8_t* p) {
  return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

struct WavInfo {
  uint32_t dataStart = 44;   // byte offset to PCM
  uint32_t dataBytes = 0;    // PCM byte count
  uint16_t fmt       = 0;    // 1 = PCM
  uint16_t channels  = 0;
  uint32_t sampleRate = 0;
  uint16_t bits       = 0;
};

// Robustly locate the "data" chunk (and read basic fmt info). This fixes clicks/pops caused by
// assuming the WAV header is always 44 bytes.
static bool parseWavHeader(File& f, WavInfo& wi, const char* tag) {
  wi = WavInfo{};
  if (!f) return false;

  // RIFF header
  uint8_t riff[12];
  f.seek(0);
  if (f.read(riff, sizeof(riff)) != sizeof(riff)) {
    Serial.printf("%s: WAV read header FAIL\n", tag);
    return false;
  }
  if (memcmp(riff + 0, "RIFF", 4) != 0 || memcmp(riff + 8, "WAVE", 4) != 0) {
    Serial.printf("%s: not RIFF/WAVE\n", tag);
    return false;
  }

  bool foundFmt  = false;
  bool foundData = false;
  uint32_t guard = 0;

  // Walk chunks
  while (f.available() && guard++ < 64) { // guard against malformed files
    uint8_t chdr[8];
    if (f.read(chdr, sizeof(chdr)) != sizeof(chdr)) break;
    char cid[5] = { (char)chdr[0], (char)chdr[1], (char)chdr[2], (char)chdr[3], 0 };
    uint32_t csz = rd32le(chdr + 4);
    uint32_t cpos = (uint32_t)f.position();

    if (memcmp(cid, "fmt ", 4) == 0) {
      if (csz < 16) {
        Serial.printf("%s: WAV fmt chunk too small (%lu)\n", tag, (unsigned long)csz);
        return false;
      }
      uint8_t fmt16[16];
      if (f.read(fmt16, sizeof(fmt16)) != sizeof(fmt16)) {
        Serial.printf("%s: WAV fmt read FAIL\n", tag);
        return false;
      }
      wi.fmt       = rd16le(fmt16 + 0);
      wi.channels  = rd16le(fmt16 + 2);
      wi.sampleRate = rd32le(fmt16 + 4);
      wi.bits      = rd16le(fmt16 + 14);
      foundFmt = true;

      // Skip rest of fmt chunk if any
      uint32_t skipTo = cpos + csz;
      if (csz > 16) skipTo = cpos + csz; // already read 16 bytes, but we'll seek absolute
      // Word-align
      if (csz & 1) skipTo++;
      f.seek(skipTo);
      continue;
    }

    if (memcmp(cid, "data", 4) == 0) {
      wi.dataStart = cpos;
      wi.dataBytes = csz;
      foundData = true;
      break;
    }

    // Skip unhandled chunk
    uint32_t skipTo = cpos + csz;
    if (csz & 1) skipTo++; // pad to word boundary
    if (!f.seek(skipTo)) break;
  }

  if (!foundData) {
    Serial.printf("%s: WAV has no data chunk\n", tag);
    return false;
  }

  // Clamp dataBytes to file size (defensive)
  uint32_t fileSize = (uint32_t)f.size();
  if (wi.dataStart >= fileSize) {
    Serial.printf("%s: WAV bad dataStart (%lu >= %lu)\n", tag,
                  (unsigned long)wi.dataStart, (unsigned long)fileSize);
    return false;
  }
  if (wi.dataBytes == 0 || (wi.dataStart + wi.dataBytes) > fileSize) {
    wi.dataBytes = fileSize - wi.dataStart;
  }

  // Print format warnings (don’t hard-fail sample rate, but do require 16-bit mono PCM)
  if (kWavDebug) {
    Serial.printf("%s: WAV fmt=%u ch=%u sr=%lu bits=%u dataStart=%lu dataBytes=%lu\n",
                  tag, (unsigned)wi.fmt, (unsigned)wi.channels, (unsigned long)wi.sampleRate,
                  (unsigned)wi.bits, (unsigned long)wi.dataStart, (unsigned long)wi.dataBytes);
  }
  if (!foundFmt) {
    Serial.printf("%s: WAV missing fmt chunk (assuming 16-bit mono PCM)\n", tag);
    // allow, but we can’t validate
  } else {
    if (wi.fmt != 1) {
      Serial.printf("%s: WAV unsupported format (fmt=%u)\n", tag, (unsigned)wi.fmt);
      return false;
    }
    if (wi.bits != 16) {
      Serial.printf("%s: WAV unsupported bits (%u), need 16-bit\n", tag, (unsigned)wi.bits);
      return false;
    }
    if (wi.channels != 1) {
      Serial.printf("%s: WAV unsupported channels (%u), need mono\n", tag, (unsigned)wi.channels);
      return false;
    }
    if (wi.sampleRate != 0 && wi.sampleRate != SAMPLE_RATE) {
      Serial.printf("%s: WAV sampleRate=%lu (engine=%u) → will play at wrong speed\n",
                    tag, (unsigned long)wi.sampleRate, (unsigned)SAMPLE_RATE);
    }
  }

  // Ensure even byte count (16-bit samples)
  wi.dataBytes &= ~1u;
  return true;
}

void listRootOnce() {
  Serial.println(F("SD root listing:"));
  File root = SD.open("/");
  if (!root) { Serial.println(F("  (cannot open /)")); return; }
  for (File e = root.openNextFile(); e; e = root.openNextFile()) {
    Serial.printf("  %s  %lu bytes\n", e.name(), (unsigned long)e.size());
    e.close();
  }
  root.close();
}

bool remountSD(uint32_t hz) {
  SD.end(); delay(2);
  SPI.end(); delay(2);
  SPI.begin(SD_SCK, SD_MISO, SD_MOSI);
  pinMode(SD_CS, OUTPUT); digitalWrite(SD_CS, HIGH);
  bool ok = SD.begin(SD_CS, SPI, hz);
  Serial.printf("SD remount %s @ %lu Hz\n", ok ? "OK" : "FAIL", (unsigned long)hz);
  return ok;
}

bool remountAndReopenAll(uint32_t hz1, uint32_t hz2) {
  if (!remountSD(hz1)) if (!remountSD(hz2)) return false;
  bool any = false;
  for (int i=0;i<4;++i) {
    if (ch[i].useRAM || ch[i].isTone) { any = true; continue; } // RAM cached or tone: nothing to reopen
    if (!ch[i].path.length()) continue;

    File f = SD.open(ch[i].path.c_str(), FILE_READ);
    if (f) {
      WavInfo wi;
      if (!parseWavHeader(f, wi, "REOPEN")) {
        Serial.printf("CH%d: REOPEN WAV PARSE FAIL %s\n", i+1, ch[i].path.c_str());
        f.close();
        continue;
      }
      ch[i].sd.f = f;
      ch[i].sd.dataStart = wi.dataStart;
      ch[i].sd.dataEnd   = wi.dataStart + wi.dataBytes;
      ch[i].sd.cur = 0;
      ch[i].sd.f.seek(ch[i].sd.dataStart);
      Serial.printf("CH%d: REOPENED %s (dataStart=%lu)\n", i+1, ch[i].path.c_str(), (unsigned long)wi.dataStart);
      any = true;
    } else {
      Serial.printf("CH%d: REOPEN FAILED %s\n", i+1, ch[i].path.c_str());
    }
  }
  return any;
}

bool openForSD(Channel& C, int idx) {
  if (C.sd.f) C.sd.f.close();
  if (!C.path.length()) {
    Serial.printf("CH%d: OPEN (no path)\n", idx+1);
    return false;
  }
  C.sd.f = SD.open(C.path.c_str(), FILE_READ);
  Serial.printf("CH%d: OPEN %s %s\n", idx+1, C.path.c_str(), C.sd.f?"OK":"FAIL");
  if (!C.sd.f) return false;

  WavInfo wi;
  char tag[12];
  snprintf(tag, sizeof(tag), "CH%d", idx+1);
  if (!parseWavHeader(C.sd.f, wi, tag)) {
    C.sd.f.close();
    return false;
  }

  C.sd.dataStart = wi.dataStart;
  C.sd.dataEnd   = wi.dataStart + wi.dataBytes;
  if (C.sd.dataEnd <= C.sd.dataStart) {
    Serial.printf("CH%d: BAD WAV data range (%lu..%lu)\n", idx+1,
                  (unsigned long)C.sd.dataStart, (unsigned long)C.sd.dataEnd);
    C.sd.f.close();
    return false;
  }
  C.sd.cur = 0;
  C.sd.f.seek(C.sd.dataStart);
  return true;
}

bool loadWavIntoRam(const char* path, const char* tag, int16_t** outBuf, size_t* outSamples) {
  File f = SD.open(path, FILE_READ);
  if (!f) { Serial.printf("%s: RAM load OPEN FAIL %s\n", tag, path); return false; }

  WavInfo wi;
  if (!parseWavHeader(f, wi, tag)) { f.close(); return false; }

  size_t dataBytes = (size_t)wi.dataBytes;
  size_t samples   = dataBytes / 2;

  int16_t* buf = (int16_t*)ps_malloc(dataBytes);
  if (!buf) { Serial.printf("%s: RAM alloc FAIL (%u bytes)\n", tag, (unsigned)dataBytes); f.close(); return false; }

  f.seek(wi.dataStart);
  size_t off=0;
  while (off < dataBytes) {
    size_t n = f.read(((uint8_t*)buf)+off, dataBytes - off);
    if (n == 0) {
      Serial.printf("%s: RAM read FAIL @%u\n", tag, (unsigned)off);
      free(buf);
      f.close();
      return false;
    }
    off += n;
    yield();
  }
  f.close();

  *outBuf = buf;
  *outSamples = samples;
  Serial.printf("%s: RAM cached %u samples (%.2f s @ %u Hz)\n",
                tag, (unsigned)samples, (double)samples/SAMPLE_RATE, SAMPLE_RATE);
  return true;
}

size_t sdReadReliable(Channel& C, uint8_t* dst, size_t want) {
  if (C.useRAM || C.isTone) return 0; // not used here
  uint8_t retries = 0; bool remounted = false;
  size_t total = 0;
  uint32_t dataBytes = C.sd.dataEnd - C.sd.dataStart;
  while (total < want) {
    if (C.sd.cur >= dataBytes) break; // EOF
    if (!C.sd.f) {
      if (retries < 2) {
        retries++;
        if (C.path.length()) {
          File f = SD.open(C.path.c_str(), FILE_READ);
          if (f) {
            // Re-parse to recover correct dataStart if the file has extra chunks.
            WavInfo wi;
            if (parseWavHeader(f, wi, "RECOVER")) {
              C.sd.dataStart = wi.dataStart;
              C.sd.dataEnd   = wi.dataStart + wi.dataBytes;
              C.sd.f = f;
              C.sd.f.seek(C.sd.dataStart + C.sd.cur);
              continue;
            }
            f.close();
          }
        }
      }
      if (!remounted) {
        remounted = true;
        if (remountAndReopenAll()) {
          if (C.sd.f) {
            C.sd.f.seek(C.sd.dataStart + C.sd.cur);
            continue;
          }
        }
      }
      break;
    }
    C.sd.f.seek(C.sd.dataStart + C.sd.cur);
    uint32_t maxNow = dataBytes - C.sd.cur;
    size_t chunk = min((size_t)maxNow, want - total);
    size_t n = C.sd.f.read(dst + total, chunk);
    if (n == 0) {
      delay(1);
      if (++retries <= 2) continue;
      if (!remounted) { remounted = true; if (remountAndReopenAll()) continue; }
      break;
    }
    C.sd.cur += n;
    total += n;
  }
  return total;
}

// ───────────────── Tone synthesis ─────────────────

// Generate a single sample for a tone channel, in [-1.0, +1.0].
static float synthNextSample(Channel& C) {
  const float sr = (float)SAMPLE_RATE;
  float s = 0.0f;

  switch (C.toneMode) {
    case TONE_SIMPLE: {
      float freq = C.toneFreq1;
      float inc  = kTwoPi * freq / sr;
      C.tonePhase += inc;
      if (C.tonePhase > kTwoPi) C.tonePhase -= kTwoPi;
      s = sinf(C.tonePhase) * 0.35f;
    } break;

    case TONE_SWEEP_UP:
    case TONE_SWEEP_DOWN: {
      if (C.toneSweepRate <= 0.0f) {
        C.toneSweepRate = 1.0f / (sr * 0.4f); // ~400ms sweep
      }
      C.toneSweepPos += C.toneSweepRate;
      if (C.toneSweepPos >= 1.0f) C.toneSweepPos -= 1.0f;

      float t = C.toneSweepPos;
      if (C.toneMode == TONE_SWEEP_DOWN) t = 1.0f - t;
      float freq = C.toneFreq1 + (C.toneFreq2 - C.toneFreq1) * t;
      float inc  = kTwoPi * freq / sr;
      C.tonePhase += inc;
      if (C.tonePhase > kTwoPi) C.tonePhase -= kTwoPi;
      s = sinf(C.tonePhase) * 0.35f;
    } break;

    case TONE_SIREN: {
      if (C.toneSweepRate <= 0.0f) {
        C.toneSweepRate = 1.0f / (sr * 1.2f); // ~1.2s LFO
      }
      C.toneSweepPos += C.toneSweepRate;
      if (C.toneSweepPos >= 1.0f) C.toneSweepPos -= 1.0f;
      float lfo = sinf(kTwoPi * C.toneSweepPos); // -1..+1

      float fMid = 0.5f * (C.toneFreq1 + C.toneFreq2);
      float fDev = 0.5f * (C.toneFreq2 - C.toneFreq1);
      float freq = fMid + fDev * lfo;

      float inc  = kTwoPi * freq / sr;
      C.tonePhase += inc;
      if (C.tonePhase > kTwoPi) C.tonePhase -= kTwoPi;
      s = sinf(C.tonePhase) * 0.35f;
    } break;

    case TONE_NOISE: {
      // Simple white noise in [-0.4, +0.4]
      int32_t r = (int32_t)random(-32768, 32767);
      s = (float)r / 32768.0f * 0.4f;
    } break;

    case TONE_DOUBLE_CLICK:
    case TONE_TRIPLE_BEEP: {
      const float freq = C.toneFreq1;
      const float inc  = kTwoPi * freq / sr;

      const uint32_t beepSamples = (uint32_t)(sr * 0.04f); // 40 ms beep
      const uint32_t gapSamples  = (uint32_t)(sr * 0.04f); // 40 ms gap

      uint32_t patternTotal = 0;
      if (C.toneMode == TONE_DOUBLE_CLICK) {
        // beep, gap, beep, long gap
        patternTotal = beepSamples + gapSamples + beepSamples + (gapSamples * 4);
      } else {
        // triple beep: beep,gap,beep,gap,beep,long gap
        patternTotal = (beepSamples * 3) + (gapSamples * 5);
      }

      uint32_t pos = C.tonePatternSamples % (patternTotal ? patternTotal : 1);
      C.tonePatternSamples++;

      bool on = false;
      if (C.toneMode == TONE_DOUBLE_CLICK) {
        if (pos < beepSamples) on = true; // first beep
        else if (pos >= (beepSamples + gapSamples) &&
                 pos < (beepSamples + gapSamples + beepSamples)) on = true; // second beep
      } else {
        // triple beep
        if (pos < beepSamples) on = true; // first
        else if (pos >= (beepSamples + gapSamples) &&
                 pos < (beepSamples + gapSamples + beepSamples)) on = true; // second
        else if (pos >= (2*beepSamples + 2*gapSamples) &&
                 pos < (2*beepSamples + 2*gapSamples + beepSamples)) on = true; // third
      }

      if (on) {
        C.tonePhase += inc;
        if (C.tonePhase > kTwoPi) C.tonePhase -= kTwoPi;
        s = sinf(C.tonePhase) * 0.4f;
      } else {
        s = 0.0f;
      }
    } break;

    default:
    case TONE_NONE:
      s = 0.0f;
      break;
  }

  return s;
}

// ───────────────── Frame filling ─────────────────

// NOTE: Our audio loop uses fixed-size frames (kFrameSamples).
// If a looping clip ends mid-frame and we pad the remainder with zeros, you'll hear a "click"/"gap"
// when the loop restarts. To avoid that, LOOPING channels wrap within the SAME frame so every frame
// stays fully filled with audio.
// We also apply a very short declick ramp at wrap boundaries.

static constexpr size_t kLoopDeclickSamples = 96; // a hair longer than before (~2.2ms @ 44.1kHz)
static uint16_t s_loopFadeIn[4] = {0,0,0,0};

static inline void rampIn(int16_t* buf, size_t n) {
  if (n == 0) return;
  if (n == 1) { buf[0] = 0; return; }
  const int32_t denom = (int32_t)(n - 1);
  for (size_t i=0;i<n;i++) {
    int32_t num = (int32_t)i;
    buf[i] = (int16_t)((int32_t)buf[i] * num / denom);
  }
}

static inline void rampOutTail(int16_t* buf, size_t total, size_t n) {
  if (n == 0 || total == 0) return;
  if (n > total) n = total;
  if (n == 1) { buf[total-1] = 0; return; }
  const int32_t denom = (int32_t)(n - 1);
  for (size_t i=0;i<n;i++) {
    int32_t num = (int32_t)(denom - (int32_t)i);
    size_t idx = (total - n) + i;
    buf[idx] = (int16_t)((int32_t)buf[idx] * num / denom);
  }
}

static inline void declickBoundaryToZero(int16_t* buf, size_t wrapAt) {
  if (wrapAt == 0 || wrapAt >= kFrameSamples) return;
  size_t N = kLoopDeclickSamples;
  if (N > wrapAt) N = wrapAt;
  if (N > (kFrameSamples - wrapAt)) N = (kFrameSamples - wrapAt);
  if (N == 0) return;
  if (N == 1) { buf[wrapAt-1] = 0; buf[wrapAt] = 0; return; }

  const int32_t denom = (int32_t)(N - 1);
  for (size_t i=0;i<N;i++) {
    const int32_t head = (int32_t)i;
    const int32_t tail = denom - head;
    size_t ti = wrapAt - N + i;
    size_t hi = wrapAt + i;
    buf[ti] = (int16_t)((int32_t)buf[ti] * tail / denom);
    buf[hi] = (int16_t)((int32_t)buf[hi] * head / denom);
  }
}

void fillChannelFrame(int idx, int16_t* dst) {
  Channel& C = ch[idx];

  if (C.state == IDLE) {
    s_loopFadeIn[idx & 3] = 0;
    memset(dst, 0, kFrameSamples * 2);
    return;
  }

  // Tone-backed channel
  if (C.isTone && C.toneMode != TONE_NONE) {
    for (size_t n = 0; n < kFrameSamples; n++) {
      float s = synthNextSample(C);
      int32_t v = (int32_t)(s * 32767.0f);
      if (v >  32767) v =  32767;
      if (v < -32768) v = -32768;
      dst[n] = (int16_t)v;
    }
    return;
  }

  // RAM mode
  if (C.useRAM && C.ram.data) {
    size_t outPos = 0;
    size_t wrapAt = (size_t)-1;

    while (outPos < kFrameSamples) {
      size_t remain = (C.ram.samples > C.idx) ? (C.ram.samples - C.idx) : 0;
      if (remain == 0) {
        if (C.state == LOOPING && C.ram.samples > 0) {
          if (wrapAt == (size_t)-1) wrapAt = outPos;
          C.idx = 0;
          remain = C.ram.samples;
        } else {
          C.state = IDLE;
          C.idx = 0;
          memset(dst + outPos, 0, (kFrameSamples - outPos) * 2);
          return;
        }
      }
      size_t run = min(remain, (size_t)(kFrameSamples - outPos));
      memcpy(dst + outPos, C.ram.data + C.idx, run * 2);
      C.idx += run;
      outPos += run;

      if (outPos == kFrameSamples && C.idx >= C.ram.samples) {
        if (C.state == LOOPING && C.ram.samples > 0) {
          size_t N = min(kLoopDeclickSamples, kFrameSamples);
          rampOutTail(dst, kFrameSamples, N);
          s_loopFadeIn[idx & 3] = (uint16_t)N;
          C.idx = 0;
        } else if (C.state != LOOPING) {
          C.state = IDLE;
          C.idx = 0;
        }
      }
    }

    if (wrapAt != (size_t)-1) declickBoundaryToZero(dst, wrapAt);
    uint16_t fin = s_loopFadeIn[idx & 3];
    if (fin) {
      rampIn(dst, min((size_t)fin, kFrameSamples));
      s_loopFadeIn[idx & 3] = 0;
    }
    return;
  }

  // SD mode
  const uint32_t dataBytes = (C.sd.dataEnd > C.sd.dataStart) ? (C.sd.dataEnd - C.sd.dataStart) : 0;
  size_t filled = 0;
  size_t wrapAt = (size_t)-1;
  uint8_t safety = 0;

  while (filled < kBytesPerCh) {
    size_t got = sdReadReliable(C, ((uint8_t*)dst) + filled, kBytesPerCh - filled);

    if (got == 0) {
      if (C.state == LOOPING && dataBytes > 0 && safety++ < 4) {
        if (wrapAt == (size_t)-1) wrapAt = filled / 2;
        C.sd.cur = 0;
        if (C.sd.f) C.sd.f.seek(C.sd.dataStart);
        continue;
      }
      memset(((uint8_t*)dst) + filled, 0, kBytesPerCh - filled);
      C.state = IDLE;
      C.sd.cur = 0;
      break;
    }

    filled += got;

    if (C.state == LOOPING && dataBytes > 0 && C.sd.cur >= dataBytes) {
      if (filled < kBytesPerCh && wrapAt == (size_t)-1) wrapAt = filled / 2;

      if (filled == kBytesPerCh) {
        size_t N = min(kLoopDeclickSamples, kFrameSamples);
        rampOutTail(dst, kFrameSamples, N);
        s_loopFadeIn[idx & 3] = (uint16_t)N;
      }

      C.sd.cur = 0;
      if (C.sd.f) C.sd.f.seek(C.sd.dataStart);
    }
  }

  if (filled < kBytesPerCh) memset(((uint8_t*)dst) + filled, 0, kBytesPerCh - filled);
  if (wrapAt != (size_t)-1) declickBoundaryToZero(dst, wrapAt);

  uint16_t fin = s_loopFadeIn[idx & 3];
  if (fin) {
    rampIn(dst, min((size_t)fin, kFrameSamples));
    s_loopFadeIn[idx & 3] = 0;
  }
}

// ───────────────── I2S init ─────────────────

void i2s_init_common(i2s_port_t port, int dout, int bclk, int lrck) {
  i2s_config_t cfg = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX),
    .sample_rate = SAMPLE_RATE,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
    .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,
    .communication_format = I2S_COMM_FORMAT_I2S,
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
    .dma_buf_count = 8,
    .dma_buf_len   = 1024,
    .use_apll      = true,
    .tx_desc_auto_clear = true
  };
  ESP_ERROR_CHECK(i2s_driver_install(port, &cfg, 0, nullptr));
  i2s_pin_config_t pins = {
    .bck_io_num   = bclk,
    .ws_io_num    = lrck,
    .data_out_num = dout,
    .data_in_num  = I2S_PIN_NO_CHANGE
  };
  ESP_ERROR_CHECK(i2s_set_pin(port, &pins));
  i2s_set_clk(port, SAMPLE_RATE, I2S_BITS_PER_SAMPLE_16BIT, I2S_CHANNEL_STEREO);
  i2s_zero_dma_buffer(port);
}

// ───────────────── Volume helpers ─────────────────

int16_t q15_from_db(int8_t db) {
  float g = powf(10.0f, db / 20.0f);
  int32_t q = (int32_t)(g * 32767.0f + 0.5f);
  if (q < 0) q = 0; if (q > 32767) q = 32767;
  return (int16_t)q;
}
int16_t q15_mul(int16_t a, int16_t b) {
  int32_t t = (int32_t)a * (int32_t)b;
  t >>= 15;
  if (t >  32767) t =  32767;
  if (t < -32768) t = -32768;
  return (int16_t)t;
}
void applyGain(int16_t* buf, size_t n, int16_t g) {
  if (g == 32767) return; // unity
  for (size_t i=0; i<n; i++) buf[i] = q15_mul(buf[i], g);
}

int16_t masterGainQ15 = q15_from_db(0);
