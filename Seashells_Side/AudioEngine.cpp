#include "AudioEngine.h"
#include <SPI.h>
#include <math.h>

// Local constants used by helpers (keep in sync with your .ino)
static constexpr size_t kFrameSamples = 1024;          // per-channel samples/frame
static constexpr size_t kBytesPerCh   = kFrameSamples * 2; // 16-bit mono
static constexpr float  kTwoPi        = 6.28318530718f;

// ───────────────── SD & WAV helpers ─────────────────

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
      ch[i].sd.f = f;
      ch[i].sd.dataStart = 44; ch[i].sd.dataEnd = f.size(); ch[i].sd.cur = 0;
      ch[i].sd.f.seek(44);
      Serial.printf("CH%d: REOPENED %s\n", i+1, ch[i].path.c_str());
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
  C.sd.dataStart = 44;
  C.sd.dataEnd   = C.sd.f.size();
  if (C.sd.dataEnd <= C.sd.dataStart) {
    Serial.printf("CH%d: BAD SIZE (%lu)\n", idx+1, (unsigned long)C.sd.dataEnd);
    C.sd.f.close(); return false;
  }
  C.sd.cur = 0;
  C.sd.f.seek(44);
  return true;
}

bool loadWavIntoRam(const char* path, const char* tag, int16_t** outBuf, size_t* outSamples) {
  File f = SD.open(path, FILE_READ);
  if (!f) { Serial.printf("%s: RAM load OPEN FAIL %s\n", tag, path); return false; }
  size_t sz = f.size();
  if (sz <= 44) { Serial.printf("%s: RAM load BAD SIZE\n", tag); f.close(); return false; }
  size_t dataBytes = sz - 44;
  size_t samples   = dataBytes / 2;
  int16_t* buf = (int16_t*)ps_malloc(dataBytes);
  if (!buf) { Serial.printf("%s: RAM alloc FAIL (%u bytes)\n", tag, (unsigned)dataBytes); f.close(); return false; }
  f.seek(44);
  size_t off=0; while (off < dataBytes) {
    size_t n = f.read(((uint8_t*)buf)+off, dataBytes - off);
    if (n == 0) { Serial.printf("%s: RAM read FAIL @%u\n", tag, (unsigned)off); free(buf); f.close(); return false; }
    off += n; yield();
  }
  f.close();
  *outBuf = buf; *outSamples = samples;
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
          if (f) { C.sd.f = f; C.sd.f.seek(C.sd.dataStart + C.sd.cur); continue; }
        }
      }
      if (!remounted) {
        remounted = true;
        if (remountAndReopenAll()) { if (C.sd.f) { C.sd.f.seek(C.sd.dataStart + C.sd.cur); continue; } }
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

void fillChannelFrame(int idx, int16_t* dst) {
  Channel& C = ch[idx];

  if (C.state == IDLE) {
    memset(dst, 0, kFrameSamples * 2);
    return;
  }

  // Tone-backed channel: synthesize samples
  if (C.isTone && C.toneMode != TONE_NONE) {
    for (size_t n = 0; n < kFrameSamples; n++) {
      float s = synthNextSample(C);
      int32_t v = (int32_t)(s * 32767.0f);
      if (v >  32767) v =  32767;
      if (v < -32768) v = -32768;
      dst[n] = (int16_t)v;
    }
    // LOOPING tones just run until stopped externally.
    // PLAYING tones also run until state is changed; no auto-stop yet.
    return;
  }

  // RAM mode
  if (C.useRAM && C.ram.data) {
    size_t remain = (C.ram.samples > C.idx) ? (C.ram.samples - C.idx) : 0;
    size_t run = min(remain, (size_t)kFrameSamples);
    if (run) memcpy(dst, C.ram.data + C.idx, run * 2);
    if (run < kFrameSamples) memset(dst + run, 0, (kFrameSamples - run) * 2);
    C.idx += run;
    if (C.idx >= C.ram.samples) {
      if (C.state == LOOPING) C.idx = 0;
      else { C.state = IDLE; C.idx = 0; }
    }
    return;
  }

  // SD mode
  size_t n = sdReadReliable(C, (uint8_t*)dst, kBytesPerCh);
  if (n < kBytesPerCh) memset(((uint8_t*)dst) + n, 0, kBytesPerCh - n);

  uint32_t dataBytes = C.sd.dataEnd - C.sd.dataStart;
  if (C.sd.cur >= dataBytes || n == 0) {
    if (C.state == LOOPING) {
      C.sd.cur = 0;
      if (C.sd.f) C.sd.f.seek(C.sd.dataStart);
    } else {
      C.state = IDLE;
      C.sd.cur = 0;
    }
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

// Default to 0 dB; set this from your .ino at startup:
//   masterGainQ15 = q15_from_db(MASTER_GAIN_DB);
int16_t masterGainQ15 = q15_from_db(0);
