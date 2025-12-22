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
#define MASTER_GAIN_DB -20

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

// ---- Blink controller (non-blocking) ----
struct BlinkCtrl {
  bool     active   = false;
  bool     phaseOn  = false;
  uint8_t  color    = 0;     // 0=red, 1=green, 2=white
  uint8_t  remaining= 0;     // on->off transitions left
  uint16_t on_ms    = 0;
  uint16_t off_ms   = 0;
  uint32_t nextMs   = 0;
} blink;

static inline void ledsAllColor(uint8_t r,uint8_t g,uint8_t b){
  for (int i=0;i<4;i++) RGBT[i]->setPixelColor(0, RGBT[i]->Color(r,g,b));
  for (int i=0;i<4;i++) RGBT[i]->show();
}

static inline void ledsAllOff(){
  for (int i=0;i<4;i++) RGBT[i]->setPixelColor(0, 0);
  for (int i=0;i<4;i++) RGBT[i]->show();
}

static void blinkStart(uint8_t color, uint16_t on_ms, uint16_t off_ms, uint8_t reps=3){
  blink.active = true;
  blink.phaseOn = true;
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
    uint8_t r=(blink.color==0||blink.color==2)?255:0;
    uint8_t g=(blink.color==1||blink.color==2)?255:0;
    uint8_t b=(blink.color==2)?255:0;
    ledsAllColor(r,g,b);
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
      ch[i].gainQ15 = masterGainQ15;  // just master trim
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
      ch[i].gainQ15 = masterGainQ15;
      Serial.printf("[SCENE] slot %d: id=%u NOT FOUND\n", i, (unsigned)ids[i]);
      continue;
    }

    // Compute per-clip gain: master * per-clip (dB -> Q15)
    int32_t clipQ = q15_from_db(cm->volume_db);
    ch[i].gainQ15 = q15_mul(masterGainQ15, clipQ);

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

void side_playSlot(uint8_t slot) {
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
  blinkStop();
  ledsAllColor(255, 255, 255);
}

void side_blinkAll(uint8_t color, uint16_t on_ms, uint16_t off_ms) {
  blinkStart(color, on_ms, off_ms, /*reps*/3);
}

void side_setGameMode(bool en){ gameMode=en; }

void side_startLoopAll(){
  for (int i=0;i<4;i++){
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
void loop() {
  Ota_loopTick();
  GameBus_pump();  // process queued ESP-NOW commands in the main loop (avoids LED glitches)

  uint32_t now = millis();
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
    for (size_t n=0;n<FRAME_SAMPLES;++n) { *o++ = L[n]; *o++ = R[n]; }
  }
  {
    int16_t* o = (int16_t*)outLR1;
    int16_t* L = tmpMono[2];
    int16_t* R = tmpMono[3];
    for (size_t n=0;n<FRAME_SAMPLES;++n) { *o++ = L[n]; *o++ = R[n]; }
  }

  size_t w0=0, w1=0;
  i2s_write(I2S_NUM_0, outLR0, OUT_BYTES, &w0, portMAX_DELAY);
  i2s_write(I2S_NUM_1, outLR1, OUT_BYTES, &w1, portMAX_DELAY);

  blinkUpdate();
}
