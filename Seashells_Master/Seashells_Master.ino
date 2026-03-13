/*
// =============================================================
// Seashells Master (Odd One Out) — ESP-NOW Controller
//
// With standardized serial protocol for the main Player Management System (PMS)
//
// Standard protocol lines always start with:
//   !PMS
//
// PMS protocol (v1) ***DO NOT REMOVE***:
//
//   PMS -> Master:
//     !PMS PING
//     !PMS START level=1        (level 1..3; default 1)
//     !PMS STOP
//
//   Master -> PMS:
//     !PMS PONG v=1 game=seashells role=server
//     !PMS STATUS v=1 state=arming|playing level=1|2|3 score=.. lives=.. tleft_ms=.. last_reason=..
//       (STATUS prints every 250ms while active; NOT emitted while idle)
//     !PMS EVENT v=1 name=game_start level=1
//     !PMS EVENT v=1 name=game_end reason=timeup|no_lives|stopped score=.. lives=..
//     !PMS EVENT v=1 name=score delta=1 total=.. bonus=0
//     !PMS EVENT v=1 name=life delta=-1 lives=..
//
// Notes:
//   - One message per line, newline '\n' terminated.
//   - PMS should ONLY parse lines starting with "!PMS" (ignore everything else).
//   - No ACK/ERR by design (PMS infers success from STATUS/EVENT).
//
// Serial console (for manual tech/debug use):
//   - Commands are LINE-based and WORD-based to prevent accidental triggers.
//   - Type: HELP
//   - Common commands:
//       START [1|2|3]      (or START level=<1..3>)
//       STOP              (end game)
//       OTA [A|B|BOTH]     (push Side OTA URL)
//       CHAN <1-13>        (set ESP-NOW channel + reboot)
//       INFO              (print channel + discovered sides)
//       DIAG ...           (type DIAG for help)
//       AUD ...            (type AUD for help)
//       MIXFIX ...         (type MIXFIX for help)
//
// Build toggles (compile-time):
//   - PMS_STD_ENABLED: enable/disable PMS protocol support
//   - PMS_DEBUG_SERIAL: when 0, suppress non-!PMS debug/legacy Serial prints (clean PMS output)
//
// =============================================================
*/
#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <Preferences.h>
#include <cstring>
#include <stdarg.h>
#include <stdio.h>

#include "Messages.h"
#include "ConfigMaster.h"
#include "MasterManifest.h"

// =============================================================
// PMS / Debug toggles
// =============================================================

#ifndef PMS_STD_ENABLED
#define PMS_STD_ENABLED 1
#endif

// 1 = Keep legacy/debug Serial prints (boot banners, manifest dump, debug logs)
// 0 = Suppress all non-!PMS output (recommended for production PMS wiring)
#ifndef PMS_DEBUG_SERIAL
#define PMS_DEBUG_SERIAL 0
#endif

// PMS STATUS tick period (ms)
#ifndef PMS_STATUS_PERIOD_MS
#define PMS_STATUS_PERIOD_MS 250
#endif

#if PMS_DEBUG_SERIAL
  #define DBG_PRINT(...)    Serial.print(__VA_ARGS__)
  #define DBG_PRINTLN(...)  Serial.println(__VA_ARGS__)
  #define DBG_PRINTF(...)   Serial.printf(__VA_ARGS__)
#else
  #define DBG_PRINT(...)    do { } while (0)
  #define DBG_PRINTLN(...)  do { } while (0)
  #define DBG_PRINTF(...)   do { } while (0)
#endif


// ---------- Tuning ----------
static const uint32_t BASE_TIMEOUT_MS[3] = {
  45000,  // Round 1 base: 45s
  40000,  // Round 2 base: 40s
  35000   // Round 3 base: 35s
};
static const float   TIME_DECAY_FACTOR = 0.8f;   // each correct: timeout *= 0.8
static const uint32_t MIN_TIMEOUT_MS   = 5000;   // never go below 5 seconds
static const uint8_t  MAX_LIVES        = 5;

// ---------- Types ----------
enum State { IDLE, BUILD, ANNOUNCE, WAIT, PAUSE };

// ---------- Globals ----------
static State g_state = IDLE;

// Blink cadence
// Number of blink pulses shown between rounds/results.
// (Increase for easier visual tracking / slower pacing.)
static const uint8_t  BLINK_REPS              = 5;
static const uint16_t BLINK_ON_MS_CORRECT     = 140;
static const uint16_t BLINK_OFF_MS_CORRECT    = 120;
static const uint16_t BLINK_ON_MS_WRONG       = 160;
static const uint16_t BLINK_OFF_MS_WRONG      = 140;

// =============================================================
// Audio mitigation / diagnostics
// =============================================================
// IMPORTANT:
// - The Side firmware outputs exactly ONE stream per speaker channel.
//   If you hear "odd + common" from a single speaker, the most common causes are:
//     (a) amp outputting a mono mix of L+R, or
//     (b) acoustic bleed from the other 7 speakers.
//
// These options are SOFTWARE workarounds that can help either scenario.
// They intentionally change the relative loudness of speakers.
//
// 0 = off (stock behavior)
// 1 = mute/attenuate ONLY the odd slot's stereo-pair mate (same Side, same I2S pair)
// 2 = attenuate ALL non-odd slots (both Sides)
static constexpr uint8_t AUDIO_MITIGATION_MODE = 0;

// Mode 1: dB applied to the odd slot's pair-mate (common) on the same Side.
// Use something like -30 or -40 to effectively mute it.
static constexpr int8_t  AUDIO_PAIR_MUTE_DB   = -40;

// Mode 2: dB applied to all non-odd slots (common). Example: -6 or -12.
static constexpr int8_t  AUDIO_COMMON_ATTEN_DB = -12;

// =============================================================
// Dedicated audio diagnostic mode (tone patterns)
// =============================================================
// This is a structured way to prove/disprove channel mixing, L/R swap, and
// crosstalk without relying on SD clips or per-clip volume differences.
//
// Usage (Serial monitor, line-based):
//   DIAG            -> prints help
//   DIAG OFF        -> exit diag
//   DIAG <n>        -> set pattern n on BOTH sides (1..15)
//   DIAG A <n>      -> set pattern n on Side A only (Side B OFF)
//   DIAG B <n>      -> set pattern n on Side B only (Side A OFF)
//   DIAG BOTH <n>   -> same as DIAG <n>
//   DIAG NEXT       -> next pattern
//   DIAG PREV       -> previous pattern
//   DIAG AUTO       -> toggle auto-cycle (every ~2.5s)
//
// While in DIAG mode, ANY button press will also advance to NEXT.
static bool     g_diagAudioEnabled   = false;
static uint8_t  g_diagPattern        = 1;     // 1..15
static uint8_t  g_diagTargetMask     = 0x03;  // bit0=A, bit1=B
static bool     g_diagAutoCycle      = false;
static uint32_t g_diagNextStepMs     = 0;
static uint32_t g_diagLastResendMs   = 0;
static constexpr uint32_t DIAG_RESEND_PERIOD_MS = 1200;
static constexpr uint32_t DIAG_AUTOCYCLE_MS     = 2500;
static volatile bool g_diagAdvanceRequested = false;

// =============================================================
// SOUND AUDITION / QA MODE (browse clips + simulate rounds)
// =============================================================
// Tech utility to:
//  - step through clips in MasterManifest (to judge distinctness)
//  - simulate odd-one-out rounds (to judge pairings in context)
//  - record per-clip trim suggestions (for SD manifest volume_db tuning)
//
// Usage (Serial, line-based):
//   AUD              -> help
//   AUD ON|OFF       -> enter/exit audition mode
//   AUD MODE CLIP|ROUND|SIM1|SIM2|SIM3
//   AUD NEXT|PREV    -> step through clips / scenes (depends on mode)
//   AUD SOLOPOS <A0..A3|B0..B3|0..7>   (CLIP mode output speaker)
//   AUD ODDPOS  <A0..A3|B0..B3|0..7>   (ROUND mode odd speaker position)
//   AUD COMMON <id|CUR|NEXT|PREV>      (ROUND mode common clip)
//   AUD ODD    <id|CUR|NEXT|PREV>      (ROUND mode odd clip)
//   AUD TRIMSET <db>                  (CLIP mode: set suggested delta dB for current clip)
//   AUD TRIMSET ODD|COMMON <db>        (ROUND mode: set suggested delta dB for odd/common clip)
//   AUD EXPORT                         (print suggested deltas to copy into manifest.csv)

enum AudMode : uint8_t { AUD_OFF=0, AUD_CLIP=1, AUD_ROUND=2, AUD_SIM=3 };
static bool     g_audEnabled       = false;
static AudMode  g_audMode          = AUD_CLIP;
static uint8_t  g_audSimLevel      = 1;      // 1..3 used when mode==AUD_SIM
static bool     g_audAuto          = false;
static uint32_t g_audAutoPeriodMs  = 5000;
static uint32_t g_audNextAutoMs    = 0;
static volatile uint8_t g_audBtnPos = 255;   // 0..7 (set from BTN_EVENT), 255=none

// Cursor over MASTER_CLIPS (filtered + exclusions skipped).
static size_t   g_audCursor        = 0;
static uint16_t g_audClipId        = 0;      // current cursor clip (CLIP mode)
static uint16_t g_audCommonId      = 0;      // ROUND mode
static uint16_t g_audOddId         = 0;      // ROUND mode
static uint8_t  g_audSoloPos       = 0;      // 0..7 (A0..A3,B0..B3) for CLIP mode
static uint8_t  g_audOddPos        = 0;      // 0..7 for ROUND mode

// Optional category filters for browsing (empty = no filter).
static String g_audFilterBase = "";
static String g_audFilterSub  = "";
static String g_audFilterSub2 = "";

// Per-clip suggested trim delta (dB). This does NOT modify SD; it only helps you decide what to edit in manifest.csv later.
struct ClipAdj { uint16_t id; int8_t db; };
static ClipAdj g_audAdj[128];
static size_t  g_audAdjCount = 0;


// AUD scene-push reliability:
// AUD mode changes clips on demand, so we do a small follow-up retry burst
// after each apply. This fixes the common ESP-NOW failure mode where one Side
// misses SET_SCENE or START_LOOP_ALL and keeps the old clip (or stays silent).
static constexpr uint8_t  AUD_PUSH_RETRY_COUNT          = 3;
static constexpr uint32_t AUD_PUSH_RETRY_FIRST_DELAY_MS = 70;
static constexpr uint32_t AUD_PUSH_RETRY_GAP_MS         = 90;
static uint8_t  g_audRetryLedA[4]  = {3,3,3,3};
static uint8_t  g_audRetryLedB[4]  = {3,3,3,3};
static int8_t   g_audRetryTrimA[4] = {0,0,0,0};
static int8_t   g_audRetryTrimB[4] = {0,0,0,0};
static uint8_t  g_audPushRetries   = 0;
static uint32_t g_audPushRetryAtMs = 0;


// Pause bookkeeping
static uint32_t resultPauseUntil = 0;
static State    nextAfterBlink   = IDLE;

static volatile uint8_t lastSide = 255, lastSlot = 255;

// LED reliability: retry LED_ALL_WHITE a couple times at the start of each WAIT phase.
// If a Side misses the initial LED update (packet loss, RF noise, etc.),
// retry a few times during the first ~1-2 seconds of WAIT.
//
// We use a "soft" white refresh (no forced OFF frame first) to avoid visible flicker.
static constexpr uint8_t  LED_WHITE_RETRY_COUNT          = 6;
static constexpr uint32_t LED_WHITE_RETRY_FIRST_DELAY_MS = 120;
static constexpr uint32_t LED_WHITE_RETRY_GAP_MS         = 250;
static uint8_t  g_ledWhiteRetries = 0;
static uint32_t g_ledWhiteRetryAtMs = 0;

// Round-start audio reliability:
// With PMS_DEBUG_SERIAL=0 the Master can emit BUILD/ANNOUNCE ESP-NOW packets
// in a very tight burst. On some installs that makes one Side miss SET_SCENE
// or START_LOOP_ALL. AUD mode already compensates with timed retries; do the
// same for normal gameplay so debug prints are no longer acting as accidental
// pacing.
static constexpr uint8_t  ROUND_PUSH_RETRY_COUNT          = 3;
static constexpr uint32_t ROUND_PUSH_RETRY_FIRST_DELAY_MS = 70;
static constexpr uint32_t ROUND_PUSH_RETRY_GAP_MS         = 90;
static constexpr uint16_t ROUND_PUSH_PACE_MS              = 8;
static uint8_t  g_roundPushRetries = 0;
static uint32_t g_roundPushRetryAtMs = 0;

// Current scene + odd markers
static uint16_t sceneA[4] {0,0,0,0};
static uint16_t sceneB[4] {0,0,0,0};
static bool     slotIsOdd_A[4] {false,false,false,false};
static bool     slotIsOdd_B[4] {false,false,false,false};

// =============================================================
// Game session bookkeeping (moved out of loop so PMS status can read it)
// =============================================================

// Round/level state:
//   roundIdx 0 => Level 1
//   roundIdx 1 => Level 2
//   roundIdx 2 => Level 3 (infinite)
static uint8_t  g_roundIdx       = 0;
static uint8_t  g_pointsInRound  = 0;      // resets every 3 correct picks
static uint16_t g_scoreTotal     = 0;      // total correct selections this session (PMS score)
static uint8_t  g_lives          = MAX_LIVES;

static uint32_t g_waitStartMs    = 0;      // when WAIT began (for timeout countdown)
static uint32_t g_curTimeoutMs   = BASE_TIMEOUT_MS[0];

// =============================================================
// PMS status/event bookkeeping
// =============================================================

enum PmsState : uint8_t { PMS_IDLE = 0, PMS_ARMING = 1, PMS_PLAYING = 2 };

static uint32_t g_pmsLastTickMs   = 0;
static PmsState g_pmsLastState    = PMS_IDLE;
static const char* g_pmsLastReason = "none";  // none|score|life|state|stale (stale unused here)

static bool g_pmsGameStarted      = false;
static bool g_pmsGameEndEmitted   = false;

// Serial line buffering (prevents accidental triggers from PMS traffic)
static String g_serialLine;


// Forward declarations for commands defined later in this file
static void cmdLedAllWhite();
static void cmdStopAll();
static void cmdOtaUpdate(const uint8_t mac[6], const char* url);

// Forward declarations for round-start reliability helpers
static inline void gameClearRoundPushRetries();
static void gamePushSceneToSidesNow(bool includeWhite);
static inline void gameScheduleRoundPushRetries();

// =============================================================
// ESP-NOW network state (declared early so serial handlers compile)
// =============================================================

// Broadcast MAC
static const uint8_t BCAST_MAC[6] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};

// NVS (Preferences) — shared namespace with Side (and Role.h)
static constexpr const char* kPrefsNs = "seashells";
static constexpr const char* kKeyChan = "chan"; // u8

// Current ESP-NOW channel and discovered side peers
static uint8_t  g_nowChannel = NOW_DEFAULT_CHANNEL;
static uint8_t  g_sideMac[2][6] = {{0},{0}};
static bool     g_sideKnown[2]  = {false,false};
static uint32_t g_sideLastHelloMs[2] = {0,0};
static uint32_t g_nextHelloReqMs = 0;
static uint32_t g_lastHelloReqSentMs = 0; // timestamp of last broadcast HELLO_REQ

// Forward declarations (implemented later)
static void sendFramed(const uint8_t mac[6], uint8_t type, const uint8_t* payload, int plen);
static inline void prefsSaveChannel(uint8_t ch);


// =============================================================
// PMS helpers
// =============================================================

static uint8_t seashellsLevelFromRoundIdx(uint8_t roundIdx) {
  return (roundIdx < 2) ? (uint8_t)(roundIdx + 1) : 3;
}

static const char* pmsStateStr(PmsState st) {
  switch (st) {
    case PMS_ARMING:  return "arming";
    case PMS_PLAYING: return "playing";
    default:          return "unknown";
  }
}

static void pmsPrintPong();

// Write a full PMS line in ONE serial write to avoid interleaving/corruption
// if other tasks (ESP-NOW recv callback, etc.) are also printing.
#if PMS_STD_ENABLED
static void pmsWriteLine(const char* fmt, ...) {
  char buf[220];
  va_list ap;
  va_start(ap, fmt);
  // Leave 2 bytes for '\n' + '\0'
  int n = vsnprintf(buf, sizeof(buf) - 2, fmt, ap);
  va_end(ap);
  if (n < 0) return;
  if (n > (int)sizeof(buf) - 2) n = (int)sizeof(buf) - 2;
  buf[n++] = '\n';
  buf[n] = '\0';
  Serial.write((const uint8_t*)buf, (size_t)n);
}
#endif

static void pmsPrintPong() {
#if PMS_STD_ENABLED
  pmsWriteLine("!PMS PONG v=1 game=seashells role=server");
#endif
}

static void pmsPrintEventGameStart(uint8_t level) {
#if PMS_STD_ENABLED
  pmsWriteLine("!PMS EVENT v=1 name=game_start level=%u", (unsigned)level);
#endif
}

static void pmsPrintEventGameEnd(const char* reason, uint16_t score, uint8_t lives) {
#if PMS_STD_ENABLED
  pmsWriteLine("!PMS EVENT v=1 name=game_end reason=%s score=%u lives=%u",
               reason, (unsigned)score, (unsigned)lives);
#endif
}

static void pmsPrintEventScore(int32_t delta, uint16_t total) {
#if PMS_STD_ENABLED
  pmsWriteLine("!PMS EVENT v=1 name=score delta=%ld total=%u bonus=0",
               (long)delta, (unsigned)total);
#endif
}

static void pmsPrintEventLife(int32_t delta, uint8_t lives) {
#if PMS_STD_ENABLED
  pmsWriteLine("!PMS EVENT v=1 name=life delta=%ld lives=%u",
               (long)delta, (unsigned)lives);
#endif
}

static void pmsPrintStatus(PmsState st, uint8_t level, uint16_t score, uint8_t lives, uint32_t tleftMs, const char* lastReason) {
#if PMS_STD_ENABLED
  pmsWriteLine("!PMS STATUS v=1 state=%s level=%u score=%u lives=%u tleft_ms=%lu last_reason=%s",
               pmsStateStr(st),
               (unsigned)level,
               (unsigned)score,
               (unsigned)lives,
               (unsigned long)tleftMs,
               lastReason);
#endif
}

// Emit game_end at most once per session
static void pmsMaybeEmitGameEnd(const char* reason) {
#if PMS_STD_ENABLED
  if (g_pmsGameStarted && !g_pmsGameEndEmitted) {
    pmsPrintEventGameEnd(reason, g_scoreTotal, g_lives);
    g_pmsGameEndEmitted = true;
  }
#else
  (void)reason;
#endif
}

// =============================================================
// Game start/stop helpers (used by both PMS and legacy serial)
// =============================================================

static void startGameAtLevel(uint8_t level) {
  if (level < 1) level = 1;
  if (level > 3) level = 3;

  // Reset session state
  g_lives         = MAX_LIVES;
  g_scoreTotal    = 0;
  g_pointsInRound = 0;
  g_roundIdx      = (uint8_t)(level - 1);
  g_curTimeoutMs  = BASE_TIMEOUT_MS[g_roundIdx];
  g_waitStartMs   = 0;

  // Clear pick latch
  lastSide = lastSlot = 255;

  // Cancel any pending reliability retries from a previous run
  g_ledWhiteRetries = 0;
  gameClearRoundPushRetries();

  // Enter active state machine
  cmdLedAllWhite();
  g_state = BUILD;

  DBG_PRINTF("Game start (level=%u, lives=%u, timeout=%lums)\n",
             (unsigned)level, (unsigned)g_lives, (unsigned long)g_curTimeoutMs);

  // PMS bookkeeping
  g_pmsGameStarted    = true;
  g_pmsGameEndEmitted = false;
  g_pmsLastReason     = "state";
  pmsPrintEventGameStart(level);
}

static void stopGameFromHost(const char* reason) {
  if (g_state == IDLE) return;

  cmdStopAll();
  g_ledWhiteRetries = 0;
  gameClearRoundPushRetries();
  g_state = IDLE;
  DBG_PRINTLN("[Master] Game ended -> IDLE");

  g_pmsLastReason = "state";
  pmsMaybeEmitGameEnd(reason);
}

// =============================================================
// Serial parsing (PMS + legacy)
// =============================================================

static int32_t parseKeyInt(const String& line, const char* key, int32_t defaultVal) {
  String pattern = String(key) + "=";
  int idx = line.indexOf(pattern);
  if (idx < 0) return defaultVal;

  idx += pattern.length();
  int end = idx;
  while (end < (int)line.length()) {
    char c = line.charAt(end);
    if (c == ' ' || c == '\t' || c == '\r' || c == '\n') break;
    end++;
  }
  String val = line.substring(idx, end);
  val.trim();
  if (val.length() == 0) return defaultVal;
  return val.toInt();
}

static String firstToken(const String& s) {
  int sp = s.indexOf(' ');
  if (sp < 0) return s;
  return s.substring(0, sp);
}

static String afterFirstToken(const String& s) {
  int sp = s.indexOf(' ');
  if (sp < 0) return "";
  return s.substring(sp + 1);
}

static void handlePmsLine(const String& rawLine) {
#if PMS_STD_ENABLED
  String line = rawLine;
  line.trim();
  if (!line.startsWith("!PMS")) return;

  String rest = line.substring(4);
  rest.trim();
  if (rest.length() == 0) return;

  String cmd = firstToken(rest);
  String args = afterFirstToken(rest);
  cmd.toUpperCase();

  if (cmd == "PING") {
    pmsPrintPong();
    return;
  }

  if (cmd == "START") {
    int32_t level = parseKeyInt(args, "level", 1);
    startGameAtLevel((uint8_t)level);
    return;
  }

  if (cmd == "STOP") {
    stopGameFromHost("stopped");
    return;
  }

  DBG_PRINT("Unknown PMS cmd: ");
  DBG_PRINTLN(rawLine);
#else
  (void)rawLine;
#endif
}

// =============================================================
// AUDITION helpers
// =============================================================

static inline int8_t clampInt8(int v, int lo=-40, int hi=20) {
  if (v < lo) v = lo;
  if (v > hi) v = hi;
  return (int8_t)v;
}

static bool audFilterMatch(const MasterClipMeta& m) {
  if (g_audFilterBase.length() && strcasecmp(m.base, g_audFilterBase.c_str()) != 0) return false;
  if (g_audFilterSub.length()  && strcasecmp(m.sub,  g_audFilterSub.c_str())  != 0) return false;
  if (g_audFilterSub2.length() && strcasecmp(m.sub2, g_audFilterSub2.c_str()) != 0) return false;
  return true;
}

static bool audIdxAllowed(size_t idx) {
  if (idx >= MASTER_CLIP_COUNT) return false;
  const MasterClipMeta& m = MASTER_CLIPS[idx];
  if (isExcludedClip(m)) return false;
  if (!audFilterMatch(m)) return false;
  return true;
}

static size_t audSeekIdx(size_t startIdx, int dir) {
  if (MASTER_CLIP_COUNT == 0) return 0;
  size_t idx = startIdx;
  for (size_t guard = 0; guard < MASTER_CLIP_COUNT; guard++) {
    if (dir >= 0) idx = (idx + 1) % MASTER_CLIP_COUNT;
    else          idx = (idx + MASTER_CLIP_COUNT - 1) % MASTER_CLIP_COUNT;
    if (audIdxAllowed(idx)) return idx;
  }
  return startIdx;
}

static void audEnsureCursorValid() {
  if (MASTER_CLIP_COUNT == 0) {
    g_audCursor = 0;
    g_audClipId = 0;
    return;
  }
  if (!audIdxAllowed(g_audCursor)) {
    // Find the first allowed index.
    g_audCursor = 0;
    if (!audIdxAllowed(g_audCursor)) {
      g_audCursor = audSeekIdx(0, +1);
    }
  }
  g_audClipId = MASTER_CLIPS[g_audCursor].id;
}

static int8_t audGetAdj(uint16_t id) {
  for (size_t i=0; i<g_audAdjCount; i++) {
    if (g_audAdj[i].id == id) return g_audAdj[i].db;
  }
  return 0;
}

static void audSetAdj(uint16_t id, int8_t db) {
  db = clampInt8((int)db);
  for (size_t i=0; i<g_audAdjCount; i++) {
    if (g_audAdj[i].id == id) {
      g_audAdj[i].db = db;
      return;
    }
  }
  if (g_audAdjCount < (sizeof(g_audAdj)/sizeof(g_audAdj[0]))) {
    g_audAdj[g_audAdjCount++] = { id, db };
  } else {
    // simple fallback: overwrite the oldest entry
    g_audAdj[0] = { id, db };
  }
}

static void audClearAdj() {
  g_audAdjCount = 0;
}

static void audPrintClipMeta(const char* label, uint16_t id) {
  const MasterClipMeta* cm = MasterManifest_find(id);
  if (!cm) {
    DBG_PRINTF("[AUD] %s id=%u (meta not found)\n", label ? label : "clip", (unsigned)id);
    return;
  }
  DBG_PRINTF("[AUD] %s id=%u  %s/%s/%s\n", label ? label : "clip", (unsigned)id, cm->base, cm->sub, cm->sub2);
}

static bool audParsePosToken(const String& tokIn, uint8_t& outPos) {
  String tok = tokIn;
  tok.trim();
  if (tok.length() == 0) return false;
  String u = tok; u.toUpperCase();
  if (u.length() == 2) {
    char s = u.charAt(0);
    char d = u.charAt(1);
    if ((s == 'A' || s == 'B') && d >= '0' && d <= '3') {
      outPos = (uint8_t)((s == 'B' ? 4 : 0) + (d - '0'));
      return true;
    }
  }
  // numeric 0..7
  bool allDigits = true;
  for (int i=0;i<(int)u.length();i++){
    if (u.charAt(i) < '0' || u.charAt(i) > '9') { allDigits = false; break; }
  }
  if (!allDigits) return false;
  int p = u.toInt();
  if (p < 0 || p > 7) return false;
  outPos = (uint8_t)p;
  return true;
}

static uint16_t audSceneIdAtPos(uint8_t pos) {
  if (pos < 4) return sceneA[pos & 3];
  return sceneB[pos & 3];
}

static uint16_t audSceneOddId() {
  for (int i=0;i<4;i++){ if (slotIsOdd_A[i] && sceneA[i]) return sceneA[i]; }
  for (int i=0;i<4;i++){ if (slotIsOdd_B[i] && sceneB[i]) return sceneB[i]; }
  return 0;
}


static void audBuildScene_clip() {
  // Clear scenes
  for (int i=0;i<4;i++){ sceneA[i]=0; sceneB[i]=0; slotIsOdd_A[i]=false; slotIsOdd_B[i]=false; }

  uint8_t side = (g_audSoloPos >= 4) ? 1 : 0;
  uint8_t slot = g_audSoloPos & 3;
  if (side == 0) sceneA[slot] = g_audClipId;
  else           sceneB[slot] = g_audClipId;
}

static void audBuildScene_round() {
  // Fill everything with common, then place odd at ODDPOS
  for (int i=0;i<4;i++){ sceneA[i]=g_audCommonId; sceneB[i]=g_audCommonId; slotIsOdd_A[i]=false; slotIsOdd_B[i]=false; }
  uint8_t side = (g_audOddPos >= 4) ? 1 : 0;
  uint8_t slot = g_audOddPos & 3;
  if (side == 0) {
    sceneA[slot] = g_audOddId;
    slotIsOdd_A[slot] = true;
  } else {
    sceneB[slot] = g_audOddId;
    slotIsOdd_B[slot] = true;
  }
}


static void audComputeSolidLeds(uint8_t ledA[4], uint8_t ledB[4]) {
  // codes: 0=red, 1=green, 2=white, 3=off
  for (int i=0;i<4;i++){ ledA[i]=3; ledB[i]=3; }

  if (g_audMode == AUD_CLIP) {
    uint8_t side = (g_audSoloPos >= 4) ? 1 : 0;
    uint8_t slot = g_audSoloPos & 3;
    if (side==0) ledA[slot] = 2; else ledB[slot] = 2; // active = white
    return;
  }

  // ROUND or SIM: show odd=green, common=white, silence=off
  for (int i=0;i<4;i++){
    if (sceneA[i] == 0) ledA[i] = 3;
    else ledA[i] = slotIsOdd_A[i] ? 1 : 2;
    if (sceneB[i] == 0) ledB[i] = 3;
    else ledB[i] = slotIsOdd_B[i] ? 1 : 2;
  }
}


static void audSendSceneToSidesNow(const uint8_t ledA[4], const uint8_t ledB[4],
                                   const int8_t trimA[4], const int8_t trimB[4],
                                   bool includeStop, bool includeStart) {
  // Ensure sides don't locally play on press (we want BTN_EVENTs for control).
  cmdGameMode(true);

  if (includeStop) cmdStopAll();

  // Push scene + trims. Re-sending these is safe and is what recovers a Side
  // that missed one packet and would otherwise keep an old clip or go silent.
  cmdSetSceneSide(0, sceneA);
  cmdSetSceneSide(1, sceneB);
  cmdSetSlotTrimSide(0, trimA);
  cmdSetSlotTrimSide(1, trimB);

  // Start once initially, then also on follow-up retries. The Side now treats
  // START_LOOP_ALL as idempotent, so retries won't rewind already-playing clips.
  if (includeStart) cmdStartLoopAll();

  // In AUD we want deterministic solid LEDs instead of whatever the normal
  // start path chooses.
  cmdLedSolidSlotsSide(0, ledA);
  cmdLedSolidSlotsSide(1, ledB);
}

static void audPushSceneToSides(const uint8_t ledA[4], const uint8_t ledB[4]) {
  // Apply per-clip trim suggestions as per-slot trims (audition only).
  int8_t tA[4] = {0,0,0,0};
  int8_t tB[4] = {0,0,0,0};
  for (int i=0;i<4;i++){
    if (sceneA[i]) tA[i] = audGetAdj(sceneA[i]);
    if (sceneB[i]) tB[i] = audGetAdj(sceneB[i]);
    g_audRetryLedA[i]  = ledA[i];
    g_audRetryLedB[i]  = ledB[i];
    g_audRetryTrimA[i] = tA[i];
    g_audRetryTrimB[i] = tB[i];
  }

  // First push: stop the old scene, send the new one, then start it.
  audSendSceneToSidesNow(ledA, ledB, tA, tB, /*includeStop*/true, /*includeStart*/true);

  // Follow-up retries use the *latest* scene and make AUD NEXT/PREV much more
  // reliable over ESP-NOW.
  g_audPushRetries   = AUD_PUSH_RETRY_COUNT;
  g_audPushRetryAtMs = millis() + AUD_PUSH_RETRY_FIRST_DELAY_MS;
}

// Normal game round-start push with small pacing gaps.
// This intentionally mirrors the AUD reliability approach, but keeps LEDs
// separate so the existing soft-white retry path still avoids flicker.
static inline void gameClearRoundPushRetries() {
  g_roundPushRetries = 0;
  g_roundPushRetryAtMs = 0;
}

static void gamePushSceneToSidesNow(bool includeWhite) {
  cmdSetSceneSide(0, sceneA);
  delay(ROUND_PUSH_PACE_MS);
  cmdSetSceneSide(1, sceneB);
  delay(ROUND_PUSH_PACE_MS);

  cmdApplyAudioMitigationForRound();
  delay(ROUND_PUSH_PACE_MS);

  cmdStartLoopAll();

  if (includeWhite) {
    delay(ROUND_PUSH_PACE_MS);
    cmdLedAllWhite();
  }
}

static inline void gameScheduleRoundPushRetries() {
  g_roundPushRetries = ROUND_PUSH_RETRY_COUNT;
  g_roundPushRetryAtMs = millis() + ROUND_PUSH_RETRY_FIRST_DELAY_MS;
}

static void audPrintState() {
  DBG_PRINTLN("\n=== AUDITION ===");
  DBG_PRINTF("mode=%s  cursor=%u/%u  soloPos=%u  oddPos=%u\n",
             (g_audMode==AUD_CLIP?"CLIP":(g_audMode==AUD_ROUND?"ROUND":"SIM")),
             (unsigned)g_audCursor, (unsigned)MASTER_CLIP_COUNT,
             (unsigned)g_audSoloPos, (unsigned)g_audOddPos);
  if (g_audFilterBase.length() || g_audFilterSub.length() || g_audFilterSub2.length()) {
    DBG_PRINTF("filter: base='%s' sub='%s' sub2='%s'\n",
               g_audFilterBase.c_str(), g_audFilterSub.c_str(), g_audFilterSub2.c_str());
  }

  if (g_audMode == AUD_CLIP) {
    audPrintClipMeta("CLIP", g_audClipId);
    DBG_PRINTF("suggested_delta_db=%d\n", (int)audGetAdj(g_audClipId));
  } else if (g_audMode == AUD_ROUND) {
    audPrintClipMeta("COMMON", g_audCommonId);
    DBG_PRINTF("common_suggested_delta_db=%d\n", (int)audGetAdj(g_audCommonId));
    audPrintClipMeta("ODD", g_audOddId);
    DBG_PRINTF("odd_suggested_delta_db=%d\n", (int)audGetAdj(g_audOddId));
  } else {
    DBG_PRINTF("SIM level=%u\n", (unsigned)g_audSimLevel);
  }

  // Print active scene IDs (8 slots).
  for (int i=0;i<4;i++) printIdInfo("  sceneA", sceneA[i]);
  for (int i=0;i<4;i++) printIdInfo("  sceneB", sceneB[i]);
}

static void audApply() {
  if (!g_audEnabled) return;
  audEnsureCursorValid();
  if (g_audClipId == 0) g_audClipId = pickAnyAllowedId();
  if (g_audCommonId == 0) g_audCommonId = g_audClipId;
  if (g_audOddId == 0) {
    size_t nx = audSeekIdx(g_audCursor, +1);
    g_audOddId = MASTER_CLIPS[nx].id;
    if (g_audOddId == g_audCommonId) {
      nx = audSeekIdx(nx, +1);
      g_audOddId = MASTER_CLIPS[nx].id;
    }
  }

  if (g_audMode == AUD_CLIP) {
    audBuildScene_clip();
  } else if (g_audMode == AUD_ROUND) {
    audBuildScene_round();
  } else {
    // AUD_SIM uses whatever is already in sceneA/sceneB (generated elsewhere).
  }

  uint8_t ledA[4], ledB[4];
  audComputeSolidLeds(ledA, ledB);
  audPushSceneToSides(ledA, ledB);
  audPrintState();
}

static void audGenerateSimScene() {
  if (g_audSimLevel <= 1) buildScenes_level1_sub2();
  else if (g_audSimLevel == 2) buildScenes_level2_randomBases();
  else buildScenes_level3_subs();
}

static void audEnable(AudMode mode, uint8_t simLevel=1) {
  // Stop any in-progress game so we're not fighting the state machine.
  stopGameFromHost("stopped");

  // Exit DIAG if active
  if (g_diagAudioEnabled) {
    diagDisable();
  }

  g_audEnabled = true;
  g_audMode = mode;
  g_audSimLevel = simLevel;
  if (g_audSimLevel < 1) g_audSimLevel = 1;
  if (g_audSimLevel > 3) g_audSimLevel = 3;

  g_audAuto = false;
  g_audBtnPos = 255;

  // Clear trims on entry so you start clean.
  int8_t z[4] = {0,0,0,0};
  cmdSetSlotTrimSide(0, z);
  cmdSetSlotTrimSide(1, z);

  audEnsureCursorValid();
  if (g_audClipId == 0) g_audClipId = pickAnyAllowedId();
  // Initialize common/odd defaults if needed
  if (g_audCommonId == 0) g_audCommonId = g_audClipId;
  if (g_audOddId == 0) {
    size_t nx = audSeekIdx(g_audCursor, +1);
    g_audOddId = MASTER_CLIPS[nx].id;
  }

  if (g_audMode == AUD_SIM) {
    audGenerateSimScene();
  }

  DBG_PRINTLN("[AUD] enabled");
  audApply();
}

static void audDisable() {
  if (!g_audEnabled) return;
  g_audEnabled = false;
  g_audAuto = false;
  g_audBtnPos = 255;
  g_audPushRetries = 0;

  cmdStopAll();
  // Clear trims so gameplay isn't affected later.
  int8_t z[4] = {0,0,0,0};
  cmdSetSlotTrimSide(0, z);
  cmdSetSlotTrimSide(1, z);
  // Turn LEDs off
  uint8_t off[4] = {3,3,3,3};
  cmdLedSolidSlotsSide(0, off);
  cmdLedSolidSlotsSide(1, off);

  DBG_PRINTLN("[AUD] disabled");
}

static void audStep(int dir) {
  if (!g_audEnabled) return;
  if (g_audMode == AUD_CLIP) {
    g_audCursor = audSeekIdx(g_audCursor, dir);
    g_audClipId = MASTER_CLIPS[g_audCursor].id;
    audApply();
    return;
  }
  if (g_audMode == AUD_ROUND) {
    // For round audition, NEXT/PREV steps the ODD clip (common stays as reference).
    g_audCursor = audSeekIdx(g_audCursor, dir);
    g_audClipId = MASTER_CLIPS[g_audCursor].id;
    g_audOddId  = g_audClipId;
    audApply();
    return;
  }
  // SIM: next scene of the same sim level
  audGenerateSimScene();
  audApply();
}

static void audTick() {
  if (!g_audEnabled) return;

  // Reliability retries for the most recent AUD scene push.
  const uint32_t nowMs = millis();
  if (g_audPushRetries && (int32_t)(nowMs - g_audPushRetryAtMs) >= 0) {
    audSendSceneToSidesNow(g_audRetryLedA, g_audRetryLedB,
                           g_audRetryTrimA, g_audRetryTrimB,
                           /*includeStop*/false, /*includeStart*/true);
    g_audPushRetries--;
    if (g_audPushRetries) {
      g_audPushRetryAtMs = nowMs + AUD_PUSH_RETRY_GAP_MS;
    }
  }

  // Button-driven control (from BTN_EVENT).
  if (g_audBtnPos != 255) {
    uint8_t pos = g_audBtnPos;
    g_audBtnPos = 255;
    if (g_audMode == AUD_CLIP) {
      g_audSoloPos = pos;
      audApply();
    } else if (g_audMode == AUD_ROUND) {
      g_audOddPos = pos;
      audApply();
    } else {
      // SIM: any press -> next scene
      audGenerateSimScene();
      audApply();
    }
  }

  // Optional auto-step.
  if (g_audAuto) {
    if ((int32_t)(nowMs - g_audNextAutoMs) >= 0) {
      g_audNextAutoMs = nowMs + g_audAutoPeriodMs;
      audStep(+1);
    }
  }
}

static void audPrintHelp() {
  DBG_PRINTLN("\n=== AUD (Sound Audition / QA Mode) ===");
  DBG_PRINTLN("This mode lets you browse clips and simulate rounds to judge:\n  (1) sound distinctness\n  (2) per-clip loudness (so you can tune manifest.csv volume_db)");
  DBG_PRINTLN("\nBasics:");
  DBG_PRINTLN("  AUD ON");
  DBG_PRINTLN("  AUD OFF");
  DBG_PRINTLN("  AUD SHOW");
  DBG_PRINTLN("  AUD NEXT    (step clip / step odd / step sim scene)");
  DBG_PRINTLN("  AUD PREV");
  DBG_PRINTLN("\nModes:");
  DBG_PRINTLN("  AUD MODE CLIP    -> play ONE clip on ONE speaker (SOLOPOS)");
  DBG_PRINTLN("  AUD MODE ROUND   -> play COMMON on 7 speakers, ODD on 1 (ODDPOS)");
  DBG_PRINTLN("  AUD MODE SIM1    -> generate real Level-1 scenes (base+sub2 family vs other base)");
  DBG_PRINTLN("  AUD MODE SIM2    -> generate real Level-2 scenes (base vs different base)");
  DBG_PRINTLN("  AUD MODE SIM3    -> generate real Level-3 scenes (sub vs different sub, same base)");
  DBG_PRINTLN("\nPosition selection:");
  DBG_PRINTLN("  AUD SOLOPOS A0..A3 | B0..B3 | 0..7");
  DBG_PRINTLN("  AUD ODDPOS  A0..A3 | B0..B3 | 0..7");
  DBG_PRINTLN("  AUD ROTATE   (ROUND: oddpos = next)");
  DBG_PRINTLN("\nChoosing clips (ROUND mode):");
  DBG_PRINTLN("  AUD COMMON <id|CUR|NEXT|PREV>");
  DBG_PRINTLN("  AUD ODD    <id|CUR|NEXT|PREV>");
  DBG_PRINTLN("\nPer-clip volume suggestions (does NOT edit SD):");
  DBG_PRINTLN("  AUD TRIMSET <db>            (CLIP mode: for current clip)");
  DBG_PRINTLN("  AUD TRIMSET ODD <db>        (ROUND mode)");
  DBG_PRINTLN("  AUD TRIMSET COMMON <db>     (ROUND mode)");
  DBG_PRINTLN("  AUD TRIMSET SLOT <pos> <db>   (any mode: apply to the clip currently assigned to that speaker position)");
  DBG_PRINTLN("  AUD EXPORT                 (print deltas you can apply to manifest.csv volume_db)\n");
  DBG_PRINTLN("Filters (for browsing):");
  DBG_PRINTLN("  AUD BASE <name>     (or BASE CLEAR)");
  DBG_PRINTLN("  AUD SUB  <name>     (or SUB CLEAR)");
  DBG_PRINTLN("  AUD SUB2 <name>     (or SUB2 CLEAR)");
  DBG_PRINTLN("  AUD CLEARFILTER");
  DBG_PRINTLN("\nButtons while AUD is ON:");
  DBG_PRINTLN("  - CLIP: press any button -> SOLOPOS = that speaker");
  DBG_PRINTLN("  - ROUND: press any button -> ODDPOS = that speaker");
  DBG_PRINTLN("  - SIM: press any button -> NEXT scene\n");
}

static void audExport() {
  DBG_PRINTLN("\n=== AUD EXPORT (suggested deltas) ===");
  DBG_PRINTLN("Format: id,delta_db,base,sub,sub2");
  for (size_t i=0;i<g_audAdjCount;i++) {
    uint16_t id = g_audAdj[i].id;
    int db = (int)g_audAdj[i].db;
    const MasterClipMeta* cm = MasterManifest_find(id);
    if (cm) {
      DBG_PRINTF("%u,%d,%s,%s,%s\n", (unsigned)id, db, cm->base, cm->sub, cm->sub2);
    } else {
      DBG_PRINTF("%u,%d,,,\n", (unsigned)id, db);
    }
  }
  DBG_PRINTLN("\nApply these deltas to the SD /manifest.csv column 'volume_db' (new = old + delta_db).\n");
}

static void handleLegacyLine(const String& rawLine) {
  String line = rawLine;
  line.trim();
  if (line.length() == 0) return;

  // ------------------------------------------------------------------------
  // New utility commands (line-based)
  // ------------------------------------------------------------------------
  {
    String up = line;
    up.toUpperCase();

    // CHAN <1-13>
    // Persist the ESP-NOW channel in NVS and tell the sides to switch too.
    // NOTE: Make sure both sides are powered when you run this.
    if (up.startsWith("CHAN")) {
      String rest = line.substring(4);
      rest.trim();
      int ch = rest.toInt();
      if (ch >= 1 && ch <= 13) {
        uint8_t payload[1] = { (uint8_t)ch };

        DBG_PRINTF("[Master] CHAN %d -> sending CHAN_SET + reboot\n", ch);

        // Unicast (reliable) to known sides
        for (uint8_t sid = 0; sid < 2; sid++) {
          if (g_sideKnown[sid]) sendFramed(g_sideMac[sid], CHAN_SET, payload, 1);
        }
        // Broadcast fallback (helps if a side is unpaired but listening)
        sendFramed(BCAST_MAC, CHAN_SET, payload, 1);

        prefsSaveChannel((uint8_t)ch);
        delay(80);
        ESP.restart();
      } else {
        DBG_PRINTLN("Usage: CHAN <1-13>");
      }
      return;
    }

    // INFO
    if (up == "INFO") {
      DBG_PRINTF("[Master] channel=%u sideA=%s sideB=%s\n",
                 (unsigned)g_nowChannel,
                 g_sideKnown[0] ? "KNOWN" : "UNKNOWN",
                 g_sideKnown[1] ? "KNOWN" : "UNKNOWN");
      if (g_sideKnown[0]) {
        DBG_PRINTF("  Side A MAC: %02X:%02X:%02X:%02X:%02X:%02X\n",
                   g_sideMac[0][0],g_sideMac[0][1],g_sideMac[0][2],
                   g_sideMac[0][3],g_sideMac[0][4],g_sideMac[0][5]);
      }
      if (g_sideKnown[1]) {
        DBG_PRINTF("  Side B MAC: %02X:%02X:%02X:%02X:%02X:%02X\n",
                   g_sideMac[1][0],g_sideMac[1][1],g_sideMac[1][2],
                   g_sideMac[1][3],g_sideMac[1][4],g_sideMac[1][5]);
      }
      return;
    }

    // DIAG ...  (audio diagnostic mode)
    if (up.startsWith("DIAG")) {
      String rest = line.substring(4);
      rest.trim();

      if (rest.length() == 0) {
        diagPrintHelp();
        return;
      }

      String a1 = rest;
      String a2 = "";
      int sp = rest.indexOf(' ');
      if (sp >= 0) {
        a1 = rest.substring(0, sp);
        a2 = rest.substring(sp + 1);
        a2.trim();
      }
      String a1u = a1; a1u.toUpperCase();
      String a2u = a2; a2u.toUpperCase();

      if (a1u == "HELP") {
        diagPrintHelp();
        return;
      }
      if (a1u == "OFF") {
        diagDisable();
        return;
      }
      if (a1u == "AUTO") {
        if (!g_diagAudioEnabled) {
          diagEnable(g_diagPattern, g_diagTargetMask, /*auto*/true);
        } else {
          g_diagAutoCycle = !g_diagAutoCycle;
          DBG_PRINTF("[DIAG] auto=%s\n", g_diagAutoCycle ? "ON" : "OFF");
          g_diagNextStepMs = millis() + DIAG_AUTOCYCLE_MS;
        }
        return;
      }
      if (a1u == "NEXT") {
        if (!g_diagAudioEnabled) diagEnable(g_diagPattern, g_diagTargetMask, /*auto*/false);
        diagStep(+1);
        return;
      }
      if (a1u == "PREV") {
        if (!g_diagAudioEnabled) diagEnable(g_diagPattern, g_diagTargetMask, /*auto*/false);
        diagStep(-1);
        return;
      }

      // DIAG A <n> / DIAG B <n> / DIAG BOTH <n>
      if (a1u == "A" || a1u == "B" || a1u == "BOTH") {
        uint8_t mask = (a1u == "A") ? 0x01 : (a1u == "B") ? 0x02 : 0x03;
        int pat = a2.toInt();
        if (pat < 1 || pat > 15) {
          DBG_PRINTLN("Usage: DIAG A <1-15>  |  DIAG B <1-15>  |  DIAG BOTH <1-15>");
          return;
        }
        diagEnable((uint8_t)pat, mask, /*auto*/false);
        return;
      }

      // DIAG <n>
      int pat = a1.toInt();
      if (pat >= 1 && pat <= 15) {
        diagEnable((uint8_t)pat, 0x03, /*auto*/false);
        return;
      }

      DBG_PRINTLN("DIAG: unknown syntax. Type DIAG for help.");
      return;
    }

    // AUD ... (sound audition / QA)
    // Type "AUD" for help.
    if (up.startsWith("AUD")) {
      String rest = line.substring(3);
      rest.trim();

      if (rest.length() == 0) {
        audPrintHelp();
        return;
      }

      String a1 = firstToken(rest);
      String a2 = afterFirstToken(rest);
      a2.trim();

      String a1u = a1; a1u.toUpperCase();
      String a2u = a2; a2u.toUpperCase();

      if (a1u == "HELP") {
        audPrintHelp();
        return;
      }
      if (a1u == "ON") {
        audEnable(AUD_CLIP, 1);
        return;
      }
      if (a1u == "OFF") {
        audDisable();
        return;
      }
      if (a1u == "SHOW") {
        if (!g_audEnabled) DBG_PRINTLN("[AUD] (currently OFF)\n");
        audPrintState();
        return;
      }
      if (a1u == "NEXT") {
        if (!g_audEnabled) audEnable(AUD_CLIP, 1);
        audStep(+1);
        return;
      }
      if (a1u == "PREV") {
        if (!g_audEnabled) audEnable(AUD_CLIP, 1);
        audStep(-1);
        return;
      }

      if (a1u == "MODE") {
        if (a2u.length() == 0) {
          DBG_PRINTLN("Usage: AUD MODE CLIP|ROUND|SIM1|SIM2|SIM3");
          return;
        }
        if (a2u == "CLIP") {
          if (!g_audEnabled) audEnable(AUD_CLIP, 1);
          else { g_audMode = AUD_CLIP; audApply(); }
          return;
        }
        if (a2u == "ROUND") {
          if (!g_audEnabled) audEnable(AUD_ROUND, 1);
          else { g_audMode = AUD_ROUND; audApply(); }
          return;
        }
        if (a2u == "SIM1" || a2u == "SIM2" || a2u == "SIM3") {
          uint8_t lvl = (a2u == "SIM2") ? 2 : (a2u == "SIM3") ? 3 : 1;
          if (!g_audEnabled) {
            audEnable(AUD_SIM, lvl);
          } else {
            g_audMode = AUD_SIM;
            g_audSimLevel = lvl;
            audGenerateSimScene();
            audApply();
          }
          return;
        }
        DBG_PRINTLN("AUD MODE: unknown mode. Use CLIP|ROUND|SIM1|SIM2|SIM3");
        return;
      }

      if (a1u == "SOLOPOS") {
        uint8_t pos;
        if (!audParsePosToken(a2, pos)) {
          DBG_PRINTLN("Usage: AUD SOLOPOS A0..A3 | B0..B3 | 0..7");
          return;
        }
        g_audSoloPos = pos;
        if (!g_audEnabled) audEnable(AUD_CLIP, 1);
        else { g_audMode = AUD_CLIP; audApply(); }
        return;
      }

      if (a1u == "ODDPOS") {
        uint8_t pos;
        if (!audParsePosToken(a2, pos)) {
          DBG_PRINTLN("Usage: AUD ODDPOS A0..A3 | B0..B3 | 0..7");
          return;
        }
        g_audOddPos = pos;
        if (!g_audEnabled) audEnable(AUD_ROUND, 1);
        else { g_audMode = AUD_ROUND; audApply(); }
        return;
      }

      if (a1u == "ROTATE") {
        g_audOddPos = (uint8_t)((g_audOddPos + 1) & 7);
        if (!g_audEnabled) audEnable(AUD_ROUND, 1);
        else { g_audMode = AUD_ROUND; audApply(); }
        return;
      }

      if (a1u == "COMMON" || a1u == "ODD") {
        if (!g_audEnabled) audEnable(AUD_ROUND, 1);
        g_audMode = AUD_ROUND;

        String arg = a2;
        arg.trim();
        String argu = arg; argu.toUpperCase();
        uint16_t id = 0;
        if (argu == "CUR") {
          id = g_audClipId;
        } else if (argu == "NEXT") {
          g_audCursor = audSeekIdx(g_audCursor, +1);
          g_audClipId = MASTER_CLIPS[g_audCursor].id;
          id = g_audClipId;
        } else if (argu == "PREV") {
          g_audCursor = audSeekIdx(g_audCursor, -1);
          g_audClipId = MASTER_CLIPS[g_audCursor].id;
          id = g_audClipId;
        } else {
          id = (uint16_t)arg.toInt();
        }
        if (id == 0) {
          DBG_PRINTLN("Usage: AUD COMMON <id|CUR|NEXT|PREV>  or  AUD ODD <id|CUR|NEXT|PREV>");
          return;
        }
        if (a1u == "COMMON") g_audCommonId = id; else g_audOddId = id;
        audApply();
        return;
      }

      if (a1u == "TRIMSET") {
        // Sets a suggested delta dB for a clip (does NOT edit SD).
        //
        //   AUD TRIMSET <db>                 -> CLIP: current clip
        //                                     ROUND: odd clip
        //                                     SIM: odd clip currently in the scene
        //   AUD TRIMSET ODD <db>             -> odd clip (ROUND/SIM)
        //   AUD TRIMSET COMMON <db>          -> common clip (ROUND only)
        //   AUD TRIMSET SLOT <pos> <db>      -> clip currently assigned to that speaker position
        //
        if (!g_audEnabled) audEnable(AUD_CLIP, 1);

        // Tokenize the remainder (a2)
        String t1 = firstToken(a2);
        String rest = afterFirstToken(a2);
        rest.trim();
        String t1u = t1; t1u.toUpperCase();

        uint16_t id = 0;
        int db = 0;

        if (t1.length() == 0) {
          DBG_PRINTLN("Usage: AUD TRIMSET <db>  |  AUD TRIMSET ODD <db>  |  AUD TRIMSET COMMON <db>  |  AUD TRIMSET SLOT <pos> <db>");
          return;
        }

        if (t1u == "SLOT") {
          String posTok = firstToken(rest);
          String dbTok  = afterFirstToken(rest);
          dbTok.trim();
          uint8_t pos;
          if (!audParsePosToken(posTok, pos) || dbTok.length() == 0) {
            DBG_PRINTLN("Usage: AUD TRIMSET SLOT A0..A3|B0..B3|0..7 <db>");
            return;
          }
          id = audSceneIdAtPos(pos);
          db = dbTok.toInt();
        } else if (t1u == "ODD" || t1u == "COMMON") {
          if (rest.length() == 0) {
            DBG_PRINTLN("Usage: AUD TRIMSET ODD <db>  |  AUD TRIMSET COMMON <db>");
            return;
          }
          db = rest.toInt();
          if (t1u == "ODD") {
            if (g_audMode == AUD_SIM) id = audSceneOddId();
            else id = g_audOddId;
          } else {
            id = g_audCommonId;
          }
        } else {
          // Assume t1 is a number (db).
          db = t1.toInt();
          if (g_audMode == AUD_CLIP) id = g_audClipId;
          else if (g_audMode == AUD_ROUND) id = g_audOddId;
          else id = audSceneOddId();
        }

        if (id == 0) {
          DBG_PRINTLN("[AUD] TRIMSET target clip id=0 (slot empty or odd not found). Try AUD SHOW to see scene IDs.");
          return;
        }

        audSetAdj(id, (int8_t)db);
        audPrintClipMeta("TRIMSET", id);
        DBG_PRINTF("[AUD] suggested_delta_db set to %d\n", (int)audGetAdj(id));
        audApply();
        return;
      }

      if (a1u == "EXPORT") {
        audExport();
        return;
      }
      if (a1u == "CLEARTRIM") {
        audClearAdj();
        DBG_PRINTLN("[AUD] cleared all suggested deltas");
        if (g_audEnabled) audApply();
        return;
      }

      if (a1u == "AUTO") {
        // AUD AUTO           -> toggle
        // AUD AUTO <ms>      -> set period + enable
        if (!g_audEnabled) audEnable(AUD_CLIP, 1);
        if (a2.length() == 0) {
          g_audAuto = !g_audAuto;
        } else {
          uint32_t ms = (uint32_t)a2.toInt();
          if (ms < 500) ms = 500;
          g_audAutoPeriodMs = ms;
          g_audAuto = true;
        }
        g_audNextAutoMs = millis() + g_audAutoPeriodMs;
        DBG_PRINTF("[AUD] auto=%s period=%lums\n", g_audAuto?"ON":"OFF", (unsigned long)g_audAutoPeriodMs);
        return;
      }

      if (a1u == "BASE" || a1u == "SUB" || a1u == "SUB2") {
        String val = a2;
        val.trim();
        String valu = val; valu.toUpperCase();
        if (valu == "CLEAR") val = "";

        if (a1u == "BASE") g_audFilterBase = val;
        else if (a1u == "SUB") g_audFilterSub = val;
        else g_audFilterSub2 = val;

        audEnsureCursorValid();
        DBG_PRINTF("[AUD] filter updated: base='%s' sub='%s' sub2='%s'\n",
                   g_audFilterBase.c_str(), g_audFilterSub.c_str(), g_audFilterSub2.c_str());
        if (g_audEnabled) audApply();
        return;
      }

      if (a1u == "CLEARFILTER") {
        g_audFilterBase = "";
        g_audFilterSub  = "";
        g_audFilterSub2 = "";
        audEnsureCursorValid();
        DBG_PRINTLN("[AUD] filters cleared");
        if (g_audEnabled) audApply();
        return;
      }

      DBG_PRINTLN("AUD: unknown syntax. Type AUD for help.");
      return;
    }

    // MIXFIX ... (runtime RIGHT-channel de-mix)
    // Usage:
    //   MIXFIX                -> help
    //   MIXFIX ON             -> enable mask=3 with k=2.0, m=1.0 on BOTH sides
    //   MIXFIX OFF            -> disable on BOTH sides
    //   MIXFIX A ON|OFF       -> enable/disable on Side A
    //   MIXFIX B ON|OFF       -> enable/disable on Side B
    //   MIXFIX BOTH ON|OFF    -> same as MIXFIX ON|OFF
    //   MIXFIX [A|B|BOTH] <mask 0-3> <k_milli> <m_milli>
    //     Example: MIXFIX A 1 2000 1000   (I2S0-right only, k=2.0, m=1.0)
    //              MIXFIX BOTH 3 2000 1000
    if (up.startsWith("MIXFIX")) {
      String rest = line.substring(6);
      rest.trim();

      auto printHelp = [](){
        DBG_PRINTLN("\n=== MIXFIX (runtime RIGHT-channel de-mix) ===");
        DBG_PRINTLN("If a RIGHT speaker amp is outputting an L+R mix, MixFix can cancel LEFT leakage by pre-distorting the RIGHT samples.");
        DBG_PRINTLN("\nCommands:");
        DBG_PRINTLN("  MIXFIX OFF");
        DBG_PRINTLN("  MIXFIX ON");
        DBG_PRINTLN("  MIXFIX A ON|OFF");
        DBG_PRINTLN("  MIXFIX B ON|OFF");
        DBG_PRINTLN("  MIXFIX BOTH ON|OFF");
        DBG_PRINTLN("  MIXFIX [A|B|BOTH] <mask 0-3> <k_milli> <m_milli>");
        DBG_PRINTLN("\nmask bits:");
        DBG_PRINTLN("  bit0 = I2S0 RIGHT (slot1 / Speaker2)");
        DBG_PRINTLN("  bit1 = I2S1 RIGHT (slot3 / Speaker4)");
        DBG_PRINTLN("\nTypical values:");
        DBG_PRINTLN("  k_milli=2000, m_milli=1000  (best for MIX ~= (L+R)/2)");
        DBG_PRINTLN("  k_milli=1000, m_milli=1000  (best for MIX ~= (L+R))\n");
      };

      if (rest.length() == 0) {
        printHelp();
        return;
      }

      // Tokenize (simple split on spaces)
      String t1 = rest;
      String t2 = "";
      String t3 = "";
      String t4 = "";

      int p1 = t1.indexOf(' ');
      if (p1 >= 0) {
        t2 = t1.substring(p1 + 1); t2.trim();
        t1 = t1.substring(0, p1);  t1.trim();
        int p2 = t2.indexOf(' ');
        if (p2 >= 0) {
          t3 = t2.substring(p2 + 1); t3.trim();
          t2 = t2.substring(0, p2);  t2.trim();
          int p3 = t3.indexOf(' ');
          if (p3 >= 0) {
            t4 = t3.substring(p3 + 1); t4.trim();
            t3 = t3.substring(0, p3);  t3.trim();
          }
        }
      }

      String t1u=t1; t1u.toUpperCase();
      String t2u=t2; t2u.toUpperCase();

      uint8_t target = 0x03; // default BOTH
      String a = t1u;
      String b = t2u;
      String c = t3;
      String d = t4;

      if (a == "A" || a == "B" || a == "BOTH") {
        target = (a == "A") ? 0x01 : (a == "B") ? 0x02 : 0x03;
        // shift args
        a = b;
        b = c;  // original t3
        c = d;  // original t4
        d = "";
      }

      if (a == "ON" || a == "OFF") {
        if (a == "OFF") {
          cmdMixFixApply(target, /*mask*/0, /*k*/q12FromMilli(2000), /*m*/q12FromMilli(1000));
          DBG_PRINTLN("[MIXFIX] OFF sent");
        } else {
          cmdMixFixApply(target, /*mask*/3, /*k*/q12FromMilli(2000), /*m*/q12FromMilli(1000));
          DBG_PRINTLN("[MIXFIX] ON sent (mask=3 k=2.0 m=1.0)");
        }
        return;
      }

      // Expect numeric: <mask> <k_milli> <m_milli>
      if (a.length() == 0 || b.length() == 0 || c.length() == 0) {
        DBG_PRINTLN("MIXFIX: missing args. Type MIXFIX for help.");
        return;
      }

      int mask = a.toInt();
      int kMilli = b.toInt();
      int mMilli = c.toInt();
      if (mask < 0 || mask > 3) {
        DBG_PRINTLN("MIXFIX: mask must be 0..3");
        return;
      }

      int16_t kQ12 = q12FromMilli(kMilli);
      int16_t mQ12 = q12FromMilli(mMilli);
      cmdMixFixApply(target, (uint8_t)mask, kQ12, mQ12);
      DBG_PRINTF("[MIXFIX] sent target=0x%02X mask=%d k=%.3f m=%.3f\n",
                 (unsigned)target, mask,
                 (double)kQ12 / 4096.0,
                 (double)mQ12 / 4096.0);
      return;
    }
  }

  // Manual tech commands (word-based).
  // These intentionally avoid single-character commands to prevent accidental triggers.
  {
    String up2 = line;
    up2.toUpperCase();
    String cmd = firstToken(up2);
    String args = afterFirstToken(line);
    args.trim();

    // HELP
    if (cmd == "HELP") {
      Serial.println();
      Serial.println("=== Seashells Master Serial Commands ===");
      Serial.println("HELP");
      Serial.println("START [1|2|3]         (or: START level=<1..3>)");
      Serial.println("STOP                  (end game)");
      Serial.println("OTA [A|B|BOTH]         (trigger Side OTA update)");
      Serial.println("CHAN <1-13>            (set ESP-NOW channel, persist to NVS, reboot)");
      Serial.println("INFO                   (print channel + discovered sides)");
      Serial.println("DIAG ...               (audio diagnostic; type DIAG for help)");
      Serial.println("AUD ...                (sound audition / QA; type AUD for help)");
      Serial.println("MIXFIX ...             (MixFix control; type MIXFIX for help)");
      Serial.println();
      Serial.println("PMS protocol (incoming):");
      Serial.println("  !PMS PING");
      Serial.println("  !PMS START level=1");
      Serial.println("  !PMS STOP");
      Serial.println();
      return;
    }

    // START [level]
    if (cmd == "START") {
      int level = 1;
      if (args.length()) {
        // Accept: START 2
        //         START level=2
        //         START LEVEL 2
        int32_t lv = parseKeyInt(args, "level", -999);
        if (lv != -999) {
          level = (int)lv;
        } else {
          String aup = args; aup.toUpperCase();
          if (aup.startsWith("LEVEL")) {
            String rest = args.substring(5);
            rest.trim();
            level = rest.toInt();
          } else {
            level = args.toInt();
          }
        }
      }
      if (level < 1 || level > 3) {
        Serial.println("Usage: START [1|2|3]   (or START level=<1..3>)");
        return;
      }
      startGameAtLevel((uint8_t)level);
      return;
    }

    // STOP / END
    if (cmd == "STOP" || cmd == "END") {
      stopGameFromHost("stopped");
      return;
    }

    // OTA [A|B|BOTH]
    if (cmd == "OTA") {
      String t = args;
      t.trim();
      t.toUpperCase();
      if (t.startsWith("SIDE")) {
        t = t.substring(4);
        t.trim();
      }

      uint8_t mask = 0x03; // default BOTH
      if (t.length() == 0 || t == "BOTH") mask = 0x03;
      else if (t == "A") mask = 0x01;
      else if (t == "B") mask = 0x02;
      else {
        Serial.println("Usage: OTA [A|B|BOTH]");
        return;
      }

      if (mask & 0x01) {
        DBG_PRINTLN("[Master] OTA Side A");
        if (g_sideKnown[0]) cmdOtaUpdate(g_sideMac[0], OTA_URL_SIDE_BIN);
        else DBG_PRINTLN("[Master] Side A not discovered yet (no OTA)");
        delay(120);
      }
      if (mask & 0x02) {
        DBG_PRINTLN("[Master] OTA Side B");
        if (g_sideKnown[1]) cmdOtaUpdate(g_sideMac[1], OTA_URL_SIDE_BIN);
        else DBG_PRINTLN("[Master] Side B not discovered yet (no OTA)");
      }
      return;
    }
  }
// Unknown legacy input: stay quiet in production
  DBG_PRINT("Unknown legacy line: ");
  DBG_PRINTLN(rawLine);
}

static void handleSerialLine(const String& rawLine) {
  String line = rawLine;
  line.trim();
  if (line.length() == 0) return;

#if PMS_STD_ENABLED
  if (line.startsWith("!PMS")) {
    handlePmsLine(line);
    return;
  }
#endif
  handleLegacyLine(line);
}

static void pollSerial() {
  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    if (c == '\r') continue;
    if (c == '\n') {
      handleSerialLine(g_serialLine);
      g_serialLine = "";
    } else {
      // Avoid unbounded growth if someone blasts data without newlines
      if (g_serialLine.length() < 256) g_serialLine += c;
    }
  }
}

// =============================================================
// PMS 250ms STATUS tick (suppressed while idle)
// =============================================================

static void pmsTick() {
#if PMS_STD_ENABLED
  const uint32_t now = millis();
  if (now - g_pmsLastTickMs < (uint32_t)PMS_STATUS_PERIOD_MS) return;
  g_pmsLastTickMs = now;

  // No STATUS while idle.
  if (g_state == IDLE) {
    // If we somehow returned to idle without emitting game_end, emit a safe fallback.
    if (g_pmsGameStarted && !g_pmsGameEndEmitted) {
      pmsMaybeEmitGameEnd("stopped");
    }
    g_pmsLastState = PMS_IDLE;
    return;
  }

  const PmsState curState = (g_state == WAIT) ? PMS_PLAYING : PMS_ARMING;

  // Default tleft_ms is meaningful only during WAIT (selection timer)
  uint32_t tleftMs = 0;
  if (curState == PMS_PLAYING && g_curTimeoutMs > 0) {
    uint32_t elapsed = now - g_waitStartMs;
    tleftMs = (elapsed >= g_curTimeoutMs) ? 0 : (g_curTimeoutMs - elapsed);
  }

  const uint8_t level = seashellsLevelFromRoundIdx(g_roundIdx);

  // last_reason: prefer explicit score/life markers, else show state changes
  const char* lr = g_pmsLastReason;
  if (strcmp(lr, "none") == 0 && curState != g_pmsLastState) {
    lr = "state";
  }

  pmsPrintStatus(curState, level, g_scoreTotal, g_lives, tleftMs, lr);

  // reset one-shot reason after a tick
  g_pmsLastReason = "none";
  g_pmsLastState = curState;
#else
  (void)0;
#endif
}



static inline bool macIsAllZero(const uint8_t mac[6]) {
  for (int i=0;i<6;i++) if (mac[i] != 0) return false;
  return true;
}

// Forward decl (used by helloReqTick)
static void sendFramed(const uint8_t mac[6], uint8_t type, const uint8_t* payload, int plen);

// Periodically solicit HELLOs from Sides so boot order doesn't matter.
//
// IMPORTANT:
//  - We only broadcast HELLO_REQ while we *haven't* discovered both sides yet.
//  - Once both sides are known, we stop sending HELLO_REQ to avoid needless traffic.
//    (Sides will still send a HELLO on boot/reboot, which is enough to re-sync mid-game.)
static void helloReqTick() {
  if (g_sideKnown[0] && g_sideKnown[1]) return;  // discovery complete

  const uint32_t now = millis();
  if (now < g_nextHelloReqMs) return;

  sendFramed(BCAST_MAC, HELLO_REQ, nullptr, 0);
  g_lastHelloReqSentMs = now;
  g_nextHelloReqMs = now + 1000; // 1 Hz while discovering
}

static inline uint8_t prefsLoadChannel() {
  Preferences p;
  p.begin(kPrefsNs, true);
  uint8_t ch = p.getUChar(kKeyChan, NOW_DEFAULT_CHANNEL);
  p.end();
  if (ch < 1 || ch > 13) ch = NOW_DEFAULT_CHANNEL;
  return ch;
}

static inline void prefsSaveChannel(uint8_t ch) {
  Preferences p;
  p.begin(kPrefsNs, false);
  p.putUChar(kKeyChan, ch);
  p.end();
}

static void addPeer(const uint8_t mac[6]) {
  if (!mac || macIsAllZero(mac)) return;
  esp_now_peer_info_t p{};
  std::memcpy(p.peer_addr, mac, 6);
  p.channel = g_nowChannel;
  p.encrypt = false;
  // ignore errors (peer may already exist)
  (void)esp_now_add_peer(&p);
}

static void sendFramed(const uint8_t mac[6], uint8_t type, const uint8_t* payload, int plen) {
  // Enough for our largest packet (OTA URL): header(4) + url_len(1) + url(200)
  uint8_t buf[4 + 1 + 210];
  int n = SS_build(buf, (int)sizeof(buf), type, payload, plen);
  if (n > 0) {
    esp_now_send(mac, buf, (size_t)n);
  }
}

static inline void setSideMac(uint8_t sideId, const uint8_t mac[6]) {
  if (sideId > 1 || !mac || macIsAllZero(mac)) return;
  if (!g_sideKnown[sideId] || memcmp(g_sideMac[sideId], mac, 6) != 0) {
    memcpy(g_sideMac[sideId], mac, 6);
    g_sideKnown[sideId] = true;
    addPeer(g_sideMac[sideId]);
    DBG_PRINTF("[NOW] Side %c learned: %02X:%02X:%02X:%02X:%02X:%02X\n",
               (sideId==0?'A':'B'),
               g_sideMac[sideId][0],g_sideMac[sideId][1],g_sideMac[sideId][2],
               g_sideMac[sideId][3],g_sideMac[sideId][4],g_sideMac[sideId][5]);
  }
  g_sideLastHelloMs[sideId] = millis();
}

static inline bool macEq(const uint8_t a[6], const uint8_t b[6]) {
  return (a && b && memcmp(a, b, 6) == 0);
}

static int8_t sideIdFromSrc(const uint8_t src[6]) {
  if (g_sideKnown[0] && macEq(src, g_sideMac[0])) return 0;
  if (g_sideKnown[1] && macEq(src, g_sideMac[1])) return 1;
  return -1;
}

static void cmdRoleAssign(const uint8_t mac[6], uint8_t sideId) {
  uint8_t payload[1] = { (uint8_t)(sideId & 1) };
  sendFramed(mac, ROLE_ASSIGN, payload, (int)sizeof(payload));
}

static void cmdGameModeOne(const uint8_t mac[6], bool en){
  uint8_t payload[1] = { (uint8_t)(en ? 1 : 0) };
  sendFramed(mac, GAME_MODE, payload, (int)sizeof(payload));
}

static void cmdLedAllWhiteOne(const uint8_t mac[6]){
  sendFramed(mac, LED_ALL_WHITE, nullptr, 0);
}

// Soft white refresh (no forced OFF frame first on the Side)
static void cmdLedAllWhiteSoftOne(const uint8_t mac[6]){
  sendFramed(mac, LED_WHITE_SOFT, nullptr, 0);
}

static void cmdLedSolidSlotsSide(uint8_t sideId, const uint8_t slotColors[4]) {
  if (sideId > 1 || !slotColors) return;
  if (!g_sideKnown[sideId]) return;
  sendFramed(g_sideMac[sideId], LED_SOLID_SLOTS, slotColors, 4);
}


static void cmdStartLoopAllOne(const uint8_t mac[6]){
  sendFramed(mac, START_LOOP_ALL, nullptr, 0);
}

static void cmdGameMode(bool en){
  uint8_t payload[1] = { (uint8_t)(en ? 1 : 0) };
  for (uint8_t sid = 0; sid < 2; sid++) {
    if (g_sideKnown[sid]) sendFramed(g_sideMac[sid], GAME_MODE, payload, (int)sizeof(payload));
  }
  // Broadcast fallback while discovery is incomplete
  if (!g_sideKnown[0] || !g_sideKnown[1]) {
    sendFramed(BCAST_MAC, GAME_MODE, payload, (int)sizeof(payload));
  }

}

static void cmdLedAllWhite(){
  for (uint8_t sid = 0; sid < 2; sid++) {
    if (g_sideKnown[sid]) sendFramed(g_sideMac[sid], LED_ALL_WHITE, nullptr, 0);
  }
  // Broadcast fallback while discovery is incomplete
  if (!g_sideKnown[0] || !g_sideKnown[1]) {
    sendFramed(BCAST_MAC, LED_ALL_WHITE, nullptr, 0);
  }

}

// Soft white refresh (no forced OFF frame first on the Side).
// Used for retrying white during WAIT without visible flicker.
static void cmdLedAllWhiteSoft(){
  for (uint8_t sid = 0; sid < 2; sid++) {
    if (g_sideKnown[sid]) cmdLedAllWhiteSoftOne(g_sideMac[sid]);
  }
  // Broadcast fallback while discovery is incomplete
  if (!g_sideKnown[0] || !g_sideKnown[1]) {
    sendFramed(BCAST_MAC, LED_WHITE_SOFT, nullptr, 0);
  }
}

static void cmdBlinkAll(uint8_t color, uint16_t on_ms, uint16_t off_ms){
  uint8_t payload[5] = {
    color,
    (uint8_t)(on_ms >> 8), (uint8_t)on_ms,
    (uint8_t)(off_ms >> 8), (uint8_t)off_ms
  };
  for (uint8_t sid = 0; sid < 2; sid++) {
    if (g_sideKnown[sid]) sendFramed(g_sideMac[sid], BLINK_ALL, payload, (int)sizeof(payload));
  }
  // Broadcast fallback while discovery is incomplete
  if (!g_sideKnown[0] || !g_sideKnown[1]) {
    sendFramed(BCAST_MAC, BLINK_ALL, payload, (int)sizeof(payload));
  }

}

// Per-slot blink pattern (red for wrong, green for correct). No broadcast fallback
// because each Side can have a different slot layout.
static void cmdBlinkSlotsOne(const uint8_t mac[6], const uint8_t slotColors[4],
                             uint16_t on_ms, uint16_t off_ms) {
  uint8_t payload[8];
  payload[0] = slotColors[0];
  payload[1] = slotColors[1];
  payload[2] = slotColors[2];
  payload[3] = slotColors[3];
  payload[4] = (uint8_t)(on_ms >> 8);
  payload[5] = (uint8_t)(on_ms & 0xFF);
  payload[6] = (uint8_t)(off_ms >> 8);
  payload[7] = (uint8_t)(off_ms & 0xFF);
  sendFramed(mac, BLINK_SLOTS, payload, (int)sizeof(payload));
}

static void cmdBlinkRevealCorrect(uint16_t on_ms, uint16_t off_ms) {
  // If discovery isn't complete, fall back to a simple red blink.
  if (!g_sideKnown[0] || !g_sideKnown[1]) {
    cmdBlinkAll(/*red*/0, on_ms, off_ms);
    return;
  }

  // Build per-side color maps: 0=red (wrong), 1=green (correct)
  uint8_t colorsA[4];
  uint8_t colorsB[4];
  for (int i=0;i<4;i++) {
    colorsA[i] = slotIsOdd_A[i] ? 1 : 0;
    colorsB[i] = slotIsOdd_B[i] ? 1 : 0;
  }

  cmdBlinkSlotsOne(g_sideMac[0], colorsA, on_ms, off_ms);
  cmdBlinkSlotsOne(g_sideMac[1], colorsB, on_ms, off_ms);
}

static void cmdStartLoopAll(){
  for (uint8_t sid = 0; sid < 2; sid++) {
    if (g_sideKnown[sid]) sendFramed(g_sideMac[sid], START_LOOP_ALL, nullptr, 0);
  }
  // Broadcast fallback while discovery is incomplete
  if (!g_sideKnown[0] || !g_sideKnown[1]) {
    sendFramed(BCAST_MAC, START_LOOP_ALL, nullptr, 0);
  }

}

static void cmdStopAll(){
  for (uint8_t sid = 0; sid < 2; sid++) {
    if (g_sideKnown[sid]) sendFramed(g_sideMac[sid], STOP_ALL, nullptr, 0);
  }
  // Broadcast fallback while discovery is incomplete
  if (!g_sideKnown[0] || !g_sideKnown[1]) {
    sendFramed(BCAST_MAC, STOP_ALL, nullptr, 0);
  }

}

static void cmdSetSceneSide(uint8_t sideId, const uint16_t ids[4]){
  if (sideId > 1 || !g_sideKnown[sideId]) return;
  uint8_t payload[8];
  for (int i=0;i<4;i++) {
    payload[i*2]     = (uint8_t)(ids[i] >> 8);
    payload[i*2 + 1] = (uint8_t)(ids[i] & 0xFF);
  }
  sendFramed(g_sideMac[sideId], SET_SCENE, payload, (int)sizeof(payload));
}

// Per-slot audio trim in dB (signed int8 per slot). Applied by the Side.
// This is a pure software workaround to reduce perceived "two sounds" when
// a speaker/amp is effectively summing L+R, or when acoustic bleed is strong.
static void cmdSetSlotTrimSide(uint8_t sideId, const int8_t trim_db[4]) {
  if (sideId > 1 || !g_sideKnown[sideId] || !trim_db) return;
  uint8_t payload[4];
  for (int i=0;i<4;i++) payload[i] = (uint8_t)trim_db[i];
  sendFramed(g_sideMac[sideId], SLOT_TRIM_DB, payload, (int)sizeof(payload));
}

// =============================================================
// MIXFIX (runtime RIGHT-channel de-mix) helpers
// =============================================================

static inline int16_t q12FromMilli(int milli) {
  if (milli < 0) milli = 0;
  if (milli > 4000) milli = 4000; // cap at 4.0
  int32_t q12 = ((int32_t)milli * 4096 + 500) / 1000; // rounded
  if (q12 > 16384) q12 = 16384;
  return (int16_t)q12;
}

static void cmdMixFixSide(uint8_t sideId, uint8_t mask, int16_t kQ12, int16_t mQ12) {
  if (sideId > 1 || !g_sideKnown[sideId]) return;
  uint8_t payload[5];
  payload[0] = (uint8_t)(mask & 0x03);
  payload[1] = (uint8_t)((uint16_t)kQ12 >> 8);
  payload[2] = (uint8_t)((uint16_t)kQ12 & 0xFF);
  payload[3] = (uint8_t)((uint16_t)mQ12 >> 8);
  payload[4] = (uint8_t)((uint16_t)mQ12 & 0xFF);

  // Send a few times to survive dropped ESP-NOW packets.
  for (int i=0;i<3;i++) {
    sendFramed(g_sideMac[sideId], MIXFIX_SET, payload, (int)sizeof(payload));
    delay(12);
  }
}

static void cmdMixFixApply(uint8_t targetMask, uint8_t mask, int16_t kQ12, int16_t mQ12) {
  if (targetMask & 0x01) cmdMixFixSide(0, mask, kQ12, mQ12);
  if (targetMask & 0x02) cmdMixFixSide(1, mask, kQ12, mQ12);

  // Broadcast fallback helps if discovery is incomplete.
  if (!g_sideKnown[0] || !g_sideKnown[1]) {
    uint8_t payload[5];
    payload[0] = (uint8_t)(mask & 0x03);
    payload[1] = (uint8_t)((uint16_t)kQ12 >> 8);
    payload[2] = (uint8_t)((uint16_t)kQ12 & 0xFF);
    payload[3] = (uint8_t)((uint16_t)mQ12 >> 8);
    payload[4] = (uint8_t)((uint16_t)mQ12 & 0xFF);
    sendFramed(BCAST_MAC, MIXFIX_SET, payload, (int)sizeof(payload));
  }
}

static inline bool findOdd(uint8_t& outSide, uint8_t& outSlot) {
  for (uint8_t i=0;i<4;i++) {
    if (slotIsOdd_A[i]) { outSide = 0; outSlot = i; return true; }
  }
  for (uint8_t i=0;i<4;i++) {
    if (slotIsOdd_B[i]) { outSide = 1; outSlot = i; return true; }
  }
  outSide = 255;
  outSlot = 255;
  return false;
}

// Apply the configured mitigation mode by sending SLOT_TRIM_DB to both sides.
// Called each round (BUILD/ANNOUNCE) to keep Sides in a known state even if a
// packet was dropped in a previous round.
static void cmdApplyAudioMitigationForRound() {
  int8_t tA[4] = {0,0,0,0};
  int8_t tB[4] = {0,0,0,0};

  if (AUDIO_MITIGATION_MODE == 0) {
    // Stock behavior: no trims.
  } else {
    uint8_t oddSide = 255, oddSlot = 255;
    (void)findOdd(oddSide, oddSlot);

    if (AUDIO_MITIGATION_MODE == 1) {
      // Mute/attenuate ONLY the odd slot's I2S pair-mate on the same Side.
      // Slots are paired as 0<->1 and 2<->3, so XOR 1 gives the mate.
      if (oddSide <= 1 && oddSlot <= 3) {
        uint8_t mate = oddSlot ^ 1;
        if (oddSide == 0) tA[mate] = AUDIO_PAIR_MUTE_DB;
        else              tB[mate] = AUDIO_PAIR_MUTE_DB;
      }
    } else if (AUDIO_MITIGATION_MODE == 2) {
      // Attenuate all common speakers, leave the odd at 0 dB.
      for (uint8_t i=0;i<4;i++) {
        if (!slotIsOdd_A[i]) tA[i] = AUDIO_COMMON_ATTEN_DB;
        if (!slotIsOdd_B[i]) tB[i] = AUDIO_COMMON_ATTEN_DB;
      }
    }
  }

  // Unicast only; trims can differ per side.
  cmdSetSlotTrimSide(0, tA);
  cmdSetSlotTrimSide(1, tB);
}

// =============================================================
// DIAG AUDIO (tone patterns) helpers
// =============================================================

static void cmdDiagAudioOne(uint8_t sideId, uint8_t pattern) {
  if (sideId > 1) return;
  if (!g_sideKnown[sideId]) return;

  // Keep Sides in gameMode so button presses send BTN_EVENT (we use it to step patterns).
  cmdGameModeOne(g_sideMac[sideId], true);

  // Clear any prior mitigation trims so diag isn't affected.
  const int8_t z[4] = {0,0,0,0};
  cmdSetSlotTrimSide(sideId, z);

  // Send the pattern a few times to survive a dropped ESP-NOW packet.
  uint8_t payload[1] = { pattern };
  for (int i=0;i<3;i++) {
    sendFramed(g_sideMac[sideId], DIAG_AUDIO, payload, 1);
    delay(12);
  }
}

static void cmdDiagAudioApply() {
  // Apply to BOTH sides, but allow targeting one side by turning the other OFF.
  uint8_t patA = (g_diagTargetMask & 0x01) ? g_diagPattern : 0;
  uint8_t patB = (g_diagTargetMask & 0x02) ? g_diagPattern : 0;

  if (g_sideKnown[0]) cmdDiagAudioOne(0, patA);
  if (g_sideKnown[1]) cmdDiagAudioOne(1, patB);

  g_diagLastResendMs = millis();
}

static void diagPrintHelp() {
  DBG_PRINTLN("\n=== DIAG AUDIO (tone test) ===");
  DBG_PRINTLN("Commands:");
  DBG_PRINTLN("  DIAG            -> help");
  DBG_PRINTLN("  DIAG OFF        -> exit diag");
  DBG_PRINTLN("  DIAG <n>        -> set pattern n on BOTH sides");
  DBG_PRINTLN("  DIAG A <n>      -> set pattern n on Side A only");
  DBG_PRINTLN("  DIAG B <n>      -> set pattern n on Side B only");
  DBG_PRINTLN("  DIAG BOTH <n>   -> set pattern n on BOTH sides");
  DBG_PRINTLN("  DIAG NEXT       -> next pattern");
  DBG_PRINTLN("  DIAG PREV       -> prev pattern");
  DBG_PRINTLN("  DIAG AUTO       -> toggle auto-cycle (~2.5s)\n");

  DBG_PRINTLN("Patterns (also shown on Side LEDs):");
  DBG_PRINTLN("  1: I2S0 LEFT  only (slot0)  440Hz");
  DBG_PRINTLN("  2: I2S0 RIGHT only (slot1)  880Hz");
  DBG_PRINTLN("  3: I2S0 L+R simultaneously 440Hz + 880Hz");
  DBG_PRINTLN("  4: I2S1 LEFT  only (slot2)  550Hz");
  DBG_PRINTLN("  5: I2S1 RIGHT only (slot3) 1100Hz");
  DBG_PRINTLN("  6: I2S1 L+R simultaneously 550Hz + 1100Hz");
  DBG_PRINTLN("  7: ALL 4 (330/660/990/1320Hz)");
  DBG_PRINTLN("  8: Odd-sim: odd slot0 (1600Hz), commons (800Hz)");
  DBG_PRINTLN("  9: Odd-sim: odd slot1 (1600Hz), commons (800Hz)");
  DBG_PRINTLN(" 10: Odd-sim: odd slot2 (1600Hz), commons (800Hz)");
  DBG_PRINTLN(" 11: Odd-sim: odd slot3 (1600Hz), commons (800Hz)");
  DBG_PRINTLN(" 12: PHASE-CANCEL I2S0: slot0=+440Hz, slot1=-440Hz (mono-sum detector)");
  DBG_PRINTLN(" 13: PHASE-CANCEL I2S1: slot2=+550Hz, slot3=-550Hz (mono-sum detector)");
  DBG_PRINTLN(" 14: MIXFIX TOGGLE I2S0: slot0=440Hz, slot1 silent; slot1 LED RED=OFF, GREEN=ON");
  DBG_PRINTLN(" 15: MIXFIX TOGGLE I2S1: slot2=550Hz, slot3 silent; slot3 LED RED=OFF, GREEN=ON\n");

  DBG_PRINTLN("Interpretation tips:");
  DBG_PRINTLN("  - Most decisive: patterns 1/2/4/5. In each, the OTHER speaker in that I2S pair should be effectively silent at the cone.");
  DBG_PRINTLN("  - Patterns 3/6: each speaker should have ONLY its own tone at the cone (440 vs 880, 550 vs 1100). If one speaker has BOTH strongly, that amp is mixing L+R.");
  DBG_PRINTLN("  - Patterns 12/13: phase-cancel tests. If an amp outputs a mono sum (L+R), the tone will cancel and that speaker will get MUCH quieter/near silent.");
  DBG_PRINTLN("    (If your speakers are truly separate, each cone still plays a tone; cancellation may only be noticeable in the room between speakers.)");
  DBG_PRINTLN("  - Patterns 14/15: MixFix toggle. If the RIGHT amp is mixing L+R, turning MixFix ON should make the RIGHT speaker get noticeably quieter.");
  DBG_PRINTLN("  - Patterns 8..11: reproduce the game scenario (odd vs common) with known tones.\n");
}

static inline uint8_t clampDiagPattern(int v) {
  if (v < 1) v = 1;
  if (v > 15) v = 15;
  return (uint8_t)v;
}

static void diagEnable(uint8_t pattern, uint8_t targetMask, bool autoCycle) {
  if (pattern < 1) pattern = 1;
  if (pattern > 15) pattern = 15;
  g_diagAudioEnabled = true;
  g_diagPattern      = pattern;
  g_diagTargetMask   = targetMask ? targetMask : 0x03;
  g_diagAutoCycle    = autoCycle;
  g_diagNextStepMs   = millis() + DIAG_AUTOCYCLE_MS;
  g_diagAdvanceRequested = false;

  // Stop any running game cleanly.
  if (g_state != IDLE) stopGameFromHost("stopped");

  DBG_PRINTF("[DIAG] enabled pattern=%u target=%s%s auto=%s\n",
             (unsigned)g_diagPattern,
             (g_diagTargetMask & 0x01) ? "A" : "",
             (g_diagTargetMask & 0x02) ? "B" : "",
             g_diagAutoCycle ? "ON" : "OFF");

  cmdDiagAudioApply();
}

static void diagDisable() {
  g_diagAudioEnabled = false;
  g_diagAutoCycle    = false;
  g_diagAdvanceRequested = false;

  // Tell both sides to exit diag.
  g_diagTargetMask = 0x03;
  g_diagPattern = 0;
  if (g_sideKnown[0]) cmdDiagAudioOne(0, 0);
  if (g_sideKnown[1]) cmdDiagAudioOne(1, 0);
  DBG_PRINTLN("[DIAG] disabled");
}

static void diagStep(int delta) {
  int v = (int)g_diagPattern + delta;
  if (v < 1) v = 15;
  if (v > 15) v = 1;
  g_diagPattern = (uint8_t)v;
  DBG_PRINTF("[DIAG] pattern=%u\n", (unsigned)g_diagPattern);
  cmdDiagAudioApply();
}

static void diagTick() {
  if (!g_diagAudioEnabled) return;
  const uint32_t now = millis();

  // Button press while in DIAG -> NEXT (flag set by ESP-NOW callback)
  if (g_diagAdvanceRequested) {
    g_diagAdvanceRequested = false;
    diagStep(+1);
    g_diagNextStepMs = now + DIAG_AUTOCYCLE_MS;
  }

  // Auto-cycle
  if (g_diagAutoCycle && (int32_t)(now - g_diagNextStepMs) >= 0) {
    diagStep(+1);
    g_diagNextStepMs = now + DIAG_AUTOCYCLE_MS;
  }

  // Periodic resend (helps if a Side rebooted)
  if ((int32_t)(now - g_diagLastResendMs) >= (int32_t)DIAG_RESEND_PERIOD_MS) {
    cmdDiagAudioApply();
  }
}

static void endGame() {
  cmdStopAll();
  gameClearRoundPushRetries();
  g_state = IDLE;
  DBG_PRINTLN("[Master] Game ended -> IDLE");
}

static void shuffleArray(uint16_t* arr, size_t n) {
  for (size_t i = 0; i + 1 < n; i++) {
    size_t j = i + (size_t)random((long)(n - i));
    uint16_t t = arr[i];
    arr[i] = arr[j];
    arr[j] = t;
  }
}

static void printIdInfo(const char* label, uint16_t id) {
  const MasterClipMeta* cm = MasterManifest_find(id);
  if (!cm) {
    DBG_PRINTF("  %s id=%u (unknown)\n", label, (unsigned)id);
  } else {
    DBG_PRINTF("  %s id=%u base=%s sub=%s sub2=%s\n",
                  label, (unsigned)id, cm->base, cm->sub, cm->sub2);
  }
}


// ---------- Clip exclusions ----------
// Some clips exist on the SD card / manifest but should never be used in gameplay.
// (Requested: remove thunder from Halloween sounds; remove walking-in-snow.)
static bool isExcludedClip(const MasterClipMeta& m) {
  switch (m.id) {
    // Halloween thunder
    case 6006:
    case 6007:
    case 6008:

    // Christmas walking-in-snow
    case 6104:
    case 6105:
    case 6106:

    // Additional requested exclusions
    case 1102:
    case 1108:
    case 5201:
    case 5202:
      return true;

    default:
      return false;
  }
}

// Fallback: pick any non-excluded clip ID (guarded)
static uint16_t pickAnyAllowedId() {
  for (int guard = 0; guard < 800; guard++) {
    size_t idx = (size_t)random((long)MASTER_CLIP_COUNT);
    const MasterClipMeta& m = MASTER_CLIPS[idx];
    if (isExcludedClip(m)) continue;
    return m.id;
  }
  // Worst-case: return the first clip even if excluded (should never happen)
  return MASTER_CLIPS[0].id;
}

// ---------- Category helper functions ----------

// Collect unique base strings
static size_t collectUniqueBases(const char* out[], size_t maxOut) {
  size_t n = 0;
  for (size_t i = 0; i < MASTER_CLIP_COUNT; i++) {
    if (isExcludedClip(MASTER_CLIPS[i])) continue;
    const char* b = MASTER_CLIPS[i].base;
    bool found = false;
    for (size_t j = 0; j < n; j++) {
      if (strcasecmp(out[j], b) == 0) { found = true; break; }
    }
    if (!found) {
      if (n < maxOut) out[n++] = b;
      else break;
    }
  }
  return n;
}

// Collect unique sub strings for a given base
static size_t collectUniqueSubsForBase(const char* base, const char* out[], size_t maxOut) {
  size_t n = 0;
  for (size_t i = 0; i < MASTER_CLIP_COUNT; i++) {
    if (isExcludedClip(MASTER_CLIPS[i])) continue;
    if (strcasecmp(MASTER_CLIPS[i].base, base) != 0) continue;
    const char* s = MASTER_CLIPS[i].sub;
    bool found = false;
    for (size_t j = 0; j < n; j++) {
      if (strcasecmp(out[j], s) == 0) { found = true; break; }
    }
    if (!found) {
      if (n < maxOut) out[n++] = s;
      else break;
    }
  }
  return n;
}

// Collect unique sub2 strings for a given base
static size_t collectUniqueSub2ForBase(const char* base, const char* out[], size_t maxOut) {
  size_t n = 0;
  for (size_t i = 0; i < MASTER_CLIP_COUNT; i++) {
    if (isExcludedClip(MASTER_CLIPS[i])) continue;
    if (strcasecmp(MASTER_CLIPS[i].base, base) != 0) continue;
    const char* s2 = MASTER_CLIPS[i].sub2;
    bool found = false;
    for (size_t j = 0; j < n; j++) {
      if (strcasecmp(out[j], s2) == 0) { found = true; break; }
    }
    if (!found) {
      if (n < maxOut) out[n++] = s2;
      else break;
    }
  }
  return n;
}

// Collect all IDs matching base
static size_t collectIdsByBase(const char* base, uint16_t* out, size_t maxOut) {
  size_t n = 0;
  for (size_t i=0; i<MASTER_CLIP_COUNT; i++) {
    if (isExcludedClip(MASTER_CLIPS[i])) continue;
    if (strcasecmp(MASTER_CLIPS[i].base, base) != 0) continue;
    if (n < maxOut) out[n++] = MASTER_CLIPS[i].id;
  }
  return n;
}

// Collect all IDs matching base+sub
static size_t collectIdsByBaseSub(const char* base, const char* sub, uint16_t* out, size_t maxOut) {
  size_t n = 0;
  for (size_t i=0; i<MASTER_CLIP_COUNT; i++) {
    if (isExcludedClip(MASTER_CLIPS[i])) continue;
    if (strcasecmp(MASTER_CLIPS[i].base, base) != 0) continue;
    if (strcasecmp(MASTER_CLIPS[i].sub,  sub)  != 0) continue;
    if (n < maxOut) out[n++] = MASTER_CLIPS[i].id;
  }
  return n;
}

// Collect all IDs matching base+sub2
static size_t collectIdsByBaseSub2(const char* base, const char* sub2, uint16_t* out, size_t maxOut) {
  size_t n = 0;
  for (size_t i=0; i<MASTER_CLIP_COUNT; i++) {
    if (isExcludedClip(MASTER_CLIPS[i])) continue;
    if (strcasecmp(MASTER_CLIPS[i].base, base) != 0) continue;
    if (strcasecmp(MASTER_CLIPS[i].sub2, sub2) != 0) continue;
    if (n < maxOut) out[n++] = MASTER_CLIPS[i].id;
  }
  return n;
}

// Pick a random ID for a base (odd/fallback)
static uint16_t pickRandomIdByBase(const char* base) {
  uint16_t ids[32];
  size_t n = collectIdsByBase(base, ids, 32);
  if (n == 0) {
    return pickAnyAllowedId();
  }
  size_t idx = (size_t)random((long)n);
  return ids[idx];
}

// Fill dest[needed] with unique IDs first, then reuse randomly from uniques if needed
static void fillWithUniqueThenReuse(uint16_t* dest, size_t needed, uint16_t* uniqueIds, size_t uniqueCount, const char* context) {
  if (uniqueCount == 0) {
    for (size_t i=0; i<needed; i++) dest[i] = 0;
    DBG_PRINTF("[Master] WARN: no IDs for context '%s'\n", context ? context : "");
    return;
  }

  shuffleArray(uniqueIds, uniqueCount);

  for (size_t i=0; i<needed; i++) {
    if (i < uniqueCount) {
      dest[i] = uniqueIds[i];  // unique
    } else {
      size_t idx = (size_t)random((long)uniqueCount);
      dest[i] = uniqueIds[idx];
    }
  }
}

// ---------- Level builders ----------

// Level 2: 7 from baseMain, 1 from baseOdd
static void buildScenes_level2_randomBases() {
  const char* bases[8];
  size_t baseCount = collectUniqueBases(bases, 8);

  if (baseCount < 2) {
    DBG_PRINTLN("[Master] Level2: need >=2 bases, falling back to trivial (all from same base)");
    baseCount = collectUniqueBases(bases, 8);
  }

  size_t idxMain = (size_t)random((long)baseCount);
  size_t idxOdd  = (baseCount > 1) ? (size_t)random((long)(baseCount - 1)) : idxMain;
  if (baseCount > 1 && idxOdd >= idxMain) idxOdd++;

  const char* baseMain = bases[idxMain];
  const char* baseOdd  = bases[idxOdd];

  uint16_t unique[32];
  uint16_t sameIds[7];

  size_t uCount = collectIdsByBase(baseMain, unique, 32);
  if (uCount == 0) {
    DBG_PRINTLN("[Master] Level2: no IDs for baseMain, using any IDs");
    for (int i=0;i<7;i++) sameIds[i] = pickAnyAllowedId();
  } else {
    fillWithUniqueThenReuse(sameIds, 7, unique, uCount, "Level2 baseMain");
  }

  uint16_t oddId;
  uint16_t uniqueOdd[32];
  size_t uOddCount = collectIdsByBase(baseOdd, uniqueOdd, 32);
  if (uOddCount == 0) {
    oddId = pickAnyAllowedId();
  } else {
    shuffleArray(uniqueOdd, uOddCount);
    oddId = uniqueOdd[0];
  }

  uint8_t sideOdd = random(2);
  uint8_t oddSlot = random(4);
  int sameIdx = 0;

  if (sideOdd == 0) {
    for (int i=0; i<4; i++) {
      if (i == oddSlot) sceneA[i] = oddId;
      else              sceneA[i] = sameIds[sameIdx++];
    }
    for (int i=0; i<4; i++) sceneB[i] = sameIds[sameIdx++];
  } else {
    for (int i=0; i<4; i++) sceneA[i] = sameIds[sameIdx++];
    for (int i=0; i<4; i++) {
      if (i == oddSlot) sceneB[i] = oddId;
      else              sceneB[i] = sameIds[sameIdx++];
    }
  }

  for (int i=0;i<4;i++) slotIsOdd_A[i] = (sceneA[i] == oddId);
  for (int i=0;i<4;i++) slotIsOdd_B[i] = (sceneB[i] == oddId);

  DBG_PRINTF("[Master] Level2: baseMain=%s baseOdd=%s sideOdd=%u oddSlot=%u\n",
                baseMain, baseOdd, (unsigned)sideOdd, (unsigned)oddSlot);
  for (int i=0;i<4;i++) printIdInfo("  sceneA", sceneA[i]);
  for (int i=0;i<4;i++) printIdInfo("  sceneB", sceneB[i]);
}

// Level 1: 7 from one sub2 family of a random base, 1 from a different base
static void buildScenes_level1_sub2() {
  const char* bases[8];
  size_t baseCount = collectUniqueBases(bases, 8);
  if (baseCount < 2) {
    DBG_PRINTLN("[Master] Level1: need >=2 bases, fallback to Level2");
    buildScenes_level2_randomBases();
    return;
  }

  size_t idxMain = (size_t)random((long)baseCount);
  const char* baseMain = bases[idxMain];

  const char* sub2List[16];
  size_t sub2Count = collectUniqueSub2ForBase(baseMain, sub2List, 16);
  if (sub2Count == 0) {
    DBG_PRINTLN("[Master] Level1: no sub2 families for baseMain, fallback to Level2");
    buildScenes_level2_randomBases();
    return;
  }

  size_t idxFamily = (size_t)random((long)sub2Count);
  const char* familySub2 = sub2List[idxFamily];

  size_t idxOdd = (size_t)random((long)(baseCount - 1));
  if (idxOdd >= idxMain) idxOdd++;
  const char* baseOdd = bases[idxOdd];

  uint16_t unique[32];
  uint16_t sameIds[7];

  size_t uCount = collectIdsByBaseSub2(baseMain, familySub2, unique, 32);
  if (uCount == 0) {
    DBG_PRINTLN("[Master] Level1: no IDs for baseMain+sub2, fallback to Level2");
    buildScenes_level2_randomBases();
    return;
  }
  fillWithUniqueThenReuse(sameIds, 7, unique, uCount, "Level1 base+sub2");

  uint16_t oddId;
  uint16_t uniqueOdd[32];
  size_t uOddCount = collectIdsByBase(baseOdd, uniqueOdd, 32);
  if (uOddCount == 0) {
    oddId = pickAnyAllowedId();
  } else {
    shuffleArray(uniqueOdd, uOddCount);
    oddId = uniqueOdd[0];
  }

  uint8_t sideOdd = random(2);
  uint8_t oddSlot = random(4);
  int sameIdx = 0;

  if (sideOdd == 0) {
    for (int i=0; i<4; i++) {
      if (i == oddSlot) sceneA[i] = oddId;
      else              sceneA[i] = sameIds[sameIdx++];
    }
    for (int i=0; i<4; i++) sceneB[i] = sameIds[sameIdx++];
  } else {
    for (int i=0; i<4; i++) sceneA[i] = sameIds[sameIdx++];
    for (int i=0; i<4; i++) {
      if (i == oddSlot) sceneB[i] = oddId;
      else              sceneB[i] = sameIds[sameIdx++];
    }
  }

  for (int i=0;i<4;i++) slotIsOdd_A[i] = (sceneA[i] == oddId);
  for (int i=0;i<4;i++) slotIsOdd_B[i] = (sceneB[i] == oddId);

  DBG_PRINTF("[Master] Level1: baseMain=%s familySub2=%s baseOdd=%s sideOdd=%u oddSlot=%u\n",
                baseMain, familySub2, baseOdd, (unsigned)sideOdd, (unsigned)oddSlot);
  for (int i=0;i<4;i++) printIdInfo("  sceneA", sceneA[i]);
  for (int i=0;i<4;i++) printIdInfo("  sceneB", sceneB[i]);
}

// Level 3: 7 from one sub of a base, 1 from a different sub of same base
static void buildScenes_level3_subs() {
  const char* bases[8];
  size_t baseCount = collectUniqueBases(bases, 8);
  if (baseCount == 0) {
    DBG_PRINTLN("[Master] Level3: no bases, fallback to Level2");
    buildScenes_level2_randomBases();
    return;
  }

  const char* baseMain = nullptr;
  const char* subs[16];
  size_t subCount = 0;

  for (size_t attempt = 0; attempt < baseCount * 2; attempt++) {
    const char* candidateBase = bases[random((long)baseCount)];
    size_t cnt = collectUniqueSubsForBase(candidateBase, subs, 16);
    if (cnt >= 2) {
      baseMain = candidateBase;
      subCount = cnt;
      break;
    }
  }

  if (!baseMain || subCount < 2) {
    DBG_PRINTLN("[Master] Level3: no base with >=2 subs, fallback to Level2");
    buildScenes_level2_randomBases();
    return;
  }

  size_t idxSame = (size_t)random((long)subCount);
  size_t idxOdd  = (size_t)random((long)(subCount - 1));
  if (idxOdd >= idxSame) idxOdd++;

  const char* subSame = subs[idxSame];
  const char* subOdd  = subs[idxOdd];

  uint16_t unique[32];
  uint16_t sameIds[7];

  size_t uCount = collectIdsByBaseSub(baseMain, subSame, unique, 32);
  if (uCount == 0) {
    DBG_PRINTLN("[Master] Level3: no IDs for baseMain+subSame, fallback to Level2");
    buildScenes_level2_randomBases();
    return;
  }
  fillWithUniqueThenReuse(sameIds, 7, unique, uCount, "Level3 base+subSame");

  uint16_t oddId;
  uint16_t uniqueOdd[32];
  size_t uOddCount = collectIdsByBaseSub(baseMain, subOdd, uniqueOdd, 32);
  if (uOddCount == 0) {
    oddId = pickRandomIdByBase(baseMain);
  } else {
    shuffleArray(uniqueOdd, uOddCount);
    oddId = uniqueOdd[0];
  }

  uint8_t sideOdd = random(2);
  uint8_t oddSlot = random(4);
  int sameIdx = 0;

  if (sideOdd == 0) {
    for (int i=0; i<4; i++) {
      if (i == oddSlot) sceneA[i] = oddId;
      else              sceneA[i] = sameIds[sameIdx++];
    }
    for (int i=0; i<4; i++) sceneB[i] = sameIds[sameIdx++];
  } else {
    for (int i=0; i<4; i++) sceneA[i] = sameIds[sameIdx++];
    for (int i=0; i<4; i++) {
      if (i == oddSlot) sceneB[i] = oddId;
      else              sceneB[i] = sameIds[sameIdx++];
    }
  }

  for (int i=0;i<4;i++) slotIsOdd_A[i] = (sceneA[i] == oddId);
  for (int i=0;i<4;i++) slotIsOdd_B[i] = (sceneB[i] == oddId);

  DBG_PRINTF("[Master] Level3: baseMain=%s subSame=%s subOdd=%s sideOdd=%u oddSlot=%u\n",
                baseMain, subSame, subOdd, (unsigned)sideOdd, (unsigned)oddSlot);
  for (int i=0;i<4;i++) printIdInfo("  sceneA", sceneA[i]);
  for (int i=0;i<4;i++) printIdInfo("  sceneB", sceneB[i]);
}

// OTA helper
static void cmdOtaUpdate(const uint8_t mac[6], const char* url) {
  const size_t ulen = strnlen(url, 200);
  uint8_t payload[1 + 200];
  payload[0] = (uint8_t)ulen;
  memcpy(payload + 1, url, ulen);
  sendFramed(mac, OTA_UPDATE, payload, 1 + (int)ulen);
}

// ---------- ESP-NOW ----------
static void onRecv(const esp_now_recv_info_t* info, const uint8_t* data, int len) {
  if (!info || !data || len < (int)SS_HDR_LEN) return;

  uint8_t type = 0;
  const uint8_t* payload = nullptr;
  int plen = 0;
  if (!SS_parse(data, len, type, payload, plen)) return;

  const uint8_t* src = info->src_addr;

  // -------- HELLO (discovery / role mapping) --------
  // payload: [sideId, poolA_hi, poolA_lo, poolB_hi, poolB_lo]
  if (type == HELLO && plen >= 5) {
    uint8_t reported = payload[0];
    uint16_t poolA = (uint16_t)payload[1] << 8 | payload[2];
    uint16_t poolB = (uint16_t)payload[3] << 8 | payload[4];

    // Determine which side this device should be.
    // Prefer the Side's persisted role, but resolve duplicates gracefully.
    uint8_t assign = 255;
    if (reported <= 1) {
      if (!g_sideKnown[reported] || macEq(g_sideMac[reported], src)) {
        assign = reported;
      } else {
        // Conflict: that role is already occupied by a different MAC.
        // If the other slot is free, use it and tell the Side to update.
        uint8_t other = reported ^ 1;
        if (!g_sideKnown[other]) assign = other;
        else assign = reported; // both claimed; treat as a possible replacement
      }
    } else {
      if (!g_sideKnown[0]) assign = 0;
      else if (!g_sideKnown[1]) assign = 1;
      else assign = 255;
    }

    DBG_PRINTF("[Master] HELLO from %02X:%02X:%02X:%02X:%02X:%02X rep=%u assign=%u poolA=%u poolB=%u\n",
               src[0],src[1],src[2],src[3],src[4],src[5],
               (unsigned)reported,
               (unsigned)assign,
               (unsigned)poolA,
               (unsigned)poolB);

    // Determine whether this HELLO is a response to our HELLO_REQ probe.
    const uint32_t nowMs = millis();
    const bool respToHelloReq =
      (g_lastHelloReqSentMs != 0) && ((nowMs - g_lastHelloReqSentMs) < 300);

    // Snapshot current mapping BEFORE we update it.
    const bool preWasKnown = (assign <= 1) ? g_sideKnown[assign] : false;
    const bool preSameMac  = (assign <= 1) ? (preWasKnown && macEq(g_sideMac[assign], src)) : false;
    const bool preJustLearnedOrReplaced = (assign <= 1) && (!preWasKnown || !preSameMac);
    const bool unsolicitedHello = !respToHelloReq;

    // Ensure we can unicast to this Side
    addPeer(src);

    if (assign <= 1) {
      setSideMac(assign, src);
      if (reported != assign) {
        cmdRoleAssign(src, assign);
      }
    } else {
      DBG_PRINTLN("[Master] HELLO ignored: both sides already claimed (or role unknown)");
    }

    // Put the side into game mode
    cmdGameModeOne(src, true);

    // Auto-apply MixFix defaults whenever a Side says HELLO (boot/reboot).
    // This ensures the fix is active after power cycles without requiring
    // a manual serial command.
#if AUTO_ENABLE_MIXFIX
    if (assign <= 1) {
      cmdMixFixSide(assign,
                    (uint8_t)AUTO_ENABLE_MIXFIX_MASK,
                    q12FromMilli(AUTO_ENABLE_MIXFIX_K_MILLI),
                    q12FromMilli(AUTO_ENABLE_MIXFIX_M_MILLI));
    }
#endif

    // If we're mid-game, (re)send state ONLY when the Side likely just came online.
    // This avoids restarting audio/LEDs on every HELLO (e.g., when probing for discovery).
    if ((g_state == ANNOUNCE || g_state == WAIT) && assign <= 1) {
      if (preJustLearnedOrReplaced || unsolicitedHello) {
        if (assign == 0)      cmdSetSceneSide(0, sceneA);
        else                  cmdSetSceneSide(1, sceneB);
        cmdLedAllWhiteOne(src);
        cmdStartLoopAllOne(src);
      }
    }
    return;
  }

  // For everything else, only accept packets from a learned Side MAC.
  const int8_t sid = sideIdFromSrc(src);
  if (sid < 0) return;

  // -------- BTN_EVENT --------
  // payload: [sideId, slot]
  if (type == BTN_EVENT && plen >= 2) {
    // In DIAG mode, any button press advances to the next pattern.
    if (g_diagAudioEnabled) {
      g_diagAdvanceRequested = true;
      return;
    }

    // In AUD mode, button presses are used as control input (do NOT affect lives/score).
    if (g_audEnabled) {
      uint8_t slot = (uint8_t)(payload[1] & 3);
      g_audBtnPos = (uint8_t)((sid * 4) + slot);
      return;
    }


    // Latch only the FIRST button press for the current round.
    //
    // Without this guard, a rapid sequence like "wrong then right" (or two players
    // pressing nearly simultaneously) can overwrite lastSide/lastSlot before the main
    // loop processes it, making life-loss on wrong presses feel inconsistent.
    if (lastSide == 255) {
      lastSide = (uint8_t)sid;
      lastSlot = (uint8_t)(payload[1] & 3);
      DBG_PRINTF("[Master] BTN_EVENT latched side=%u slot=%u\n", (unsigned)lastSide, (unsigned)lastSlot);
    }
    return;
  }

  // -------- OTA_STATUS --------
  // payload: [sideId, code] OR [sideId, OTA_STATUS_PROGRESS, percent]
  if (type == OTA_STATUS && plen >= 2) {
    const char* sideName = (sid==0) ? "Side A" : "Side B";
    uint8_t code = payload[1];

    if (code == OTA_STATUS_PROGRESS && plen >= 3) {
      uint8_t pct = payload[2];
      DBG_PRINTF("[Master] OTA %s: %3u%%\n", sideName, (unsigned)pct);
    } else {
      const char* msg =
        (code==OTA_STATUS_BEGIN)     ? "BEGIN" :
        (code==OTA_STATUS_OK)        ? "OK" :
        (code==OTA_STATUS_FAIL_WIFI) ? "FAIL_WIFI" :
        (code==OTA_STATUS_FAIL_HTTP) ? "FAIL_HTTP" :
        (code==OTA_STATUS_FAIL_UPD)  ? "FAIL_UPDATE" : "UNKNOWN";
      DBG_PRINTF("[Master] OTA %s: %s\n", sideName, msg);
    }
    return;
  }
}

static void nowInit() {
  g_nowChannel = prefsLoadChannel();

  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  esp_wifi_set_promiscuous(true);
  esp_wifi_set_channel(g_nowChannel, WIFI_SECOND_CHAN_NONE);
  esp_wifi_set_promiscuous(false);

  if (esp_now_init() != ESP_OK) {
    DBG_PRINTLN("[NOW] init failed");
    return;
  }
  esp_now_register_recv_cb(onRecv);
  addPeer(BCAST_MAC);
}

// ---------- Arduino ----------
void setup() {
  Serial.begin(115200);
  delay(100);
  DBG_PRINTLN("[Master] Odd One Out (Rounds 1/2/3 + unique-first + lives + shrinking timeout)");

  nowInit();

  uint8_t mac[6];
  esp_wifi_get_mac(WIFI_IF_STA, mac);
  DBG_PRINTF("Master STA MAC: %02X:%02X:%02X:%02X:%02X:%02X\n",
                mac[0],mac[1],mac[2],mac[3],mac[4],mac[5]);

  randomSeed(esp_timer_get_time());

  DBG_PRINTLN("[Master] Manifest summary:");
  for (size_t i=0; i<MASTER_CLIP_COUNT; i++) {
    DBG_PRINTF("  id=%u base=%s sub=%s sub2=%s\n",
                  (unsigned)MASTER_CLIPS[i].id,
                  MASTER_CLIPS[i].base,
                  MASTER_CLIPS[i].sub,
                  MASTER_CLIPS[i].sub2);
  }

  cmdGameMode(true);
}


void loop() {
  // Non-blocking serial processing (PMS + legacy line-based)
  pollSerial();

  // PMS 250ms status tick (suppressed while idle)
  pmsTick();

  // Periodically solicit HELLOs from Sides (boot-order independent)
  helloReqTick();

  // Dedicated audio diagnostic mode (tone patterns)
  if (g_diagAudioEnabled) {
    diagTick();
    return;
  }

  // Sound audition / QA mode (browsing clips + simulating rounds)
  if (g_audEnabled) {
    audTick();
    return;
  }


  switch (g_state) {
    case IDLE:
      break;

    case BUILD: {
      if (g_roundIdx == 0) {
        buildScenes_level1_sub2();
        DBG_PRINTLN("[Master] Using Level 1 (round 1)");
      } else if (g_roundIdx == 1) {
        buildScenes_level2_randomBases();
        DBG_PRINTLN("[Master] Using Level 2 (round 2)");
      } else {
        buildScenes_level3_subs();
        DBG_PRINTLN("[Master] Using Level 3 (round 3 - infinite)");
      }

      // Send the new scenes to each Side (unicast once each Side is discovered).
      // Add a tiny pacing gap so we don't blast both packets back-to-back.
      gameClearRoundPushRetries();
      cmdSetSceneSide(0, sceneA);
      delay(ROUND_PUSH_PACE_MS);
      cmdSetSceneSide(1, sceneB);
      delay(ROUND_PUSH_PACE_MS);

      // Optional software mitigation / diagnostic trims (sent every round so
      // Sides don't get stuck with stale trims if a packet was dropped).
      cmdApplyAudioMitigationForRound();

      DBG_PRINTF("[Master] BUILD done -> ANNOUNCE (curTimeoutMs=%lums)\n",
                 (unsigned long)g_curTimeoutMs);
      g_state = ANNOUNCE;
      break;
    }

    case ANNOUNCE:
      // Push the current round again with pacing. When debug Serial is OFF this
      // prevents one Side from missing SET_SCENE/START due to a too-tight burst.
      gamePushSceneToSidesNow(/*includeWhite*/true);
      lastSide = lastSlot = 255;
      g_waitStartMs = millis();
      // Retry LED_ALL_WHITE a couple times shortly after entering WAIT.
      g_ledWhiteRetries  = LED_WHITE_RETRY_COUNT;
      g_ledWhiteRetryAtMs = g_waitStartMs + LED_WHITE_RETRY_FIRST_DELAY_MS;
      // Also retry scene/start a few times during the first few hundred ms.
      gameScheduleRoundPushRetries();
      DBG_PRINTLN("[Master] ANNOUNCE -> WAIT");
      g_state = WAIT;
      break;

    case WAIT: {
      // Reliability: resend scene/start a few times right after entering WAIT.
      // This fixes the "works only when debug serial is on" symptom where the
      // initial burst is too fast and a Side misses SET_SCENE or START_LOOP_ALL.
      if (g_roundPushRetries && lastSide == 255) {
        const uint32_t nowMs = millis();
        if ((int32_t)(nowMs - g_roundPushRetryAtMs) >= 0) {
          gamePushSceneToSidesNow(/*includeWhite*/false);
          g_roundPushRetries--;
          g_roundPushRetryAtMs = nowMs + ROUND_PUSH_RETRY_GAP_MS;
        }
      }

      // Reliability: resend LED_ALL_WHITE a couple times right after entering WAIT.
      // This helps if a single ESP-NOW packet was missed (no longer relying on HELLO spam).
      if (g_ledWhiteRetries && lastSide == 255) {
        const uint32_t nowMs = millis();
        if ((int32_t)(nowMs - g_ledWhiteRetryAtMs) >= 0) {
          cmdLedAllWhiteSoft();
          g_ledWhiteRetries--;
          g_ledWhiteRetryAtMs = nowMs + LED_WHITE_RETRY_GAP_MS;
        }
      }

      // TIMEOUT = lose a life
      if (millis() - g_waitStartMs > g_curTimeoutMs) {
        cmdStopAll();
        g_ledWhiteRetries = 0;
        gameClearRoundPushRetries();
        if (g_lives > 0) g_lives--;
        DBG_PRINTF("[Master] TIMEOUT -> LIFE LOST (lives=%u)\n", (unsigned)g_lives);

        // PMS event + reason marker
        g_pmsLastReason = "life";
        pmsPrintEventLife(-1, g_lives);

        cmdBlinkAll(/*red*/0, BLINK_ON_MS_WRONG, BLINK_OFF_MS_WRONG);
        resultPauseUntil = millis() + BLINK_REPS * (BLINK_ON_MS_WRONG + BLINK_OFF_MS_WRONG) + 100;

        if (g_lives == 0) {
          DBG_PRINTLN("[Master] OUT OF LIVES -> GAME OVER");
          nextAfterBlink = IDLE;

          // Emit game_end now (before idle silence)
          pmsMaybeEmitGameEnd("no_lives");
        } else {
          nextAfterBlink = BUILD;  // try again, same round/points/timeout
        }
        g_state = PAUSE;
        break;
      }

      if (lastSide != 255) {
        DBG_PRINTF("[Master] PICK side=%u slot=%u\n", lastSide, lastSlot);
        cmdStopAll();
        g_ledWhiteRetries = 0;
        gameClearRoundPushRetries();

        bool correct = (lastSide==0) ? slotIsOdd_A[lastSlot & 3]
                                     : slotIsOdd_B[lastSlot & 3];

        if (correct) {
          DBG_PRINTLN("[Master] PICK -> CORRECT");
          cmdBlinkAll(/*green*/1, BLINK_ON_MS_CORRECT, BLINK_OFF_MS_CORRECT);
          resultPauseUntil = millis() + BLINK_REPS * (BLINK_ON_MS_CORRECT + BLINK_OFF_MS_CORRECT) + 100;

          // PMS score event
          g_scoreTotal++;
          g_pmsLastReason = "score";
          pmsPrintEventScore(1, g_scoreTotal);

          // SPEED UP timeout after each correct
          uint32_t newTimeout = (uint32_t)(g_curTimeoutMs * TIME_DECAY_FACTOR);
          if (newTimeout < MIN_TIMEOUT_MS) newTimeout = MIN_TIMEOUT_MS;
          g_curTimeoutMs = newTimeout;
          DBG_PRINTF("[Master] Timeout decayed to %lums\n", (unsigned long)g_curTimeoutMs);

          // Round progression logic
          if (++g_pointsInRound >= 3) {
            g_pointsInRound = 0;

            if (g_roundIdx < 2) {
              // Finished round 1 or 2 -> go to next round, reset timeout for that round
              g_roundIdx++;
              g_curTimeoutMs = BASE_TIMEOUT_MS[g_roundIdx];
              DBG_PRINTF("[Master] Round %u complete -> next round (timeout reset to %lums)\n",
                         (unsigned)g_roundIdx, (unsigned long)g_curTimeoutMs);
              nextAfterBlink = BUILD;
            } else {
              // Round 3 is infinite: stay in round 3, don't "win" by points
              DBG_PRINTLN("[Master] Round 3: correct point, staying in infinite round");
              nextAfterBlink = BUILD;
            }
          } else {
            DBG_PRINTF("[Master] Point %u in current round\n", (unsigned)g_pointsInRound);
            nextAfterBlink = BUILD;
          }
          g_state = PAUSE;

        } else {
          // WRONG PICK -> lose a life
          if (g_lives > 0) g_lives--;
          DBG_PRINTF("[Master] PICK -> WRONG (lives=%u)\n", (unsigned)g_lives);

          // PMS life event + reason marker
          g_pmsLastReason = "life";
          pmsPrintEventLife(-1, g_lives);

          // Wrong press: blink all wrong (red) and reveal the correct one (green).
          cmdBlinkRevealCorrect(BLINK_ON_MS_WRONG, BLINK_OFF_MS_WRONG);
          resultPauseUntil = millis() + BLINK_REPS * (BLINK_ON_MS_WRONG + BLINK_OFF_MS_WRONG) + 100;

          if (g_lives == 0) {
            DBG_PRINTLN("[Master] OUT OF LIVES -> GAME OVER");
            nextAfterBlink = IDLE;

            // Emit game_end now (before idle silence)
            pmsMaybeEmitGameEnd("no_lives");
          } else {
            nextAfterBlink = BUILD;  // same round, same points, same timeout
          }
          g_state = PAUSE;
        }
      }
    } break;

    case PAUSE:
      if (millis() >= resultPauseUntil) {
        DBG_PRINTF("[Master] PAUSE done -> %s\n",
                   (nextAfterBlink==IDLE) ? "IDLE" : "BUILD");
        g_state = nextAfterBlink;
      }
      break;
  }
}


