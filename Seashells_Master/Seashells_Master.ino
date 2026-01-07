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
// Legacy serial (for manual tech/debug use):
//   - This sketch used to react to single incoming characters ('s','e','u','a','b').
//   - To prevent accidental triggers from PMS traffic (e.g., the 'e' in "level="),
//     legacy commands are now line-based:
//       "s" = start, "e" = end, "u" = OTA both, "a" = OTA side A, "b" = OTA side B
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
#include <cstring>

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
#define PMS_DEBUG_SERIAL 1
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
static const uint8_t  BLINK_REPS              = 3;
static const uint16_t BLINK_ON_MS_CORRECT     = 140;
static const uint16_t BLINK_OFF_MS_CORRECT    = 120;
static const uint16_t BLINK_ON_MS_WRONG       = 160;
static const uint16_t BLINK_OFF_MS_WRONG      = 140;

// Pause bookkeeping
static uint32_t resultPauseUntil = 0;
static State    nextAfterBlink   = IDLE;

static volatile uint8_t lastSide = 255, lastSlot = 255;

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

static void pmsPrintPong() {
#if PMS_STD_ENABLED
  Serial.println(F("!PMS PONG v=1 game=seashells role=server"));
#endif
}

static void pmsPrintEventGameStart(uint8_t level) {
#if PMS_STD_ENABLED
  Serial.print(F("!PMS EVENT v=1 name=game_start level="));
  Serial.println(level);
#endif
}

static void pmsPrintEventGameEnd(const char* reason, uint16_t score, uint8_t lives) {
#if PMS_STD_ENABLED
  Serial.print(F("!PMS EVENT v=1 name=game_end reason="));
  Serial.print(reason);
  Serial.print(F(" score="));
  Serial.print(score);
  Serial.print(F(" lives="));
  Serial.println(lives);
#endif
}

static void pmsPrintEventScore(int32_t delta, uint16_t total) {
#if PMS_STD_ENABLED
  Serial.print(F("!PMS EVENT v=1 name=score delta="));
  Serial.print(delta);
  Serial.print(F(" total="));
  Serial.print(total);
  Serial.println(F(" bonus=0"));
#endif
}

static void pmsPrintEventLife(int32_t delta, uint8_t lives) {
#if PMS_STD_ENABLED
  Serial.print(F("!PMS EVENT v=1 name=life delta="));
  Serial.print(delta);
  Serial.print(F(" lives="));
  Serial.println(lives);
#endif
}

static void pmsPrintStatus(PmsState st, uint8_t level, uint16_t score, uint8_t lives, uint32_t tleftMs, const char* lastReason) {
#if PMS_STD_ENABLED
  Serial.print(F("!PMS STATUS v=1 state="));
  Serial.print(pmsStateStr(st));
  Serial.print(F(" level="));
  Serial.print(level);
  Serial.print(F(" score="));
  Serial.print(score);
  Serial.print(F(" lives="));
  Serial.print(lives);
  Serial.print(F(" tleft_ms="));
  Serial.print(tleftMs);
  Serial.print(F(" last_reason="));
  Serial.println(lastReason);
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

static void handleLegacyLine(const String& rawLine) {
  String line = rawLine;
  line.trim();
  if (line.length() == 0) return;

  // Legacy commands are line-based to avoid accidental triggers from PMS traffic.
  if (line.length() == 1) {
    char c = line.charAt(0);
    if (c >= 'A' && c <= 'Z') c = (char)(c - 'A' + 'a');

    if (c == 's') {
      startGameAtLevel(1);
      return;
    }
    if (c == 'e') {
      stopGameFromHost("stopped");
      return;
    }
    if (c == 'u') {
      DBG_PRINTLN("[Master] OTA both sides");
      cmdOtaUpdate(SIDE_A_MAC, OTA_URL_SIDE_BIN);
      delay(200);
      cmdOtaUpdate(SIDE_B_MAC, OTA_URL_SIDE_BIN);
      return;
    }
    if (c == 'a') {
      cmdOtaUpdate(SIDE_A_MAC, OTA_URL_SIDE_BIN);
      return;
    }
    if (c == 'b') {
      cmdOtaUpdate(SIDE_B_MAC, OTA_URL_SIDE_BIN);
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



// ---------- ESP-NOW helpers ----------
static void addPeer(const uint8_t mac[6]) {
  esp_now_peer_info_t p{};
  std::memcpy(p.peer_addr, mac, 6);
  p.channel = WIFI_CHANNEL;
  p.encrypt = false;
  esp_now_add_peer(&p);
}

static void sendPkt(const uint8_t mac[6], const void* data, size_t n) {
  esp_now_send(mac, (const uint8_t*)data, n);
}

static void cmdRoleAssign(const uint8_t mac[6], uint8_t sideId) {
  uint8_t m[2] = { ROLE_ASSIGN, (uint8_t)(sideId&1) };
  sendPkt(mac, m, sizeof(m));
}

static void cmdGameModeOne(const uint8_t mac[6], bool en){
  uint8_t m[2] = { GAME_MODE, (uint8_t)(en ? 1 : 0) };
  sendPkt(mac, m, sizeof(m));
}
static void cmdLedAllWhiteOne(const uint8_t mac[6]){
  uint8_t m = LED_ALL_WHITE;
  sendPkt(mac, &m, 1);
}
static void cmdStartLoopAllOne(const uint8_t mac[6]){
  uint8_t m = START_LOOP_ALL;
  sendPkt(mac, &m, 1);
}

static void cmdGameMode(bool en){
  uint8_t m[2] = { GAME_MODE, (uint8_t)(en ? 1 : 0) };
  sendPkt(SIDE_A_MAC, m, sizeof(m));
  sendPkt(SIDE_B_MAC, m, sizeof(m));
}
static void cmdLedAllWhite(){
  uint8_t m = LED_ALL_WHITE;
  sendPkt(SIDE_A_MAC, &m, 1);
  sendPkt(SIDE_B_MAC, &m, 1);
}
static void cmdBlinkAll(uint8_t color, uint16_t on_ms, uint16_t off_ms){
  uint8_t m[6] = { BLINK_ALL,
                   color,
                   (uint8_t)(on_ms >> 8), (uint8_t)on_ms,
                   (uint8_t)(off_ms >> 8), (uint8_t)off_ms };
  sendPkt(SIDE_A_MAC, m, sizeof(m));
  sendPkt(SIDE_B_MAC, m, sizeof(m));
}
static void cmdStartLoopAll(){
  uint8_t m = START_LOOP_ALL;
  sendPkt(SIDE_A_MAC, &m, 1);
  sendPkt(SIDE_B_MAC, &m, 1);
}
static void cmdStopAll(){
  uint8_t m = STOP_ALL;
  sendPkt(SIDE_A_MAC, &m, 1);
  sendPkt(SIDE_B_MAC, &m, 1);
}
static void cmdSetScene(const uint8_t mac[6], const uint16_t ids[4]){
  uint8_t m[1 + 8];
  m[0] = SET_SCENE;
  for (int i=0;i<4;i++) {
    m[1 + i*2] = (uint8_t)(ids[i] >> 8);
    m[2 + i*2] = (uint8_t)(ids[i] & 0xFF);
  }
  sendPkt(mac, m, sizeof(m));
}

static void endGame() {
  cmdStopAll();
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

// ---------- Category helper functions ----------

// Collect unique base strings
static size_t collectUniqueBases(const char* out[], size_t maxOut) {
  size_t n = 0;
  for (size_t i = 0; i < MASTER_CLIP_COUNT; i++) {
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
    if (strcasecmp(MASTER_CLIPS[i].base, base) != 0) continue;
    if (n < maxOut) out[n++] = MASTER_CLIPS[i].id;
  }
  return n;
}

// Collect all IDs matching base+sub
static size_t collectIdsByBaseSub(const char* base, const char* sub, uint16_t* out, size_t maxOut) {
  size_t n = 0;
  for (size_t i=0; i<MASTER_CLIP_COUNT; i++) {
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
    size_t idx = random((long)MASTER_CLIP_COUNT);
    return MASTER_CLIPS[idx].id;
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
    for (int i=0;i<7;i++) sameIds[i] = MASTER_CLIPS[random((long)MASTER_CLIP_COUNT)].id;
  } else {
    fillWithUniqueThenReuse(sameIds, 7, unique, uCount, "Level2 baseMain");
  }

  uint16_t oddId;
  uint16_t uniqueOdd[32];
  size_t uOddCount = collectIdsByBase(baseOdd, uniqueOdd, 32);
  if (uOddCount == 0) {
    oddId = MASTER_CLIPS[random((long)MASTER_CLIP_COUNT)].id;
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
    oddId = MASTER_CLIPS[random((long)MASTER_CLIP_COUNT)].id;
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
  uint8_t m[1 + 1 + 200];
  m[0] = OTA_UPDATE;
  m[1] = (uint8_t)ulen;
  memcpy(m+2, url, ulen);
  sendPkt(mac, m, 2 + ulen);
}

// ---------- ESP-NOW ----------
static void onRecv(const esp_now_recv_info_t* info, const uint8_t* data, int len) {
  if (!info || !data || len < 1) return;
  const uint8_t type = data[0];
  const bool isA = (std::memcmp(info->src_addr, SIDE_A_MAC, 6) == 0);

  if (type == HELLO && len >= 6) {
    const uint8_t* mac = info->src_addr;

    DBG_PRINTF("[Master] HELLO from %s sideId=%u poolA=%u poolB=%u\n",
                  isA ? "Side A" : "Side B",
                  data[1],
                  (uint16_t)(data[2] << 8 | data[3]),
                  (uint16_t)(data[4] << 8 | data[5]));

    cmdRoleAssign(mac, isA ? 0 : 1);
    cmdGameModeOne(mac, true);

    if (g_state == ANNOUNCE || g_state == WAIT) {
      cmdSetScene(mac, isA ? sceneA : sceneB);
      cmdLedAllWhiteOne(mac);
      cmdStartLoopAllOne(mac);
    }
    return;
  }

  if (type == BTN_EVENT) {
    if (len >= 3) {
      lastSide = data[1];
      lastSlot = data[2];
      DBG_PRINTF("[Master] BTN_EVENT side=%u slot=%u\n", lastSide, lastSlot);
    }
    return;
  }

  if (type == OTA_STATUS && len >= 3) {
    const char* sideName = (data[1]==0) ? "Side A" : (data[1]==1 ? "Side B" : "Side ?");
    uint8_t code = data[2];

    if (code == OTA_STATUS_PROGRESS && len >= 4) {
      uint8_t pct = data[3];
      DBG_PRINTF("[Master] OTA %s: %3u%%\n", sideName, pct);
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
  WiFi.mode(WIFI_STA);
  esp_wifi_set_promiscuous(true);
  esp_wifi_set_channel(WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE);
  esp_wifi_set_promiscuous(false);

  if (esp_now_init() != ESP_OK) {
    DBG_PRINTLN("[NOW] init failed");
    return;
  }
  esp_now_register_recv_cb(onRecv);
  addPeer(SIDE_A_MAC);
  addPeer(SIDE_B_MAC);
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

      cmdSetScene(SIDE_A_MAC, sceneA);
      cmdSetScene(SIDE_B_MAC, sceneB);

      DBG_PRINTF("[Master] BUILD done -> ANNOUNCE (curTimeoutMs=%lums)\n",
                 (unsigned long)g_curTimeoutMs);
      g_state = ANNOUNCE;
      break;
    }

    case ANNOUNCE:
      cmdStartLoopAll();
      cmdLedAllWhite();
      lastSide = lastSlot = 255;
      g_waitStartMs = millis();
      DBG_PRINTLN("[Master] ANNOUNCE -> WAIT");
      g_state = WAIT;
      break;

    case WAIT: {
      // TIMEOUT = lose a life
      if (millis() - g_waitStartMs > g_curTimeoutMs) {
        cmdStopAll();
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

          cmdBlinkAll(/*red*/0, BLINK_ON_MS_WRONG, BLINK_OFF_MS_WRONG);
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


