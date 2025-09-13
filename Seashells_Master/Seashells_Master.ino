/*
  Seashells Master – "Odd One Out" (robust)
  - ESP32 (Feather S3 ok)
  - Talks to two Sides over ESP-NOW
  - Requests random sets with retries, validates replies, then assigns scenes
  - Announce stage loops until selection
*/

#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <cstring>        // std::memcmp

#include "Messages.h"     // MUST match the Sides (enum values)
#include "ConfigMaster.h" // WIFI_CHANNEL, SIDE_A_MAC, SIDE_B_MAC

// ---------- Tuning ----------
static const uint8_t  REQ_RETRIES   = 4;      // how many times to resend REQUEST_RANDOM_SET
static const uint16_t REQ_GAP_MS    = 120;    // ms between retries
static const uint16_t PICKS_WAIT_MS = 2000;   // overall wait window for replies (ms)
static const uint32_t PICK_WINDOW_MS[3] = {30000, 30000, 30000}; // R1,R2,R3 time limits

// ---------- Types ----------
struct Picks {
  uint8_t  nA = 0, nB = 0;
  uint16_t A[4] {0,0,0,0};
  uint16_t B[4] {0,0,0,0};
  bool     have = false;
};

// ---------- Globals ----------
enum State { IDLE, BUILD, ANNOUNCE, WAIT, PAUSE };
static State g_state = IDLE;

// Blink cadence (match what you send in cmdBlinkAll)
static const uint8_t  BLINK_REPS              = 3;
static const uint16_t BLINK_ON_MS_CORRECT     = 140;
static const uint16_t BLINK_OFF_MS_CORRECT    = 120;
static const uint16_t BLINK_ON_MS_WRONG       = 160;
static const uint16_t BLINK_OFF_MS_WRONG      = 140;

// Pause bookkeeping
static uint32_t resultPauseUntil = 0;
static State    nextAfterBlink   = IDLE;

static Picks picksA, picksB;                      // replies from Side A/B
static volatile uint8_t lastSide = 255, lastSlot = 255;  // BTN_EVENT inbox

// Current scene (what each side will play) + "odd" markers
static uint16_t sceneA[4] {0,0,0,0};
static uint16_t sceneB[4] {0,0,0,0};
static bool     slotIsOdd_A[4] {false,false,false,false};
static bool     slotIsOdd_B[4] {false,false,false,false};

// ---------- Helpers ----------
static void addPeer(const uint8_t mac[6]) {
  esp_now_peer_info_t p{};
  std::memcpy(p.peer_addr, mac, 6);
  p.channel = WIFI_CHANNEL;
  p.encrypt = false;
  esp_now_add_peer(&p);
}

static void sendPkt(const uint8_t mac[6], const void* data, size_t n) {
  // Send twice for resilience on noisy RF
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
  uint8_t m[1 + 8]; // type + 4*uint16
  m[0] = SET_SCENE;
  for (int i=0;i<4;i++) {
    m[1 + i*2] = (uint8_t)(ids[i] >> 8);
    m[2 + i*2] = (uint8_t)(ids[i] & 0xFF);
  }
  sendPkt(mac, m, sizeof(m));
}

static void endGame() {
  cmdStopAll();         // stop announce loops
  g_state = IDLE;
  Serial.println("[Master] Game ended -> IDLE");
}

static void shuffle4(uint16_t v[4]) {
  for (int i=0;i<8;i++) { int a = random(4), b = random(4); uint16_t t = v[a]; v[a] = v[b]; v[b] = t; }
}
static bool containsN(const uint16_t arr[4], uint8_t n, uint16_t val) {
  for (uint8_t i=0;i<n;i++) if (arr[i] == val) return true;
  return false;
}

// ---------- ESP-NOW ----------
static void onRecv(const esp_now_recv_info_t* info, const uint8_t* data, int len) {
  if (!info || !data || len < 1) return;
  const uint8_t type = data[0];
  const bool isA = (std::memcmp(info->src_addr, SIDE_A_MAC, 6) == 0);

  if (type == HELLO && len >= 6) {
    const bool isA = (std::memcmp(info->src_addr, SIDE_A_MAC, 6) == 0);
    const uint8_t* mac = info->src_addr;

    // Assign/confirm role and force game mode for THIS side
    cmdRoleAssign(mac, isA ? 0 : 1);
    cmdGameModeOne(mac, true);

    // If we’re mid-round, re-sync ONLY this side
    if (g_state == ANNOUNCE || g_state == WAIT) {
      cmdSetScene(mac, isA ? sceneA : sceneB);
      cmdLedAllWhiteOne(mac);
      cmdStartLoopAllOne(mac);
    }
    return;
  }

  if (type == BTN_EVENT) {
    if (len >= 3) { lastSide = data[1]; lastSlot = data[2]; }
    return;
  }

  if (type == RANDOM_SET_REPLY) {
    // Side sends: type(1) + nA(1) + nB(1) + 4*A(2B ea, padded) + 4*B(2B ea, padded) = 19
    if (len < 19) return; // too short -> ignore
    Picks &p = isA ? picksA : picksB;
    p.nA = data[1];
    p.nB = data[2];
    int idx = 3;
    for (uint8_t i=0;i<4;i++) { p.A[i] = (uint16_t)data[idx] << 8 | data[idx+1]; idx += 2; }
    for (uint8_t i=0;i<4;i++) { p.B[i] = (uint16_t)data[idx] << 8 | data[idx+1]; idx += 2; }
    p.have = true;
    Serial.printf("[Master] RANDOM_SET_REPLY from %s  nA=%u nB=%u\n",
                  isA ? "Side A" : "Side B", p.nA, p.nB);
    return;
  }

  if (type == OTA_STATUS && len >= 3) {
    const char* sideName = (data[1]==0) ? "Side A" : (data[1]==1 ? "Side B" : "Side ?");
    uint8_t code = data[2];

    if (code == OTA_STATUS_PROGRESS && len >= 4) {
      uint8_t pct = data[3];
      Serial.printf("[Master] OTA %s: %3u%%\n", sideName, pct);
    } else {
      const char* msg =
        (code==OTA_STATUS_BEGIN)     ? "BEGIN" :
        (code==OTA_STATUS_OK)        ? "OK" :
        (code==OTA_STATUS_FAIL_WIFI) ? "FAIL_WIFI" :
        (code==OTA_STATUS_FAIL_HTTP) ? "FAIL_HTTP" :
        (code==OTA_STATUS_FAIL_UPD)  ? "FAIL_UPDATE" : "UNKNOWN";
      Serial.printf("[Master] OTA %s: %s\n", sideName, msg);
    }
    return;
  }

}

static void nowInit() {
  WiFi.mode(WIFI_STA);
  // Set a fixed channel for all three devices
  esp_wifi_set_promiscuous(true);
  esp_wifi_set_channel(WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE);
  esp_wifi_set_promiscuous(false);

  if (esp_now_init() != ESP_OK) {
    Serial.println("[NOW] init failed");
    return;
  }
  esp_now_register_recv_cb(onRecv);
  addPeer(SIDE_A_MAC);
  addPeer(SIDE_B_MAC);
}

// ---------- Picks orchestration ----------
static void sendReqRandom(uint8_t sideOdd) {
  const uint8_t reqOdd[3]  = { REQUEST_RANDOM_SET, 3, 1 }; // need 3×A + 1×B
  const uint8_t reqNorm[3] = { REQUEST_RANDOM_SET, 4, 0 }; // need 4×A
  if (sideOdd == 0) {
    sendPkt(SIDE_A_MAC, reqOdd,  3);
    sendPkt(SIDE_B_MAC, reqNorm, 3);
  } else {
    sendPkt(SIDE_A_MAC, reqNorm, 3);
    sendPkt(SIDE_B_MAC, reqOdd,  3);
  }
}

static bool countsOK(uint8_t sideOdd) {
  if (sideOdd == 0) return (picksA.nA >= 3 && picksA.nB >= 1 && picksB.nA >= 4);
  else               return (picksB.nA >= 3 && picksB.nB >= 1 && picksA.nA >= 4);
}

static bool getRandomSets(uint8_t sideOdd) {
  picksA = Picks(); picksB = Picks();

  const uint32_t start = millis();
  // Retry loop
  for (uint8_t attempt = 0; attempt < REQ_RETRIES; ++attempt) {
    sendReqRandom(sideOdd);
    uint32_t t0 = millis();
    while (millis() - t0 < REQ_GAP_MS) {
      if (picksA.have && picksB.have) break;
      delay(2);
    }
    if (picksA.have && picksB.have) break;
  }
  // Final wait up to window
  while (!(picksA.have && picksB.have) && (millis() - start < PICKS_WAIT_MS)) {
    delay(5);
  }

  if (!(picksA.have && picksB.have)) {
    Serial.println("[Master] picks timeout; no replies from one/both sides");
    return false;
  }
  if (!countsOK(sideOdd)) {
    Serial.printf("[Master] picks present but insufficient clips for sideOdd=%u (need 3A+1B on odd, 4A on other)\n",
                  (unsigned)sideOdd);
    return false;
  }
  return true;
}

static void cmdOtaUpdate(const uint8_t mac[6], const char* url) {
  const size_t ulen = strnlen(url, 200);
  uint8_t m[1 + 1 + 200]; // type + len + url (<=200)
  m[0] = OTA_UPDATE;
  m[1] = (uint8_t)ulen;
  memcpy(m+2, url, ulen);
  sendPkt(mac, m, 2 + ulen);
}

// ---------- Arduino ----------
void setup() {
  Serial.begin(115200);
  delay(100);
  Serial.println("[Master] Odd One Out");

  nowInit();

  // Print Master STA MAC so you can set it on the Sides as MASTER_MAC
  uint8_t mac[6];
  esp_wifi_get_mac(WIFI_IF_STA, mac);
  Serial.printf("Master STA MAC: %02X:%02X:%02X:%02X:%02X:%02X\n",
                mac[0],mac[1],mac[2],mac[3],mac[4],mac[5]);

  randomSeed(esp_timer_get_time());

  // Put Sides into game mode (buttons report only; no local auto-play)
  cmdGameMode(true);
}

void loop() {
  static uint8_t roundIdx = 0, points = 0, sideOdd = 0;
  static uint32_t t0 = 0;

  if (Serial.available()) {
    char c = Serial.read();
    if (c=='s') { 
      Serial.println("Game start");
      points = 0;
      roundIdx = 0;
      cmdLedAllWhite(); 
      g_state = BUILD; 
    }
    else if (c=='e') { endGame(); }
    else if (c=='u') { // update both
      Serial.println("[Master] OTA both sides");
      cmdOtaUpdate(SIDE_A_MAC, OTA_URL_SIDE_BIN);
      delay(200);
      cmdOtaUpdate(SIDE_B_MAC, OTA_URL_SIDE_BIN);
    }
    else if (c=='a') { cmdOtaUpdate(SIDE_A_MAC, OTA_URL_SIDE_BIN); }
    else if (c=='b') { cmdOtaUpdate(SIDE_B_MAC, OTA_URL_SIDE_BIN); }
  }

  switch (g_state) {
    case IDLE:
      break;

    case BUILD: {
      sideOdd = random(2);

      if (!getRandomSets(sideOdd)) {
        // Try swapping the odd side once before giving up
        uint8_t alt = sideOdd ^ 1;
        Serial.println("[Master] retry with swapped odd side");
        if (!getRandomSets(alt)) {
          Serial.println("[Master] picks timeout; retry");
          break; // stay in BUILD; loop will try again
        }
        sideOdd = alt;
      }

      // Build scenes
      if (sideOdd == 0) {
        // A: 3×A then 1×B
        uint8_t ai=0, bi=0;
        for (int i=0;i<4;i++) sceneA[i] = (i < 3) ? picksA.A[ai++] : picksA.B[bi++];
        // B: 4×A
        for (int i=0;i<4;i++) sceneB[i] = picksB.A[i];
      } else {
        // A: 4×A
        for (int i=0;i<4;i++) sceneA[i] = picksA.A[i];
        // B: 3×A then 1×B
        uint8_t ai=0, bi=0;
        for (int i=0;i<4;i++) sceneB[i] = (i < 3) ? picksB.A[ai++] : picksB.B[bi++];
      }

      // Shuffle per side for random button placement
      shuffle4(sceneA);
      shuffle4(sceneB);

      // Mark which slots are "odd" (pool B) for judging
      for (int i=0;i<4;i++) slotIsOdd_A[i] = containsN(picksA.B, picksA.nB, sceneA[i]);
      for (int i=0;i<4;i++) slotIsOdd_B[i] = containsN(picksB.B, picksB.nB, sceneB[i]);

      // Send assignments
      cmdSetScene(SIDE_A_MAC, sceneA);
      cmdSetScene(SIDE_B_MAC, sceneB);

      g_state = ANNOUNCE;
      break;
    }

    case ANNOUNCE:
      // Lights white; loop all slots continuously until selection or timeout
      cmdStartLoopAll();
      cmdLedAllWhite();
      lastSide = lastSlot = 255;
      t0 = millis();
      g_state = WAIT;
      break;

    case WAIT: {
      // Timeout -> lose
      if (millis() - t0 > PICK_WINDOW_MS[roundIdx]) {
        cmdStopAll();
        cmdBlinkAll(/*red*/0, BLINK_ON_MS_WRONG, BLINK_OFF_MS_WRONG);
        resultPauseUntil = millis() + BLINK_REPS * (BLINK_ON_MS_WRONG + BLINK_OFF_MS_WRONG) + 100;
        nextAfterBlink   = IDLE;
        g_state          = PAUSE;
        break;
      }

      if (lastSide != 255) {
        cmdStopAll(); // stop announce loops
        bool correct = (lastSide==0) ? slotIsOdd_A[lastSlot & 3] : slotIsOdd_B[lastSlot & 3];

        if (correct) {
          cmdBlinkAll(/*green*/1, BLINK_ON_MS_CORRECT, BLINK_OFF_MS_CORRECT);
          resultPauseUntil = millis() + BLINK_REPS * (BLINK_ON_MS_CORRECT + BLINK_OFF_MS_CORRECT) + 100;

          // Decide what comes AFTER the blink:
          if (++points >= 3) {
            points = 0;
            if (++roundIdx >= 3) {
              // Win (keep your white-win blink if you like; if you do, recalc pause for that cadence)
              // cmdBlinkAll(/*white*/2, 220, 120);
              // resultPauseUntil = millis() + BLINK_REPS * (220 + 120) + 120;
              nextAfterBlink = IDLE;
            } else {
              nextAfterBlink = BUILD;  // next round (shorter timer)
            }
          } else {
            nextAfterBlink = BUILD;    // next point in same round
          }
          g_state = PAUSE;

        } else {
          cmdBlinkAll(/*red*/0, BLINK_ON_MS_WRONG, BLINK_OFF_MS_WRONG);
          resultPauseUntil = millis() + BLINK_REPS * (BLINK_ON_MS_WRONG + BLINK_OFF_MS_WRONG) + 100;
          nextAfterBlink   = IDLE;
          g_state          = PAUSE;
        }
      }
    } break;

    case PAUSE:
      if (millis() >= resultPauseUntil) {
        g_state = nextAfterBlink;
      }
      break;
  }
}
