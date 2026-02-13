#include <cstring>
#include <esp_now.h>
#include <Preferences.h>

#include "GameBusSide.h"
#include "Manifest.h"
#include "Role.h"
#include "OtaUpdate.h"

// Externs implemented in the .ino (audio/led functions)
extern void side_setScene(uint16_t ids[4]);
extern void side_playSlot(uint8_t slot);
extern void side_ledAllWhite();
// Like side_ledAllWhite(), but does NOT force an "all off" frame first.
// Used for reliability refreshes during WAIT without visible flicker.
extern void side_ledAllWhiteSoft();
extern void side_blinkAll(uint8_t color, uint16_t on_ms, uint16_t off_ms);
extern void side_setGameMode(bool en);
extern void side_startLoopAll();
extern void side_stopAll();

// ─────────────────────────────────────────────────────────────────────────────
// IMPORTANT: ESP-NOW receive callbacks run in the WiFi task context.
// Doing SD I/O, NeoPixel .show(), or other heavy work inside the callback can
// cause random glitches (including missed LED updates).
//
// Fix: queue inbound commands in the callback, then process them in the main
// Arduino loop via GameBus_pump().
// ─────────────────────────────────────────────────────────────────────────────

static constexpr uint8_t kCmdQSize      = 16;
static constexpr uint8_t kCmdMaxPayload = 210; // enough for OTA url packets

static const uint8_t BCAST_MAC[6] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};

// NVS (Preferences) keys — shared namespace with Role.h
static constexpr const char* kPrefsNs      = "seashells";
static constexpr const char* kKeyChan      = "chan";   // u8
static constexpr const char* kKeyMasterMac = "m_mac";  // 6 bytes

static uint8_t  s_nowChannel = NOW_DEFAULT_CHANNEL;
static uint8_t  s_masterMac[6] = {0};
static bool     s_masterKnown  = false;

// Remember last "HELLO" pool counts so we can answer HELLO_REQ even if the
// main sketch only sent HELLO once at boot.
static uint16_t s_poolA_last = 0;
static uint16_t s_poolB_last = 0;

static inline bool macIsAllZero(const uint8_t mac[6]) {
  for (int i=0;i<6;i++) if (mac[i] != 0) return false;
  return true;
}

static inline void prefsLoadRadio() {
  Preferences p;
  p.begin(kPrefsNs, true);
  uint8_t ch = p.getUChar(kKeyChan, NOW_DEFAULT_CHANNEL);
  if (ch < 1 || ch > 13) ch = NOW_DEFAULT_CHANNEL;
  s_nowChannel = ch;

  uint8_t tmp[6] = {0};
  size_t n = p.getBytes(kKeyMasterMac, tmp, sizeof(tmp));
  if (n == 6 && !macIsAllZero(tmp)) {
    memcpy(s_masterMac, tmp, 6);
    s_masterKnown = true;
  } else {
    memset(s_masterMac, 0, 6);
    s_masterKnown = false;
  }
  p.end();
}

static inline void prefsSaveChannel(uint8_t ch) {
  Preferences p;
  p.begin(kPrefsNs, false);
  p.putUChar(kKeyChan, ch);
  p.end();
}

static inline void prefsSaveMasterMac(const uint8_t mac[6]) {
  Preferences p;
  p.begin(kPrefsNs, false);
  p.putBytes(kKeyMasterMac, mac, 6);
  p.end();
}

static inline void ensurePeer(const uint8_t mac[6]) {
  esp_now_peer_info_t p{};
  memcpy(p.peer_addr, mac, 6);
  p.channel = s_nowChannel;
  p.encrypt = false;
  // ignore errors (peer may already exist)
  (void)esp_now_add_peer(&p);
}

static inline void setMasterIfNeeded(const uint8_t mac[6], bool persist) {
  if (!mac || macIsAllZero(mac)) return;
  if (s_masterKnown && memcmp(s_masterMac, mac, 6) == 0) return;

  memcpy(s_masterMac, mac, 6);
  s_masterKnown = true;
  ensurePeer(s_masterMac);
  if (persist) prefsSaveMasterMac(s_masterMac);

  Serial.printf("[NOW] Master learned: %02X:%02X:%02X:%02X:%02X:%02X (ch=%u)\n",
                s_masterMac[0],s_masterMac[1],s_masterMac[2],
                s_masterMac[3],s_masterMac[4],s_masterMac[5],
                (unsigned)s_nowChannel);
}

static inline const uint8_t* destMasterOrBcast() {
  return s_masterKnown ? s_masterMac : BCAST_MAC;
}

static inline void sendFramed(const uint8_t mac[6], uint8_t type, const uint8_t* payload, int plen) {
  // Enough for our largest packet (OTA URL): header(4) + url_len(1) + url(200) = 205
  uint8_t buf[4 + 1 + 210];
  int n = SS_build(buf, (int)sizeof(buf), type, payload, plen);
  if (n > 0) {
    esp_now_send(mac, buf, (size_t)n);
  }
}

struct CmdMsg {
  uint8_t type = 0;
  uint8_t len  = 0;
  uint8_t payload[kCmdMaxPayload];
};

static CmdMsg cmdQ[kCmdQSize];
static volatile uint8_t qHead = 0;
static volatile uint8_t qTail = 0;
static portMUX_TYPE qMux = portMUX_INITIALIZER_UNLOCKED;

// Pending master switch (set in WiFi callback, applied in GameBus_pump)
static uint8_t  s_pendingMasterMac[6] = {0};
static bool     s_pendingMasterValid  = false;
static bool     s_pendingMasterPersist= false;

static inline void pendingSetMaster(const uint8_t mac[6], bool persist) {
  if (!mac) return;
  portENTER_CRITICAL(&qMux);
  memcpy(s_pendingMasterMac, mac, 6);
  s_pendingMasterValid   = true;
  s_pendingMasterPersist = persist;
  portEXIT_CRITICAL(&qMux);
}

static inline bool pendingTakeMaster(uint8_t outMac[6], bool& outPersist) {
  bool ok = false;
  portENTER_CRITICAL(&qMux);
  if (s_pendingMasterValid) {
    memcpy(outMac, s_pendingMasterMac, 6);
    outPersist = s_pendingMasterPersist;
    s_pendingMasterValid = false;
    s_pendingMasterPersist = false;
    ok = true;
  }
  portEXIT_CRITICAL(&qMux);
  return ok;
}

static inline void applyPendingMaster() {
  uint8_t mac[6];
  bool persist = false;
  if (pendingTakeMaster(mac, persist)) {
    setMasterIfNeeded(mac, persist);
  }
}


static inline void qPush(uint8_t type, const uint8_t* payload, uint8_t len) {
  if (len > kCmdMaxPayload) len = kCmdMaxPayload;

  portENTER_CRITICAL(&qMux);
  uint8_t next = (uint8_t)((qHead + 1) % kCmdQSize);

  // If full, drop the oldest entry (keeps newest state like SET_SCENE)
  if (next == qTail) {
    qTail = (uint8_t)((qTail + 1) % kCmdQSize);
  }

  cmdQ[qHead].type = type;
  cmdQ[qHead].len  = len;
  if (len && payload) {
    memcpy(cmdQ[qHead].payload, payload, len);
  }
  qHead = next;
  portEXIT_CRITICAL(&qMux);
}

static inline bool qPop(CmdMsg& out) {
  portENTER_CRITICAL(&qMux);
  if (qTail == qHead) {
    portEXIT_CRITICAL(&qMux);
    return false;
  }

  out.type = cmdQ[qTail].type;
  out.len  = cmdQ[qTail].len;
  if (out.len) memcpy(out.payload, cmdQ[qTail].payload, out.len);

  qTail = (uint8_t)((qTail + 1) % kCmdQSize);
  portEXIT_CRITICAL(&qMux);
  return true;
}

void GameBus_sendOtaStatus(uint8_t code) {
  uint8_t sid = (Role::get()==0xFF) ? 255 : Role::get();   // 255 = UNASSIGNED
  uint8_t payload[2] = { sid, code };
  sendFramed(destMasterOrBcast(), OTA_STATUS, payload, sizeof(payload));
}

void GameBus_sendOtaProgress(uint8_t percent) {
  uint8_t sid = (Role::get()==0xFF) ? 255 : Role::get();
  uint8_t payload[3] = { sid, OTA_STATUS_PROGRESS, percent };
  sendFramed(destMasterOrBcast(), OTA_STATUS, payload, sizeof(payload));
}

// v3 core signature
// v3 core signature
static void onDataRecv(const esp_now_recv_info_t* info, const uint8_t* data, int len) {
  if (!info || !data || len < (int)SS_HDR_LEN) return;

  uint8_t type = 0;
  const uint8_t* payload = nullptr;
  int plen = 0;
  if (!SS_parse(data, len, type, payload, plen)) return;

  const uint8_t* src = info->src_addr;

  // NOTE: keep the callback light. Do NOT write NVS, print a lot, or do SD/LED work here.
  // Master learning/pinning is deferred to GameBus_pump() via a pending flag.

  if (type == HELLO_REQ) {
    // Always accept HELLO_REQ so we can re-pair if the Master is replaced.
    pendingSetMaster(src, /*persist*/true);
    qPush(type, nullptr, 0);
    return;
  }

  // For all other packets:
  //  - If we already have a master, only accept packets from it.
  //  - If we don't have a master yet, accept the first valid framed packet and learn its MAC later.
  if (s_masterKnown) {
    if (memcmp(src, s_masterMac, 6) != 0) return;
  } else {
    pendingSetMaster(src, /*persist*/true);
  }

  const uint8_t qlen = (uint8_t)min(plen, (int)kCmdMaxPayload);
  qPush(type, (qlen ? payload : nullptr), qlen);
}

void GameBus_init() {
  prefsLoadRadio();

  WiFi.mode(WIFI_STA);
  esp_wifi_set_promiscuous(true);
  esp_wifi_set_channel(s_nowChannel, WIFI_SECOND_CHAN_NONE);
  esp_wifi_set_promiscuous(false);

  if (esp_now_init()!=ESP_OK) { Serial.println("[NOW] init failed"); return; }
  esp_now_register_recv_cb(onDataRecv);

  // Broadcast peer (for HELLO/bootstrapping + channel changes)
  ensurePeer(BCAST_MAC);

  // Optional persisted master peer
  if (s_masterKnown) {
    ensurePeer(s_masterMac);
  }
}

void GameBus_deinit() {
  esp_now_deinit();
}

void GameBus_sendHello(uint16_t poolA_count, uint16_t poolB_count) {
  s_poolA_last = poolA_count;
  s_poolB_last = poolB_count;

  uint8_t payload[1+2+2];
  payload[0] = (Role::get()==0xFF) ? 255 : Role::get();
  payload[1] = (uint8_t)(poolA_count >> 8);
  payload[2] = (uint8_t)(poolA_count & 0xFF);
  payload[3] = (uint8_t)(poolB_count >> 8);
  payload[4] = (uint8_t)(poolB_count & 0xFF);
  sendFramed(destMasterOrBcast(), HELLO, payload, sizeof(payload));
}

void GameBus_sendBtnEvent(uint8_t slotIdx) {
  uint8_t payload[2] = {
    (uint8_t)(Role::get()==0xFF ? 255 : Role::get()),
    (uint8_t)slotIdx
  };
  sendFramed(destMasterOrBcast(), BTN_EVENT, payload, sizeof(payload));
}

// Pump queued commands from the Arduino loop (safe context)
void GameBus_pump() {
  applyPendingMaster();
  CmdMsg m;
  while (qPop(m)) {
    switch (m.type) {
      case HELLO_REQ: {
        // Master is requesting a HELLO (use last-known pool counts; 0,0 if unknown)
        GameBus_sendHello(s_poolA_last, s_poolB_last);
      } break;

      case SET_SCENE: {
        if (m.len < 8) break;
        uint16_t ids[4];
        for (int i = 0; i < 4; i++) {
          ids[i] = (uint16_t)m.payload[i*2] << 8 | m.payload[i*2 + 1];
        }
        GB_onSetScene(ids);
      } break;

      case REQUEST_RANDOM_SET: {
        if (m.len < 2) break;
        GB_onRequestRandom(m.payload[0], m.payload[1]);
      } break;

      case PLAY_SLOT: {
        if (m.len < 1) break;
        GB_onPlaySlot(m.payload[0] & 3);
      } break;

      case LED_ALL_WHITE: {
        GB_onLedAllWhite();
      } break;

      case LED_WHITE_SOFT: {
        // Reliability refresh: keep LEDs steady white without forcing an "all off" frame.
        side_ledAllWhiteSoft();
      } break;

      case BLINK_ALL: {
        if (m.len < 5) break;
        uint8_t  color  = m.payload[0];
        uint16_t on_ms  = ((uint16_t)m.payload[1] << 8) | m.payload[2];
        uint16_t off_ms = ((uint16_t)m.payload[3] << 8) | m.payload[4];
        GB_onBlinkAll(color, on_ms, off_ms);
      } break;

      case GAME_MODE: {
        if (m.len < 1) break;
        GB_onGameMode(m.payload[0] != 0);
      } break;

      case START_LOOP_ALL: {
        GB_onStartLoopAll();
      } break;

      case STOP_ALL: {
        GB_onStopAll();
      } break;

      case ROLE_ASSIGN: {
        if (m.len < 1) break;
        uint8_t newId = m.payload[0] & 1;          // 0=A, 1=B
        Serial.printf("[SIDE] ROLE_ASSIGN %u\n", newId);
        Role::set(newId, /*persist*/true);
      } break;

      case OTA_UPDATE: {
        if (m.len < 1) break;
        uint8_t ulen = m.payload[0];
        if (ulen == 0) break;
        if (ulen > (uint8_t)(m.len - 1)) break;
        side_setOtaUrl((const char*)(m.payload + 1), ulen);
        side_requestOtaStart();
      } break;

      case CHAN_SET: {
        if (m.len < 1) break;
        uint8_t ch = m.payload[0];
        if (ch < 1 || ch > 13) break;

        Serial.printf("[NOW] CHAN_SET %u -> saving + reboot\n", (unsigned)ch);
        prefsSaveChannel(ch);
        delay(40);
        ESP.restart();
      } break;

      default:
        break;
    }
  }
}

// Default mappings to the .ino functions
void GB_onSetScene(uint16_t ids[4]) {
  side_ledAllWhite();
  side_setScene(ids);
}

// NEW: category-based random selection for proof-of-concept
// For now:
//   - "same" (A bucket)  = base="animals"
//   - "odd"  (B bucket)  = base!="animals"  (tones, and any other non-animal bases later)
void GB_onRequestRandom(uint8_t needA, uint8_t needB) {
  uint16_t a[4]={0}, b[4]={0};

  // Same pool: animals
  uint8_t nA = Manifest_pickRandomByBase("animals", needA, a, 4);

  // Odd pool: anything not animals (currently tones only)
  uint8_t nB = Manifest_pickRandomByBaseNot("animals", needB, b, 4);

  uint8_t payload[1+1+2*4+2*4]; // nA + nB + idsA + idsB
  uint8_t idx=0;
  payload[idx++]=nA;
  payload[idx++]=nB;

  for (uint8_t i=0;i<nA && i<4;i++){ payload[idx++]=a[i]>>8; payload[idx++]=a[i]&0xFF; }
  for (uint8_t pad=nA; pad<4; pad++){ payload[idx++]=0; payload[idx++]=0; }

  for (uint8_t i=0;i<nB && i<4;i++){ payload[idx++]=b[i]>>8; payload[idx++]=b[i]&0xFF; }
  for (uint8_t pad=nB; pad<4; pad++){ payload[idx++]=0; payload[idx++]=0; }

  sendFramed(destMasterOrBcast(), RANDOM_SET_REPLY, payload, idx);
}

void GB_onPlaySlot(uint8_t slot) { side_playSlot(slot); }
void GB_onLedAllWhite() { side_ledAllWhite(); }
void GB_onBlinkAll(uint8_t color, uint16_t on_ms, uint16_t off_ms) {
  side_blinkAll(color, on_ms, off_ms);
}
void GB_onGameMode(bool enabled) { side_setGameMode(enabled); }
void GB_onStartLoopAll() {
  side_ledAllWhite();
  side_startLoopAll();
}
void GB_onStopAll() { side_stopAll(); }
