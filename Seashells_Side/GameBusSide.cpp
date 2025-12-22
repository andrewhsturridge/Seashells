#include <cstring>
#include <esp_now.h>

#include "GameBusSide.h"
#include "Manifest.h"
#include "Role.h"
#include "OtaUpdate.h"

// Externs implemented in the .ino (audio/led functions)
extern void side_setScene(uint16_t ids[4]);
extern void side_playSlot(uint8_t slot);
extern void side_ledAllWhite();
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

struct CmdMsg {
  uint8_t type = 0;
  uint8_t len  = 0;
  uint8_t payload[kCmdMaxPayload];
};

static CmdMsg cmdQ[kCmdQSize];
static volatile uint8_t qHead = 0;
static volatile uint8_t qTail = 0;
static portMUX_TYPE qMux = portMUX_INITIALIZER_UNLOCKED;

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
  uint8_t pkt[3] = { OTA_STATUS, sid, code };
  esp_now_send(MASTER_MAC, pkt, sizeof(pkt));
}

void GameBus_sendOtaProgress(uint8_t percent) {
  uint8_t sid = (Role::get()==0xFF) ? 255 : Role::get();
  uint8_t pkt[4] = { OTA_STATUS, sid, OTA_STATUS_PROGRESS, percent };
  esp_now_send(MASTER_MAC, pkt, sizeof(pkt));
}

// v3 core signature
static void onDataRecv(const esp_now_recv_info_t* info, const uint8_t* data, int len) {
  if (!info || !data || len < 1) return;

  // ONLY accept packets from Master
  if (std::memcmp(info->src_addr, MASTER_MAC, 6) != 0) {
    return;
  }

  const uint8_t type = data[0];
  const uint8_t plen = (len > 1) ? (uint8_t)min(len - 1, (int)kCmdMaxPayload) : 0;

  // Queue payload bytes (everything after the type)
  qPush(type, (plen ? (data + 1) : nullptr), plen);
}

void GameBus_init() {
  WiFi.mode(WIFI_STA);
  esp_wifi_set_promiscuous(true);
  esp_wifi_set_channel(WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE);
  esp_wifi_set_promiscuous(false);

  if (esp_now_init()!=ESP_OK) { Serial.println("[NOW] init failed"); return; }
  esp_now_register_recv_cb(onDataRecv);

  esp_now_peer_info_t p{}; memcpy(p.peer_addr, MASTER_MAC, 6);
  p.channel = WIFI_CHANNEL; p.encrypt = false;
  esp_now_add_peer(&p);
}

void GameBus_deinit() {
  esp_now_deinit();
}

void GameBus_sendHello(uint16_t poolA_count, uint16_t poolB_count) {
  uint8_t pkt[1+1+2+2];
  pkt[0]=HELLO;
  pkt[1]=Role::get()==0xFF ? 255 : Role::get();   // report 255 if unassigned
  pkt[2]=poolA_count>>8; pkt[3]=poolA_count&0xFF;
  pkt[4]=poolB_count>>8; pkt[5]=poolB_count&0xFF;
  esp_now_send(MASTER_MAC, pkt, sizeof(pkt));
}

void GameBus_sendBtnEvent(uint8_t slotIdx) {
  uint8_t pkt[3] = { BTN_EVENT, (uint8_t)(Role::get()==0xFF?255:Role::get()), (uint8_t)slotIdx };
  esp_now_send(MASTER_MAC, pkt, sizeof(pkt));
}

// Pump queued commands from the Arduino loop (safe context)
void GameBus_pump() {
  CmdMsg m;
  while (qPop(m)) {
    switch (m.type) {
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

  uint8_t pkt[1+1+1+2*4+2*4]; // type + countA + countB + idsA + idsB
  uint8_t idx=0;
  pkt[idx++]=RANDOM_SET_REPLY;
  pkt[idx++]=nA;
  pkt[idx++]=nB;

  for (uint8_t i=0;i<nA && i<4;i++){ pkt[idx++]=a[i]>>8; pkt[idx++]=a[i]&0xFF; }
  for (uint8_t pad=nA; pad<4; pad++){ pkt[idx++]=0; pkt[idx++]=0; }

  for (uint8_t i=0;i<nB && i<4;i++){ pkt[idx++]=b[i]>>8; pkt[idx++]=b[i]&0xFF; }
  for (uint8_t pad=nB; pad<4; pad++){ pkt[idx++]=0; pkt[idx++]=0; }

  esp_now_send(MASTER_MAC, pkt, idx);
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
