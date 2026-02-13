#pragma once
#include <stdint.h>
#include <string.h>

// ============================================================================
// Seashells ESP-NOW protocol framing
//
// When multiple ESP-NOW games share RF space (and possibly the same WiFi
// channel), packets from one game can arrive at another game's receive
// callback. If the other game only checks the first byte as "message type",
// then different games can accidentally interpret each other's packets.
//
// To prevent cross-talk, every Seashells packet is framed with a 4-byte header:
//   [ magic0, magic1, proto_ver, msg_type, ...payload ]
//
// Receivers MUST validate magic/proto_ver before acting on msg_type.
// ============================================================================

static constexpr uint8_t SS_MAGIC0    = (uint8_t)'S';
static constexpr uint8_t SS_MAGIC1    = (uint8_t)'H'; // "SeasHell" (distinct from Soccer's "SC")
static constexpr uint8_t SS_PROTO_VER = 1;
static constexpr uint8_t SS_HDR_LEN   = 4;

static inline bool SS_parse(const uint8_t* data, int len,
                            uint8_t& outType,
                            const uint8_t*& outPayload,
                            int& outPayloadLen) {
  if (!data || len < (int)SS_HDR_LEN) return false;
  if (data[0] != SS_MAGIC0 || data[1] != SS_MAGIC1) return false;
  if (data[2] != SS_PROTO_VER) return false;
  outType = data[3];
  outPayload = data + SS_HDR_LEN;
  outPayloadLen = len - (int)SS_HDR_LEN;
  return true;
}

static inline int SS_build(uint8_t* out, int outMax,
                           uint8_t type,
                           const void* payload,
                           int payloadLen) {
  if (!out || outMax < (int)SS_HDR_LEN) return -1;
  out[0] = SS_MAGIC0;
  out[1] = SS_MAGIC1;
  out[2] = SS_PROTO_VER;
  out[3] = type;
  if (payloadLen < 0) payloadLen = 0;
  int copyLen = payloadLen;
  if (copyLen > (outMax - (int)SS_HDR_LEN)) copyLen = outMax - (int)SS_HDR_LEN;
  if (copyLen > 0 && payload) {
    memcpy(out + SS_HDR_LEN, payload, (size_t)copyLen);
  }
  return (int)SS_HDR_LEN + copyLen;
}

enum MsgType : uint8_t {
  HELLO_REQ          = 0,
  HELLO              = 1,
  SET_SCENE          = 2,  // type + 4×uint16 = 9 bytes total
  REQUEST_RANDOM_SET = 3,  // type + needA(uint8) + needB(uint8) = 3
  RANDOM_SET_REPLY   = 4,  // type + nA + nB + 4*A(2B ea) + 4*B(2B ea) = 19
  PLAY_SLOT          = 5,  // type + slot(uint8) = 2
  LED_ALL_WHITE      = 6,  // type = 1
  BLINK_ALL          = 7,  // type + color(1) + on_ms(2) + off_ms(2) = 6
  GAME_MODE          = 8,  // type + enabled(1) = 2
  BTN_EVENT          = 9,  // type + side(1) + slot(1) = 3
  START_LOOP_ALL     = 10, // type = 1
  STOP_ALL           = 11, // type = 1
  OTA_UPDATE         = 12, // payload: url_len(uint8), url bytes...
  OTA_STATUS         = 13, // payload: side_id(uint8), code(uint8)  [0=BEGIN,1=OK,2=FAIL_WIFI,3=FAIL_HTTP,4=FAIL_UPDATE]
  ROLE_ASSIGN        = 14, // payload: sideId(uint8)  0=A, 1=B
  CHAN_SET           = 15  // payload: wifi_channel(uint8)
};

// OTA_STATUS codes (data[2]) and optional payload
#define OTA_STATUS_BEGIN     0   // payload: [type, side, 0]
#define OTA_STATUS_OK        1   // payload: [type, side, 1]
#define OTA_STATUS_FAIL_WIFI 2   // ...
#define OTA_STATUS_FAIL_HTTP 3
#define OTA_STATUS_FAIL_UPD  4
#define OTA_STATUS_PROGRESS  5   // payload: [type, side, 5, percent]
