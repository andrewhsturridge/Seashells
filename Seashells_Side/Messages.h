#pragma once
#include <stdint.h>

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
  ROLE_ASSIGN        = 14  // payload: sideId(uint8)  0=A, 1=B
};

// OTA_STATUS codes (data[2]) and optional payload
#define OTA_STATUS_BEGIN     0   // payload: [type, side, 0]
#define OTA_STATUS_OK        1   // payload: [type, side, 1]
#define OTA_STATUS_FAIL_WIFI 2   // ...
#define OTA_STATUS_FAIL_HTTP 3
#define OTA_STATUS_FAIL_UPD  4
#define OTA_STATUS_PROGRESS  5   // payload: [type, side, 5, percent]
