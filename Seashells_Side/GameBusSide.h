#pragma once
#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include "Messages.h"
#include "ConfigSide.h"

void GameBus_init();
void GameBus_deinit();
void GameBus_pump();  // call this from loop() to process queued ESP-NOW commands
void GameBus_sendHello(uint16_t poolA_count, uint16_t poolB_count);
void GameBus_sendBtnEvent(uint8_t slotIdx);
void GameBus_sendOtaStatus(uint8_t code);
void GameBus_sendOtaProgress(uint8_t percent);

// Handlers called by GameBus when packets arrive:
void GB_onSetScene(uint16_t ids[4]);
void GB_onRequestRandom(uint8_t needA, uint8_t needB);
void GB_onPlaySlot(uint8_t slot);
void GB_onLedAllWhite();
void GB_onLedSolidSlots(const uint8_t slotColors[4]);
void GB_onBlinkAll(uint8_t color, uint16_t on_ms, uint16_t off_ms);
void GB_onBlinkSlots(const uint8_t slotColors[4], uint16_t on_ms, uint16_t off_ms);
void GB_onSlotTrimDb(const int8_t trim_db[4]);
void GB_onDiagAudio(uint8_t pattern_id);
void GB_onMixFixSet(uint8_t mask, int16_t kQ12, int16_t mQ12);
void GB_onGameMode(bool enabled);
void GB_onStartLoopAll();
void GB_onStopAll();
