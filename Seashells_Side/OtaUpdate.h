#pragma once
#include <Arduino.h>

// Called from GameBusSide.cpp's OTA_UPDATE handler:
void side_setOtaUrl(const char* p, uint8_t n);
void side_requestOtaStart();

// If you ever want to kick an OTA directly:
bool side_doOTA(const String& url);

// Call this once near the top of loop(); it will run an OTA
// if side_requestOtaStart() was called.
void Ota_loopTick();
