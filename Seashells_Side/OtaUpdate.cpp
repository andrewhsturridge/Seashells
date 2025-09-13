#include "OtaUpdate.h"

#include <WiFi.h>
#include <esp_wifi.h>
#include <HTTPClient.h>
#include <Update.h>

#include "ConfigSide.h"   // OTA_WIFI_SSID, OTA_WIFI_PASS, OTA_CONNECT_TIMEOUT_MS, OTA_HTTP_TIMEOUT_MS
#include "OtaUpdate.h"

// --- extern helpers provided by your Side .ino ---
// Visuals
extern void side_blinkAll(uint8_t color, uint16_t on_ms, uint16_t off_ms);
extern void otaShowProgress(uint8_t pct);
// Optional: pause audio/loops if you want during OTA
extern void side_stopAll();

// --- Module state ---
static volatile bool s_otaStartRequested = false;
static String        s_otaUrl;

// --- Public entry points used by the ESP-NOW handler ---
void side_setOtaUrl(const char* p, uint8_t n) {
  s_otaUrl = ""; s_otaUrl.reserve(n + 1);
  for (uint8_t i = 0; i < n; i++) s_otaUrl += p[i];
  Serial.printf("[OTA] URL set: %s\n", s_otaUrl.c_str());
}
void side_requestOtaStart() {
  s_otaStartRequested = true;
}

// --- Internal TREX-style OTA implementation ---
static bool doOtaFromUrl(const String& url) {
  Serial.printf("[OTA] URL: %s\n", url.c_str());

  // Optional: quiet local playback/loops
  if (side_stopAll) side_stopAll();

  // Visual cue + progress bar reset
  side_blinkAll(/*white*/2, 60, 60);
  otaShowProgress(0);

  // 1) Join Wi-Fi (STA), no ESPNOW deinit (matches TREX)
  WiFi.persistent(false);
  WiFi.disconnect(true, true);
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  esp_wifi_set_ps(WIFI_PS_NONE);

  Serial.printf("[OTA] STA connect → SSID='%s'\n", OTA_WIFI_SSID);
  WiFi.begin(OTA_WIFI_SSID, OTA_WIFI_PASS);

  uint32_t t0 = millis();
  while (WiFi.status() != WL_CONNECTED) {
    if (millis() - t0 > OTA_CONNECT_TIMEOUT_MS) {
      Serial.println("[OTA] WiFi connect timeout");
      return false;
    }
    delay(100);
  }
  Serial.printf("[OTA] WiFi OK ch=%d ip=%s\n",
                WiFi.channel(), WiFi.localIP().toString().c_str());

  // 2) HTTP (no keep-alive), 15s+ timeout; require 200 OK
  HTTPClient http;
  WiFiClient client;
  http.setReuse(false);
  http.setTimeout(OTA_HTTP_TIMEOUT_MS);
  http.setFollowRedirects(HTTPC_STRICT_FOLLOW_REDIRECTS);

  if (!http.begin(client, url)) {
    Serial.println("[OTA] http.begin FAIL");
    return false;
  }

  int code = http.GET();
  Serial.printf("[OTA] HTTP code %d\n", code);
  if (code != HTTP_CODE_OK) {
    http.end();
    return false;
  }

  int total = http.getSize();            // may be -1
  Serial.printf("[OTA] total bytes: %d\n", total);

  // 3) Begin Update with/without known length
  if (total > 0) {
    if (!Update.begin(total)) {
      Serial.printf("[OTA] Update.begin fail: %s need=%d\n", Update.errorString(), total);
      http.end(); return false;
    }
  } else {
    if (!Update.begin()) {
      Serial.printf("[OTA] Update.begin(no len) fail: %s\n", Update.errorString());
      http.end(); return false;
    }
  }

  // 4) Chunked copy + 15s inactivity watchdog + LED progress
  WiFiClient* stream = http.getStreamPtr();
  const size_t BUF = 2048;
  uint8_t buf[BUF];
  uint32_t got = 0, lastDraw = 0, lastActivity = millis();

  while ( (total < 0) || (got < (uint32_t)total) ) {
    size_t avail = stream->available();
    if (avail) {
      size_t toRead = (avail > BUF) ? BUF : avail;
      int n = stream->readBytes((char*)buf, toRead);
      if (n <= 0) { delay(1); continue; }

      size_t w = Update.write(buf, (size_t)n);
      if (w != (size_t)n) {
        Serial.printf("[OTA] write err: %s @%lu/%d\n",
                      Update.errorString(), (unsigned long)got, total);
        Update.end(); http.end(); return false;
      }

      got += w; lastActivity = millis();

      if (total > 0 && (got - lastDraw) >= 16384) {   // update every 16 KB
        uint8_t pct = (uint8_t)((got * 100UL) / (uint32_t)total);
        otaShowProgress(pct);
        lastDraw = got;
      }
    } else {
      delay(1);

      if (total > 0 && got >= (uint32_t)total) break;
      if (total < 0 && !stream->connected() && stream->available()==0) break;

      if (millis() - lastActivity > 15000) {
        Serial.println("[OTA] Stream timeout (no data)");
        Update.end(); http.end(); return false;
      }
    }
  }

  // 5) Verify & finish (like TREX)
  bool ok = Update.end(true);
  http.end();

  if (!ok || !Update.isFinished()) {
    Serial.printf("[OTA] verify error: %s (wrote %lu/%d)\n",
                  Update.errorString(), (unsigned long)got, total);
    return false;
  }

  // 6) Success → show 100%, blink green, reboot
  otaShowProgress(100);
  side_blinkAll(/*green*/1, 140, 120);
  delay(200);
  ESP.restart();
  return true; // not reached
}

// --- Public wrapper (kept for compatibility) ---
bool side_doOTA(const String& url) { return doOtaFromUrl(url); }

// --- Pump from loop() ---
void Ota_loopTick() {
  if (!s_otaStartRequested) return;
  s_otaStartRequested = false;

  if (!s_otaUrl.length()) {
    Serial.println("[OTA] No URL set");
    return;
  }

  bool ok = doOtaFromUrl(s_otaUrl);
  if (!ok) {
    // Visual fail and clean radio state
    side_blinkAll(/*red*/0, 160, 120);
    WiFi.disconnect(true, true);
    WiFi.mode(WIFI_OFF);
  }
}
