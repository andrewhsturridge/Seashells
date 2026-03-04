#pragma once
#include <Arduino.h>

#include "Role.h"

// ------- ESP-NOW / WiFi -------
// Default ESP-NOW channel used on first boot. After that, the Side will use the
// channel stored in NVS (set via Master command "CHAN <n>").
#define NOW_DEFAULT_CHANNEL 6

// These are ONLY used during OTA updates, not for ESP-NOW.
// Set them to your PHONE HOTSPOT SSID + password.
#define OTA_WIFI_SSID  "AndrewiPhone"
#define OTA_WIFI_PASS  "12345678"

#define OTA_CONNECT_TIMEOUT_MS 15000
#define OTA_HTTP_TIMEOUT_MS    45000

// NOTE: Master MAC is learned automatically over ESP-NOW and stored in NVS.

// ------- AUDIO SETTINGS -------
#define SAMPLE_RATE     44100  // 44100 or 48000; keep all files at the same rate

// ------- SD on SPI1 pins (Unexpected Maker Feather S3) -------
#define SD_CS    5
#define SD_MOSI 35
#define SD_MISO 37
#define SD_SCK  36

// ------- I2S #0 (Speakers 1 & 2) -------
#define I2S0_DOUT 12
#define I2S0_BCLK 43
#define I2S0_LRCK 44

// ------- I2S #1 (Speakers 3 & 4) -------
#define I2S1_DOUT 14
#define I2S1_BCLK 8
#define I2S1_LRCK 9

// ------- Diagnostic / Workaround: RIGHT-channel amp in L+R mix mode -------
// Some mono I2S amp boards can be strapped for LEFT, RIGHT, or MIX (L+R) output.
// If your RIGHT speakers are accidentally in MIX mode, you will hear "odd+common"
// on that speaker whenever its paired LEFT speaker plays a different sound.
//
// Use DIAG patterns to detect:
//  - DIAG 1/2/4/5 confirm basic channel routing.
//  - DIAG 12/13 are phase-cancel tests (helpful, but can be fooled by acoustic cancellation).
//  - DIAG 14/15 are the most decisive: they *toggle* the MixFix algorithm ON/OFF and show the state
//    on the RIGHT LED (RED=OFF, GREEN=ON). If the RIGHT speaker gets noticeably quieter on GREEN,
//    that amp is almost certainly mixing L+R.
//
// These set the DEFAULT enabled mask at boot. You can also control MixFix at runtime from the Master:
//   MIXFIX ON / MIXFIX OFF / MIXFIX A ... / MIXFIX B ...
#define FIX_RIGHT_MIX_I2S0 1  // default enable for slot1 (Speaker2)
#define FIX_RIGHT_MIX_I2S1 1  // default enable for slot3 (Speaker4)

// Mix model for the mis-strapped amp:
//  1 = amp output ~= (L+R)/2   (most common "average" mix)
//  0 = amp output ~= (L+R)     (rare "sum" mix)
#define RIGHT_MIX_MODEL_AVG 1

// ------- Buttons (external pull-ups, active-low) -------
#define BTN1_PIN 10
#define BTN2_PIN 18
#define BTN3_PIN 11
#define BTN4_PIN 1

// ------- RGB pins (one NeoPixel per button) -------
#define RGB1_PIN 33
#define RGB2_PIN 7
#define RGB3_PIN 6
#define RGB4_PIN 3  // strap pin ok as OUTPUT; keep pulled up at boot; 330Ω series on data

// Debounce
#define DEBOUNCE_MS   20
#define BRIGHTNESS    255
