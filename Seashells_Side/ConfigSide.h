#pragma once
#include <Arduino.h>

#include "Role.h"

// ------- ESP-NOW / WiFi -------
#define WIFI_CHANNEL 6
#define OTA_WIFI_SSID  "GUD"
#define OTA_WIFI_PASS  "EscapE66"
#define OTA_CONNECT_TIMEOUT_MS 15000
#define OTA_HTTP_TIMEOUT_MS    45000

// Fill with your Master Feather's STA MAC (print on Master at boot)
static uint8_t MASTER_MAC[6] = {0xEC,0xDA,0x3B,0x5B,0x8C,0x30};

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
