#pragma once
#include <Arduino.h>

// WiFi/ESP-NOW
#define WIFI_CHANNEL 6

// Fill with your Side Feathers' STA MACs (print on Sides at boot)
static uint8_t SIDE_A_MAC[6] = {0x7C,0xDF,0xA1,0xF8,0xF1,0x40};
static uint8_t SIDE_B_MAC[6] = {0x7C,0xDF,0xA1,0xF8,0xF0,0x4C};

#define OTA_URL_SIDE_BIN  "http://172.20.10.3:8000/Seashells/Seashells_Side/build/esp32.esp32.um_feathers3/Seashells_Side.ino.bin"
