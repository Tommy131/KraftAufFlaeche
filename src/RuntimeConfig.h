#pragma once

#include <ArduinoOTA.h>
#include <FS.h>
#include <SPIFFS.h>
#include <ESPmDNS.h>
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <SPIFFSEditor.h>
#include <Preferences.h>
#include <ArduinoJson.h>

#include "PidData.h"

#define SSID "kraftaufflaeche"
#define PSK "fosbosprojekt"
#define HOSTNAME "esp32"

#define KEY_KP_PID "KP_PID"
#define KEY_KI_PID "KI_PID"
#define KEY_KD_PID "KD_PID"

#define KEY_DISTANCE "DISTANCE"
#define KEY_SPEED "SPEED"

#define PREF_RW_MODE false
#define PREF_RO_MODE true
#define PREF_DOMAIN "TRIM"
#define PREF_INITIALIZED "PREF_INITI"

#define MAX_PREF_LEN 15

template <typename T>
struct persist_pair {
    char key[MAX_PREF_LEN+1]; // max length is 15 (by library)
    T& value;
};


void setupRuntimeConfig();
void setOnTrimeUpdateCallback(std::function<void(pid::pid_trim_t& updated)> onTrimUpdate);
void setOnDistanceUpdateCallback(std::function<void(uint16_t distance)> _onDistanceUpdate);
void setOnSpeedUpdate(std::function<void(int8_t speed)> _onSpeedUpdate);
