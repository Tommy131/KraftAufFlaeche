#pragma once

#include "constants.h"

#if defined(RUNTIME_CONFIG_ENABLE) && defined(ARDUINO_ARCH_ESP32)

#include <FS.h>
#include <SPIFFS.h>
#include <ESPmDNS.h>
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <SPIFFSEditor.h>
#include <Preferences.h>
#include <ArduinoJson.h>
#include <DNSServer.h>

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

namespace runtimeconfig {

template <typename T>
struct persist_pair {
    char key[MAX_PREF_LEN+1]; // max length is 15 (by library)
    T& value;
};

class RuntimeConfig {

public:
    RuntimeConfig(SerialType& _debug_serial, DNSServer& dns);

    void setupRuntimeConfig();
    void loopRuntimeConfig();
    void setOnTrimeUpdateCallback(std::function<void(pid::pid_trim_t& updated)> onTrimUpdate);
    void setOnDistanceUpdateCallback(std::function<void(uint16_t distance)> _onDistanceUpdate);
    void setOnSpeedUpdate(std::function<void(int8_t speed)> _onSpeedUpdate);
private:
    SerialType& debug_serial;

    std::function<void(pid::pid_trim_t& updated)> onTrimUpdate = [](pid::pid_trim_t& u){};
    std::function<void(uint16_t distance)> onDistanceUpdate = [](uint16_t distance){};
    std::function<void(int8_t speed)> onSpeedUpdate = [](int8_t  speed){};


    // int8_t
    std::function<size_t(const char* name, int8_t val)> putInt8 = [&](const char* name, int8_t val) { return preferences.putChar(name, val); };
    std::function<int8_t(const char* name)> getInt8 = [&](const char* name) { return preferences.getChar(name); };

    // uint16_t
    std::function<size_t(const char* name, uint16_t val)> putUint16 = [&](const char* name, uint16_t val) { return preferences.putUShort(name, val); };
    std::function<uint16_t(const char* name)> getUint16 = [&](const char* name) { return preferences.getUShort(name); };

    // float
    std::function<size_t(const char* name, float val)> putFloat = [&](const char* name, float val) { return preferences.putFloat(name, val); };
    std::function<float(const char* name)> getFloat = [&](const char* name) { return preferences.getFloat(name); };

    // generic
    std::function<bool(const char* name)> testValue = [&](const char* name) { return preferences.isKey(name); };


    std::vector<persist_pair<int8_t>> settingsInt8 {
        (persist_pair<int8_t>) { KEY_SPEED, speed },
    };

    std::vector<persist_pair<uint16_t>> settingsUint16 {
        (persist_pair<uint16_t>) { KEY_DISTANCE, distance },
    };

    std::vector<persist_pair<float>> settingsFloat {
        (persist_pair<float>)  { KEY_KP_PID, currentTrim.kp },
        (persist_pair<float>) { KEY_KI_PID, currentTrim.ki },
        (persist_pair<float>) { KEY_KD_PID, currentTrim.kd },
    };


    template <typename T>
    bool check_pairs(Preferences& prefs, std::vector<persist_pair<T>>& pairs);

    template <typename T>
    void writeValues(
        Preferences& prefs,
        std::vector<persist_pair<T>>& pairs,
        std::function<size_t(const char* name, T val)> valSetter);

    template <typename T>
    void readValues(
        Preferences& prefs,
        std::vector<persist_pair<T>>& pairs,
        std::function<T(const char* name)> valGetter);

    template <typename T>
    void addJsonKV(JsonDocument& data, std::vector<persist_pair<T>>& pairs);

    template <typename T>
    void handleIntParam(AsyncWebServerRequest *request, std::vector<persist_pair<T>>& pairs);

    void update_persisted_prefs(Preferences& prefs, pid::pid_trim_t& trim);
    bool read_persisted_prefs(Preferences& prefs);
    void init_persisted_prefs(Preferences& prefs);


    pid::pid_trim_t currentTrim;
    int8_t speed = MAX_SPEED;
    uint16_t distance = DEFAULT_DISTANCE;

    Preferences preferences;
    AsyncWebServer server;
    DNSServer& dnsServer;

};

} // runtimeconfig

#endif
