#include "RuntimeConfig.h"
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
#include "PidData.h"

pid::pid_trim_t currentTrim(default_pid_trim);
int8_t speed = MAX_SPEED;
uint16_t distance = DEFAULT_DISTANCE;

Preferences preferences;

std::function<void(pid::pid_trim_t& updated)> onTrimUpdate = [](pid::pid_trim_t& u){};
std::function<void(uint16_t distance)> onDistanceUpdate = [](uint16_t distance){};
std::function<void(int8_t speed)> onSpeedUpdate = [](int8_t  speed){};

AsyncWebServer server(80);

// int8_t
std::function<size_t(const char* name, int8_t val)> putInt8 = [](const char* name, int8_t val) { return preferences.putChar(name, val); };
std::function<int8_t(const char* name)> getInt8 = [](const char* name) { return preferences.getChar(name); };

// uint16_t
std::function<size_t(const char* name, uint16_t val)> putUint16 = [](const char* name, uint16_t val) { return preferences.putUShort(name, val); };
std::function<uint16_t(const char* name)> getUint16 = [](const char* name) { return preferences.getUShort(name); };

// float
std::function<size_t(const char* name, float val)> putFloat = [](const char* name, float val) { return preferences.putFloat(name, val); };
std::function<float(const char* name)> getFloat = [](const char* name) { return preferences.getFloat(name); };

// generic
std::function<bool(const char* name)> testValue = [](const char* name) { return preferences.isKey(name); };



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


void setOnTrimeUpdateCallback(std::function<void(pid::pid_trim_t& updated)> _onTrimUpdate) {
    onTrimUpdate = _onTrimUpdate;
}

void setOnDistanceUpdateCallback(std::function<void(uint16_t distance)> _onDistanceUpdate) {
  onDistanceUpdate = _onDistanceUpdate;
}

void setOnSpeedUpdate(std::function<void(int8_t speed)> _onSpeedUpdate) {
  onSpeedUpdate = _onSpeedUpdate;
}

void notFound(AsyncWebServerRequest *request) {
    request->redirect("/");
}

template <typename T>
bool check_pairs(Preferences& prefs, std::vector<persist_pair<T>>& pairs) {
  for (persist_pair<T>& setting : pairs) {
    if (!prefs.isKey(setting.key)) {
      return false;
    }
  }
  return true;
}

template <typename T>
void writeValues(
    Preferences& prefs,
    std::vector<persist_pair<T>>& pairs,
    std::function<size_t(const char* name, T val)> valSetter) {
  for (persist_pair<T>& setting: pairs) {
    valSetter(setting.key, setting.value);
  }
}

template <typename T>
void readValues(
    Preferences& prefs,
    std::vector<persist_pair<T>>& pairs,
    std::function<T(const char* name)> valGetter) {
  for (persist_pair<T>& setting: pairs) {
    setting.value = valGetter(setting.key);
  }
}


void update_persisted_prefs(Preferences& prefs, pid::pid_trim_t& trim) {
  writeValues(prefs, settingsInt8, putInt8);
  writeValues(prefs, settingsUint16, putUint16);
  writeValues(prefs, settingsFloat, putFloat);

  if (!prefs.isKey(PREF_INITIALIZED) || !prefs.getBool(PREF_INITIALIZED)) {
    prefs.putBool(PREF_INITIALIZED, true);
  }
}

bool read_persisted_prefs(Preferences& prefs) {
  if (!prefs.isKey(PREF_INITIALIZED)) {
    return false;
  }

  // make sure data is valid
  if (!check_pairs(prefs, settingsInt8)
    || !check_pairs(prefs, settingsUint16)
    || !check_pairs(prefs, settingsFloat))
  {
    return false;
  }

  // read data later
  readValues(prefs, settingsInt8, getInt8);
  readValues(prefs, settingsUint16, getUint16);
  readValues(prefs, settingsFloat, getFloat);
  return true;
}


void init_persisted_prefs(Preferences& prefs) {
  const bool hasPref = read_persisted_prefs(prefs);
  if (!hasPref) {
    update_persisted_prefs(prefs, default_pid_trim);
    return;
  }
}

template <typename T>
void addJsonKV(JsonDocument& data, std::vector<persist_pair<T>>& pairs) {
  for (persist_pair<T>& setting : pairs) {
    data[setting.key] = setting.value;
  }
}

template <typename T>
void handleIntParam(AsyncWebServerRequest *request, std::vector<persist_pair<T>>& pairs) {
  for (persist_pair<T>& setting : pairs) {
  String val = "";
  if (request->hasParam(setting.key)
    && (val = request->getParam(setting.key)->value()).length()) {
      Serial.print(setting.key);
      Serial.print(": ");
      Serial.println(val);
      setting.value = val.toInt();
    }
}
}

void setupRuntimeConfig() {
  WiFi.mode(WIFI_AP);
  WiFi.softAP(SSID, PSK);

  preferences.begin(PREF_DOMAIN, PREF_RW_MODE);
  SPIFFS.begin();

  MDNS.addService("http", "tcp", 80);

  if (MDNS.begin("esp32")) {
    Serial.println("MDNS responder started");
  }


  server.on("/getconfig", HTTP_GET, [](AsyncWebServerRequest *request) {
    AsyncResponseStream *response = request->beginResponseStream("application/json");
    JsonDocument data;
    addJsonKV(data, settingsInt8);
    addJsonKV(data, settingsUint16);
    addJsonKV(data, settingsFloat);
    serializeJson(data, *response);
    request->send(response);
  });

  server.on("/updateconfig", HTTP_GET, [](AsyncWebServerRequest *request) {
    handleIntParam(request, settingsInt8);
    handleIntParam(request, settingsUint16);
    for (persist_pair<float>& setting : settingsFloat) {
      String val = "";
      if (request->hasParam(setting.key)
        && (val = request->getParam(setting.key)->value()).length()) {
          Serial.print(setting.key);
          Serial.print(": ");
          Serial.println(val);
          setting.value = val.toFloat();

      }
    }
    update_persisted_prefs(preferences, currentTrim);
    request->redirect("/");
    onTrimUpdate(currentTrim);
    onSpeedUpdate(speed);
    onDistanceUpdate(distance);
  });

  server.serveStatic("/", SPIFFS, "/").setDefaultFile("index.html");

  server.onNotFound(notFound);
  
  Serial.println("===");
  IPAddress myIP = WiFi.softAPIP();
  Serial.print("AP IP address: ");
  Serial.println(myIP);


  server.begin();
  Serial.println("HTTP server started");

  init_persisted_prefs(preferences);
  onTrimUpdate(currentTrim);
  onSpeedUpdate(speed);
  onDistanceUpdate(distance);
}

#endif
