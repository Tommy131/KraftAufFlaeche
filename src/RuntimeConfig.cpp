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


namespace runtimeconfig {

RuntimeConfig::RuntimeConfig(SerialType& _debug_serial) :
  server(80), preferences(), currentTrim(default_pid_trim), debug_serial(_debug_serial)
{

}


void RuntimeConfig::setOnTrimeUpdateCallback(std::function<void(pid::pid_trim_t& updated)> _onTrimUpdate) {
    onTrimUpdate = _onTrimUpdate;
}

void RuntimeConfig::setOnDistanceUpdateCallback(std::function<void(uint16_t distance)> _onDistanceUpdate) {
  onDistanceUpdate = _onDistanceUpdate;
}

void RuntimeConfig::setOnSpeedUpdate(std::function<void(int8_t speed)> _onSpeedUpdate) {
  onSpeedUpdate = _onSpeedUpdate;
}

void notFound(AsyncWebServerRequest *request) {
    request->redirect("/");
}

template <typename T>
bool RuntimeConfig::check_pairs(Preferences& prefs, std::vector<persist_pair<T>>& pairs) {
  for (persist_pair<T>& setting : pairs) {
    if (!prefs.isKey(setting.key)) {
      return false;
    }
  }
  return true;
}

template <typename T>
void RuntimeConfig::writeValues(
    Preferences& prefs,
    std::vector<persist_pair<T>>& pairs,
    std::function<size_t(const char* name, T val)> valSetter) {
  for (persist_pair<T>& setting: pairs) {
    valSetter(setting.key, setting.value);
  }
}

template <typename T>
void RuntimeConfig::readValues(
    Preferences& prefs,
    std::vector<persist_pair<T>>& pairs,
    std::function<T(const char* name)> valGetter) {
  for (persist_pair<T>& setting: pairs) {
    setting.value = valGetter(setting.key);
  }
}


void RuntimeConfig::update_persisted_prefs(Preferences& prefs, pid::pid_trim_t& trim) {
  writeValues(prefs, settingsInt8, putInt8);
  writeValues(prefs, settingsUint16, putUint16);
  writeValues(prefs, settingsFloat, putFloat);

  if (!prefs.isKey(PREF_INITIALIZED) || !prefs.getBool(PREF_INITIALIZED)) {
    prefs.putBool(PREF_INITIALIZED, true);
  }
}

bool RuntimeConfig::read_persisted_prefs(Preferences& prefs) {
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


void RuntimeConfig::init_persisted_prefs(Preferences& prefs) {
  const bool hasPref = read_persisted_prefs(prefs);
  if (!hasPref) {
    update_persisted_prefs(prefs, default_pid_trim);
    return;
  }
}

template <typename T>
void RuntimeConfig::addJsonKV(JsonDocument& data, std::vector<persist_pair<T>>& pairs) {
  for (persist_pair<T>& setting : pairs) {
    data[setting.key] = setting.value;
  }
}

template <typename T>
void RuntimeConfig::handleIntParam(AsyncWebServerRequest *request, std::vector<persist_pair<T>>& pairs) {
  for (persist_pair<T>& setting : pairs) {
  String val = "";
  if (request->hasParam(setting.key)
    && (val = request->getParam(setting.key)->value()).length()) {
      debug_serial.print(setting.key);
      debug_serial.print(": ");
      debug_serial.println(val);
      setting.value = val.toInt();
    }
}
}

void RuntimeConfig::setupRuntimeConfig() {
  WiFi.mode(WIFI_AP);
  WiFi.softAP(SSID, PSK);

  preferences.begin(PREF_DOMAIN, PREF_RW_MODE);
  SPIFFS.begin();

  MDNS.addService("http", "tcp", 80);

  if (MDNS.begin("esp32")) {
    debug_serial.println("MDNS responder started");
  }


  server.on("/getconfig", HTTP_GET, [&](AsyncWebServerRequest *request) {
    AsyncResponseStream *response = request->beginResponseStream("application/json");
    JsonDocument data;
    addJsonKV(data, settingsInt8);
    addJsonKV(data, settingsUint16);
    addJsonKV(data, settingsFloat);
    serializeJson(data, *response);
    request->send(response);
  });

  server.on("/updateconfig", HTTP_GET, [&](AsyncWebServerRequest *request) {
    handleIntParam(request, settingsInt8);
    handleIntParam(request, settingsUint16);
    for (persist_pair<float>& setting : settingsFloat) {
      String val = "";
      if (request->hasParam(setting.key)
        && (val = request->getParam(setting.key)->value()).length()) {
          debug_serial.print(setting.key);
          debug_serial.print(": ");
          debug_serial.println(val);
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
  
  debug_serial.println("===");
  IPAddress myIP = WiFi.softAPIP();
  debug_serial.print("AP IP address: ");
  debug_serial.println(myIP);


  server.begin();
  debug_serial.println("HTTP server started");

  init_persisted_prefs(preferences);
  onTrimUpdate(currentTrim);
  onSpeedUpdate(speed);
  onDistanceUpdate(distance);
}

} // runtimeconfig

#endif
