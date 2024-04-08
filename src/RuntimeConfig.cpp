#include "RuntimeConfig.h"


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

pid::pid_trim_t currentTrim(default_pid_trim);

Preferences preferences;

std::function<void(pid::pid_trim_t& updated)> onTrimUpdate = [](pid::pid_trim_t& u){};

AsyncWebServer server(80);

std::vector<persist_float_pair_t> settings {
  (persist_float_pair_t)  { KEY_KP_PID, currentTrim.kp },
  (persist_float_pair_t){ KEY_KI_PID, currentTrim.ki },
  (persist_float_pair_t) { KEY_KD_PID, currentTrim.kd },
};


void setOnTrimeUpdateCallback(std::function<void(pid::pid_trim_t& updated)> _onTrimUpdate) {
    onTrimUpdate = _onTrimUpdate;
}

void notFound(AsyncWebServerRequest *request) {
    request->redirect("/");
}

void update_persisted_prefs(Preferences& prefs, pid::pid_trim_t& trim) {
  for (persist_float_pair_t& setting : settings) {
    prefs.putFloat(setting.key, setting.value);
  }
  if (!prefs.isKey(PREF_INITIALIZED) || !prefs.getBool(PREF_INITIALIZED)) {
    prefs.putBool(PREF_INITIALIZED, true);
  }
}

bool read_persisted_prefs(Preferences& prefs) {
  if (!prefs.isKey(PREF_INITIALIZED)) {
    return false;
  }

  // make sure data is valid
  for (persist_float_pair_t& setting : settings) {
    if (!prefs.isKey(setting.key)) {
      return false;
    }
  }

  // read data later
  for (persist_float_pair_t& setting : settings) {
    const float val = prefs.getFloat(setting.key);
    setting.value = val;
  }
  return true;
}


void init_persisted_prefs(Preferences& prefs) {
  const bool hasPref = read_persisted_prefs(prefs);
  if (!hasPref) {
    update_persisted_prefs(prefs, default_pid_trim);
    return;
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
    for (persist_float_pair_t& setting : settings) {
      data[setting.key] = setting.value;
    }
    serializeJson(data, *response);
    request->send(response);
  });

  server.on("/updateconfig", HTTP_GET, [](AsyncWebServerRequest *request) {


    for (persist_float_pair_t& setting : settings) {
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
}

