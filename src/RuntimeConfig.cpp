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


void setOnTrimeUpdateCallback(std::function<void(pid::pid_trim_t& updated)> _onTrimUpdate) {
    onTrimUpdate = _onTrimUpdate;
}

void notFound(AsyncWebServerRequest *request) {
    request->redirect("/");
}

void update_persisted_prefs(Preferences& prefs, pid::pid_trim_t& trim) {
  prefs.putFloat(KEY_KP_PID, trim.kp);
  prefs.putFloat(KEY_KI_PID, trim.ki);
  prefs.putFloat(KEY_KD_PID, trim.kd);
  if (!prefs.isKey(PREF_INITIALIZED) || !prefs.getBool(PREF_INITIALIZED)) {
    prefs.putBool(PREF_INITIALIZED, true);
  }
}

bool read_persisted_prefs(Preferences& prefs, pid::pid_trim_t& trim) {
  if (!prefs.isKey(PREF_INITIALIZED)
      || !prefs.isKey(KEY_KP_PID)
      || !prefs.isKey(KEY_KI_PID)
      || !prefs.isKey(KEY_KD_PID)) {
    return false;
  }
  const float kp = prefs.getFloat(KEY_KP_PID);
  const float ki = prefs.getFloat(KEY_KI_PID);
  const float kd = prefs.getFloat(KEY_KD_PID);

  trim.kp = kp;
  trim.ki = ki;
  trim.kd = kd;
  return true;
}


void init_persisted_prefs(Preferences& prefs) {
  const bool hasPref = read_persisted_prefs(prefs, currentTrim);
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
    data[KEY_KP_PID] = currentTrim.kp;
    data[KEY_KI_PID] = currentTrim.ki;
    data[KEY_KD_PID] = currentTrim.kd;
    serializeJson(data, *response);
    request->send(response);
  });

  server.on("/updateconfig", HTTP_GET, [](AsyncWebServerRequest *request) {
    String val;
    if(request->hasParam(KEY_KP_PID) 
      && (val = request->getParam(KEY_KP_PID)->value()).length()) {
      Serial.print(KEY_KP_PID ": ");
      Serial.println(val);
      currentTrim.kp = val.toFloat();
    }

    if(request->hasParam(KEY_KI_PID)
      && (val = request->getParam(KEY_KI_PID)->value()).length()) {
      Serial.print(KEY_KI_PID ": ");
      Serial.println(val);
      currentTrim.ki = val.toFloat();
    }

    if(request->hasParam(KEY_KD_PID)
      && (val = request->getParam(KEY_KD_PID)->value()).length()) {
      Serial.print(KEY_KD_PID ": ");
      Serial.println(val);
      currentTrim.kd = val.toFloat();
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

