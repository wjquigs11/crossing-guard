#include "include.h"

AsyncWebServer server(HTTP_PORT);
AsyncEventSource events("/events");
AsyncWebSocket ws("/ws");
bool serverStarted;
JsonDocument readings;
String host = "Xguard";
using namespace reactesp;
extern ReactESP app;

String getSensorReadings() {
  readings["boatHeading"] = "180"; //String(pBD->TrueHeading,0);
  String jsonString;
  serializeJson(readings,jsonString); // returns an int?
  return jsonString;
}

String processor(const String& var) {
  Serial.println(var);
  if(var == "TIMERDELAY") {
    return String(WebTimerDelay);
  }
  return String();
}

void startWebServer() {
  logTo::logToAll("starting web server");

  // start serving from SPIFFS
  server.serveStatic("/", SPIFFS, "/").setDefaultFile("index.html");
  server.serveStatic("/", SPIFFS, "/");

  server.on("/", HTTP_GET, [](AsyncWebServerRequest * request) {
    logTo::logToAll("index.html");
    request->send(SPIFFS, "/index.html", "text/html", false, processor);
  });

  // Request latest sensor readings
  server.on("/readings", HTTP_GET, [](AsyncWebServerRequest *request) {
    //logToAll("getSensorReadings\n");
    String json = getSensorReadings();
    //logToAll(readings);
    request->send(200, "application/json", json);
    json = String();
  });

  server.on("/host", HTTP_GET, [](AsyncWebServerRequest *request) {
    String buf = "hostname: " + host;
    buf += "ESP local MAC addr: " + String(WiFi.macAddress() + "\n");
    logTo::logToAll(buf);
    request->send(200, "text/plain", buf.c_str());
    buf = String();
  });

  server.on("/config", HTTP_GET, [](AsyncWebServerRequest *request) {
    logTo::logToAll("config\n");
    String response = "none";
    if (request->hasParam("hostname")) {
      Serial.printf("hostname %s\n", request->getParam("hostname")->value().c_str());
      host = request->getParam("hostname")->value();
      response = "change hostname to " + host + "\n";
      logTo::logToAll(response);
      preferences.putString("hostname",host);
      logTo::logToAll("preferences " + preferences.getString("hostname", "unknown") + "\n");
    } else if (request->hasParam("webtimer")) {
      WebTimerDelay = atoi(request->getParam("webtimer")->value().c_str());
      if (WebTimerDelay < 0) WebTimerDelay = DEFDELAY;
      if (WebTimerDelay > 10000) WebTimerDelay = 10000;
      response = "change web timer to " + String(WebTimerDelay);
      logTo::logToAll(response);
      preferences.putInt("timerdelay",WebTimerDelay); 
    }
    //request->send(SPIFFS, "/index.html", "text/html");
    //request->redirect("/index.html");
    request->send(200, "text/plain", response.c_str());
    response = String();
  }); 

  events.onConnect([](AsyncEventSourceClient *client){
    if(client->lastId()){
      Serial.printf("Client reconnected! Last message ID that it got is: %u\n", client->lastId());
    }
    // send event with message "hello!", id current millis
    // and set reconnect delay to 1 second
    client->send("hello!", NULL, millis(), 1000);
  });
  server.addHandler(&events);
}
