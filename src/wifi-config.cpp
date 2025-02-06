#include "include.h"

const long interval = 10000;  // interval to wait for Wi-Fi connection (milliseconds)

// timing
unsigned long previousMillis, previousDisplay, previousReading;
unsigned int readingId = 0;
unsigned long currentMillis = millis();

// Search for parameter in HTTP POST request
const char* P_INPUT_1 = "ssid";
const char* P_INPUT_2 = "pass";
const char* P_INPUT_3 = "ip";
const char* P_INPUT_4 = "gateway";

int num_nets;
String ssid[MAX_NETS];
String pass[MAX_NETS];
String ip[MAX_NETS];
String gateway[MAX_NETS];

const char* wifiPath = "/wifi.txt";

IPAddress localIP;
//IPAddress localIP(192, 168, 1, 200); // hardcoded

// Set your Gateway IP address
IPAddress localGateway;
//IPAddress localGateway(192, 168, 1, 1); //hardcoded
IPAddress subnet(255, 255, 0, 0);

// Define NTP Client to get time
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP);

String readFile(fs::FS &fs, const char * path){
  //Serial.printf("Reading file: %s\r\n", path);

  File file = fs.open(path);
  if(!file || file.isDirectory()){
    logTo::All("Failed to open file for reading: " + String(path));
    return String();
  }
  
  String fileContent;
  while(file.available()){
    fileContent = file.readStringUntil('\n');
    break;     
  }
  return fileContent;
}

void writeWiFi(int priority, String ssidNew, String passwdNew) {
  ssid[priority] = ssidNew;
  File wifiFile = SPIFFS.open(wifiPath, FILE_WRITE);
  if(!wifiFile) {
    logTo::All("Failed to open file for writing: " + String(wifiPath));
    return;
  }
  for (int i=0; i<MAX_NETS; i++) {  
    if (i == priority) {
      wifiFile.printf("%s:%s::\n",ssidNew,passwdNew);
    } else {
      wifiFile.printf("%s:%s:%s:%s\n",ssid[i],ssid[i],ip[i],gateway[i]);
    }
  }
  close(wifiFile);
}

void writeFile(fs::FS &fs, const char * path, const char * message){
  //Serial.printf("Writing file: %s\r\n", path);

  File file = fs.open(path, FILE_WRITE);
  if(!file){
    logTo::All("Failed to open file for writing: " + String(path));
    return;
  }
  if(file.print(message)){
    //Serial.println("- file written");
  } else {
    //Serial.println("- write failed");
  }
}

bool readWiFi() {
  File file = SPIFFS.open(wifiPath);
  if(!file || file.isDirectory()) {
    logTo::All("failed to open wifi file for reading");
    return false;
  }
  int i=0;
  while(file.available()) {
    ssid[i] = file.readStringUntil(':');
    pass[i] = file.readStringUntil(':');
    ip[i] = file.readStringUntil(':');
    gateway[i] = file.readStringUntil('\n');
    logTo::All("SSID " + ssid[i]);
    logTo::All("passwd " + pass[i]);
    logTo::All("IP " + ip[i]);
    logTo::All("gateway " + gateway[i]);
    i++;
  }
  num_nets=i;
  logTo::All("found " + String(num_nets) + " networks");
  return true;
}

bool initWiFi() {

  int num_tries = 0;
  const int MAX_TRIES = 8;

  if (!readWiFi()) {
    logTo::All("Failed to read WiFi credentials");
    return false;
  }
  WiFi.mode(WIFI_STA);
  for (int i=0; i<num_nets; i++) {
    logTo::All("Found SSID: " + String(i) + " " + ssid[i]);
    if(ssid[i]=="") {
      logTo::All("Undefined SSID.");
      return false;
    }
    localIP.fromString(ip[i].c_str());
    localGateway.fromString(gateway[i].c_str());
    if (!WiFi.config(localIP, localGateway, subnet)) {
      logTo::All("STA Failed to configure");
      return false;
    }
    logTo::All("Connecting to WiFi " + ssid[i]);
    WiFi.begin(ssid[i].c_str(), pass[i].c_str());

    unsigned long currentMillis = millis();
    previousMillis = currentMillis;

    while(WiFi.status() != WL_CONNECTED && num_tries++ < MAX_TRIES) {
      delay(1000);
      //Serial.print(".");
      currentMillis = millis();
      if (currentMillis - previousMillis >= interval) {
        logTo::All("Failed to connect.");
        //return false;
      }
    }
    num_tries = 0;
    if (WiFi.status() == WL_CONNECTED) {
      logTo::All("connected: " + WiFi.localIP().toString());      
      if (!MDNS.begin(host.c_str()))
        logTo::All(F("Error starting MDNS responder"));
      else {
        logTo::All("MDNS started " + host);
      }
      // Add service to MDNS-SD
      if (!MDNS.addService("http", "tcp", HTTP_PORT))
        logTo::All("MDNS add service failed");
#ifdef MDNS
      int n = MDNS.queryService("http", "tcp");
      if (n == 0) {
        logTo::All("No services found");
      } else {
        for (int i = 0; i < n; i++) {
          logTo::All("Service found: ");
          logTo::All(MDNS.hostname(i) + " (" + String(MDNS.IP(i)) + ":" + String(MDNS.port(i)) + ")\n");
        }
      }
#endif
      // Update the time
      #define RETRIES 10
      int count=0;
      while(!timeClient.update() && count++ < RETRIES) {
          //timeClient.forceUpdate();
      }
      logTo::All(timeClient.getFormattedDate());

      return true;
    }
  }
  return false;
}

void startAP() {
    // Connect to Wi-Fi network with SSID and password
    logTo::All("Setting AP (Access Point)");
    // NULL sets an open Access Point
    WiFi.softAP("ESP-WIFI-MANAGER", NULL);

    IPAddress IP = WiFi.softAPIP();
    logTo::All("AP IP address: " + IP.toString());

    // Web Server Root URL
    server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
      request->send(SPIFFS, "/wifimanager.html", "text/html");
    });
  
    server.serveStatic("/", SPIFFS, "/");
    
    server.on("/", HTTP_POST, [](AsyncWebServerRequest *request) {
      int params = request->params();
      for(int i=0;i<params;i++){
        const AsyncWebParameter* p = request->getParam(i);
        if(p->isPost()) {
          //Serial.printf("POST[%s]: %s\n", p->name().c_str(), p->value().c_str());
#if 0
          // HTTP POST ssid value
          if (p->name() == P_INPUT_1) {
            ssid = p->value().c_str();
            Serial.print("SSID set to: ");
            Serial.println(ssid);
            // Write file to save value
            writeFile(SPIFFS, ssidPath, ssid.c_str());
          }
          // HTTP POST pass value
          if (p->name() == P_INPUT_2) {
            pass = p->value().c_str();
            Serial.print("Password set to: ");
            Serial.println(pass);
            // Write file to save value
            writeFile(SPIFFS, passPath, pass.c_str());
          }
          // HTTP POST ip value
          if (p->name() == P_INPUT_3) {
            ip = p->value().c_str();
            Serial.print("IP Address set to: ");
            Serial.println(ip);
            // Write file to save value
            writeFile(SPIFFS, ipPath, ip.c_str());
          }
          // HTTP POST gateway value
          if (p->name() == P_INPUT_4) {
            gateway = p->value().c_str();
            Serial.print("Gateway set to: ");
            Serial.println(gateway);
            // Write file to save value
            writeFile(SPIFFS, gatewayPath, gateway.c_str());
          }
          //Serial.printf("POST[%s]: %s\n", p->name().c_str(), p->value().c_str());
#endif
        }
      }
      request->send(200, "text/plain", "Done. ESP will restart, connect to your router");
      delay(3000);
      ESP.restart();
    });
    server.begin();
}
