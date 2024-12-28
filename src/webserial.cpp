#include "include.h"

#define PRBUF 128
char prbuf[PRBUF];

bool logToSerial = true;
bool debugNMEA = false;

void logTo::logToAll(String s) {
  if (logToSerial) {
    if (s.endsWith("\n")) s.remove(s.length() - 1);
    //String t = "[" + String(millis() / 1000) + "]: ";
    Serial.println(s);
    //consLog.println(s);
    if (serverStarted) {
      WebSerial.print(s);
      //WebSerial.flush();
    }
    s = String();
  }
}

String formatMacAddress(const String& macAddress) {
  String result = "{";
  int len = macAddress.length();
  for (int i = 0; i < len; i += 3) {
    if (i > 0) {
      result += ", ";
    }
    result += "0x" + macAddress.substring(i, i + 2);
  }
  result += "};";
  return result;
}

String commandList[] = {"?", "format", "restart", "ls", "scan", "hostname", "status", "wificonfig", "teleplot", "log", "light", "stop", "go", "relay", "light", "calibrate"};
#define ASIZE(arr) (sizeof(arr) / sizeof(arr[0]))
String words[10]; // Assuming a maximum of 10 words

void WebSerialonMessage(uint8_t *data, size_t len) {
  Serial.printf("Received %lu bytes from WebSerial: ", len);
  Serial.write(data, len);
  Serial.println();
  //WebSerial.print("Received: ");
  String dataS = String((char*)data);
  // Split the String into an array of Strings using spaces as delimiters
  String words[10]; // Assuming a maximum of 10 words
  int wordCount = 0;
  int startIndex = 0;
  int endIndex = 0;
  while (endIndex != -1) {
    endIndex = dataS.indexOf(' ', startIndex);
    if (endIndex == -1) {
      words[wordCount++] = dataS.substring(startIndex);
    } else {
      words[wordCount++] = dataS.substring(startIndex, endIndex);
      startIndex = endIndex + 1;
    }
  }
  for (int i = 0; i < wordCount; i++) {
    int j;
    WebSerial.print(words[i]);
    if (words[i].equals("?")) {
      for (j = 1; j < ASIZE(commandList); j++) {
        WebSerial.print(String(j) + ":" + commandList[j]);
      }
      return;
    }
    if (words[i].equals("format")) {
      SPIFFS.format();
      WebSerial.println("SPIFFS formatted");
      return;
    }
    if (words[i].equals("restart")) {
      WebSerial.println("restarting...");
      ESP.restart();
    }
    if (words[i].equals("ls")) {
      File root = SPIFFS.open("/");
      File file = root.openNextFile();
      while (file) {
        WebSerial.println(file.name());
        file.close(); // Close the file after reading its name
        file = root.openNextFile();
      }
      root.close();
      WebSerial.println("done");
      return;
    }
    if (words[i].equals("scan")) {
      //i2cScan();
      return;
    }
    if (words[i].startsWith("host")) {
      if (!words[++i].isEmpty()) {
        host = words[i];
        preferences.putString("hostname", host);
        logTo::logToAll("hostname set to " + host + "\n");
        logTo::logToAll("restart to change hostname\n");
        logTo::logToAll("preferences " + preferences.getString("hostname") + "\n");
      } else {
        logTo::logToAll("hostname: " + host + "\n");
      }
      return;
    }
    if (words[i].equals("status")) {
      String buf = "";
      unsigned long uptime = millis() / 1000;
      logTo::logToAll("uptime: " + String(uptime));
      buf = String();
      return;
    }
    if (words[i].startsWith("wifi")) {
      String buf = "hostname: " + host;
      buf += " wifi: " + WiFi.SSID();
      buf += " ip: " + WiFi.localIP().toString();
      buf += "  MAC addr: " + formatMacAddress(WiFi.macAddress());
      logTo::logToAll(buf);
      buf = String();
      return;
    }
    if (words[i].startsWith("log")) {
      logToSerial = !logToSerial;
      logTo::logToAll("log: " + String(logToSerial ? "on" : "off"));
      return;
    }
    if (words[i].startsWith("teleplot")) {
      teleplot = !teleplot;
      logTo::logToAll("teleplot: " + String(teleplot ? "on" : "off"));
      return;
    }    
    if (words[i].startsWith("light")) {
      showLight();
      if (!words[++i].isEmpty()) {
        int j;
        String threshS = words[i];
        int thresh = atoi(threshS.c_str());
        logTo::logToAll("setting light threshold to " + threshS + "%");
        for (j=0; j < trigSize; j++)
          setTrigger(&triggers[j], thresh);
      }
#if 0
        String sensorS = words[i];
        int sensor = atoi(sensorS.c_str());
        if (!words[++i].isEmpty()) {
          String levelS = words[i];
          int level = atoi(levelS.c_str());
          logTo::logToAll("setting sensor " + sensorS + " to threshold " + levelS);
          // now go do it
        }
#endif
      return;
    }
    if (words[i].startsWith("stop")) {
      if (!words[++i].isEmpty()) {
        if (words[i].startsWith("i")) {
          logTo::logToAll("stopping inner loop");
          digitalWrite(RELAYIN, HIGH);
        } else if (words[i].startsWith("o")) {
          logTo::logToAll("stopping outer loop");
          digitalWrite(RELAYOUT, HIGH);
        }
      } else {
        logTo::logToAll("stopping both");
        digitalWrite(RELAYIN, HIGH);
        digitalWrite(RELAYOUT, HIGH);          
      }
      return;
    }    
    if (words[i].startsWith("start")) {
      if (!words[++i].isEmpty()) {
        if (words[i].startsWith("i")) {
          logTo::logToAll("starting inner loop");
          digitalWrite(RELAYIN, LOW);
        } else if (words[i].startsWith("o")) {
          logTo::logToAll("starting outer loop");
          digitalWrite(RELAYOUT, LOW);
        }
      } else {
        logTo::logToAll("starting both");
        digitalWrite(RELAYIN, LOW);
        digitalWrite(RELAYOUT, LOW);
      }
      return;
    }
    if (words[i].startsWith("rel")) {
      int i,j;
      i = digitalRead(RELAYIN);
      j = digitalRead(RELAYOUT);
      logTo::logToAll("relay status inner/mountain loop: " + String(i) + " outer/flat loop: " + String(j));
      return;
    }    
    if (words[i].startsWith("cal")) {
      calibrateLight(LIGHT_CYCLES);
      return;
    }

    logTo::logToAll("Unknown command: " + words[i]);
  }
  for (int i=0; i<wordCount; i++) words[i] = String();
  dataS = String();
}
