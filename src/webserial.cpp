#include "include.h"

#define PRBUF 128
char prbuf[PRBUF];

bool logToSerial = true;
bool debugNMEA = false;

void logTo::All(String s) {
  if (s.endsWith("\n")) s.remove(s.length() - 1);
  if (logToSerial) {
    Serial.println(s);
    //consLog.println(s);
  }
  // to test serial connection to turnout controller
  Serial2.println(s);
  if (serverStarted) {
    WebSerial.print(s);
    //WebSerial.flush();
  }
  s = String();
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

String commandList[] = {"?", "host","wifi","log","teleplot","sens","stop [i/o]","start [i/o]","relay","calibrate","state","<","speed","brake","resume","disable","enable","loco [ns/ew]","cab","serial"};
#define ASIZE(arr) (sizeof(arr) / sizeof(arr[0]))

void WebSerialonMessage(uint8_t *data, size_t len) {
  //Serial.printf("Received %lu bytes from WebSerial: ", len);
  //Serial.write(data, len);
  //Serial.println();
  //WebSerial.print("Received: ");
  String dataS = String((char*)data);
  serialWrapper(dataS);
  dataS = String();
}

void serialWrapper(String dataS) {
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
        preferences.begin("ESPprefs", false);
        host = words[i];
        preferences.putString("hostname", host);
        logTo::All("hostname set to " + host);
        logTo::All("restart to change hostname");
        logTo::All("preferences " + preferences.getString("hostname"));
        preferences.end();
      } else {
        logTo::All("hostname: " + host + "\n");
      }
      return;
    }
    if (words[i].equals("status")) {
      String buf = "";
      unsigned long uptime = millis() / 1000;
      logTo::All("uptime: " + String(uptime));
      buf = String();
      return;
    }
    if (words[i].startsWith("wifi")) {
      String buf = "hostname: " + host;
      buf += " wifi: " + WiFi.SSID();
      buf += " ip: " + WiFi.localIP().toString();
      buf += "  MAC addr: " + formatMacAddress(WiFi.macAddress());
      logTo::All(buf);
      buf = String();
      return;
    }
    if (words[i].startsWith("log")) {
      logToSerial = !logToSerial;
      logTo::All("log: " + String(logToSerial ? "on" : "off"));
      return;
    }
    if (words[i].startsWith("teleplot")) {
      teleplot = !teleplot;
      logTo::All("teleplot: " + String(teleplot ? "on" : "off"));
      return;
    }    
    if (words[i].startsWith("sens")) {
      showSensors();
      if (!words[++i].isEmpty()) {
        int j;
        String threshS = words[i];
        int thresh = atoi(threshS.c_str());
        logTo::All("setting light threshold to " + threshS + "%");
        for (j=0; j < trigSize; j++)
          setTrigger(&sensors[j], thresh);
      }
#if 0
        String sensorS = words[i];
        int sensor = atoi(sensorS.c_str());
        if (!words[++i].isEmpty()) {
          String levelS = words[i];
          int level = atoi(levelS.c_str());
          logTo::All("setting sensor " + sensorS + " to threshold " + levelS);
          // now go do it
        }
#endif
      return;
    }
    if (words[i].startsWith("stop")) {  // TBD change to DCC-EX
      if (!words[++i].isEmpty()) {
        if (words[i].startsWith("i")) {
          logTo::All("stopping inner loop");
          digitalWrite(RELAYIN, HIGH);
        } else if (words[i].startsWith("o")) {
          logTo::All("stopping outer loop");
          digitalWrite(RELAYOUT, HIGH);
        }
      } else {
        logTo::All("stopping both");
        digitalWrite(RELAYIN, HIGH);
        digitalWrite(RELAYOUT, HIGH);          
      }
      return;
    }    
    if (words[i].startsWith("start")) {
      if (!words[++i].isEmpty()) {
        if (words[i].startsWith("i")) {
          logTo::All("starting inner loop");
          digitalWrite(RELAYIN, LOW);
        } else if (words[i].startsWith("o")) {
          logTo::All("starting outer loop");
          digitalWrite(RELAYOUT, LOW);
        }
      } else {
        logTo::All("starting both");
        digitalWrite(RELAYIN, LOW);
        digitalWrite(RELAYOUT, LOW);
      }
      return;
    }
    if (words[i].startsWith("rela")) {
      int i,j;
      i = digitalRead(RELAYIN);
      j = digitalRead(RELAYOUT);
      String statusS = enableRelays ? "enabled" : "disabled";
      logTo::All("relay status: " + statusS);
      logTo::All("inner/mountain loop: " + String(i) + " outer/flat loop: " + String(j));
      return;
    }    
    if (words[i].startsWith("turn")) {
      if (!words[++i].isEmpty()) {
        int turn = atoi(words[i].c_str());
        if (turn > 0) {
          if (!words[++i].isEmpty()) {
            if (words[i].startsWith("s")) {
              logTo::All("throw " + String(turn) + " straight");
              throwTurnout(turn, true);
            } else if (words[i].startsWith("r")) {
              logTo::All("throw " + String(turn) + " branch");
              throwTurnout(turn, false);
            }
          }
        }
      } else {
        // right now I haven't figured out how to give diamond-main access to turnout state.
        // perhaps via response from throwturn()
        //showTurnsigState();
      }
      return;
    }
#if 0
    if (words[i].startsWith("trig")) {
      if (!words[++i].isEmpty()) {
        int trigtime = atoi(words[i].c_str());
        if (trigtime > 30000) trigtime = 30000;
        if (trigtime < 0) trigtime = 1;
        turnTime = trigtime;
      }
      logTo::All("turnout trigger time: " + String(turnTime));
      return;
    }
#endif
    if (words[i].startsWith("loop")) {
      if (!words[++i].isEmpty()) {
        if (words[i].startsWith("on"))
          loopMode = true;
        else if (words[i].startsWith("off"))
          loopMode = false;
      }
      logTo::All("loop mode: " + String(loopMode));
      return;
    }
    if (words[i].startsWith("cali")) {
      calibrateSensors(LIGHT_CYCLES);
      return;
    }
    if (words[i].startsWith("state")) {
      logTo::All("signals: ");
      printSigsigState();
      logTo::All("crossing: " + printCrosssigState(crosssigState));
      return;
    }
    if (words[i].startsWith("<")) {
      // echo "<>" commands to DCC-EX command station (entire string not just words[])
      WebSerial.print(dataS);
      //Serial.println((char*)data);
      return;
    }
#if 0
    if (words[i].startsWith("speed")) {
      if (!words[++i].isEmpty()) {
        int loco = atoi(words[i].c_str());
        int speed = getSpeed(loco,1000);
        logTo::All("loco " + String(loco) + " speed is " + String(speed));
      }
      return;
    }
    if (words[i].startsWith("brake")) {
       if (!words[++i].isEmpty()) {
        int loco = atoi(words[i].c_str());
        brake(loco);
      }   
      return;
    }
    if (words[i].startsWith("resume")) {
      if (!words[++i].isEmpty()) {
        int loco = atoi(words[i].c_str());
        resume(loco);
      }    
      return;
    }
#endif
    if (words[i].startsWith("disable")) {
      enableRelays = false;
      return;
    }
    if (words[i].startsWith("enable")) {
      enableRelays = true;
      for (int i=0; i<NUM_SIGS; i++) {
        if (signals[i].state != Y)
          setLED(i,Y);
      }
      crosssigState = Clear;
      return;
    }
    if (words[i].startsWith("loco")) {
      int s;
      if (!words[++i].isEmpty()) {
        if (words[i].startsWith("ns")) {
          if (!words[++i].isEmpty()) {
            int loco = atoi(words[i].c_str());
#if 0
            if (int speed = getSpeed(loco,10000) >= 0) {
              // don't set loco if we can't talk to DCC
              logTo::All("loco " + String(loco) + " speed is " + String(speed));
              NSlocoID = loco;
              preferences.begin("ESPprefs", false);
              preferences.putInt("NSlocoID", NSlocoID);
              preferences.putUInt("ns-loco", NSlocoID);
              logTo::All("DCC ping OK setting NSloco");
              preferences.end();
            } else logTo::All("cannot reach DCC");
#endif
          }
        } else if (words[i].startsWith("ew")) {
          if (!words[++i].isEmpty()) {
            int loco = atoi(words[i].c_str());
#if 0
            if (int speed = getSpeed(loco,10000) >= 0) {
              logTo::All("loco " + String(loco) + " speed is " + String(speed));
              // don't set loco if we can't talk to DCC
              EWlocoID = loco;
              preferences.begin("ESPprefs", false);
              preferences.putInt("EWlocoID", EWlocoID);
              preferences.putUInt("ew-loco", EWlocoID);
              logTo::All("DCC ping OK setting EWloco");
              preferences.end();
            } else logTo::All("cannot reach DCC");
#endif
          }
        }
        setSigLoco();
      }
      for (s=0; s<NUM_SIGS; s++) {
        logTo::All("signal:" + String(s) + " loco:" + String(signals[s].loco->getAddress()));
      }
      logTo::All("NSloco: " + String(NSlocoID));
      logTo::All("EWloco: " + String(EWlocoID));
      return;
    }
#if 0
    if (words[i].startsWith("cab")) {
      if (!words[++i].isEmpty())
        int cabNo = atoi(words[i].c_str());
      for (int i=0; i<MAXLOC; i++) {
        logTo::All("cab: " + String(i) + " speed: " + String(cabDCC[i].speed));
      }
      return;
    }
#endif
    if (words[i].startsWith("serial")) {
      unsigned long startTime = millis();
      String receivedString;
      int timeout=60;
      if (!words[++i].isEmpty())
        int timeout = atoi(words[i].c_str());
      if (timeout > 300) timeout = 300;
      while (!Serial1.available()) {
        unsigned long now = millis();
        if (now - startTime > timeout*1000) {
          // Handle timeout condition
          logTo::All("timed out waiting for status response from DCC-EX " + String(now) + "-" + String(startTime) + "=" + String(now-startTime) + " " + String(timeout));
          break;
        }
        yield();
      }
      while (Serial1.available() > 0) {
        receivedString = Serial1.readString();
        logTo::All("DCC rx: " + receivedString);
      }
      return;
    }
    logTo::All("Unknown command: " + words[i]);
  }
  for (int i=0; i<wordCount; i++) words[i] = String();
  dataS = String();
}
