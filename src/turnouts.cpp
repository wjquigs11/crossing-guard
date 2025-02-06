#include "include.h"

// this is probably a terrible idea but I will fix it "someday"
// the master ESP32 (DIAMOND) will simply send turnout throw data to the slave (TURNOUT)
// the same function in the slave will actually move the turnout, set LEDs, respond to toggles, etc
#ifdef DIAMOND
void throwTurnout(int t, bool d) {
  char buf[256];
  sprintf(buf,"T:%d:%c\n",t,(d?'s':'b'));
  logTo::All(buf);
  //myMux.setPort(7);
  if (turnControlConnected) {
    Wire.beginTransmission(I2C_DEV_ADDR);
    Wire.printf(buf);
    uint8_t error = Wire.endTransmission(true);
    if (error) {
      turnControlConnected = false;
    }
    //Serial.printf("endTransmission: %u\n", error);

    // Read 16 bytes from the slave
    uint8_t bytesReceived = Wire.requestFrom(I2C_DEV_ADDR, 16);
    
    Serial.printf("requestFrom: %u\n", bytesReceived);
    if ((bool)bytesReceived) {  //If received more than zero bytes
      uint8_t temp[bytesReceived];
      Wire.readBytes(temp, bytesReceived);
      sprintf(buf,"%s",temp);
      logTo::All("throw response: " + String(buf));
      //log_print_buf(temp, bytesReceived);
    }
  }
}
#endif
void showTurnsigState() {
  for (int j=0; j<TURNOUTS; j++)
    logTo::All("turnout: " + String(j+1) + " state:" + ((layout[j].state ? "straight" : "branch")));
}

#if STANDALONE
String commandList[] = {"?", "format", "restart", "ls", "scan", "hostname", "status", "wificonfig", "teleplot", "log", "sens", "stop", "go", "relay", "calibrate", "disable", "enable"};
#define ASIZE(arr) (sizeof(arr) / sizeof(arr[0]))

void parseSerial(String dataS) {
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
    //WebSerial.print(words[i]);
    if (words[i].equals("?")) {
      for (j = 1; j < ASIZE(commandList); j++) {
        Serial.printf("%d:%s\n",j,commandList[j].c_str());
      }
      return;
    }
    int turn = atoi(words[i].c_str());
    if (turn>0 && turn<=TURNOUTS) {
      turn--;
      // switch number
      if (!words[++i].isEmpty()) {
        words[i].toUpperCase();
        if (words[i].startsWith("S")) {
          // straight/normal direction
          throwTurnout(turn,true);
        } else
          throwTurnout(turn,false);
      }
      return;
    }
    if (words[i].startsWith("state")) {
      for (int i=0; i<TURNOUTS; i++) {
        Serial.printf("%d: %s\n", i+1, layout[i].state ? "straight" : "branch");
      }
      return;
    }
  }
}
#endif

#if STANDALONE
void loop() {
#else
void loopTurnout() {
#endif
  checkToggles();
#if STANDALONE
  String input = "";
  if (Serial.available() > 0) {
    input = Serial.readString();
    input.trim();  // Remove whitespace and newlines
    Serial.print("serial: ");
    Serial.println(input);
    parseSerial(input);
  }
#endif
}
