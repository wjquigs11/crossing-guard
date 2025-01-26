#include "include.h"

Adafruit_MCP23X17 mcp[MCP];
int MCPaddr[] = {0x21, 0x20};
// note B0 == GPIO8
// also note B0 is across from A7

// PROBLEM: GPIO is going high but driver output is not going to 19V!!!
// would help if power was on

TurnoutQ layout[TURNOUTS] = {
  // mcp, togA, togB, togStateA, togStateB, solA, solB, ledA, ledB, state
  {0, 12, 13, false, false, 14, 15, 0, 0, true}, // 1: westernmost turnout at tunnel entrance
  {0, 2, 3, false, false, 0, 1, 0, 0, true},     // 2: S2 double slip (going west)
  {0, 6, 7, false, false, 4, 5, 0, 0, true},     // 3: S3 double slip (going east)
  // MCP 1 in future so LEDs can be on same MCP as their switches
  {0, 8, 9, false, false, 11, 10, 0, 0, true}    // 4: easternmost turnout (solenoid doesn't look like it's working both ways)
};
// NOTE: when 2 goes reverse, 1 should also go reverse, if 1 goes reverse, 2 should go reverse, 
//       but if 1 or 2 goes straight, do nothing to the other
// NOTE: for "single loop mode" if sensor triggers 3x, also set 2x, and if sensor triggers 3s, also set 2s
//                           if sensor triggers 2s, set 3s...
// and set turnout 4 to reverse in single loop mode
// I will either need another sensor in the tunnel on the outer track, 
// or I will switch #1 to reverse when the eastern sensor is triggered

void setupTurnout() {
  int i;
#if 0 // init in main
  for (i=0; i<MCP; i++) {
    Serial.printf("init MCP %d\n", i);
    if (!mcp[i].begin_I2C(MCPaddr[i])) {
      Serial.printf("Error on init of mcp[%d] at 0x%x", i, MCPaddr[i]);
      while (1);
    }
  }
#endif
  Serial.println("init turnouts");
  for (i=0; i<TURNOUTS; i++) {
    int thisMCP = layout[i].mcp;
    if (thisMCP < MCP) {
      Serial.printf("%d: mcp: %d togA: %d togB: %d solA: %d solB: %d\n", i, thisMCP, layout[i].toggleA, layout[i].toggleB,layout[i].solenoidA, layout[i].solenoidB);
      mcp[thisMCP].pinMode(layout[i].toggleA,INPUT_PULLUP);
      mcp[thisMCP].pinMode(layout[i].toggleB,INPUT_PULLUP);
      mcp[thisMCP].pinMode(layout[i].solenoidA,OUTPUT);
      mcp[thisMCP].pinMode(layout[i].solenoidB,OUTPUT);
      mcp[thisMCP].digitalWrite(layout[i].solenoidA,LOW);
      mcp[thisMCP].digitalWrite(layout[i].solenoidB,LOW);
      // should I set all turnouts to straight at the beginning? If not how do I know which direction they are pointed?
      //throwTurnout(i, false); 
      //throwTurnout(i, true);
    }
  }
}

unsigned long now, startTime;
int turnTime = TRIGTIME;
bool loopMode = false;

void checkToggles() {
  int i, thisMCP;
  for (i=0; i<TURNOUTS; i++) {
    thisMCP = layout[i].mcp;
    bool currentStateA = mcp[thisMCP].digitalRead(layout[i].toggleA);
    bool currentStateB = mcp[thisMCP].digitalRead(layout[i].toggleB);
    bool lastStateA = layout[i].togStateA;
    bool lastStateB = layout[i].togStateB;
    // Only trigger when state changes from HIGH to LOW
    now = startTime = millis();
    if (currentStateA == LOW && lastStateA == HIGH) {
      throwTurnout(i,true);
    } else if (currentStateB == LOW && lastStateB == HIGH) {
      throwTurnout(i,false);
  }
  layout[i].togStateA = currentStateA;
  layout[i].togStateB = currentStateB;
  }
}

void throwWrapper(int t, bool d) {
  throwTurnout(t, d);
  if (loopMode) {
    if (t == 2) { // throwing turnout 2
      throwTurnout(3,d);  // throw 3 same direction
    }
    if (t == 3) { // turnout 3
      throwTurnout(2,d);
    }
  }
  // now evaluate all turnouts that face other turnouts and throw if necessary
  if (t == 2 && !d) {
    // if I throw turnout 2 reverse, need to throw 1 reverse also
    throwTurnout(1,false);  
  }
  // but I also need to throw 2 reverse if 1 gets thrown...will this result in deadly embrace or will state prevail?
  if (t == 1 && !d) {
    throwTurnout(2,false);  
  }
  if (t == 1 && d) {
    // 1 straight means 2 must also be straight
    throwTurnout(2,true);
  }
  if (t == 2 && d) {
    throwTurnout(1,true);
  }
}

void throwTurnout(int t, bool d) {
  // t = turnout number, straight == true/false
  // TBD check state first
  // this function gets a "logical" turnout numbered 1..X
  logTo::logToAll("evaluate t " + String(t) + " state " + (layout[t-1].state ? "straight" : "reverse"));
  t--;
  int thisMCP = layout[t].mcp;
  now = startTime = millis();
  ledOn();
  if (!layout[t].state && d) {
    logTo::logToAll("turnout " + String(t+1) + " straight, mcp " + String(thisMCP) + " sol " + String(layout[t].solenoidA) + " time " + String(turnTime));
    mcp[thisMCP].digitalWrite(layout[t].solenoidA,HIGH);
    while (now - startTime < turnTime) {
      now = millis();
      delay(3);
    }
    logTo::logToAll("going low");
    mcp[thisMCP].digitalWrite(layout[t].solenoidA,LOW);
    layout[t].state = true;
  } else if (layout[t].state && !d) {
    // reverse/curve
    if (t == 3) turnTime *= 4; // desperate attempt to get peco to work
    logTo::logToAll("turnout " + String(t+1) + " reverse, mcp " + String(thisMCP) + " sol " + String(layout[t].solenoidB) + " time " + String(turnTime));
    mcp[thisMCP].digitalWrite(layout[t].solenoidB,HIGH);
    while (now - startTime < turnTime) {
      now = millis();
      delay(3);
    }
    logTo::logToAll("going low");
    mcp[thisMCP].digitalWrite(layout[t].solenoidB,LOW);
    if (t+1 == 4) turnTime /= 4; // desperate attempt to get peco to work
    layout[t].state = false; 
  }
  char tName[10];
  sprintf(tName,"turnout%d",t);
  preferences.putBool(tName, layout[t].state);
  ledOff();
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
        Serial.printf("%d: %s\n", i+1, layout[i].state ? "straight" : "reverse");
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
