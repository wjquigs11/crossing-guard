#ifdef TURNOUT
#include "include.h"

#define SDA1 17
#define SCL1 16
Adafruit_MCP23X17 mcp;
int mcpT = 0x21;
using namespace reactesp;
ReactESP app;
Preferences preferences;

uint32_t i = 0;

// note B0 == GPIO8
// also note B0 is across from A7

TurnoutQ layout[TURNOUTS] = {
  // mcp, togA, togB, togsigStateA, togsigStateB, solA, solB, ledA, ledB, state
  // note solenoid GPIOs are now directly connected to ESP32, not MCP
  // LEDs and toggles are on MCP
  {0, 0, 1, false, false, 13, 12, 4, 5, true}, // 1: westernmost turnout at tunnel entrance
  {0, 11, 13, false, false, 33, 32, 7, 8, true},     // 2: S3 double slip (going west)
  {0, 2, 3, false, false, 26, 25, 6, 5, true},     // 3: S2 double slip (going east)
  {0, 14, 15, false, false, 27, 14, 9, 10, true}    // 4: easternmost turnout (solenoid doesn't look like it's working both ways)
  // TEMP: connecting 14 to relay
};
// NOTE: when 2 goes branch, 1 should also go branch, if 1 goes branch, 2 should go branch, 
//       but if 1 or 2 goes straight, do nothing to the other
// NOTE: for "single loop mode" if sensor triggers 3x, also set 2x, and if sensor triggers 3s, also set 2s
//                           if sensor triggers 2s, set 3s...
// and set turnout 4 to branch in single loop mode
// I will either need another sensor in the tunnel on the outer track, 
// or I will switch #1 to branch when the eastern sensor is triggered

unsigned long now, startTime;
int turnTime = TRIGTIME;

void checkToggles() {
  int i;
  for (i=0; i<TURNOUTS; i++) {
    bool currentsigStateA = mcp.digitalRead(layout[i].toggleA);
    bool currentsigStateB = mcp.digitalRead(layout[i].toggleB);
    //Serial.printf("turn %d tog %d/%d: %d/%d\n",i+1,layout[i].toggleA,layout[i].toggleB,mcp.digitalRead(layout[i].toggleA),mcp.digitalRead(layout[i].toggleB));
    bool lastsigStateA = layout[i].togsigStateA;
    bool lastsigStateB = layout[i].togsigStateB;
    // Only trigger when state changes from HIGH to LOW
    now = startTime = millis();
    if (currentsigStateA == LOW && lastsigStateA == HIGH) {
      throwTurnout(i+1,true);
    } else if (currentsigStateB == LOW && lastsigStateB == HIGH) {
      throwTurnout(i+1,false);
  }
  layout[i].togsigStateA = currentsigStateA;
  layout[i].togsigStateB = currentsigStateB;
  }
}

void printsigState() {
  for (i=0; i<TURNOUTS; i++) {
    Serial.printf("turn %d tog %d/%d: %d/%d sol %d %d\n",i+1,layout[i].toggleA,layout[i].toggleB,mcp.digitalRead(layout[i].toggleA),
      mcp.digitalRead(layout[i].toggleB),digitalRead(layout[i].solenoidA),digitalRead(layout[i].solenoidB));
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
    // if I throw turnout 2 branch, need to throw 1 branch also
    throwTurnout(1,false);  
  }
  // but I also need to throw 2 branch if 1 gets thrown...will this result in deadly embrace or will state prevail?
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
  // t ranges from 1..X
  // TBD check state first
  // this function gets a "logical" turnout numbered 1..X
  logTo::All("evaluate t " + String(t) + " state " + (layout[t-1].state ? "straight" : "branch"));
  t--;
  now = startTime = millis();
  ledOn();
  if (!layout[t].state && d) {
    logTo::All("turnout: " + String(t+1) + " straight, sol: " + String(layout[t].solenoidA) + " time: " + String(turnTime) + "LED on/off: " + String(layout[t].LEDB) + "/" + String(layout[t].LEDA));
    mcp.digitalWrite(layout[t].LEDB,HIGH);
    mcp.digitalWrite(layout[t].LEDA,LOW);
    digitalWrite(layout[t].solenoidA,HIGH);
    while (now - startTime < turnTime) {
      now = millis();
      delay(3);
    }
    logTo::All("going low");
    digitalWrite(layout[t].solenoidA,LOW);
    layout[t].state = true;
  } else if (layout[t].state && !d) {
    int TT = turnTime;
    //if (t==3) TT = turnTime*50;
    // branch/curve
    logTo::All("turnout: " + String(t+1) + " branch, sol: " + String(layout[t].solenoidA) + " time: " + String(TT) + " LED on/off: " + String(layout[t].LEDA) + "/" + String(layout[t].LEDB));
    mcp.digitalWrite(layout[t].LEDA,HIGH);
    mcp.digitalWrite(layout[t].LEDB,LOW);
    digitalWrite(layout[t].solenoidB,HIGH);
    while (now - startTime < TT) {
      now = millis();
      delay(3);
    }
    logTo::All("going low");
    digitalWrite(layout[t].solenoidB,LOW);
    /*if (t == 3) {
      logTo::All("turnout 4 again...");
      digitalWrite(layout[t].solenoidB,HIGH);
      while (now - startTime < turnTime) {
        now = millis();
        delay(3);
      }      
      digitalWrite(layout[t].solenoidB,LOW);
    }*/
    layout[t].state = false; 
  }
  char tName[10];
  sprintf(tName,"turnout%d",t);
  preferences.putBool(tName, layout[t].state);
  ledOff();
}

// master requests data
void onRequest() {
  Wire.print(i++);
  Wire.print(" Packets.");
  Serial.println("onRequest");
  Serial.println();
}

// master sends data
void onReceive(int len) {
  String message = "";
  char prefix;
  int t;
  bool d;
  char dir;
  
  // Read the full message first
  while (Wire.available()) {
    message += (char)Wire.read();
  }
  //Serial.println(message);
  // Check if message matches expected format
  if (len >= 5 && message[0] == 'T' && message[1] == ':') {
    prefix = message[0];
    t = message.substring(2, message.lastIndexOf(':')).toInt();
    dir = message[message.length()-2];
    Serial.printf("len: %d Prefix: %c, Turnout: %d, Dir: %c\n", len, prefix, t, dir);
    // Validate direction is 's' or 'b'
    if (dir == 's' || dir == 'b') {
      d = (dir == 's'?true:false);
    } else {
      Serial.println("Invalid direction format");
    }
  } else {
    Serial.println("Invalid message format");
  }
}

void setup() {
  Serial.begin(115200); delay(300);
  Serial.printf("slave begin SDA: %d SCL: %d\n",SDA,SCL);
  Serial.setDebugOutput(true);
  Wire.onReceive(onReceive);
  Wire.onRequest(onRequest);
  Wire.begin((uint8_t)I2C_DEV_ADDR);

  // start 2nd i2c bus for MCP
  Wire1.begin(SDA1,SCL1);
  Serial.println("init MCP");
  if (!mcp.begin_I2C(mcpT,&Wire1)) {
    Serial.printf("Error on init of mcp at 0x%x\n", mcpT);
    i2cScan(Wire);
  }

  Serial.println("init turnouts");
  preferences.begin("ESPprefs", false);
  for (i=0; i<TURNOUTS; i++) {
    char tName[10];
    sprintf(tName,"turnout%d",i);
    layout[i].state = preferences.getBool(tName, false);
    char buf[64];
    sprintf(buf,"%d: togA: %d togB: %d solA: %d solB: %d, state:%s", i+1, layout[i].toggleA, layout[i].toggleB,layout[i].solenoidA, layout[i].solenoidB, (layout[i].state?"straight":"branch"));
    logTo::All(buf);
    mcp.pinMode(layout[i].toggleA,INPUT_PULLUP);
    mcp.pinMode(layout[i].toggleB,INPUT_PULLUP);
    // motor drivers direct to ESP32
    pinMode(layout[i].solenoidA,OUTPUT);
    pinMode(layout[i].solenoidB,OUTPUT);
    digitalWrite(layout[i].solenoidA,LOW);
    digitalWrite(layout[i].solenoidB,LOW);
    mcp.pinMode(layout[i].LEDA,OUTPUT);
    mcp.pinMode(layout[i].LEDB,OUTPUT);
    mcp.digitalWrite(layout[i].LEDA,LOW);
    mcp.digitalWrite(layout[i].LEDB,LOW);
    bool currentsigStateA = mcp.digitalRead(layout[i].toggleA);
    bool currentsigStateB = mcp.digitalRead(layout[i].toggleB);
    Serial.printf("%d A:%d/%d/%d B:%d/%d/%d\n",i+1,layout[i].toggleA,currentsigStateA,mcp.digitalRead(layout[i].toggleA),layout[i].toggleB,currentsigStateB,mcp.digitalRead(layout[i].toggleB));
    // pin 12 is always reading low even with toggle disengaged.
    // will swap for motor (output) pins after everything is working
    // should I set all turnouts to straight at the beginning? If not how do I know which direction they are pointed?
    //throwTurnout(i+1, false); 
    //throwTurnout(i+1, true);
  }

  // check toggle switches for turnouts
  // turnouts can also be thrown by commands over i2c from diamond-main, via onReceive() which doesn't need to be scheduled
  app.onRepeat(LOOPDELAY, []() {
    checkToggles();
  });

#if 0
  app.onRepeat(1000, []() {
    printsigState();
  });
#endif
/*#if CONFIG_IDF_TARGET_ESP32
  char message[64];
  snprintf(message, 64, "%lu Packets.", i++);
  Wire.slaveWrite((uint8_t *)message, strlen(message));
  Serial.print('Printing config %lu', i);
#endif*/
}

void loop() {
  app.tick(); 
}
#endif