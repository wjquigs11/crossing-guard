//
// diamond crossing guard: turn off one track if there's a risk of collision at a diamond crossing
// also controls signals on each branch of the crossing
//

#include "include.h"

#define SDA 25 // crossing guard under layout
#define SCL 15

File consLog;
using namespace reactesp;
ReactESP app;
Preferences preferences;
int WebTimerDelay;
bool startWifi = false;
bool teleplot = false;

bool stopInner = false;
bool stopOuter = false;
bool enableRelays = true;

int NSlocoID, EWlocoID;
int saveSpeed = -1; // to restore after braking
Direction saveDir;
String DCCEXhostname;

// each sensor represents a photoresistor or IR sensor
// the west side of my crossing has a turnout so it needs two sensors
// note sensor numbering starts at 0 but turnout numbering starts at 1
// TBD: some sensors might have 2 turnouts associated, depending on train direction
// e.g. sensors 2 and 5 should trigger turnout 4 going east and 2 or 3 going west
struct Sensor sensors[] = {
  // location, activated, initial value, current value, average value, sensor index, trigger level, 
  //    current state, previous state, turnout East, direction East, turnout West, direction West
  {E, false, 0, 0, MOVINGAVG, 3, TRIGLEVEL, false, false, 0, true, 0, false},
  {W, false, 0, 0, MOVINGAVG, 0, TRIGLEVEL, false, false, 4, false, 3, true},      
  {S, false, 0, 0, MOVINGAVG, 2, TRIGLEVEL, false, false, 0, true, 0, false},      
  {N, false, 0, 0, MOVINGAVG, 1, TRIGLEVEL, false, false, 3, false, 0, false},       
  {W, false, 0, 0, MOVINGAVG, 4, TRIGLEVEL, false, false, 2, true, 0, false},      
  {nulldir, false, 0, 0, MOVINGAVG, 5, TRIGLEVEL, false, false, 2, true, 0, false} // 5 is in the tunnel west of the double slip, not at the crossing
};
int trigSize = sizeof(sensors) / sizeof(sensors[0]);
//int trigSize = 1;

QWIICMUX myMux;
SFEVL53L1X irSensor[MUXMAX];
bool sensorOn[MUXMAX];

// last time any sensor was triggered
// used to debounce gaps between cars that allow light on sensor
// wait CLEARDELAY seconds after lastTrigTime before clearing signals
unsigned long lastTrigTime; 

// total light level measured across all sensors
int totalLight, newTotalLight;


CrossDir lastTrigCrossDir = nulldir;  // Store the direction of last trigger
bool waitingForClear = false;  // Flag to indicate we're waiting for opposite direction

blockState crossState = Clear;
CrossDir fromDir = nulldir;

// signals are defined by the GPIOs each LED is connected to, the location, and the state
// NB at initialization all locoIDs will be 0
Signal signals[NUM_SIGS] = {
    {{23, 22, 21}, S, Y, NORTHSOUTH, 0},
    {{19, 18, 5}, N, Y, NORTHSOUTH, 0},
    {{17, 16, 4}, W, Y, EASTWEST, 0},
    {{14, 12, 13}, E, Y, EASTWEST, 0}
};

CrossDir complement(CrossDir dir) {
    switch (dir) {
        case N: return S;
        case S: return N;
        case E: return W;
        case W: return E;
        default: return nulldir;
    }
}

void setTrigger(Sensor *trigger, int trigLevel) {
  trigger->trigLevel = trigLevel;
}

String printState(State S) {
  if (S == Y) return("Y");
  if (S == G) return("G");
  if (S == R) return("R");
  return("unknown");
}

String printCrossDir(CrossDir D) {
  if (D == N) return("N");
  if (D == S) return("S");
  if (D == E) return("E");
  if (D == W) return("W");
  return ("unknown");
}

String printCrossState(blockState BS) {
  if (BS == Clear) return("Clear");
  if (BS == Clearing) return("Clearing");
  if (BS == Occupied) return("Occupied");
  return ("unknown");
}

void printSigState() {
  for (int i=0; i<NUM_SIGS; i++) {
    logTo::logToAll(String(i) + "(" + printCrossDir(signals[i].location) + "): " + printState(signals[i].state));// + " loco:" + String(signals[j].loco->getAddress()));
    logTo::logToAll("LED GPIO state G: " + String(digitalRead(signals[i].GPIO[0])) + " Y: " + String(digitalRead(signals[i].GPIO[1])) + " R: " + String(digitalRead(signals[i].GPIO[2])));
  }
}

// set both LED and state of signal
// also set readings json var for web interface
void setLED(int sigIdx, State state) {
  //logTo::logToAll("setting " + String(sigIdx) + " to " + printState(state));
  if (sigIdx==NUM_SIGS && state == Y) {
    // set all to yellow
    for (int sig=0; sig<NUM_SIGS; sig++) {
      gpio_set_level((gpio_num_t)signals[sig].GPIO[0], 1); // G
      gpio_set_level((gpio_num_t)signals[sig].GPIO[1], 0); // Y
      gpio_set_level((gpio_num_t)signals[sig].GPIO[2], 1); // R
      readings[printCrossDir(signals[sig].location)] = printState(signals[sig].state);
      logTo::logToAll("LED GPIO state G: " + String(digitalRead(signals[sig].GPIO[0])) + " Y: " + String(digitalRead(signals[sig].GPIO[1])) + " R: " + String(digitalRead(signals[sig].GPIO[2])));
    }
  } else {
    // set state of signal data structure
    signals[sigIdx].state = state;
    readings[printCrossDir(signals[sigIdx].location)] = printState(signals[sigIdx].state);
    if (state == Y) {
      gpio_set_level((gpio_num_t)signals[sigIdx].GPIO[0], 1); // G
      gpio_set_level((gpio_num_t)signals[sigIdx].GPIO[1], 0); // Y
      gpio_set_level((gpio_num_t)signals[sigIdx].GPIO[2], 1); // R
    } else if (state == G) {
      gpio_set_level((gpio_num_t)signals[sigIdx].GPIO[0], 0); // G
      gpio_set_level((gpio_num_t)signals[sigIdx].GPIO[1], 1); // Y
      gpio_set_level((gpio_num_t)signals[sigIdx].GPIO[2], 1); // R
    } else if (state == R) {
      gpio_set_level((gpio_num_t)signals[sigIdx].GPIO[0], 1); // G
      gpio_set_level((gpio_num_t)signals[sigIdx].GPIO[1], 1); // Y
      gpio_set_level((gpio_num_t)signals[sigIdx].GPIO[2], 0); // R
    } else {
      logTo::logToAll("setLED error invalid location/state");
    }
  }
}

void setSigLoco() {
  logTo::logToAll("initializing DCC locoIDs");
  for (Loco *loco = dccexProtocol.roster->getFirst(); loco; loco = loco->getNext()) {
    int id = loco->getAddress();
    for (int sig=0; sig<NUM_SIGS; sig++) {
      if (id == NSlocoID && (signals[sig].location == N || signals[sig].location == S)) {
        logTo::logToAll("setting NS signal " + String(sig) + " loco: " + String(id));
        //signals[sig].loco = loco;
        signals[sig].loco = loco;
      } else if (id == EWlocoID && (signals[sig].location == E || signals[sig].location == W)) {
        logTo::logToAll("setting EW signal " + String(sig) + " loco: " + String(id));
        //signals[sig].loco = loco;
        signals[sig].loco = loco;
      }
    }
  }
}

void calibrateSensors(int cycles) {
  int i, j;
  totalLight = 0;
  logTo::logToAll("Starting IR measurement...");
  for (i=0; i < 0; i++) {
    sensors[i].initVal=0;   
    if (sensorOn[i]) {
      myMux.setPort(i);
      irSensor[sensors[i].sensIdx].setDistanceModeShort();
      irSensor[sensors[i].sensIdx].setTimingBudgetInMs(15);
      int budget = irSensor[sensors[i].sensIdx].getTimingBudgetInMs();
      int dMode = irSensor[sensors[i].sensIdx].getDistanceMode();
      int dModeS = irSensor[sensors[i].sensIdx].getDistanceMode();
      int MP = irSensor[sensors[i].sensIdx].getIntermeasurementPeriod();
      char buf[128];
      sprintf(buf,"sensor: %d budget: %d distance mode: %d/%d measurement period: %d", i, budget, dMode, dModeS, MP);
      logTo::logToAll(buf);
    }
  }
  int dSamples=0;
  for (i=0; i < cycles; i++) {
    for (j=0; j<MUXMAX; j++) {
      if (sensorOn[j]) {
        Serial.print(j);
        myMux.setPort(j);
        irSensor[sensors[j].sensIdx].startRanging();
        while (!irSensor[sensors[j].sensIdx].checkForDataReady()) {
          Serial.print(".");
          delay(5);
        }       
        int dist = irSensor[sensors[j].sensIdx].getDistance();
        irSensor[sensors[j].sensIdx].clearInterrupt();
        Serial.printf("%d:%d ", j, dist);
        sensors[j].initVal += dist;
        irSensor[sensors[j].sensIdx].stopRanging();
      }
    }
    delay(10);
    dSamples++;
  }
  if (dSamples != i) logTo::logToAll("distance samples mismatch: " + String(dSamples) + "/" + String(i));
  logTo::logToAll("Startup done, light measurements:");
  // average initial readings
  for (i = 0; i < MUXMAX; i++) {
    if (sensorOn[i]) {
      sensors[i].initVal /= dSamples;
      logTo::logToAll(String(i) + ": loc " + printCrossDir(sensors[i].location) + ", " + String(sensors[i].initVal) + " sensor:" + String(sensors[i].sensIdx));
      totalLight += sensors[i].initVal;
    }
  }
  logTo::logToAll("total IR level: " + String(totalLight));
}

void showSensors() {
  int i;
  newTotalLight = 0;
  for (i = 0; i < MUXMAX; i++) {
    if (sensorOn[i]) {
      myMux.setPort(i);
      irSensor[sensors[i].sensIdx].startRanging();
      while (!irSensor[sensors[i].sensIdx].checkForDataReady()) {
        Serial.print(".");
        delay(5);
      }       
      sensors[i].curVal = irSensor[sensors[i].sensIdx].getDistance();
      irSensor[sensors[i].sensIdx].stopRanging();
      logTo::logToAll(String(i) + " curr: " + String(sensors[i].curVal) + " init: " + String(sensors[i].initVal)
        + " ratio: " + String((float)sensors[i].curVal/sensors[i].initVal*100, 2) 
        + " avg: " + String(sensors[i].avgVal.getAvg())
        + " avgratio: " + String((float)sensors[i].avgVal.getAvg()/sensors[i].initVal*100, 2)
        + " % thresh: " + sensors[i].trigLevel
        + " state: " + sensors[i].curState
        + " prev: " + sensors[i].prevState
        );
      newTotalLight += sensors[i].curVal;
    }
  }
  logTo::logToAll("total IR level (old/new): " + String(totalLight) + "/" + String(newTotalLight));
}

void ToggleLed() {
  static bool led_state = false;
  digitalWrite(LED_BUILTIN, led_state);
  led_state = !led_state;
}

void ledOn() {
  digitalWrite(LED_BUILTIN, 1);
}

void ledOff() {
  digitalWrite(LED_BUILTIN, 0);
}

volatile bool buttonPressed = false;

void IRAM_ATTR buttonISR() {
    buttonPressed = true;
}

static int greenSig;  // can this be local to checkTriggers?

int analogValue, triggerValue;
float trigRatio;
unsigned long cTime;
int i,trigIdx;
int numTrigs=0;
bool trig=false;

void checkSensor(int i) {
  cTime = millis();
  sensors[i].curState = false;
  myMux.setPort(i);
  irSensor[sensors[i].sensIdx].startRanging();
//#if 0
  while (!irSensor[sensors[i].sensIdx].checkForDataReady()) {
    //Serial.printf("%d.",sensors[i].sensIdx);
    delay(5);
  }       
//#endif
//  if (irSensor[sensors[i].sensIdx].checkForDataReady())
  sensors[i].curVal = irSensor[sensors[i].sensIdx].getDistance();  // this reading

  irSensor[sensors[i].sensIdx].stopRanging();
  analogValue = sensors[i].avgVal.reading(sensors[i].curVal); // moving average of last X readings
  trigRatio = (analogValue/(float)sensors[i].initVal)*100;
  if (teleplot)
    Serial.printf(">[%i]trig:%d:%ld\n",i,analogValue,cTime);
  if (trigRatio < sensors[i].trigLevel) {
    //Serial.printf("sensor %d trig\n", i);
    trig = true;
    sensors[i].curState = true;
    lastTrigTime = cTime;
    numTrigs++;
    trigIdx = i;  // index of last triggered sensor
    String RS = "S" + String(i);
    readings[RS] = "T"; // JSON string like: S0TS3T...
  } 
}

void checkTriggers() {
  //printSigState();
  //int analogValue, triggerValue;
  numTrigs = 0;
  trig = false;
  for (i = 0; i < CROSSMAX; i++) {
    cTime = millis();
    if (sensorOn[i]) {
      checkSensor(i);
      if (trigRatio < sensors[i].trigLevel) {
        if (crossState == Occupied && sensors[i].location == complement(fromDir)) {
          // train is in crossing and hit second sensor
          crossState = Clearing;
        }
        for (int j=0; j<NUM_SIGS; j++) {
          if (sensors[i].location == signals[j].location) {
            // we found the signal at the triggered sensor
            if (signals[j].state == Y) {
              //logTo::logToAll("sensor " + String(i) + " loc " + printCrossDir(sensors[i].location) + " trigger signal " + String(j) + " loc " + printState(signals[j].state));
              // signal is yellow implying no other triggers
              crossState = Occupied;
              fromDir = sensors[i].location;
              for (int k=0; k<NUM_SIGS; k++) {
                if (k == j) {
                  setLED(j,G);
                  greenSig = j;
                } else {
                  setLED(k,R);
                }
                // now we should have one G and NUM_LEDS-1 signals R
                // turn on this track if it was off
                // check for DCC and restore to lastSpeed
                if (digitalRead(signals[j].relay)) { // if relay pin is HIGH we're already stopped
                  logTo::logToAll("proceeding; turn on relay");
                  digitalWrite(signals[j].relay, LOW);      
                } else if (signals[j].loco && saveSpeed >= 0) {
                  // kludge: we hit this the first time any signal is triggered
                  logTo::logToAll("proceeding on DCC track");
                  dccexProtocol.setThrottle(signals[j].loco, saveSpeed, saveDir);
                }
              } // for k (LEDs)
            // state at triggered signal was not Y
            } else if (signals[j].state == R) {
              // we can go through a red if it's facing the other direction on the same track
              if (signals[j].location == complement(signals[greenSig].location)) {
                // passing a red signal implies train occupying crossing is leaving
                crossState = Clearing;
                //logTo::logToAll("second trigger, clearing");
              } else if (enableRelays) {
                // other direction, stop train
                if (digitalRead(signals[j].relay) == 0) { // if relay pin is high we're already stopped so check for low
                  // turn off this block! state was red and we got a trigger
                  logTo::logToAll("sensor triggered on R @" + String(cTime/1000));
                  // if loco != 0; it's DCC
                  if (signals[j].loco) {
                    logTo::logToAll("sending brake command for loco " + String(signals[j].loco->getAddress()) + " to " + DCCEXhostname);          
                    saveSpeed = signals[j].loco->getSpeed();
                    saveDir = signals[j].loco->getDirection();
                    logTo::logToAll("speed/dir:" + String(saveSpeed) + "/" + String(saveDir));
                    //dccexProtocol.setThrottle(signals[j].loco, 0, saveDir);
                    char cmd[64];
                    sprintf(cmd,"t %d -1 1",signals[j].loco->getAddress());
                    dccexProtocol.sendCommand(cmd);
                  } else {
                    // not DCC or unknown cab; turn off relay
                    logTo::logToAll("stopping track on relay " + String(signals[j].relay));
                    digitalWrite(signals[j].relay, HIGH); // writing HIGH switches relay (track is on NC so it's turned off)
                  }
                }
              }
            } // else signal was green already; do nothing
          } // if location 
        } // for j
      } // if trigRatio
    } // if SensorOn
  } // for i trigger loop
  // we've checked all signals
  //if (numTrigs > 0)
  //  logTo::logToAll("triggers: " + String(numTrigs));
  if (numTrigs == 1) {
    // one trigger; either single train still near crossing (Clearing), do nothing
    // or train waiting in opposite direction for Clear
    // if trigger direction is not on same track as last passing train, we can reset to yellow
    if (crossState == Clearing && sensors[trigIdx].location != complement(fromDir)) {
      logTo::logToAll("one trigger, state clearing, different track, proceed");
      crossState = Clear;
      for (int i=0; i<NUM_SIGS; i++) {
        if (signals[i].state != Y)
          setLED(i,Y);
      }
    }
  }
  if (numTrigs == 0) {
    // if no triggers, either a short train is between sensors (Occupied), or it's a false trigger, so set a 30-second timeout
    int clearTime;
    if (crossState != Occupied)
      clearTime = CLEARDELAY*500;
    else
      clearTime = 30000;
    // reset all directions to yellow if they're not already
    // X seconds after lastTrigTime (helps with short trains)
    crossState = Clear;
    //logTo::logToAll("no triggers");
    if (millis() - lastTrigTime > clearTime)
    for (int i=0; i<NUM_SIGS; i++) {
      if (signals[i].state != Y)
        setLED(i,Y);
    }
  }
  // now address additional sensors for turnouts
  for (; i < trigSize; i++) {
    cTime = millis();
    if (sensorOn[i])
      checkSensor(i);
  }
  for (i = 0; i < trigSize; i++) {
    // if state has changed from last cycle, throw the turnout (if assigned) associated with this sensor
    if (sensors[i].curState != sensors[i].prevState) {
      int tE = sensors[i].turnE; // numbered 1..n
      bool dE = sensors[i].dirE;
      int tW = sensors[i].turnW;
      bool dW = sensors[i].dirW;
      logTo::logToAll("turnout state change in sensor " + String(i));
      Serial.printf("trig %d curVal %d curState %d prevState %d\n", i, sensors[i].curVal, sensors[i].curState, sensors[i].prevState);
      if (tE) {
        throwWrapper(tE, dE);
      }
      if (tW) {
        throwWrapper(tW, dW);
      }
    }
    // reset state
    sensors[i].prevState = sensors[i].curState;
  }
}

void setup() {
  int i;
  Serial.begin(115200); delay(500);
  Serial.println("diamond crossing signal controller");

  // disable brownout detector
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);  

  pinMode(0, INPUT_PULLUP); // boot button
  attachInterrupt(digitalPinToInterrupt(0), buttonISR, FALLING);

  Wire.begin(SDA,SCL);
  // slow clock to see if it helps with more devices
  Wire.setClock(100000);
  //i2cScan(Wire);
  if (myMux.begin() == false) {
    Serial.println("Mux not detected!");
  } else
    Serial.println("Mux detected");
  //myMux.setPort(0);
  //byte currentPortNumber = myMux.getPort();
  //Serial.print("CurrentPort: ");
  //Serial.println(currentPortNumber);

  if (SPIFFS.begin())
    Serial.println("opened SPIFFS");
  else
    Serial.println("failed to open SPIFFS");

  // start a console.log file in case we crash before Webserial starts
  if (SPIFFS.exists("/console.log")) {
    SPIFFS.remove("/console.log");
  }
  consLog = SPIFFS.open("/console.log", "w", true);
  if (!consLog) {
    logTo::logToAll("failed to open console log");
  }
  if (consLog.println("ESP console log.")) {
    logTo::logToAll("console log written");
  } else {
    logTo::logToAll("console log write failed");
  }

  preferences.begin("ESPprefs", false);

  WebTimerDelay = preferences.getInt("timerdelay", 1000);
  if (WebTimerDelay<200) {
    WebTimerDelay = 200;
    preferences.putInt("timerdelay", 200);
  }
  logTo::logToAll("WebTimerDelay " + String(WebTimerDelay));

  //startWifi = preferences.getBool("wifi", false);
  host = preferences.getString("hostname", host);
  logTo::logToAll("hostname: " + host);
  preferences.end();

  if (startWifi)
    if (initWiFi()) {
      WiFi.setTxPower(WIFI_POWER_MINUS_1dBm);  // Lowest power setting at 120mA
      startWebServer();
      ElegantOTA.begin(&server);
      WebSerial.begin(&server);
      // Attach a callback function to handle incoming messages
      WebSerial.onMessage(WebSerialonMessage);
      server.begin();
      serverStarted = true;
      logTo::logToAll("HTTP server started @" + WiFi.localIP().toString() + "\n");
      // update web page
      app.onRepeat(WebTimerDelay, []() {
        events.send(getSensorReadings().c_str(),"new_readings" ,millis());
        // print some stuff
        consLog.flush();
      });
    } else {
      startAP();
    } else {
      WiFi.disconnect(true);
      WiFi.mode(WIFI_OFF);
    }

  for (i=0; i<MUXMAX; i++) {
    Serial.printf("%d:",i);
    sensors[i].avgVal.begin();   // init moving average
    myMux.setPort(i);
    if (irSensor[i].begin() != 0) {
      Serial.println("Sensor failed to begin.");
      //while (1);
    } else {
      sensorOn[i] = true;
      Serial.println(" Sensor online!");
      irSensor[i].setDistanceModeShort();
      //irSensor[i].startRanging();
    }
  }

  for (i=0; i<MCP; i++) {
    Serial.printf("init MCP %d\n", i);
    if (!mcp[i].begin_I2C(MCPaddr[i])) {
      Serial.printf("Error on init of mcp[%d] at 0x%x\n", i, MCPaddr[i]);
      i2cScan(Wire);
    }
  }

  Serial.println("init turnouts");
  preferences.begin("ESPprefs", false);
  for (i=0; i<TURNOUTS; i++) {
    char tName[10];
    sprintf(tName,"turnout%d",i);
    layout[i].state = preferences.getBool(tName, false);
    int thisMCP = layout[i].mcp;
    if (thisMCP < MCP) {
      char buf[64];
      sprintf(buf,"%d: mcp: %d togA: %d togB: %d solA: %d solB: %d, state:%s", i+1, thisMCP, layout[i].toggleA, layout[i].toggleB,layout[i].solenoidA, layout[i].solenoidB, (layout[i].state?"straight":"reverse"));
      logTo::logToAll(buf);
      mcp[thisMCP].pinMode(layout[i].toggleA,INPUT_PULLUP);
      mcp[thisMCP].pinMode(layout[i].toggleB,INPUT_PULLUP);
      mcp[thisMCP].pinMode(layout[i].solenoidA,OUTPUT);
      mcp[thisMCP].pinMode(layout[i].solenoidB,OUTPUT);
      mcp[thisMCP].digitalWrite(layout[i].solenoidA,LOW);
      mcp[thisMCP].digitalWrite(layout[i].solenoidB,LOW);
      // should I set all turnouts to straight at the beginning? If not how do I know which direction they are pointed?
      //throwTurnout(i+1, false); 
      //throwTurnout(i+1, true);
    }
  }

  // Direct logs to CONSOLE
  dccexProtocol.setLogStream(&CONSOLE);
  // Set the delegate for broadcasts/responses
  dccexProtocol.setDelegate(&myDelegate);
  // Connect to the CS via CLIENT
  CLIENT.begin(115200, SERIAL_8N1, S1RX, S1TX);
  dccexProtocol.connect(&CLIENT);
  CONSOLE.println(F("DCC-EX connected"));
  dccexProtocol.requestServerVersion();
  dccexProtocol.getLists(true, false, false, false);

#if 0
  app.onRepeat(LOOPDELAY, []() {
    //logTo::logToAll("check dcc");
    dccexProtocol.check();
    dccexProtocol.getLists(true, false, false, false);
  });
#endif

  //preferences.begin("ESPprefs", false);
  //logTo::logToAll("reading speed");
  NSlocoID = preferences.getInt("NSlocoID",0);
  logTo::logToAll("NSloco: " + String(NSlocoID));
  EWlocoID = preferences.getInt("EWlocoID",0);
  logTo::logToAll("EWloco: " + String(EWlocoID));
  //preferences.end();
  //DCCEXhostname = preferences.getString("DCC-EX","");
  //logTo::logToAll("DCCEXhostname: " + String(DCCEXhostname));

  // toggle the LED pin at rate of 1 Hz
  pinMode(LED_BUILTIN, OUTPUT);
  //app.onRepeatMicros(1e6 / 1, []() { ToggleLed(); });

  // set LED GPIOs to active low
  for (int sig=0; sig<NUM_SIGS; sig++) {
    gpio_config_t io_conf = {};
    io_conf.pin_bit_mask = (1ULL << (gpio_num_t)signals[sig].GPIO[0] | 1ULL << (gpio_num_t)signals[sig].GPIO[1] | 1ULL << (gpio_num_t)signals[sig].GPIO[2]);
    //io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.mode = GPIO_MODE_INPUT_OUTPUT;
    io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
    io_conf.intr_type = GPIO_INTR_DISABLE;
    gpio_config(&io_conf);
  }
  // test crossing signals at startup
  logTo::logToAll("test LEDs...");
  for (int i=0; i<NUM_SIGS; i++) {
    setLED(i, G);
    logTo::logToAll("G: " + String(digitalRead(signals[i].GPIO[0])) + " Y: " + String(digitalRead(signals[i].GPIO[1])) + " R: " + String(digitalRead(signals[i].GPIO[2])));
  }
  delay(500);
  for (int i=0; i<NUM_SIGS; i++) {
    setLED(i,R);
    logTo::logToAll("G: " + String(digitalRead(signals[i].GPIO[0])) + " Y: " + String(digitalRead(signals[i].GPIO[1])) + " R: " + String(digitalRead(signals[i].GPIO[2])));
  }
  delay(500);
  setLED(NUM_SIGS,Y);

  logTo::logToAll("setting relays LOW");
  // set pins for controlling voltage to track
  pinMode(RELAYIN, OUTPUT);  // inner (mountain) loop
  digitalWrite(RELAYIN, LOW);
  pinMode(RELAYOUT, OUTPUT); // outer (flat) loop
  digitalWrite(RELAYOUT, LOW);
 
  calibrateSensors(LIGHT_CYCLES);
  printSigState();
  logTo::logToAll("starting loop...");

  logToSerial = true;  // false = stop logging to serial after setup

  // check toggle switches for turnouts
#if 0
  app.onRepeat(LOOPDELAY, []() {
    checkToggles();
  });
#endif
  // here's where the magic happens...determine if any sensors have triggered and set signals
  app.onRepeat(LOOPDELAY, []() {
    checkTriggers();
  });
}

String input = "";

void loop() {
  app.tick(); 
  dccexProtocol.check();
  if (buttonPressed) {
      Serial.println("Boot button pressed!");
      buttonPressed = false;
  }
  ElegantOTA.loop();  
  WebSerial.loop();

  if (Serial.available() > 0) {
    input = Serial.readString();
    //input.trim();  // Remove whitespace and newlines
    logTo::logToAll(input);
    serialWrapper(input);
  }
  input = String();
}

void i2cScan(TwoWire Wire) {
  byte error, address;
  int nDevices = 0;
  logTo::logToAll("Scanning...");
  for (address = 1; address < 127; address++) {
    Wire.beginTransmission(address);
    error = Wire.endTransmission(); 
    char buf[16];
    sprintf(buf, "%2X", address); // Formats value as uppercase hex
    if (error == 0) {
      logTo::logToAll("I2C device found at address 0x" + String(buf));
      nDevices++;
    }
    else if (error == 4) {
      logTo::logToAll("error at address 0x" + String(buf));
    }
  }
  if (nDevices == 0) {
    logTo::logToAll("No I2C devices found\n");
  }
  else {
    logTo::logToAll("done\n");
  }
}


