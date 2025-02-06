//
// diamond crossing guard: turn off one track if there's a risk of collision at a diamond crossing
// also controls signals on each branch of the crossing
//
#include "include.h"

#ifdef DIAMOND

File consLog;
using namespace reactesp;
ReactESP app;
Preferences preferences;
int WebTimerDelay;
bool startWifi = true;
bool teleplot = false;
bool stopInner = false;
bool stopOuter = false;
bool enableRelays = true;
bool loopMode = false;
bool turnControlConnected = true; // we will assume we're connected to i2c slave turnout controller unless we get an error on write

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
SFEVL53L1X dSensor[MUXMAX];
bool sensorOn[MUXMAX];

// last time any sensor was triggered
// used to debounce gaps between cars that allow light on sensor
// wait CLEARDELAY seconds after lastTrigTime before clearing signals
unsigned long lastTrigTime; 

// total light level measured across all sensors
int totalLight, newTotalLight;


cDirection lastTrigcDirection = nulldir;  // Store the direction of last trigger
bool waitingForClear = false;  // Flag to indicate we're waiting for opposite direction

blocksigState crosssigState = Clear;
cDirection fromDir = nulldir;

// signal LED MCP23017
Adafruit_MCP23X17 mcp;
int mcpD = 0x20;

// signals are defined by the GPIOs each LED is connected to, the location, and the state
// NB at initialization all locoIDs will be 0
Signal signals[NUM_SIGS] = {
    {{0, 1, 2}, S, Y, NORTHSOUTH, 0},
    {{3, 4, 5}, N, Y, NORTHSOUTH, 0},
    {{6, 7, 8}, W, Y, EASTWEST, 0},
    {{9, 10, 11}, E, Y, EASTWEST, 0}
};

cDirection complement(cDirection dir) {
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

String printsigState(sigState S) {
  if (S == Y) return("Y");
  if (S == G) return("G");
  if (S == R) return("R");
  return("unknown");
}

String printcDirection(cDirection D) {
  if (D == N) return("N");
  if (D == S) return("S");
  if (D == E) return("E");
  if (D == W) return("W");
  return ("unknown");
}

String printCrosssigState(blocksigState BS) {
  if (BS == Clear) return("Clear");
  if (BS == Clearing) return("Clearing");
  if (BS == Occupied) return("Occupied");
  return ("unknown");
}

void printSigsigState() {
  for (int i=0; i<NUM_SIGS; i++) {
    logTo::All(String(i) + "(" + printcDirection(signals[i].location) + "): " + printsigState(signals[i].state));// + " loco:" + String(signals[j].loco->getAddress()));
    //logTo::All("LED GPIO state G: " + String(digitalRead(signals[i].GPIO[0])) + " Y: " + String(digitalRead(signals[i].GPIO[1])) + " R: " + String(digitalRead(signals[i].GPIO[2])));
  }
}

// set both LED and state of signal
// also set readings json var for web interface
void setLED(int sigIdx, sigState state) {
  //logTo::All("setting " + String(sigIdx) + " to " + printsigState(state));
  // set state of signal data structure
  signals[sigIdx].state = state;
  readings[printcDirection(signals[sigIdx].location)] = printsigState(signals[sigIdx].state);
  if (signals[sigIdx].GPIO[0] >= 0)
    if (state == Y) {
      mcp.digitalWrite(signals[sigIdx].GPIO[0],OFF); // G
      mcp.digitalWrite(signals[sigIdx].GPIO[1],ON);  // Y
      mcp.digitalWrite(signals[sigIdx].GPIO[2],OFF); // R
    } else if (state == G) {
      mcp.digitalWrite(signals[sigIdx].GPIO[0],ON); // G
      mcp.digitalWrite(signals[sigIdx].GPIO[1],OFF);  // Y
      mcp.digitalWrite(signals[sigIdx].GPIO[2],OFF); // R
    } else if (state == R) {
      mcp.digitalWrite(signals[sigIdx].GPIO[0],OFF); // G
      mcp.digitalWrite(signals[sigIdx].GPIO[1],OFF);  // Y
      mcp.digitalWrite(signals[sigIdx].GPIO[2],ON); // R
    } else {
      logTo::All("setLED error invalid location/state");
    }
}

void setSigLoco() {
  logTo::All("initializing DCC locoIDs");
  for (Loco *loco = dccexProtocol.roster->getFirst(); loco; loco = loco->getNext()) {
    int id = loco->getAddress();
    for (int sig=0; sig<NUM_SIGS; sig++) {
      if (id == NSlocoID && (signals[sig].location == N || signals[sig].location == S)) {
        logTo::All("setting NS signal " + String(sig) + " loco: " + String(id));
        //signals[sig].loco = loco;
        signals[sig].loco = loco;
      } else if (id == EWlocoID && (signals[sig].location == E || signals[sig].location == W)) {
        logTo::All("setting EW signal " + String(sig) + " loco: " + String(id));
        //signals[sig].loco = loco;
        signals[sig].loco = loco;
      }
    }
  }
}

void calibrateSensors(int cycles) {
  int i, j;
  totalLight = 0;
  logTo::All("Starting IR measurement...");
  for (i=0; i < NUM_SIGS; i++) {
    sensors[i].initVal=0;   
    if (sensorOn[i]) {
      myMux.setPort(i);
      dSensor[sensors[i].sensIdx].setDistanceModeShort();
      dSensor[sensors[i].sensIdx].setTimingBudgetInMs(15);
      int budget = dSensor[sensors[i].sensIdx].getTimingBudgetInMs();
      int dMode = dSensor[sensors[i].sensIdx].getDistanceMode();
      int dModeS = dSensor[sensors[i].sensIdx].getDistanceMode();
      int MP = dSensor[sensors[i].sensIdx].getIntermeasurementPeriod();
      char buf[128];
      sprintf(buf,"sensor: %d budget: %d distance mode: %d/%d measurement period: %d", i, budget, dMode, dModeS, MP);
      logTo::All(buf);
    }
  }
  int dSamples=0;
  for (i=0; i < cycles; i++) {
    for (j=0; j<MUXMAX; j++) {
      if (sensorOn[j]) {
        Serial.print(j);
        myMux.setPort(j);
        dSensor[sensors[j].sensIdx].startRanging();
        while (!dSensor[sensors[j].sensIdx].checkForDataReady()) {
          Serial.print(".");
          delay(5);
        }       
        int dist = dSensor[sensors[j].sensIdx].getDistance();
        dSensor[sensors[j].sensIdx].clearInterrupt();
        Serial.printf("%d:%d ", j, dist);
        sensors[j].initVal += dist;
        dSensor[sensors[j].sensIdx].stopRanging();
      }
    }
    delay(10);
    dSamples++;
  }
  if (dSamples != i) logTo::All("distance samples mismatch: " + String(dSamples) + "/" + String(i));
  logTo::All("Startup done, light measurements:");
  // average initial readings
  for (i = 0; i < MUXMAX; i++) {
    if (sensorOn[i]) {
      sensors[i].initVal /= dSamples;
      logTo::All(String(i) + ": loc " + printcDirection(sensors[i].location) + ", " + String(sensors[i].initVal) + " sensor:" + String(sensors[i].sensIdx));
      totalLight += sensors[i].initVal;
    }
  }
  logTo::All("total IR level: " + String(totalLight));
}

void showSensors() {
  int i;
  newTotalLight = 0;
  for (i = 0; i < MUXMAX; i++) {
    if (sensorOn[i]) {
      myMux.setPort(i);
      dSensor[sensors[i].sensIdx].startRanging();
      while (!dSensor[sensors[i].sensIdx].checkForDataReady()) {
        Serial.print(".");
        delay(5);
      }       
      sensors[i].curVal = dSensor[sensors[i].sensIdx].getDistance();
      dSensor[sensors[i].sensIdx].stopRanging();
      logTo::All(String(i) + " curr: " + String(sensors[i].curVal) + " init: " + String(sensors[i].initVal)
        + " ratio: " + String((float)sensors[i].curVal/sensors[i].initVal*100, 2) 
        + " avg: " + String(sensors[i].avgVal.getAvg())
        + " avgratio: " + String((float)sensors[i].avgVal.getAvg()/sensors[i].initVal*100, 2)
        + " % thresh: " + sensors[i].trigLevel
        + " state: " + sensors[i].cursigState
        + " prev: " + sensors[i].prevsigState
        );
      newTotalLight += sensors[i].curVal;
    }
  }
  logTo::All("total IR level (old/new): " + String(totalLight) + "/" + String(newTotalLight));
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
  sensors[i].cursigState = false;
  myMux.setPort(i);
  dSensor[sensors[i].sensIdx].startRanging();
//#if 0
  while (!dSensor[sensors[i].sensIdx].checkForDataReady()) {
    //Serial.printf("%d.",sensors[i].sensIdx);
    delay(5);
  }       
//#endif
//  if (dSensor[sensors[i].sensIdx].checkForDataReady())
  sensors[i].curVal = dSensor[sensors[i].sensIdx].getDistance();  // this reading

  dSensor[sensors[i].sensIdx].stopRanging();
  analogValue = sensors[i].avgVal.reading(sensors[i].curVal); // moving average of last X readings
  trigRatio = (analogValue/(float)sensors[i].initVal)*100;
  if (teleplot)
    Serial.printf(">[%i]trig:%d:%ld\n",i,analogValue,cTime);
  if (trigRatio < sensors[i].trigLevel) {
    //Serial.printf("sensor %d trig\n", i);
    trig = true;
    sensors[i].cursigState = true;
    lastTrigTime = cTime;
    numTrigs++;
    trigIdx = i;  // index of last triggered sensor
    String RS = "S" + String(i);
    readings[RS] = "T"; // JSON string like: S0TS3T...
  } 
}

void checkTriggers() {
  //printSigsigState();
  //int analogValue, triggerValue;
  numTrigs = 0;
  trig = false;
  for (i = 0; i < CROSSMAX; i++) {
    cTime = millis();
    if (sensorOn[i]) {
      checkSensor(i);
      if (trigRatio < sensors[i].trigLevel) {
        if (crosssigState == Occupied && sensors[i].location == complement(fromDir)) {
          // train is in crossing and hit second sensor
          crosssigState = Clearing;
        }
        for (int j=0; j<NUM_SIGS; j++) {
          if (sensors[i].location == signals[j].location) {
            // we found the signal at the triggered sensor
            if (signals[j].state == Y) {
              //logTo::All("sensor " + String(i) + " loc " + printcDirection(sensors[i].location) + " trigger signal " + String(j) + " loc " + printsigState(signals[j].state));
              // signal is yellow implying no other triggers
              crosssigState = Occupied;
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
                  logTo::All("proceeding; turn on relay");
                  digitalWrite(signals[j].relay, LOW);      
                } else if (signals[j].loco && saveSpeed >= 0) {
                  // kludge: we hit this the first time any signal is triggered
                  logTo::All("proceeding on DCC track");
                  dccexProtocol.setThrottle(signals[j].loco, saveSpeed, saveDir);
                }
              } // for k (LEDs)
            // state at triggered signal was not Y
            } else if (signals[j].state == R) {
              //logTo::All("trigger on red");
              // we can go through a red if it's facing the other direction on the same track
              if (signals[j].location == complement(signals[greenSig].location)) {
                // passing a red signal implies train occupying crossing is leaving
                crosssigState = Clearing;
                logTo::All("second trigger, clearing");
              } else if (enableRelays) {
                // other direction, stop train
                if (digitalRead(signals[j].relay) == 0) { // if relay pin is high we're already stopped so check for low
                  // turn off this block! state was red and we got a trigger
                  logTo::All("sensor triggered on R @" + String(cTime/1000));
                  // if loco != 0; it's DCC
                  if (signals[j].loco) {
                    logTo::All("sending brake command for loco " + String(signals[j].loco->getAddress()) + " to " + DCCEXhostname);          
                    saveSpeed = signals[j].loco->getSpeed();
                    saveDir = signals[j].loco->getDirection();
                    logTo::All("speed/dir:" + String(saveSpeed) + "/" + String(saveDir));
                    //dccexProtocol.setThrottle(signals[j].loco, 0, saveDir);
                    char cmd[64];
                    sprintf(cmd,"t %d -1 1",signals[j].loco->getAddress());
                    dccexProtocol.sendCommand(cmd);
                  } else {
                    // not DCC or unknown cab; turn off relay
                    logTo::All("stopping track on relay " + String(signals[j].relay) + " at " + printcDirection(signals[j].location));
                    digitalWrite(signals[j].relay, HIGH); // writing HIGH switches relay (track is on NC so it's turned off)
                  }
                } //else logTo::All("relay was high");
              } else logTo::All("relays not enabled");
            } //else logTo::All("signal green");
          } // if location 
        } // for j
      } // if trigRatio
    } // if SensorOn
  } // for i trigger loop
  // we've checked all signals
  //if (numTrigs > 0)
  //  logTo::All("triggers: " + String(numTrigs));
  if (numTrigs == 1) {
    // one trigger; either single train still near crossing (Clearing), do nothing
    // or train waiting in opposite direction for Clear
    // if trigger direction is not on same track as last passing train, we can reset to yellow
    if (crosssigState == Clearing && sensors[trigIdx].location != complement(fromDir)) {
      logTo::All("one trigger, state clearing, different track, proceed");
      crosssigState = Clear;
      for (int i=0; i<NUM_SIGS; i++) {
        if (signals[i].state != Y)
          setLED(i,Y);
      }
    }
  }
  if (numTrigs == 0) {
    // if no triggers, either a short train is between sensors (Occupied), or it's a false trigger, so set a 30-second timeout
    int clearTime;
    if (crosssigState != Occupied)
      clearTime = CLEARDELAY*500;
    else
      clearTime = 30000;
    // reset all directions to yellow if they're not already
    // X seconds after lastTrigTime (helps with short trains)
    crosssigState = Clear;
    //logTo::All("no triggers");
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
    if (sensors[i].cursigState != sensors[i].prevsigState) {
      int tE = sensors[i].turnE; // numbered 1..n
      bool dE = sensors[i].dirE;
      int tW = sensors[i].turnW;
      bool dW = sensors[i].dirW;
      logTo::All("turnout state change in sensor " + String(i) + " location " + printcDirection(sensors[i].location));
      Serial.printf("trig %d curVal %d cursigState %d prevsigState %d\n", i, sensors[i].curVal, sensors[i].cursigState, sensors[i].prevsigState);
      if (tE) {
        throwTurnout(tE, dE);
      }
      if (tW) {
        throwTurnout(tW, dW);
      }
    }
    // reset state
    sensors[i].prevsigState = sensors[i].cursigState;
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

  Wire.begin();
  Serial.println("init mcp");
  if (!mcp.begin_I2C(mcpD)) {
    Serial.printf("Error on init of mcp at 0x%x\n",mcpD);
    i2cScan(Wire);
  }
  for (i=0; i<NUM_SIGS; i++) {
    mcp.pinMode(signals[i].GPIO[0],OUTPUT);
    mcp.digitalWrite(signals[i].GPIO[0],OFF);
    mcp.pinMode(signals[i].GPIO[1],OUTPUT);
    mcp.digitalWrite(signals[i].GPIO[1],OFF);
    mcp.pinMode(signals[i].GPIO[2],OUTPUT);
    mcp.digitalWrite(signals[i].GPIO[2],OFF);
  }
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
    logTo::All("failed to open console log");
  }
  if (consLog.println("ESP console log.")) {
    logTo::All("console log written");
  } else {
    logTo::All("console log write failed");
  }

  preferences.begin("ESPprefs", false);

  WebTimerDelay = preferences.getInt("timerdelay", 1000);
  if (WebTimerDelay<200) {
    WebTimerDelay = 200;
    preferences.putInt("timerdelay", 200);
  }
  logTo::All("WebTimerDelay " + String(WebTimerDelay));

  //startWifi = preferences.getBool("wifi", false);
  host = preferences.getString("hostname", host);
  logTo::All("hostname: " + host);
  preferences.end();

  if (startWifi)
    if (initWiFi()) {
      //WiFi.setTxPower(WIFI_POWER_MINUS_1dBm);  // Lowest power setting at 120mA
      startWebServer();
      ElegantOTA.begin(&server);
      WebSerial.begin(&server);
      // Attach a callback function to handle incoming messages
      WebSerial.onMessage(WebSerialonMessage);
      server.begin();
      serverStarted = true;
      logTo::All("HTTP server started @" + WiFi.localIP().toString() + "\n");
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
    if (dSensor[i].begin() != 0) {
      Serial.println("Sensor failed to begin.");
      //while (1);
    } else {
      sensorOn[i] = true;
      Serial.println(" Sensor online!");
      dSensor[i].setDistanceModeShort();
      //dSensor[i].startRanging();
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

  Serial.println("serial2 connection to turnout controller");
  Serial2.begin(115200, SERIAL_8N1, S2RX, S2TX);

#if 0
  app.onRepeat(LOOPDELAY, []() {
    //logTo::All("check dcc");
    dccexProtocol.check();
    dccexProtocol.getLists(true, false, false, false);
  });
#endif

  //preferences.begin("ESPprefs", false);
  //logTo::All("reading speed");
  NSlocoID = preferences.getInt("NSlocoID",0);
  logTo::All("NSloco: " + String(NSlocoID));
  EWlocoID = preferences.getInt("EWlocoID",0);
  logTo::All("EWloco: " + String(EWlocoID));
  //preferences.end();
  //DCCEXhostname = preferences.getString("DCC-EX","");
  //logTo::All("DCCEXhostname: " + String(DCCEXhostname));

  // toggle the LED pin at rate of 1 Hz
  pinMode(LED_BUILTIN, OUTPUT);
  //app.onRepeatMicros(1e6 / 1, []() { ToggleLed(); });

  // test crossing signals at startup
  logTo::All("test LEDs...");
  for (int i=0; i<NUM_SIGS; i++)
    if (signals[i].GPIO[0]>=0)
      setLED(i, G);
  delay(500);
  for (int i=0; i<NUM_SIGS; i++)
    if (signals[i].GPIO[0]>=0)
      setLED(i,R);
  delay(500);
  for (int i=0; i<NUM_SIGS; i++)
    if (signals[i].GPIO[0]>=0)
      setLED(i,Y);
  logTo::All("setting relays LOW");
  // set pins for controlling voltage to track
  pinMode(RELAYIN, OUTPUT);  // inner (mountain) loop
  digitalWrite(RELAYIN, LOW);
  pinMode(RELAYOUT, OUTPUT); // outer (flat) loop
  digitalWrite(RELAYOUT, LOW);
 
  calibrateSensors(LIGHT_CYCLES);
  printSigsigState();
  logTo::All("starting loop...");

  logToSerial = true;  // false = stop logging to serial after setup

  // here's where the magic happens...determine if any sensors have triggered and set signals
  // this will send commands to slave ESP32 running "turnout-main" via i2c if sensors trigger turnout throws
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
    logTo::All(input);
    serialWrapper(input);
  }
  input = String();
}

#endif
// needed by turnout-main also

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

void i2cScan(TwoWire Wire) {
  byte error, address;
  int nDevices = 0;
  logTo::All("Scanning...");
  for (address = 1; address < 127; address++) {
    Wire.beginTransmission(address);
    error = Wire.endTransmission(); 
    char buf[16];
    sprintf(buf, "%2X", address); // Formats value as uppercase hex
    if (error == 0) {
      logTo::All("I2C device found at address 0x" + String(buf));
      nDevices++;
    }
    else if (error == 4) {
      logTo::All("error at address 0x" + String(buf));
    }
  }
  if (nDevices == 0) {
    logTo::All("No I2C devices found\n");
  }
  else {
    logTo::All("done\n");
  }
}

