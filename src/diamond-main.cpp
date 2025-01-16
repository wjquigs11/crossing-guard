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
bool startWifi = true;
bool teleplot = false;

bool stopInner = false;
bool stopOuter = false;
bool enableRelays = true;

int NSlocoID, EWlocoID;
int lastSpeed;
String DCCEXhostname;

// each sensor represents a photoresistor or IR sensor
// the west side of my crossing has a turnout so it needs two sensors
struct Sensor sensors[] = {
  {E, false, 0, 0, MOVINGAVG, 3, TRIGLEVEL},
  {W, false, 0, 0, MOVINGAVG, 0, TRIGLEVEL},      
  {S, false, 0, 0, MOVINGAVG, 2, TRIGLEVEL},      
  {N, false, 0, 0, MOVINGAVG, 1, TRIGLEVEL},       
  {W, false, 0, 0, MOVINGAVG, 4, TRIGLEVEL},      
  {nulldir, false, 0, 0, MOVINGAVG, 5, TRIGLEVEL} // 5 is in the tunnel west of the double slip, not at the crossing
};

QWIICMUX myMux;
SFEVL53L1X irSensor[MUXMAX];
bool sensorOn[MUXMAX];

// last time any sensor was triggered
// used to debounce gaps between cars that allow light on sensor
// wait CLEARDELAY seconds after lastTrigTime before clearing signals
unsigned long lastTrigTime; 

// total light level measured across all sensors
int totalLight, newTotalLight;

int trigSize = sizeof(sensors) / sizeof(sensors[0]);
//int trigSize = 1;

Direction lastTrigDirection = nulldir;  // Store the direction of last trigger
bool waitingForClear = false;  // Flag to indicate we're waiting for opposite direction

blockState crossState = Clear;
Direction fromDir = nulldir;

// signals are defined by the GPIOs each LED is connected to, the location, and the state
// NB at initialization all locoIDs will be 0
Signal signals[NUM_SIGS] = {
    {{23, 22, 21}, S, Y, NORTHSOUTH, 0},
    {{19, 18, 5}, N, Y, NORTHSOUTH, 0},
    {{17, 16, 4}, W, Y, EASTWEST, 0},
    {{14, 12, 13}, E, Y, EASTWEST, 0}
};

Direction complement(Direction dir) {
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

String printDirection(Direction D) {
  if (D == N) return("N");
  if (D == S) return("S");
  if (D == E) return("E");
  if (D == W) return("W");
  return ("unknown");
}

void printSigState() {
  for (int j=0; j<NUM_SIGS; j++) {
    logTo::logToAll(String(j) + "(" + printDirection(signals[j].location) + "): " + printState(signals[j].state));
  }
}

// set both LED and state of signal
// also set readings json var for web interface
void setLED(int sigIdx, State state) {
  //logTo::logToAll("setting " + String(sigIdx) + " to " + printState(state));
  if (sigIdx>NUM_SIGS && state == Y) {
    // set all to yellow
    for (int sig=0; sig<NUM_SIGS; sig++) {
      gpio_set_level((gpio_num_t)signals[sig].GPIO[0], 1); // G
      gpio_set_level((gpio_num_t)signals[sig].GPIO[1], 0); // Y
      gpio_set_level((gpio_num_t)signals[sig].GPIO[2], 1); // R
      readings[printDirection(signals[sig].location)] = printState(signals[sig].state);
      // while we're initializing, set loco for DCC-signal tracks
      if (signals[sig].location == N || signals[sig].location == S)
        signals[sig].loco = NSlocoID;
      else
        signals[sig].loco = EWlocoID;
    }
  } else {
    // set state of signal data structure
    signals[sigIdx].state = state;
    readings[printDirection(signals[sigIdx].location)] = printState(signals[sigIdx].state);
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

void calibrateSensors(int cycles) {
  int i, j;
  totalLight = 0;
  logTo::logToAll("Starting IR measurement...");
  for (j=0; j < MUXMAX; j++) {
    sensors[j].initVal=0;   
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
      logTo::logToAll(String(i) + ": loc " + printDirection(sensors[i].location) + ", " + String(sensors[i].initVal) + " sensor:" + String(sensors[i].sensIdx));
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
        + " % thresh: " + sensors[i].trigLevel);
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

volatile bool buttonPressed = false;

void IRAM_ATTR buttonISR() {
    buttonPressed = true;
}

void setup() {
  Serial.begin(115200); delay(500);
  Serial.println("diamond crossing signal controller");

  pinMode(0, INPUT_PULLUP); // boot button
  attachInterrupt(digitalPinToInterrupt(0), buttonISR, FALLING);

  Wire.begin(SDA,SCL);
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
  logTo::logToAll("hostname: " + host + "\n");

  if (startWifi)
    if (initWiFi()) {
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
    }

  for (int i=0; i<MUXMAX; i++) {
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
      irSensor[i].startRanging();
    }
  }

  int loco = preferences.getInt("NSlocoID",0);
  if (getSpeed(loco) >= 0) {
    // don't set loco if we can't talk to DCC
    NSlocoID = loco;
    logTo::logToAll("NSloco: " + String(NSlocoID));
  } else logTo::logToAll("cannot talk to DCC-EX not setting NSlocoID");
  loco = preferences.getInt("EWlogoID",0);
  if (getSpeed(loco) >= 0) {
    // don't set loco if we can't talk to DCC
    EWlocoID = loco;
    logTo::logToAll("EWloco: " + String(EWlocoID));
  } else logTo::logToAll("cannot talk to DCC-EX not setting EWlocoID");
  DCCEXhostname = preferences.getString("DCC-EX","");
  logTo::logToAll("DCCEXhostname: " + String(DCCEXhostname));

  // toggle the LED pin at rate of 1 Hz
  pinMode(LED_BUILTIN, OUTPUT);
  app.onRepeatMicros(1e6 / 1, []() { ToggleLed(); });

  // set LED GPIOs to active low
  for (int sig=0; sig<NUM_SIGS; sig++) {
    gpio_config_t io_conf = {};
    io_conf.pin_bit_mask = (1ULL << (gpio_num_t)signals[sig].GPIO[0] | 1ULL << (gpio_num_t)signals[sig].GPIO[1] | 1ULL << (gpio_num_t)signals[sig].GPIO[2]);
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
    io_conf.intr_type = GPIO_INTR_DISABLE;
    gpio_config(&io_conf);
  }
  // test crossing signals at startup
  for (int i=0; i<NUM_SIGS; i++) {
    setLED(i, G);
    delay(500);
    setLED(i,R);
    delay(500);
    //setLED(dir,Y);
    setLED(i,Y);
  }

  logTo::logToAll("setting relays LOW");
  // set pins for controlling voltage to track
  pinMode(RELAYIN, OUTPUT);  // inner (mountain) loop
  digitalWrite(RELAYIN, LOW);
  pinMode(RELAYOUT, OUTPUT); // outer (flat) loop
  digitalWrite(RELAYOUT, LOW);
 
  calibrateSensors(LIGHT_CYCLES);
  printSigState();
  logTo::logToAll("starting loop...");

  logToSerial = false;  // stop logging to serial after setup so we can talk to command station

  // here's where the magic happens...determine if any sensors have triggered and set signals
  static int greenSig;
  app.onRepeat(LOOPDELAY, []() {
    int analogValue, triggerValue;
    float trigRatio;
    unsigned long cTime = millis();
    int i,trigIdx;
    int numTrigs=0;
    bool trig=false;

    //printSigState();
    for (i = 0; i < CROSSMAX; i++) {
      cTime = millis();
      if (sensorOn[i]) {
        myMux.setPort(i);
        irSensor[sensors[i].sensIdx].startRanging();
        while (!irSensor[sensors[i].sensIdx].checkForDataReady()) {
          //Serial.print(".");
          delay(5);
        }       
        sensors[i].curVal = irSensor[sensors[i].sensIdx].getDistance();  // this reading
        irSensor[sensors[i].sensIdx].stopRanging();
        analogValue = sensors[i].avgVal.reading(sensors[i].curVal); // moving average of last X readings
        trigRatio = (analogValue/(float)sensors[i].initVal)*100;
        if (teleplot)
          Serial.printf(">[%ld]trig%d:%d:[%ld]\n",i,analogValue,cTime);
        if (trigRatio < sensors[i].trigLevel) {
          trig = true;
          lastTrigTime = cTime;
          numTrigs++;
          trigIdx = i;  // index of last triggered sensor
          char buf[256];
          sprintf(buf,"sensor %d triggered %s", i, printDirection(sensors[i].location));
          logTo::logToAll(String(buf));
          if (crossState == Occupied && sensors[i].location == complement(fromDir)) {
            // train is in crossing and hit second sensor
            crossState = Clearing;
          }
          for (int j=0; j<NUM_SIGS; j++) {
            if (sensors[i].location == signals[j].location) {
              // we found the signal at the triggered sensor
              if (signals[j].state == Y) {
                logTo::logToAll("sensor " + String(i) + " loc " + printDirection(sensors[i].location) + " trigger signal " + String(j) + " loc " + printState(signals[j].state));
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
                  if (digitalRead(signals[j].solenoid)) { // if relay pin is HIGH we're already stopped
                    logTo::logToAll("proceeding; turn on solenoid");
                    digitalWrite(signals[j].solenoid, LOW);      
                  } else if (signals[j].loco && getSpeed(signals[j].loco == 0)) {
                    logTo::logToAll("proceeding on DCC track");
                    resume(signals[j].loco);
                  }
                } // for k (LEDs)
              // state at triggered signal was not Y
              } else if (signals[j].state == R) {
                // we can go through a red if it's facing the other direction on the same track
                if (signals[j].location == complement(signals[greenSig].location)) {
                  // passing a red signal implies train occupying crossing is leaving
                  crossState = Clearing;
                  logTo::logToAll("second trigger, clearing");
                } else if (enableRelays) {
                  // other direction, stop train
                  if (digitalRead(signals[j].solenoid) == 0) { // if relay pin is high we're already stopped so check for low
                    // turn off this block! state was red and we got a trigger
                    logTo::logToAll("sensor triggered on R shut down block! " + String(cTime/1000));
                    if (signals[j].loco) {
                      // if loco != 0; it's DCC
                      if (getSpeed(signals[j].loco) < 0) {
                        // timed out, turn of relay
                        digitalWrite(signals[j].solenoid, HIGH); // writing HIGH switches relay (track is on NC so it's turned off)
                      }
                      brake(signals[j].loco);
                      logTo::logToAll("sending brake command for loco " + String(NSlocoID) + " to " + DCCEXhostname);
                    } else {
                      // not DCC; turn off solenoid
                      logTo::logToAll("stopping track on solenoid " + String(signals[j].solenoid));
                      digitalWrite(signals[j].solenoid, HIGH); // writing HIGH switches relay (track is on NC so it's turned off)
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
    if (numTrigs > 0)
      logTo::logToAll("triggers: " + String(numTrigs));
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
    if (numTrigs == 0 && crossState != Occupied) {
      // reset all directions to yellow if they're not already
      // X seconds after lastTrigTime (helps with short trains)
      crossState = Clear;
      //logTo::logToAll("no triggers");
      if (millis() - lastTrigTime > CLEARDELAY*500)
      for (int i=0; i<NUM_SIGS; i++) {
        if (signals[i].state != Y)
          setLED(i,Y);
      }
    }
    for (i = CROSSMAX; i < MUXMAX; i++) {
      // now address sensors not at crossing (turnouts)
    }
  });
}

void loop() {
  app.tick(); 
  if (buttonPressed) {
      Serial.println("Boot button pressed!");
      buttonPressed = false;
  }
  ElegantOTA.loop();  
  WebSerial.loop();
  String input = "";
  if (Serial.available() > 0) {
    while (Serial.available() > 0) {
      char c = Serial.read();
      if (c == '\n' || c == '\r') {
        // Ignore newline and carriage return characters
        break;
      }
      input += c;
    }
  }
  if (input.length() > 0) {
    //Serial.print("Received: ");
    //Serial.println(input);
    logTo::logToAll(input);
    if (input.startsWith("<")) {
      // do something with data from DCC-EX
    } else if (input.startsWith("wif")) {
      if (startWifi) {
        Serial.println("turning off wifi next reboot");
        preferences.putBool("wifi",false);
      } else {
        Serial.println("turning on wifi next reboot");
        preferences.putBool("wifi",true);
      }
    } else if (input.startsWith("rest")) {
      ESP.restart();
    }
  }
}
