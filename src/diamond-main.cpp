//
// diamond crossing guard: turn off one track if there's a risk of collision at a diamond crossing
// also controls signals on each branch of the crossing
//

#include "include.h"

#define SDA 25 // crossing guard under layout
#define SCL 15

#define STARTTRIG 1 // if STARTTRIG == 0, use photoresistors

File consLog;
using namespace reactesp;
ReactESP app;
Preferences preferences;
int WebTimerDelay;
bool startWifi = true;
bool teleplot = false;

bool stopInner = false;
bool stopOuter = false;

int NSlocoID, EWlocoID;
int lastSpeed;
String DCCEXhostname;

SFEVL53L1X dSens;

// each sensor represents a photoresistor or IR sensor
// the west side of my crossing has a turnout so it needs two sensors
struct Sensor sensors[] = {
  {S, false, 0, 0, MOVINGAVG, 36, TRIGLEVEL},  // 0 is a north/south trigger on the south side of the junction
  {W, false, 0, 0, MOVINGAVG, 39, TRIGLEVEL},  // 1 
  {W, false, 0, 0, MOVINGAVG, 34, TRIGLEVEL},  // 2
  {N, false, 0, 0, MOVINGAVG, 35, TRIGLEVEL},  // 3
  {E, false, 0, 0, MOVINGAVG, 32, TRIGLEVEL}   // 4
};

// last time any sensor was triggered
// used to debounce gaps between cars that allow light on sensor
// wait CLEARDELAY seconds after lastTrigTime before clearing signals
unsigned long lastTrigTime; 

// total light level measured across all sensors
int totalLight, newTotalLight;

int trigSize = sizeof(sensors) / sizeof(sensors[0]);
//int trigSize = 1;

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

void calibrateLight(int cycles) {
  int i, j;
  totalLight = 0;
  logTo::logToAll("Starting light measurement...");
  for (j=STARTTRIG; j < trigSize; j++) {
    sensors[j].avgVal.begin();      
  }
  int dSamples;
  for (i=0; i < cycles; i++) {
    int distance;
    if (dSens.checkForDataReady()) {
      distance = dSens.getDistance();
      dSens.clearInterrupt();
      sensors[0].initVal += distance;
      dSamples++;
    }
    for (j=STARTTRIG; j < trigSize; j++) {
      sensors[j].initVal += analogRead(sensors[j].GPIO);
    }
    delay(10);
  }
  if (dSamples != i) logTo::logToAll("distance samples: " + String(dSamples));
  logTo::logToAll("Startup done, light measurements:");
  // average initial readings
  for (i = 0; i < trigSize; i++) {
    sensors[i].initVal /= cycles;
    logTo::logToAll(String(i) + ": loc " + printDirection(sensors[i].location) + ", " + String(sensors[i].initVal) + " gpio:" + String(sensors[i].GPIO));
    totalLight += sensors[i].initVal;
  }

  logTo::logToAll("total light level: " + String(totalLight));
}

void showLight() {
  int i;
  newTotalLight = 0;
  for (i = 0; i < trigSize; i++) {
    if (i == 0) {
      sensors[i].curVal = dSens.getDistance();
    } else {
      sensors[i].curVal = analogRead(sensors[i].GPIO);
    }
    logTo::logToAll(String(i) + " curr: " + String(sensors[i].curVal) + " init: " + String(sensors[i].initVal)
      + " ratio: " + String((float)sensors[i].curVal/sensors[i].initVal*100, 2) + "% thresh: " + sensors[i].trigLevel);
    newTotalLight += sensors[i].curVal;
  }
  logTo::logToAll("total light level (old/new): " + String(totalLight) + "/" + String(newTotalLight));
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
#if MUX
  if (myMux.begin() == false) {
    Serial.println("Mux not detected. Freezing...");
    while (1);
  }
  Serial.println("i2c mux detected");

  myMux.setPort(1); //Connect master to port labeled '1' on the mux

  // i2c mux
  for (int i=0; i<7; i++)
    myMux.disablePort(i);
#endif
  if (dSens.begin()){
    Serial.println("IR sensor not found!");
    while (1);
  }
  dSens.setDistanceModeShort();
  dSens.startRanging();

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

  NSlocoID = preferences.getInt("NSlocoID",0);
  logTo::logToAll("NSloco: " + String(NSlocoID));
  EWlocoID = preferences.getInt("EWlogoID",0);
  logTo::logToAll("EWloco: " + String(EWlocoID));
  DCCEXhostname = preferences.getString("DCC-EX","");
  logTo::logToAll("DCCEXhostname: " + String(DCCEXhostname));
  
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
 
  // set the ADC attenuation to 11 dB (up to ~3.3V input)
  analogSetAttenuation(ADC_11db);

  // measure ambient light on photoresistors
  calibrateLight(LIGHT_CYCLES);
  printSigState();

  logToSerial = false;  // stop logging to serial so we can talk to command station

  // here's where the magic happens...determine if any sensors have triggered and set signals
  static int greenSig;
  app.onRepeat(LOOPDELAY, []() {
    int analogValue, triggerValue;
    float trigRatio;
    unsigned long cTime = millis();
    int trigIdx;
    int numTrigs=0;
    bool trig=false;

    //printSigState();
    for (int i = 0; i < trigSize; i++) {
      cTime = millis();
      if (i == 0) {
        analogValue = sensors[i].curVal = dSens.getDistance();
      } else {
        sensors[i].curVal = analogRead(sensors[i].GPIO);
        analogValue = sensors[i].avgVal.reading(sensors[i].curVal);
      }
      trigRatio = (analogValue/(float)sensors[i].initVal)*100;
      if (teleplot)
        Serial.printf(">[%ld]trig%d:%d:[%ld]\n",i,analogValue,cTime);
      if (trigRatio < sensors[i].trigLevel) {
        trig = true;
        lastTrigTime = cTime;
        numTrigs++;
        trigIdx = i;  // index of last triggered sensor
        for (int j=0; j<NUM_SIGS; j++) {
          if (sensors[i].location == signals[j].location) {
            // we found the signal at the triggered sensor
            if (signals[j].state == Y) {
              // signal is yellow implying no other triggers
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
                } else if (getSpeed(signals[j].loco == 0)) {
                  logTo::logToAll("proceeding on DCC track");
                  resume(signals[j].loco);
                }
              } // for k (LEDs)
            // state at triggered signal was not Y
            } else if (signals[j].state == R) {
              // we can go through a red if it's facing the other direction on the same track
              if (signals[j].location != complement(signals[greenSig].location)) {
                if (digitalRead(signals[j].solenoid) == 0) { // if relay pin is high we're already stopped so check for low
                  // turn off this block! state was red and we got a trigger
                  logTo::logToAll("sensor triggered on R shut down block! " + String(cTime/1000));
                  if (signals[j].loco) {
                    // if loco != 0; it's DCC
                    getSpeed(signals[j].loco);
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
          }
        } // for j
      } // if trigger
    } // for trigger loop
    if (numTrigs == 1) {
      // one trigger, either:
      //    a) train on green has not yet passed completely or
      //    b) train in other direction is waiting
      if (sensors[trigIdx].location != complement(signals[greenSig].location)) {
        // triggered signal is not on same track as green signal
        for (int i=0; i<NUM_SIGS; i++) {
          if (sensors[trigIdx].location == signals[i].location) {
            if (signals[i].state == R) {
              // found a red signal associated with this trigger but since it's the only trigger it can proceed
              logTo::logToAll("clear to proceed in " + String(CLEARDELAY));
              for (int k=0; k<NUM_SIGS; k++) {
                setLED(k,Y);
              }
              delay(CLEARDELAY*1000);
              // this direction will proceed next iteration since it's now Y
              // unless another train comes in at a lower trigger index between iterations
              // which seems unlikely since the loop iterates at LOOPDELAY
              // but you should lower LOOPDELAY interval in "production" just to be safe
            }
          }
        }
      } // trigger location is same track as greenSig so do nothing until it clears
    }
    if (numTrigs == 0) {
      // reset all directions to yellow if they're not already
      // X seconds after lastTrigTime (helps with short trains)
      if (millis() - lastTrigTime > CLEARDELAY*1000)
        for (int i=0; i<NUM_SIGS; i++) {
          if (signals[i].state != Y)
            setLED(i,Y);
        }
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
