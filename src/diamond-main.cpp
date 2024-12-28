//
// diamond crossing guard: turn off one track if there's a risk of collision at a diamond crossing
// also controls signals on each branch of the crossing
//

#include "include.h"

// using TOF sensor for NS 0 trigger (instead of photoresistor)
// although IR sensor would be better, but multiple require I2C hub
// https://www.adafruit.com/product/6064
// https://www.adafruit.com/product/5626
#define SDA 15
#define SCL 25
#define IRQ_PIN 2
#define XSHUT_PIN 3

Adafruit_VL53L1X vl53 = Adafruit_VL53L1X(XSHUT_PIN, IRQ_PIN);
int tof;
#define STARTTRIG 0 // if STARTTRIG == 0, use photoresistors
                    // else skip trigger 0 to test TOF/IR sensor


File consLog;
using namespace reactesp;
ReactESP app;
Preferences preferences;
int WebTimerDelay;
bool teleplot = false;

bool stopInner = false;
bool stopOuter = false;

struct Trigger triggers[] = {
  {N, S, false, 0, 0, MOVINGAVG, 36, TRIGLEVEL},  // 0 is a north/south trigger on the south side of the junction
  {E, W, false, 0, 0, MOVINGAVG, 39, TRIGLEVEL},  // 1 
  {E, W, false, 0, 0, MOVINGAVG, 34, TRIGLEVEL},  // 2
  {N, N, false, 0, 0, MOVINGAVG, 35, TRIGLEVEL},  // 3
  {E, E, false, 0, 0, MOVINGAVG, 32, TRIGLEVEL}   // 4
};

// last time *any* sensor was triggered
// used to debounce gaps between cars that allow light on sensor
// wait CLEARDELAY seconds after lastTrigTime before clearing signals
unsigned long lastTrigTime; 

// total light level measured across all sensors
int totalLight, newTotalLight;

int trigSize = sizeof(triggers) / sizeof(triggers[0]);

DirectionState nsState = {N, Y};
DirectionState ewState = {E, Y};

#define NUM_SIGS 4  // 4 signals on the diamond crossing
#define NUM_LEDS 3  // each signal has 3 LEDs

// GPIOs for each signal green/yellow/red
int signals[NUM_SIGS][NUM_LEDS] = 
   {{23, 22, 21},  // north/south
    {19, 18, 5},
    {17, 16, 4},    // east/west
    {14, 12, 13}};

void setTrigger(Trigger *trigger, int trigLevel) {
  trigger->trigLevel = trigLevel;
}

// TBD: I should track which trigger changed and set the opposite light to red so traffic can't come in both directions
// which would never happen in DC but could happen in DCC
void setLED(Direction direction, State state) {
  int sig, led;
  if (state == Y) {
    // set all to yellow, direction doesn't matter
    for (sig=0; sig<NUM_SIGS; sig++) {
      gpio_set_level((gpio_num_t)signals[sig][0], 1); // G
      gpio_set_level((gpio_num_t)signals[sig][1], 0); // Y (0)
      gpio_set_level((gpio_num_t)signals[sig][2], 1); // R
    }
    Serial.println();
  } else
  if (direction == N || direction == S) {
    // NS yellow then green, EW red
    for (sig=0; sig<2; sig++) {
      gpio_set_level((gpio_num_t)signals[sig][0], 1);
      gpio_set_level((gpio_num_t)signals[sig][1], 0);
      gpio_set_level((gpio_num_t)signals[sig][2], 1);           
    }    
    for (sig=2; sig<NUM_SIGS; sig++) {
      gpio_set_level((gpio_num_t)signals[sig][0], 1);
      gpio_set_level((gpio_num_t)signals[sig][1], 1);
      gpio_set_level((gpio_num_t)signals[sig][2], 0);           
    }
    delay(50);
    for (sig=0; sig<2; sig++) {
      gpio_set_level((gpio_num_t)signals[sig][0], 0);
      gpio_set_level((gpio_num_t)signals[sig][1], 1);
      gpio_set_level((gpio_num_t)signals[sig][2], 1);           
    }        
  } else { // direction == E or W
    for (sig=0; sig<2; sig++) {
      gpio_set_level((gpio_num_t)signals[sig][0], 1);
      gpio_set_level((gpio_num_t)signals[sig][1], 1);
      gpio_set_level((gpio_num_t)signals[sig][2], 0);           
    }    
    for (sig=2; sig<NUM_SIGS; sig++) {
      gpio_set_level((gpio_num_t)signals[sig][0], 1);
      gpio_set_level((gpio_num_t)signals[sig][1], 0);
      gpio_set_level((gpio_num_t)signals[sig][2], 1);           
    }
    delay(50);
    for (sig=2; sig<NUM_SIGS; sig++) {
      gpio_set_level((gpio_num_t)signals[sig][0], 0);
      gpio_set_level((gpio_num_t)signals[sig][1], 1);
      gpio_set_level((gpio_num_t)signals[sig][2], 1);           
    }    
  }
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

void calibrateLight(int cycles) {
  int i, j;
  totalLight = 0;
  logTo::logToAll("Starting light measurement...");
  for (i=0; i < cycles; i++) {
    for (j=0; j < trigSize; j++) {
      triggers[j].initVal += analogRead(triggers[j].GPIO);
      triggers[j].avgVal.begin();      
    }
    delay(10);
  }
  logTo::logToAll("Startup done, light measurements:");
  // average initial readings
  for (i = 0; i < trigSize; i++) {
    triggers[i].initVal /= cycles;
    logTo::logToAll(String(i) + ": dir " + printDirection(triggers[i].trackDir) + ", " + String(triggers[i].initVal) + " gpio:" + String(triggers[i].GPIO) + " loc: " + printDirection(triggers[i].location));
    totalLight += triggers[i].initVal;
  }

  logTo::logToAll("total light level: " + String(totalLight));
}

void showLight() {
  int i;
  newTotalLight = 0;
  logTo::logToAll("tof=" + String(tof));
  for (i = 0; i < trigSize; i++) {
    logTo::logToAll(String(i) + " curr: " + String(triggers[i].curVal) + " init: " + String(triggers[i].initVal)
      + " ratio: " + String((float)triggers[i].curVal/triggers[i].initVal*100, 2) + "% thresh: " + triggers[i].trigLevel);
    newTotalLight += triggers[i].curVal;
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

int checkTOF() {
  int distance;
  if (vl53.dataReady()) {
    // new measurement for the taking!
    distance = vl53.distance();
    if (distance == -1) {
      // something went wrong!
      //logTo::logToAll("Couldn't get distance: " + String(vl53.vl_status));
      Serial.printf("Couldn't get distance: %d\n",vl53.vl_status);
      return -1;
    }
    Serial.print(F("Distance: "));
    Serial.println(distance);
    // data is read out, time for another reading!
    vl53.clearInterrupt();
    return distance;
  }
  return -2;
}

void setup() {
  Serial.begin(115200); delay(500);
  Serial.println("diamond crossing signal controller");

  pinMode(0, INPUT_PULLUP); // boot button
  attachInterrupt(digitalPinToInterrupt(0), buttonISR, FALLING);

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

  host = preferences.getString("hostname", host);
  logTo::logToAll("hostname: " + host + "\n");

  Wire.begin(SDA,SCL);
  
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

  for (int sig=0; sig<NUM_SIGS; sig++) {
    gpio_config_t io_conf = {};
    io_conf.pin_bit_mask = (1ULL << (gpio_num_t)signals[sig][0] | 1ULL << (gpio_num_t)signals[sig][1] | 1ULL << (gpio_num_t)signals[sig][2]);
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
    io_conf.intr_type = GPIO_INTR_DISABLE;
    gpio_config(&io_conf);
    // test crossing signals at startup
    // turn on green
    gpio_set_level((gpio_num_t)signals[sig][0], 0);
    gpio_set_level((gpio_num_t)signals[sig][1], 1);
    gpio_set_level((gpio_num_t)signals[sig][2], 1);
    delay(200);
    // turn on red
    gpio_set_level((gpio_num_t)signals[sig][0], 1);
    gpio_set_level((gpio_num_t)signals[sig][1], 1);
    gpio_set_level((gpio_num_t)signals[sig][2], 0);
    delay(200);
    // turn on yellow
    gpio_set_level((gpio_num_t)signals[sig][0], 1);
    gpio_set_level((gpio_num_t)signals[sig][1], 0); 
    gpio_set_level((gpio_num_t)signals[sig][2], 1);
  }

  // set pins for controlling voltage to track
  pinMode(RELAYIN, OUTPUT);  // inner (mountain) loop
  digitalWrite(RELAYIN, LOW);
  pinMode(RELAYOUT, OUTPUT); // outer (flat) loop
  digitalWrite(RELAYOUT, LOW);
 
  // set the ADC attenuation to 11 dB (up to ~3.3V input)
  analogSetAttenuation(ADC_11db);

  // measure ambient light on photoresistors
  calibrateLight(LIGHT_CYCLES);

  // initialize TOF sensor
  Wire.begin(SDA,SCL);
  if (! vl53.begin(0x29, &Wire)) {
    logTo::logToAll("Error on init of VL sensor: " + String(vl53.vl_status));
  }
  logTo::logToAll("VL53L1X sensor OK!");

  logTo::logToAll("Sensor ID: 0x" + String(vl53.sensorID(), HEX));

  if (! vl53.startRanging()) {
    logTo::logToAll("Couldn't start ranging: " + String(vl53.vl_status));
  }
  logTo::logToAll("Ranging started");

  // Valid timing budgets: 15, 20, 33, 50, 100, 200 and 500ms!
  vl53.setTimingBudget(50);
  logTo::logToAll("Timing budget (ms): " + String(vl53.getTimingBudget()));

  /*
  vl.VL53L1X_SetDistanceThreshold(100, 300, 3, 1);
  vl.VL53L1X_SetInterruptPolarity(0);
  */

  // here's where the magic happens...determine if any sensors have triggered and set signals
  app.onRepeat(LOOPDELAY, []() {
    int i, analogValue, triggerValue;
    float trigRatio;
    unsigned long cTime = millis();

    bool nsTriggerHigh = false;
    bool ewTriggerHigh = false;
    // first check to see if any sensors have triggered in either direction
    if (STARTTRIG > 0) {
      if (int x = checkTOF() >=0) // sometimes it can't read state don't know why
        tof = x;
      if (tof > -1 && tof < 100) {  // arbitrary value right now
        nsTriggerHigh = true;
        logTo::logToAll("tof trigger " + String(tof));
        if (teleplot)
          Serial.printf(">tof:%d\n", tof);
      }
    }
    for (i = STARTTRIG; i < trigSize; i++) {
      cTime = millis();
      triggers[i].curVal = analogRead(triggers[i].GPIO);
      analogValue = triggers[i].avgVal.reading(triggers[i].curVal);
      if (teleplot)
        Serial.printf(">trig%d:%d\n",i,analogValue);
      trigRatio = (analogValue/(float)triggers[i].initVal)*100;
      if (trigRatio < triggers[i].trigLevel) {
        //logTo::logToAll("sensor " + String(i) + " dir: " + printDirection(triggers[i].trackDir));
        if (triggers[i].trackDir == N || triggers[i].trackDir == S) {
          nsTriggerHigh = true;
        }
        if (triggers[i].trackDir == E || triggers[i].trackDir == W) {
          ewTriggerHigh = true;
        }
        lastTrigTime = cTime;
      }
    }
    if (nsTriggerHigh) {
      if (nsState.state == Y) { // state change
        logTo::logToAll("NS sensor triggered on Y go to green " + String(cTime/1000));
        showLight();
        nsState.state = G;
        ewState.state = R;
        setLED(N, G);
        setLED(S, G);
      } else { // my state is red and there's still a train on the other block
        //logTo::logToAll("NS trigger high and state (ns/ew): " + printState(nsState.state) + "/" + printState(ewState.state));
        if (nsState.state == R)
          if (ewTriggerHigh) {
            // turn off this block! state was red and we got a trigger
            logTo::logToAll("NS sensor triggered on R shut down block! " + String(cTime/1000));
            digitalWrite(RELAYIN, HIGH);
            showLight();
          } else {
            logTo::logToAll("NS clear to proceed "  + printState(nsState.state) + " " + String(cTime/1000) + " in " + String(CLEARDELAY*500) + " ewtrig: " + String(ewTriggerHigh));
            showLight();
            nsState.state = Y;
            ewState.state = R;
            setLED(N,Y);
            setLED(S,Y);
            setLED(E,R);
            setLED(W,R);
            delay(CLEARDELAY*500);
            digitalWrite(RELAYIN, LOW);
          }
      }
    }
    if (ewTriggerHigh) {
      if (ewState.state == Y) { // state change
        logTo::logToAll("EW sensor triggered on Y go to green " + String(cTime/1000));
        showLight();
        ewState.state = G;
        nsState.state = R;
        setLED(E,G);
        setLED(W,G);
      // if my state was set to R and I'm triggering, I can go if the other direction ISN'T triggered
      } else {
        //logTo::logToAll("EW trigger high and state (ns/ew): " + printState(nsState.state) + "/" + printState(ewState.state));
        if (ewState.state == R)
          if (nsTriggerHigh) {
            // turn off this block! state was red and we got a trigger
            logTo::logToAll("EW sensor triggered on R shut down block! " + String(cTime/1000));
            digitalWrite(RELAYOUT, HIGH);
            showLight();
          } else {
            logTo::logToAll("EW clear to proceed "  + printState(ewState.state) + " " + String(cTime/1000) + " in " + String(CLEARDELAY*500) + " NStrig: " + String(nsTriggerHigh));
            showLight();
            nsState.state = R;
            ewState.state = Y;
            setLED(E,Y); 
            setLED(W,Y); 
            setLED(N,R);
            setLED(S,R);
            delay(CLEARDELAY*500);
            digitalWrite(RELAYOUT, LOW);
          }
      }
    }
    // no triggers high so set all back to yellow
    // wait CLEARDELAY seconds since last trigger before clearing
    if (!nsTriggerHigh && !ewTriggerHigh) {  
      //logTo::logToAll("NS=" + printState(nsState.state) + " EW=" + printState(ewState.state));
      if (nsState.state != Y && ewState.state != Y) {
        if ((cTime - lastTrigTime) > CLEARDELAY*1000) {
          logTo::logToAll("no triggers go to Y, last trigger delta: " + String(lastTrigTime/1000) + " " + String(cTime/1000));
          showLight();
          //calibrateLight(LIGHT_CYCLES);
          nsState.state = Y;
          ewState.state = Y;
          setLED(nulldir, Y);
          // turn on both relays in case train is stopped with coupler over photosensor
          digitalWrite(RELAYIN, LOW);
          digitalWrite(RELAYOUT, LOW);
        }
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
  if (Serial.available() > 0) {
    String input = "";
    while (Serial.available() > 0) {
      char c = Serial.read();
      if (c == '\n' || c == '\r') {
        // Ignore newline and carriage return characters
        break;
      }
      input += c;
    }
    if (input.length() > 0) {
      Serial.print("Received: ");
      Serial.println(input);
    }
  }
}
