//
// diamond crossing guard stage 3
// multiple sensors on SparkFun I2C Mux and multiple signals on MCP23017
// now we will associate a signal with a sensor on the same track
//
// i2c mux: https://www.sparkfun.com/sparkfun-qwiic-mux-breakout-8-channel-tca9548a.html
// MCP23017: https://www.adafruit.com/product/5346

#include <Arduino.h>
#include <SparkFun_VL53L1X.h>
#include <Adafruit_MCP23X17.h>
#include <SparkFun_I2C_Mux_Arduino_Library.h>
#include "logto.h"

#define MUXMAX 7  // number of MUX ports
QWIICMUX myMux;
SFEVL53L1X dSensor[MUXMAX];
bool sensorOn[MUXMAX];

Adafruit_MCP23X17 mcp;

#define NUM_SIGS 4
#define NUM_LEDS 3
#define ON LOW
#define OFF HIGH

#define TRIGLEVEL 10 // trigger sensor if it drops to % of start value

// direction north, south, east, or west
typedef enum {
    N, S, E, W, 
    nulldir
} cDirection;

// state of the signal
typedef enum {
    Y, // Yellow
    G, // Green
    R  // Red
} sigState;

// sensors represent the TOF lasers in the track and their associated GPIOs (on the MCP)
struct Sensor {
    cDirection location; // e.g, located at W end of E/W track
    bool active;
    int initVal;  // value at startup (with no trains over it)
    int curVal; // value at current reading
    int sensIdx;    // Sensor[] isn't necessarly in same order as dSensor[]
    int trigLevel {TRIGLEVEL};

    Sensor(cDirection location, bool active, int initVal, int curVal, int sensIdx, int trigLevel)
        : location(location),
          active(active),
          initVal(initVal),
          curVal(curVal),
          sensIdx(sensIdx),
          trigLevel(trigLevel) {}
};

struct Sensor sensors[] = {
  {E, false, 0, 0, 3, TRIGLEVEL},
  {W, false, 0, 0, 0, TRIGLEVEL},      
  {S, false, 0, 0, 2, TRIGLEVEL},      
  {N, false, 0, 0, 1, TRIGLEVEL},       
  {W, false, 0, 0, 4, TRIGLEVEL}
}; 

struct Signal {
    int GPIO[NUM_LEDS]; // one GPIO for each of green, yellow, red
    cDirection location;  // location (N/S/E/W) of the signal/LEDs
    sigState state; // state = G/Y/R
};

// signals are defined by the GPIOs each LED is connected to, the location, and the state
Signal signals[NUM_SIGS] = {
    {{0, 1, 2}, S, Y},
    {{3, 4, 5}, N, Y},
    {{6, 7, 8}, W, Y},
    {{9, 10, 11}, E, Y}
};

void setup() {
  int i,j;
  Serial.begin(115200); delay(300);
  Serial.println("Model RR Diamond Crossing");

  Wire.begin(SDA,SCL);
  Wire.setClock(100000);
  if (myMux.begin() == false) {
    Serial.println("Mux not detected!");
  } else
    Serial.println("Mux detected");
  for (i=0; i<MUXMAX; i++) {
    myMux.setPort(i);
    if (dSensor[i].begin() != 0) {
      Serial.printf("Sensor %d failed to begin or not installed.\n", i);
    } else {
      sensorOn[i] = true;
      dSensor[i].setDistanceModeShort();
    }
  }
  if (!mcp.begin_I2C(0x20)) {
    Serial.printf("Error on init of mcp at 0x%x\n", 0x0);
    //while(1);
  } else Serial.println("MCP online.");
  Serial.println("init mcp pins");
  for (i=0; i<NUM_SIGS; i++) {
    for (j=0; j<NUM_LEDS; j++) {
      Serial.printf("%d.%d: %d\n",i,j,led[i][j]);
      mcp.pinMode(led[i][j],OUTPUT);
      mcp.digitalWrite(led[i][j],OFF);
    }
  }
}

void loop() {
  int i, j, dist;
  for (i=0; i<MUXMAX; i++) {
    if (sensorOn[i]) {
      myMux.setPort(i);
      dSensor[i].startRanging();
      while (!dSensor[i].checkForDataReady()) {
        delay(1);
      }       
      dist = dSensor[i].getDistance();
      dSensor[i].clearInterrupt();
      dSensor[i].stopRanging();
      if (dist < MINDIST) {
        // this example just turns all signals to red when a train approaches
        // we don't know where the train is until next example
        for (j=0; i<NUM_SIGS; j++) {
          digitalWrite(led[j][0],OFF);
          digitalWrite(led[j][1],OFF);
          digitalWrite(led[j][2],ON);
        }
      } else {
        // sensor not detecting train; go back to yellow
        for (j=0; i<NUM_SIGS; j++) {
          digitalWrite(led[j][0],OFF);
          digitalWrite(led[j][1],ON);
          digitalWrite(led[j][2],OFF); 
        }       
      }
    }
  }
}

