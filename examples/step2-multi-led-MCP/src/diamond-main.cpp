//
// diamond crossing guard stage 2
// multiple sensors on SparkFun I2C Mux and multiple signals on MCP23017
//
// i2c mux: https://www.sparkfun.com/sparkfun-qwiic-mux-breakout-8-channel-tca9548a.html
// MCP23017: https://www.adafruit.com/product/5346

#include <Arduino.h>
#include <SparkFun_VL53L1X.h>
#include <Adafruit_MCP23X17.h>
#include <SparkFun_I2C_Mux_Arduino_Library.h>

#define MUXMAX 7  // number of MUX ports
QWIICMUX myMux;
SFEVL53L1X dSensor[MUXMAX];
bool sensorOn[MUXMAX];

Adafruit_MCP23X17 mcp;

#define NUM_SIGS 4
#define NUM_LEDS 3
#define ON LOW
#define OFF HIGH

// 4 signals (N/S/E/W), each with 3 LED (G/Y/R)
// each cell refers to a GPIO for that LED
int led[NUM_SIGS][NUM_LEDS] = 
  {{0, 1, 2},
   {3, 4, 5},
   {6, 7, 8}, 
   {9, 10, 11}};

#define MINDIST 10  // mm distance from sensor

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

