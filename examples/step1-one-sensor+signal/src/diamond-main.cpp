//
// diamond crossing guard stage 1
// one sensor and one signal
//
#include <Arduino.h>
#include <SparkFun_VL53L1X.h>

SFEVL53L1X dSensor;

int led[] = { 25, 26, 27}; // GPIOs for green, yellow, and red
bool ledState = false;

#define MINDIST 10  // mm distance from sensor

void setup() {
  Serial.begin(115200); delay(300);
  Serial.println("Model RR Diamond Crossing");

  Wire.begin(SDA,SCL);
  if (dSensor.begin() != 0) { // Begin returns 0 on a good init
    Serial.println("Sensor failed to begin. Please check wiring. Freezing...");
    while (1);
  }
  Serial.println("Sensor online.");
  for (int i=0; i<3; i++) {
    pinMode(led[i], OUTPUT);
    digitalWrite(led[i], HIGH); // Active low: HIGH turns LED off  
  }
  Serial.println("testing LEDs...");
  for (int i=0; i<3; i++) {
    digitalWrite(led[i], LOW);
    delay(1000);
    digitalWrite(led[i], HIGH);
  }  
  // turn on yellow
  digitalWrite(led[1],LOW);
}

void loop() {
  dSensor.startRanging(); // Write configuration bytes to initiate measurement
  while (!dSensor.checkForDataReady()) {
    delay(1);
  }
  int distance = dSensor.getDistance(); // Get the result of the measurement from the sensor
  dSensor.clearInterrupt();
  dSensor.stopRanging();
  if (distance < MINDIST) {
    // train present; change LED from yellow to green
    digitalWrite(led[1],HIGH);
    digitalWrite(led[0],LOW);
    ledState = true;
  } else if (ledState) {
    // train was present but has left; go back to yellow
    digitalWrite(led[0],HIGH);
    digitalWrite(led[1],LOW);
    ledState = false;
  }
}