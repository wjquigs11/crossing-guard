#include <Arduino.h>
#include <WiFi.h>
#include "SPIFFS.h"
#include <ArduinoJson.h>
#include <ESPmDNS.h>
#include <ESPAsyncWebServer.h>
#include <WebSerialPro.h>
#include <ElegantOTA.h>
#include <NTPClient.h>
#include <ReactESP.h>
#include <Preferences.h>
#include <Arduino.h>
#include <Time.h>
#include <Wire.h>
#include <SPI.h>
#include <MovingAvg.h>
#include <SparkFun_VL53L1X.h>
#include <SparkFun_I2C_Mux_Arduino_Library.h>

#include "logto.h"

extern AsyncWebServer server;
extern bool serverStarted;
extern AsyncEventSource events;
extern String host;
extern JsonDocument readings;
extern int WebTimerDelay;
extern Preferences preferences;
extern File consLog;

#define HTTP_PORT 80
#define MAX_NETS 4

// Timer variables
#define DEFDELAY 1000
extern unsigned long lastTime;
extern int WebTimerDelay;
extern int minReadRate;

extern bool teleplot;
extern bool logToSerial;

#define LIGHT_CYCLES 10
#define TRIGLEVEL 10 // % trigger sensor if it drops to % of start value
#define MOVINGAVG 10
#define CLEARDELAY 8  // seconds to wait after crossing is clear to reset signals
#define LOOPDELAY 10 // (msecs) configurable in case trains move faster

// GPIOs to prevent collision at crossing
// switching to relays from MOSFET because MOSFET is polarized
#define RELAYOUT 26  // outer (flat) loop
#define RELAYIN 27 // inner (mountain) loop
#define NORTHSOUTH RELAYIN
#define EASTWEST RELAYOUT
extern bool stopInner;
extern bool stopOuter;
extern bool enableRelays;

typedef enum {
    Y, // Yellow
    G, // Green
    R  // Red
} State;

typedef enum {
    N, S, E, W, 
    nulldir
} Direction;

typedef enum {
    Clear, Occupied, Clearing,
} blockState;

// sensors represent the photresistors in the track and their associated GPIOs for ESP32 ADCs
struct Sensor {
    Direction location; // e.g, located at W end of E/W track
    bool active;
    int initVal;
    int curVal;
    movingAvg avgVal; 
    int sensIdx;    // Sensor[] isn't necessarly in same order as irSensor[]
    int trigLevel {TRIGLEVEL};

    Sensor(Direction location, bool active, int initVal, int curVal, movingAvg avgVal, int sensIdx, int trigLevel)
        : location(location), active(active), initVal(initVal), curVal(curVal), avgVal(avgVal), sensIdx(sensIdx), trigLevel(TRIGLEVEL) {}
};

extern struct Sensor sensors[];
extern int trigSize;

#define NUM_SIGS 4  // 4 signals on the diamond crossing
#define NUM_LEDS 3  // each signal has 3 LEDs

struct Signal {
    int GPIO[NUM_LEDS];
    Direction location;  // location (N/S/E/W) of the signal/LEDs
    State state; // state = G/Y/R
    int solenoid; // GPIO of solenoid to turn off the track at this sensor
    int loco; // if DCC, which locomotive to brake
};
extern Signal signals[];

extern QWIICMUX myMux;
#define MUXMAX 8  // number of MUX ports
extern SFEVL53L1X irSensor[MUXMAX];
extern bool sensorOn[MUXMAX];
#define CROSSMAX 5  // MUX ports 0..4 are for crossing

extern blockState crossState;
extern Direction fromDir;

// wifi/web functions
bool initWiFi();
void startAP();
void startWebServer();
String getSensorReadings();
void logToAll(String s);
void WebSerialonMessage(uint8_t *data, size_t len);

// track control functions
void showSensors();
void setTrigger(Sensor *trigger, int trigLevel);
void calibrateSensors(int cycles);
void printSigState();
void setLED(int sigIdx, State state);

extern int NSlocoID, EWlocoID;
extern int lastSpeed;
extern String DCCEXhostname;

extern int proxTrig; // temporary to get trigger from other esp32

// DCC functions
int getSpeed(int cab);
void brake(int cab);
void resume(int cab);