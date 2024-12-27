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
#include "MovingAvg.h"
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

#define LIGHT_CYCLES 100
#define TRIGLEVEL 55 // % trigger sensor if light drops to % of start value
#define MOVINGAVG 10
#define CLEARDELAY 8  // seconds to wait after crossing is clear to reset signals
#define LOOPDELAY 100 // (msecs) configurable in case trains move faster

// GPIOs to prevent collision at crossing
// switching to relays from MOSFET because MOSFET is polarized
#define RELAYOUT 26  // outer (flat) loop
#define RELAYIN 27 // inner (mountain) loop
extern bool stopInner;
extern bool stopOuter;

typedef enum {
    Y, // Yellow
    G, // Green
    R  // Red
} State;

typedef enum {
    NS, // North-South
    EW,  // East-West
    nulldir
} Direction;

// triggers represent the photresistors in the track and their associated GPIOs for ESP32 ADCs
struct Trigger {
    bool active;
    int initVal;
    int curVal;
    movingAvg avgVal; 
    int GPIO;
    int trigLevel; // threshold for triggering; defaults to TRIGLEVEL

    Trigger(bool active, int initVal, int curVal, int avgSize, int gpio, bool wasTriggered)
        : active(active), initVal(initVal), curVal(curVal), avgVal(avgSize), GPIO(gpio), trigLevel(TRIGLEVEL) {}
};

typedef struct {
    Direction dir;           // Direction (NS or EW)
    State state;         // Current state information
    struct Trigger* triggers;  // Pointer to an array of triggers
    int numTriggers;         // Number of triggers in the array
} DirectionState;

extern struct Trigger nsTriggers[];
extern struct Trigger ewTriggers[];
extern int nsTrigSize;
extern int ewTrigSize;

// wifi/web functions
bool initWiFi();
void startAP();
void startWebServer();
String getSensorReadings();
void logToAll(String s);
void WebSerialonMessage(uint8_t *data, size_t len);

// track control functions
void showLight();
void setTrigger(Trigger *trigger, int trigLevel);
void calibrateLight(int cycles);