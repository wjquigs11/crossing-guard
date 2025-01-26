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
#include <DCCEXProtocol.h>
#include <Adafruit_MCP23X17.h>
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"

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
} CrossDir;

typedef enum {
    Clear, Occupied, Clearing,
} blockState;

// sensors represent the photresistors in the track and their associated GPIOs for ESP32 ADCs
struct Sensor {
    CrossDir location; // e.g, located at W end of E/W track
    bool active;
    int initVal;
    int curVal;
    movingAvg avgVal; 
    int sensIdx;    // Sensor[] isn't necessarly in same order as irSensor[]
    int trigLevel {TRIGLEVEL};
    bool curState;
    bool prevState;
    int turnE;  // turnout to throw for eastbound trains
    bool dirE;  // which direction to throw (straight/reverse)
    int turnW;  // turnout to throw for westbound trains
    bool dirW;  // direction

    Sensor(CrossDir location, bool active, int initVal, int curVal, movingAvg avgVal, int sensIdx, int trigLevel, 
        bool curState, bool prevState, int turnE, bool dirE, int turnW, bool dirW)
        : location(location),
          active(active),
          initVal(initVal),
          curVal(curVal),
          avgVal(avgVal),
          sensIdx(sensIdx),
          trigLevel(trigLevel),
          curState(curState),
          prevState(prevState),
          turnE(turnE),
          dirE(dirE),
          turnW(turnW),
          dirW(dirW) {}
};

extern struct Sensor sensors[];
extern int trigSize;

#define NUM_SIGS 4  // 4 signals on the diamond crossing
#define NUM_LEDS 3  // each signal has 3 LEDs

struct Signal {
    int GPIO[NUM_LEDS];
    CrossDir location;  // location (N/S/E/W) of the signal/LEDs
    State state; // state = G/Y/R
    int relay; // GPIO of relay to turn off the track at this sensor
    Loco *loco; // if DCC, which locomotive to brake
    //int loco;
};
extern Signal signals[];

extern QWIICMUX myMux;
// SF MUX i2c @ 0x70
#define MUXMAX 7  // number of MUX ports
// TEMP!!! saving last MUX port for second MCP !!!
// TOF sensors i2c @ 0x29
extern SFEVL53L1X irSensor[MUXMAX];
extern bool sensorOn[MUXMAX];
#define CROSSMAX 5  // MUX ports 0..4 are for crossing

extern blockState crossState;
extern CrossDir fromDir;

// maximum number of locomotives
#define MAXLOC 3
struct loco {
    int cabnum, speed, saveSpeed, direction=1, funcmap;
};
extern loco cabDCC[MAXLOC];
// TBD don't use globals maybe a Cab class

// wifi/web functions
bool initWiFi();
void startAP();
void startWebServer();
String getSensorReadings();
void logToAll(String s);
void serialWrapper(String);
void WebSerialonMessage(uint8_t *data, size_t len);
void ledOn();
void ledOff();

// track control functions
void showSensors();
void setTrigger(Sensor *trigger, int trigLevel);
void calibrateSensors(int cycles);
void printSigState();
String printCrossState(blockState BS);
void setLED(int sigIdx, State state);
void setSigLoco();

extern int NSlocoID, EWlocoID;
extern String DCCEXhostname;
extern int saveSpeed; // to restore after braking
extern Direction saveDir;

void i2cScan(TwoWire Wire);

// DCC functions
int getSpeed(int cab, int timeout);
void brake(int cab);
void resume(int cab);

// for DCC-EX library
#define CONSOLE Serial
#define CLIENT Serial1
#define S1RX 32
#define S1TX 33

// Declare functions to call from our delegate
void printRoster();

#ifndef MY_DELEGATE_H
#define MY_DELEGATE_H
class MyDelegate : public DCCEXProtocolDelegate {
public:
    void receivedServerVersion(int major, int minor, int patch);
    void receivedTrackPower(TrackPower state);
    void receivedRosterList();
    void receivedScreenUpdate(int screen, int row, char *message);
    void receivedLocoUpdate(Loco *loco);
};
#endif

extern DCCEXProtocol dccexProtocol;
extern MyDelegate myDelegate;
extern Loco *NSloco, *EWloco;

// turnouts
#define MCP 1
// Adafruit i2c 0x21
// generic i2c 0x20
extern Adafruit_MCP23X17 mcp[MCP];
extern int MCPaddr[];
void throwTurnout(int t, bool straight);
void throwWrapper(int t, bool d);
void checkToggles();
//#define straight true
//#define reverse false

struct TurnoutQ {
    // define GPIOs to control a turnout
    int mcp;   // which MCP is this connected to?
    int toggleA;  // normal or straight direction (left for double slip)
    int toggleB;  // reverse or curved direction (right for double slip)
    bool togStateA;  // last state of toggle switch GPIO (for debounce)
    bool togStateB;  // need state for each direction
    int solenoidA;
    int solenoidB;
    int LEDA;
    int LEDB;
    bool state; // true = straight, false = curved
};

#define TURNOUTS 4
extern TurnoutQ layout[];
#define TRIGTIME 40 // msecs to run motor driver to throw turnout
//#define TRIGTIME 10000
extern int turnTime;
extern bool loopMode;