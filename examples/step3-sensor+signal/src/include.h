

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
} sigState;

typedef enum {
    N, S, E, W, 
    nulldir
} cDirection;

typedef enum {
    Clear, Occupied, Clearing,
} blocksigState;

// sensors represent the photresistors in the track and their associated GPIOs for ESP32 ADCs
struct Sensor {
    cDirection location; // e.g, located at W end of E/W track
    bool active;
    int initVal;
    int curVal;
    movingAvg avgVal; 
    int sensIdx;    // Sensor[] isn't necessarly in same order as irSensor[]
    int trigLevel {TRIGLEVEL};
    bool cursigState;
    bool prevsigState;
    int turnE;  // turnout to throw for eastbound trains
    bool dirE;  // which direction to throw (straight/branch)
    int turnW;  // turnout to throw for westbound trains
    bool dirW;  // direction

    Sensor(cDirection location, bool active, int initVal, int curVal, movingAvg avgVal, int sensIdx, int trigLevel, 
        bool cursigState, bool prevsigState, int turnE, bool dirE, int turnW, bool dirW)
        : location(location),
          active(active),
          initVal(initVal),
          curVal(curVal),
          avgVal(avgVal),
          sensIdx(sensIdx),
          trigLevel(trigLevel),
          cursigState(cursigState),
          prevsigState(prevsigState),
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
    cDirection location;  // location (N/S/E/W) of the signal/LEDs
    sigState state; // state = G/Y/R
    int relay; // GPIO of relay to turn off the track at this sensor
    Loco *loco; // if DCC, which locomotive to brake
    //int loco;
};
extern Signal signals[];

extern QWIICMUX myMux;
// SF MUX i2c @ 0x70
#define MUXMAX 7  // number of MUX ports
// TOF sensors i2c @ 0x29
extern SFEVL53L1X irSensor[MUXMAX];
extern bool sensorOn[MUXMAX];
#define CROSSMAX 5  // MUX ports 0..4 are for crossing

extern blocksigState crosssigState;
extern cDirection fromDir;

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
void printSigsigState();
String printCrosssigState(blocksigState BS);
void setLED(int sigIdx, sigState state);
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

// for turnout controller
#define S2RX 34
#define S2TX 2

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
//#define MCP 1
// Adafruit i2c 0x21
// generic i2c 0x20
//extern Adafruit_MCP23X17 mcp[MCP];
//extern int MCPaddr[];
// changing to two independent MCPs
//Adafruit_MCP23X17 mcp[MCP];
//extern Adafruit_MCP23X17 mcpD;
//extern Adafruit_MCP23X17 mcpT;
//extern int mcpD, mcpT;
//int MCPaddr[] = {0x21, 0x20};
void throwTurnout(int t, bool straight);
void throwWrapper(int t, bool d);
void checkToggles();
void showTurnsigState();
//#define straight true
//#define branch false
// i2c address for ESP32 turnout controller
#define I2C_DEV_ADDR 0x55

struct TurnoutQ {
    // define GPIOs to control a turnout
    int mcp;   // which MCP is this connected to?
    int toggleA;  // GPIO for main or straight direction (left for double slip)
    int toggleB;  // branch or reverse direction (right for double slip)
    bool togsigStateA;  // last state of toggle switch GPIO (for debounce)
    bool togsigStateB;  // need state for each direction
    int solenoidA;
    int solenoidB;
    int LEDA;   // LED to light for main
    int LEDB;   // LED to light for branch
    bool state; // true = main, false = branch
};

#define TURNOUTS 4
extern TurnoutQ layout[];
#define TRIGTIME 40 // msecs to run motor driver to throw turnout
//#define TRIGTIME 10000
extern int turnTime;
extern bool loopMode;