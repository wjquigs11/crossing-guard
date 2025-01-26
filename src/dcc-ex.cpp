//
// interact with DCC-EX Command Station
//
#include "include.h"

#if 0
// maximum number of tokens in a DCC-EX response
#define MAXDCC 10
int values[MAXDCC];
int idx = 0;

loco cabDCC[MAXLOC];

/*
Speed is, inexplicably, asymmetric. Speed to controller:
 > speed: 0-127
 > dir: one of
    • 1=forward
    • 0=reverse
Speed from controller:
speedbyte: Speed in DCC speedstep format.
    • reverse - 2-127 = speed 1-126, 0 = stop
    • forward - 130-255 = speed 1-126, 128 = stop
*/

void parseDCC(String input)
{
    char *ptr = strtok((char *)input.c_str(), " <>"); // Split by spaces and brackets

    while (ptr != NULL && idx < MAXDCC)
    {
        if (isdigit(ptr[0]) || ptr[0] == '-')
        { // Check if token is a number
            values[idx++] = atoi(ptr);
        }
        ptr = strtok(NULL, " <>");
    }
    input = String();
    idx = 0;
}

// this isn't going to work. Needs to be async. If one cab slows down while I'm waiting for speed update from other cab...
int getSpeed(int cab, int timeout)
{
    String speedS = "<t " + String(cab) + ">";
    logTo::logToAll("getSpeed tx: " + speedS);
    Serial1.println(speedS);
    unsigned long startTime = millis();
    String receivedString;
    // min/max timeout 100 msecs to 5 secs
    if (timeout < 100)
        timeout = 100;
    if (timeout > 5000)
        timeout = 5000;
    while (!Serial1.available())
    {
        unsigned long now = millis();
        if (now - startTime > timeout)
        {
            // Handle timeout condition
            logTo::logToAll("timed out waiting for response from DCC-EX " + String(now) + " " + String(startTime) + " " + String(timeout));
            return -1;
        }
        yield();
    }
    if (Serial1.available() > 0)
    {
        receivedString = Serial1.readString();
        // receivedString.trim();  // Remove whitespace and newlines
        logTo::logToAll("getSpeed rx: " + receivedString);
    }
    parseDCC(receivedString);
    int cabRX = values[0];
#if 0
    if (values[0] != cab) {
        logTo::logToAll("DCC response wrong cab: " + String(values[0]) + " expected " + String(cab));
        return -1;
    }
#endif
    cabDCC[cabRX].cabnum = cabRX;
    int dccSpeed = values[2];
    if (dccSpeed > 0 && dccSpeed < 128)
    {
        cabDCC[cabRX].saveSpeed = dccSpeed - 1;
        cabDCC[cabRX].direction = 0;
    }
    else if (dccSpeed > 129 && dccSpeed < 256)
    {
        cabDCC[cabRX].saveSpeed = dccSpeed - 129;
        cabDCC[cabRX].direction = 1;
    }
    else
    {
        cabDCC[cabRX].saveSpeed = 0;
    }
    // funcmap = values[3];
    logTo::logToAll("cab " + String(cabRX) + " speed is " + String(dccSpeed) + " direction " + String(cabDCC[cabRX].direction));
    return cabDCC[cabRX].saveSpeed;
}

void brake(int cab)
{
    cabDCC[cab].saveSpeed = getSpeed(cab, 100);
    logTo::logToAll("stopping cab " + String(cab) + " from speed " + String(cabDCC[cab].saveSpeed));
    int newSpeed = cabDCC[cab].saveSpeed;
    String speedS;
#if MOMENTUM
    while (newSpeed > 0)
    {
        newSpeed /= 3;
        speedS = "<t" + String(cab) + " " + String(newSpeed) + " " + String(cabDCC[cab].direction) + ">";
        Serial1.println(speedS);
        delay(200);
    }
#else
    speedS = "<t" + String(cab) + " 0 " + String(cabDCC[cab].direction) + ">";
    Serial1.println(speedS);
#endif
}

void resume(int cab)
{
    logTo::logToAll("resuming cab " + String(cab) + " to speed " + String(cabDCC[cab].saveSpeed));
    String speedS;
#if MOMENTUM
    int newSpeed = cabDCC[cab].saveSpeed / 20;
    while (newSpeed < cabDCC[cab].saveSpeed)
    {
        speedS = "<t" + String(cab) + " " + String(newSpeed) + " " + String(cabDCC[cab].direction) + ">";
        newSpeed += 10;
        Serial1.println(speedS);
        delay(500);
    }
#else
    speedS = "<t" + String(cab) + " " + String(cabDCC[cab].saveSpeed) + " " + String(cabDCC[cab].direction) + ">";
    Serial1.println(speedS);
#endif
}
#endif

DCCEXProtocol dccexProtocol;
MyDelegate myDelegate;
Loco *NSloco, *EWloco;

void MyDelegate::receivedServerVersion(int major, int minor, int patch) {
    CONSOLE.print("\n\nReceived version: ");
    CONSOLE.print(major);
    CONSOLE.print(".");
    CONSOLE.print(minor);
    CONSOLE.print(".");
    CONSOLE.println(patch);
}

void MyDelegate::receivedTrackPower(TrackPower state) {
    CONSOLE.print("\n\nReceived Track Power: ");
    CONSOLE.println(state);
    CONSOLE.println("\n\n");
}

void MyDelegate::receivedRosterList() {
    CONSOLE.println("\n\nReceived Roster");
    printRoster();
    setSigLoco();
}

void MyDelegate::receivedScreenUpdate(int screen, int row, char *message) {
    CONSOLE.println("\n\nReceived screen|row|message");
    CONSOLE.print(screen);
    CONSOLE.print("|");
    CONSOLE.print(row);
    CONSOLE.print("|");
    CONSOLE.println(message);
}

void MyDelegate::receivedLocoUpdate(Loco *loco) {
    //Serial.print("Received Loco update for DCC address: ");
    //Serial.print(loco->getAddress());
    //Serial.print(" speed: ");
    //Serial.println(loco->getSpeed());
    // sometimes the "get roster" command doesn't work so I will set signal->loco when I get a speed update
    if (!NSlocoID && !EWlocoID) setSigLoco();
}

void printRoster() {
    logTo::logToAll("print roster");
    for (Loco *loco = dccexProtocol.roster->getFirst(); loco; loco = loco->getNext()) {
        int id = loco->getAddress();
        char *name = loco->getName();
        CONSOLE.print(id);
        CONSOLE.print(" ~");
        CONSOLE.print(name);
        CONSOLE.println("~");
#if 0
        for (int i = 0; i < 32; i++)
        {
            char *fName = loco->getFunctionName(i);
            if (fName != nullptr)
            {
                CONSOLE.print("loadFunctionLabels() ");
                CONSOLE.print(fName);
                if (loco->isFunctionMomentary(i))
                {
                    CONSOLE.print(" - Momentary");
                }
                CONSOLE.println();
            }
        }
#endif
    }
    CONSOLE.println("\n");
}
