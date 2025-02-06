//
// interact with DCC-EX Command Station
//
#include "include.h"

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
    logTo::All("print roster");
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
