//
// interact with DCC-EX Command Station
//
#include "include.h"

#define MAXDCC 10
int values[MAXDCC];
int idx = 0;
// TBD don't use globals maybe a Cab class
int cab, speed, saveSpeed, direction=1, funcmap;

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

void parseDCC(String input) {
    char* ptr = strtok((char*)input.c_str(), " <>"); // Split by spaces and brackets

    while (ptr != NULL && idx < MAXDCC) {
        if (isdigit(ptr[0]) || ptr[0] == '-') { // Check if token is a number
            values[idx++] = atoi(ptr);
        }
        ptr = strtok(NULL, " <>");
    }
    input = String();
    idx = 0;
}

int getSpeed(int cab) {
    String speedS = "<t " + String(cab) + ">";
    logTo::logToAll("getSpeed tx: " + speedS);
    Serial.println(speedS);
    unsigned long timeout = 500;
    unsigned long startTime = millis();
    String receivedString;
    while (!Serial.available()) {
        if (millis() - startTime > timeout) {
            // Handle timeout condition
            logTo::logToAll("timed out waiting for response from DCC-EX");
            return -1;
        }
        yield();
    }
    if (Serial.available() > 0) {
        receivedString = Serial.readString();
        receivedString.trim();  // Remove whitespace and newlines
        logTo::logToAll("getSpeed rx: " + receivedString);
    }
    parseDCC(receivedString);
    cab = values[0];
    int dccSpeed = values[2];
    if (dccSpeed > 0 && dccSpeed < 128) {
        speed = dccSpeed-1;
        direction = 0;
    } else if (dccSpeed > 129 && dccSpeed < 256) {
        speed = dccSpeed - 129;
        direction = 1;
    } else {
        speed = 0;
    }
    funcmap = values[3];
    logTo::logToAll("cab " + String(cab) + " speed is " + String(dccSpeed) + " direction " + String(direction));
    return speed;
}

void brake(int cab) {
    saveSpeed = speed = getSpeed(cab);
    logTo::logToAll("stopping cab " + String(cab) + " from speed " + String(speed));
    int newSpeed = speed;
    String speedS;
    while (newSpeed > 0) {  
        newSpeed /= 3;
        speedS = "<t" + String(cab) + " " + String(newSpeed) + " " + String(direction) + ">";
        Serial.println(speedS);
        delay(200);
    }
}

void resume(int cab) {
    logTo::logToAll("resuming cab " + String(cab) + " to speed " + String(speed));
    String speedS;
    int newSpeed = saveSpeed / 20;
    while (newSpeed < saveSpeed) {
        speedS = "<t" + String(cab) + " " + String(newSpeed) + " " + String(direction) + ">";
        newSpeed += 10;
        Serial.println(speedS);
        delay(500);
    }
}