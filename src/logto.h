
#ifndef LOGTO_H
#define LOGTO_H

class logTo {
public:
    static void logToAll(String s);
    logTo() {}
    static const int ASIZE = 20;
    static String commandList[ASIZE];
};
#endif