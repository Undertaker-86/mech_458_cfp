#ifndef LOGGER_H
#define LOGGER_H

#include <Arduino.h>
#include "MotorControl.h"
#include "config.h"

class Logger {
public:
    static void logData();
private:
    static unsigned long lastLogTime;
};

#endif
