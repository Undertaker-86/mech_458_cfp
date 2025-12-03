#include "Logger.h"

unsigned long Logger::lastLogTime = 0;

void Logger::logData() {
    if (millis() - lastLogTime > LOG_INTERVAL) {
        // Update RPM before logging
        MotorControl::updateRPM(LOG_INTERVAL);
        
        // CSV Output
        Serial.print(millis()); Serial.print(",");
        Serial.print(MotorControl::getTargetPos()); Serial.print(",");
        Serial.print(MotorControl::getM1Pos()); Serial.print(",");
        Serial.print(MotorControl::getM2Pos()); Serial.print(",");
        Serial.print(MotorControl::getM1RPM()); Serial.print(",");
        Serial.print(MotorControl::getM2RPM()); Serial.print(",");
        Serial.print(MotorControl::getPWM1()); Serial.print(",");
        Serial.println(MotorControl::getPWM2());
        
        lastLogTime = millis();
    }
}
