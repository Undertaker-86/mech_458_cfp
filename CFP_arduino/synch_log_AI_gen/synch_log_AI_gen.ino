#include <Arduino.h>
#include "config.h"
#include "MotorControl.h"
#include "Logger.h"

void setup() {
  Serial.begin(115200);
  MotorControl::init();
  Serial.println("System Ready. Speed Limit: 0.1 m/s");
}

void loop() {
  
  //// 1. READ COMMANDS ////
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    input.trim();
    
    // Manual Jogging
    if (input.equals("q")) { MotorControl::manualJog(1, true); }
    else if (input.equals("a")) { MotorControl::manualJog(1, false); }
    else if (input.equals("w")) { MotorControl::manualJog(2, true); }
    else if (input.equals("s")) { MotorControl::manualJog(2, false); }
    
    // Set Zero
    else if (input.equalsIgnoreCase("z")) {
      MotorControl::setZero();
      Serial.println("MSG: Zero Set!"); 
    }
    
    // Emergency Halt (Sets target to current position)
    else if (input.equalsIgnoreCase("h")) {
      MotorControl::emergencyHalt();
      Serial.println("MSG: EMERGENCY HALT TRIGGERED");
    }

    // Move to Target
    else {
      long new_target = input.toInt();
      if (new_target == 0 && input != "0") { /* Invalid */ } 
      else {
         MotorControl::setTarget(new_target);
      }
    }
  }

  //// 2. SYNC LOOP ////
  MotorControl::update();

  //// 3. LOGGING ////
  Logger::logData();
}