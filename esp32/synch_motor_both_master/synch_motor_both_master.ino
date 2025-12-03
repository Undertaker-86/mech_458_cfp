#include <Arduino.h>
#include "config.h"
#include "MotorControl.h"
#include "Logger.h"
#include "AutoTuner.h"

void setup() {
  Serial.begin(115200);
  MotorControl::init();
  Serial.println("System Ready. Dual-master position control. Speed Limit: 0.1 m/s");
}

void loop() {
  
  //// 1. READ COMMANDS ////
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    input.trim();
    
    if (AutoTuner::isRunning() && 
        !input.equalsIgnoreCase("TUNE_STOP")) {
      Serial.println("TUNE: in progress, only TUNE_STOP allowed");
    }
    else
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

    // Full sequence: tune M1 then M2
    else if (input.startsWith("TUNE_BOTH")) {
      long target1 = TUNE_STEP_TARGET;
      long target2 = TUNE_STEP_TARGET_M2;
      bool apply = false;
      String rest = input.substring(String("TUNE_BOTH").length());
      rest.trim();
      if (rest.length() > 0) {
        int sp = rest.indexOf(' ');
        String tok1 = (sp >= 0) ? rest.substring(0, sp) : rest;
        tok1.trim();
        if (tok1.length() > 0 && !(tok1.toInt() == 0 && tok1 != "0")) {
          target1 = tok1.toInt();
        }
        if (sp >= 0) {
          rest = rest.substring(sp + 1); rest.trim();
          sp = rest.indexOf(' ');
          String tok2 = (sp >= 0) ? rest.substring(0, sp) : rest;
          tok2.trim();
          if (tok2.equalsIgnoreCase("APPLY")) {
            apply = true;
          } else if (tok2.length() > 0 && !(tok2.toInt() == 0 && tok2 != "0")) {
            target2 = tok2.toInt();
          }
          if (sp >= 0) {
            String tok3 = rest.substring(sp + 1); tok3.trim();
            if (tok3.equalsIgnoreCase("APPLY")) apply = true;
          }
        }
      }
      AutoTuner::startBoth(target1, target2, apply, apply);
    }

    // Step-response autotune (Motor 2)
    else if (input.startsWith("TUNE_STEP_M2")) {
      long target = TUNE_STEP_TARGET_M2;
      bool apply = false;
      int firstSpace = input.indexOf(' ');
      if (firstSpace > 0) {
        String arg1 = input.substring(firstSpace + 1);
        arg1.trim();
        int secondSpace = arg1.indexOf(' ');
        String argTarget = (secondSpace >= 0) ? arg1.substring(0, secondSpace) : arg1;
        if (argTarget.length() > 0) target = argTarget.toInt();
        if (secondSpace >= 0) {
          String argApply = arg1.substring(secondSpace + 1);
          argApply.trim();
          if (argApply.equalsIgnoreCase("APPLY")) apply = true;
        }
      }
      AutoTuner::start(target, apply, AutoTuner::M2);
    }
    // Step-response autotune (Motor 1)
    else if (input.startsWith("TUNE_STEP")) {
      long target = TUNE_STEP_TARGET;
      bool apply = false;
      int firstSpace = input.indexOf(' ');
      if (firstSpace > 0) {
        String arg1 = input.substring(firstSpace + 1);
        arg1.trim();
        int secondSpace = arg1.indexOf(' ');
        String argTarget = (secondSpace >= 0) ? arg1.substring(0, secondSpace) : arg1;
        if (argTarget.length() > 0) target = argTarget.toInt();
        if (secondSpace >= 0) {
          String argApply = arg1.substring(secondSpace + 1);
          argApply.trim();
          if (argApply.equalsIgnoreCase("APPLY")) apply = true;
        }
      }
      AutoTuner::start(target, apply);
    }
    else if (input.equalsIgnoreCase("TUNE_STOP")) {
      AutoTuner::abort();
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

  //// 2. CONTROL LOOP ////
  AutoTuner::tick();
  MotorControl::update();

  //// 3. LOGGING ////
  if (!AutoTuner::isRunning()) {
    Logger::logData();
  }
}
