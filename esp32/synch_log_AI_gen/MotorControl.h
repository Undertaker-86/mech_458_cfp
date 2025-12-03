#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include <Arduino.h>
#include "config.h"

// Motor Control Class or Namespace
// Using a class with static members or a namespace is common for singletons like this.
// For simplicity in Arduino, I'll use a class with static members or just functions.
// Let's use a class to keep it clean.

class MotorControl {
public:
    static void init();
    static void update();
    static void manualJog(int motor, bool up);

    static void setTarget(long target, bool immediate = false);
    static void setZero();

    static void emergencyHalt();
    static void updateRPM(unsigned long intervalMs);
    
    // Telemetry getters
    static long getM1Pos();
    static long getM2Pos();
    static long getTargetPos();
    static float getM1RPM();
    static float getM2RPM();
    static int getPWM1();
    static int getPWM2();
    static void getMasterPID(float &kp, float &ki, float &kd);
    static void getSlavePID(float &kp, float &ki, float &kd);
    static void setMasterPID(float kp, float ki, float kd);
    static void setSlavePID(float kp, float ki, float kd);
    
    // ISRs need to be public/accessible
    static void countM1();
    static void countM2();

private:
    static volatile long m1_pos;
    static volatile long m2_pos;

    static long target_pos; // This will now be the instantaneous setpoint for PID
    static long final_target_pos; // The ultimate destination
    static float current_setpoint; // Floating point for smooth ramping
    static unsigned long last_ramp_time;

    
    // RPM Calculation
    static long last_m1_pos;
    static long last_m2_pos;
    static float m1_rpm;
    static float m2_rpm;
    static unsigned long lastRpmCalcTime;

    // Control
    // Control
    static int global_pwm1;
    static int global_pwm2;
    
    // Master PID (Position)
    static float kp_m1, ki_m1, kd_m1;
    static float integral_error_m1;
    static long last_error_m1;

    // Slave PID (Sync)
    static float kp_m2, ki_m2, kd_m2;
    static float integral_error_m2;
    static long last_error_m2;

    static bool isSynced;
};

#endif
