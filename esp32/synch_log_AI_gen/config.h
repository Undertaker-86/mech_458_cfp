#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

//// CONFIGURATION ////
// Encoder Settings (AMT102-V: Set Switch 4 ON for 100 PPR)
const float ENCODER_PPR = 100.0; 

// Speed Limit for 0.1 m/s (approx 3000 RPM on T8 screw)
// Max PWM is 255 (~5400 RPM). 150 is roughly 0.1 m/s.
const int SPEED_CRUISE = 50; 
const int SPEED_SLOW   = 30;  // Landing speed
const float RAMP_SPEED_PULSES_PER_MS = 0.8; // Ramp rate

// Data Logging Rate
const int LOG_INTERVAL = 50; 

// Step-response autotune defaults
const long TUNE_STEP_TARGET = 1000;           // pulses to move during tune
const long TUNE_STEP_TARGET_M2 = 1000;        // pulses for slave tune (smaller default)
const unsigned long TUNE_SETTLE_DWELL = 4000; // ms inside band to call settled
const unsigned long TUNE_TIMEOUT = 12000;     // ms before aborting

//// PINS ////
#if defined(ESP32) || defined(ARDUINO_ARCH_ESP32)
// ESP32 pinout (3.3V logic)
// Motor 1 (Left/Master)
const int M1_PWM   = 25;
const int M1_BRAKE = 33;
const int M1_DIR   = 32;
const int M1_ENC_A = 18; // Interrupt Pin
const int M1_ENC_B = 34; // Encoder B (Input Only, No Pullup)

// Motor 2 (Right/Slave)
const int M2_PWM   = 26;
const int M2_BRAKE = 27;
const int M2_DIR   = 14;
const int M2_ENC_A = 19; // Interrupt Pin
const int M2_ENC_B = 35; // Encoder B (Input Only, No Pullup)

// LEDC PWM setup
const int M1_PWM_CHANNEL = 0;
const int M2_PWM_CHANNEL = 1;
const int PWM_FREQ = 20000;      // 20 kHz for quiet drive
const int PWM_RESOLUTION = 8;    // 0-255 duty to reuse existing PID gains
#else
// Motor 1 (Left/Master)
const int M1_PWM = 5;
const int M1_BRAKE = 4;
const int M1_DIR = 8;
const int M1_ENC_A = 2; // Interrupt Pin

// Motor 2 (Right/Slave)
const int M2_PWM = 6;
const int M2_BRAKE = 7;
const int M2_DIR = 9;
const int M2_ENC_A = 3; // Interrupt Pin

// Dummy channels to keep MotorControl code unified
const int M1_PWM_CHANNEL = 0;
const int M2_PWM_CHANNEL = 1;
const int PWM_FREQ = 20000;
const int PWM_RESOLUTION = 8;
#endif

//// PID TUNING ////
// Master (Position Control)
const float KP_M1 = 0.47;
const float KI_M1 = 0.55;
const float KD_M1 = 0.0;

// Slave (Sync Control)
// const float KP_M2 = 0.3;
// const float KI_M2 = 0.0005;
// const float KD_M2 = 0.02;
const float KP_M2 = 0.4;
const float KI_M2 = 0.1;
const float KD_M2 = 0.00;
const int SYNC_DEADBAND = 5; // Pulses (Stop correcting if error is small)

#endif
