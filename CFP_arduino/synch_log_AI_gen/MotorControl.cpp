#include "MotorControl.h"

// Define static members
volatile long MotorControl::m1_pos = 0;
volatile long MotorControl::m2_pos = 0;
long MotorControl::target_pos = 0;

long MotorControl::last_m1_pos = 0;
long MotorControl::last_m2_pos = 0;
float MotorControl::m1_rpm = 0;
float MotorControl::m2_rpm = 0;
unsigned long MotorControl::lastRpmCalcTime = 0;

int MotorControl::global_pwm1 = 0;
int MotorControl::global_pwm2 = 0;
// Master PID
float MotorControl::kp_m1 = KP_M1;
float MotorControl::ki_m1 = KI_M1;
float MotorControl::kd_m1 = KD_M1;
float MotorControl::integral_error_m1 = 0;
long MotorControl::last_error_m1 = 0;

// Slave PID
float MotorControl::kp_m2 = KP_M2;
float MotorControl::ki_m2 = KI_M2;
float MotorControl::kd_m2 = KD_M2;
float MotorControl::integral_error_m2 = 0;
long MotorControl::last_error_m2 = 0;

bool MotorControl::isSynced = false;

void MotorControl::init() {
    // Pin Setup
    pinMode(M1_PWM, OUTPUT); pinMode(M1_BRAKE, OUTPUT); pinMode(M1_DIR, OUTPUT);
    pinMode(M2_PWM, OUTPUT); pinMode(M2_BRAKE, OUTPUT); pinMode(M2_DIR, OUTPUT);
    pinMode(M1_ENC_A, INPUT_PULLUP); pinMode(M2_ENC_A, INPUT_PULLUP);

    attachInterrupt(digitalPinToInterrupt(M1_ENC_A), countM1, RISING);
    attachInterrupt(digitalPinToInterrupt(M2_ENC_A), countM2, RISING);

    // Initial Stop
    digitalWrite(M1_BRAKE, LOW); digitalWrite(M2_BRAKE, LOW);
    analogWrite(M1_PWM, 0); analogWrite(M2_PWM, 0);
}

void MotorControl::update() {
    // RPM Calculation Logic (moved from logging to here or keep separate? 
    // The original code calculated RPM inside the logging block. 
    // It's better to calculate it here if we want it updated regularly, 
    // but to match original logic, I'll add a method to update RPM or just let the logger call it?
    // Actually, the original code calculated RPM *only* when logging. 
    // Let's keep that behavior for now to avoid changing timing, 
    // but I'll expose a method to update RPM state that the Logger can call, 
    // or just do it in update() if we want better control.
    // For now, I'll leave RPM calc to be triggered by the Logger or a separate timer.
    // Wait, the original code did it inside the `if (millis() - lastLogTime > LOG_INTERVAL)` block.
    // I will implement a `calculateRPM` method that can be called.
    
    if (isSynced) {
        // ---- MASTER MOTOR (Position Control) ----
        long error_m1 = target_pos - m1_pos;
        
        // Deadband
        if (abs(error_m1) < 10) {
            error_m1 = 0;
            integral_error_m1 = 0;
        }

        integral_error_m1 += error_m1;
        integral_error_m1 = constrain(integral_error_m1, -1000, 1000); // Anti-windup

        long derivative_m1 = error_m1 - last_error_m1;
        last_error_m1 = error_m1;

        float output_m1 = (error_m1 * kp_m1) + (integral_error_m1 * ki_m1) + (derivative_m1 * kd_m1);
        
        // Speed Limiting (Cruise Speed)
        // We can clamp the output to SPEED_CRUISE
        int pwm_m1 = constrain(output_m1, -SPEED_CRUISE, SPEED_CRUISE);
        
        // ---- SLAVE MOTOR (Sync Control) ----
        // Target for M2 is M1's position (or we can use target_pos, but syncing to M1 is safer)
        // Error = Master - Slave
        long error_sync = m1_pos - m2_pos;
        
        // Deadband for Sync
        if (abs(error_sync) < SYNC_DEADBAND) {
             error_sync = 0;
             integral_error_m2 = 0; 
        }
        
        integral_error_m2 += error_sync;
        integral_error_m2 = constrain(integral_error_m2, -500, 500);

        long derivative_m2 = error_sync - last_error_m2;
        last_error_m2 = error_sync;

        float adjustment = (error_sync * kp_m2) + (integral_error_m2 * ki_m2) + (derivative_m2 * kd_m2);
        
        // Slave PWM is Master PWM + Adjustment
        // But we need to handle direction. 
        // If pwm_m1 is positive, we are moving forward.
        // If pwm_m1 is negative, we are moving backward.
        // The adjustment should add/subtract speed.
        
        int pwm_m2 = pwm_m1 + adjustment;
        
        // ---- APPLY TO HARDWARE ----
        
        // Motor 1
        if (pwm_m1 == 0) {
            analogWrite(M1_PWM, 0);
            digitalWrite(M1_BRAKE, LOW); // Or HIGH to hold? Let's use LOW for now as per original
        } else {
            digitalWrite(M1_BRAKE, HIGH);
            if (pwm_m1 > 0) digitalWrite(M1_DIR, LOW);
            else digitalWrite(M1_DIR, HIGH);
            analogWrite(M1_PWM, abs(pwm_m1));
        }

        // Motor 2
        if (pwm_m2 == 0 && pwm_m1 == 0) {
             analogWrite(M2_PWM, 0);
             digitalWrite(M2_BRAKE, LOW);
        } else {
             digitalWrite(M2_BRAKE, HIGH);
             // Direction depends on sign of pwm_m2
             if (pwm_m2 > 0) digitalWrite(M2_DIR, LOW); // Assuming same wiring
             else digitalWrite(M2_DIR, HIGH);
             
             // Safety clamp
             int safe_pwm2 = constrain(abs(pwm_m2), 0, 255);
             analogWrite(M2_PWM, safe_pwm2);
        }
        
        global_pwm1 = pwm_m1;
        global_pwm2 = pwm_m2;
    }
}

void MotorControl::manualJog(int motor, bool up) {
    isSynced = false; 
    digitalWrite(M1_BRAKE, LOW); digitalWrite(M2_BRAKE, LOW);
    int pwm = SPEED_CRUISE; int duration = 100; 
    if (motor == 1) {
        digitalWrite(M1_BRAKE, HIGH); digitalWrite(M1_DIR, up ? LOW : HIGH);
        analogWrite(M1_PWM, pwm); delay(duration); analogWrite(M1_PWM, 0); digitalWrite(M1_BRAKE, LOW);
    } else {
        digitalWrite(M2_BRAKE, HIGH); digitalWrite(M2_DIR, up ? LOW : HIGH);
        analogWrite(M2_PWM, pwm); delay(duration); analogWrite(M2_PWM, 0); digitalWrite(M2_BRAKE, LOW);
    }
}

void MotorControl::setTarget(long target) {
    target_pos = target;
    isSynced = true; 
    digitalWrite(M1_BRAKE, HIGH); digitalWrite(M2_BRAKE, HIGH);
}

void MotorControl::setZero() {
    noInterrupts(); m1_pos = 0; m2_pos = 0; interrupts();
    noInterrupts(); m1_pos = 0; m2_pos = 0; interrupts();
    target_pos = 0; 
    integral_error_m1 = 0; last_error_m1 = 0;
    integral_error_m2 = 0; last_error_m2 = 0;
    isSynced = true;
}

void MotorControl::emergencyHalt() {
    target_pos = m1_pos;
    isSynced = true;
}

// ISRs
void MotorControl::countM1() { if (digitalRead(M1_DIR) == LOW) m1_pos++; else m1_pos--; }
void MotorControl::countM2() { if (digitalRead(M2_DIR) == LOW) m2_pos++; else m2_pos--; }

// Getters
long MotorControl::getM1Pos() { return m1_pos; }
long MotorControl::getM2Pos() { return m2_pos; }
long MotorControl::getTargetPos() { return target_pos; }
int MotorControl::getPWM1() { return global_pwm1; }
int MotorControl::getPWM2() { return global_pwm2; }

// RPM needs to be calculated periodically
void MotorControl::updateRPM(unsigned long intervalMs) {
    float rpm_factor = (1000.0 / intervalMs) * 60.0 / ENCODER_PPR;
    m1_rpm = (m1_pos - last_m1_pos) * rpm_factor;
    m2_rpm = (m2_pos - last_m2_pos) * rpm_factor;
    last_m1_pos = m1_pos; 
    last_m2_pos = m2_pos;
}

float MotorControl::getM1RPM() { return m1_rpm; }
float MotorControl::getM2RPM() { return m2_rpm; }

