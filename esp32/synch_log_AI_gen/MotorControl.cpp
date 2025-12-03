#include "MotorControl.h"

// Use analogWrite for both AVR and ESP32 (Arduino core provides compatibility on ESP32)
static const int PWM_MAX = 255;
static void setupPwmPin(int pin, int /*channel*/) {
    pinMode(pin, OUTPUT);
    analogWrite(pin, 0);
}
static void writePwm(int pin, int /*channel*/, int value) {
    analogWrite(pin, constrain(abs(value), 0, PWM_MAX));
}

// Define static members
volatile long MotorControl::m1_pos = 0;
volatile long MotorControl::m2_pos = 0;

long MotorControl::target_pos = 0;
long MotorControl::final_target_pos = 0;
float MotorControl::current_setpoint = 0;
unsigned long MotorControl::last_ramp_time = 0;


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
    pinMode(M1_BRAKE, OUTPUT); pinMode(M1_DIR, OUTPUT);
    pinMode(M2_BRAKE, OUTPUT); pinMode(M2_DIR, OUTPUT);
    setupPwmPin(M1_PWM, M1_PWM_CHANNEL);
    setupPwmPin(M2_PWM, M2_PWM_CHANNEL);
    pinMode(M1_ENC_A, INPUT_PULLUP); pinMode(M2_ENC_A, INPUT_PULLUP);
    pinMode(M1_ENC_B, INPUT); pinMode(M2_ENC_B, INPUT); // Input only pins, external pullup required if open collector

    attachInterrupt(digitalPinToInterrupt(M1_ENC_A), countM1, RISING);
    attachInterrupt(digitalPinToInterrupt(M2_ENC_A), countM2, RISING);

    // Initial Stop
    digitalWrite(M1_BRAKE, LOW); digitalWrite(M2_BRAKE, LOW);
    writePwm(M1_PWM, M1_PWM_CHANNEL, 0); writePwm(M2_PWM, M2_PWM_CHANNEL, 0);
}

void MotorControl::update() {
    // RPM Calculation Logic
    // I will implement a `calculateRPM` method that can be called.

    
    if (isSynced) {
        // ---- RAMP GENERATION ----
        unsigned long now = millis();
        if (last_ramp_time == 0) last_ramp_time = now;
        unsigned long dt = now - last_ramp_time;
        last_ramp_time = now;

        if (dt > 0) {
            float dist = final_target_pos - current_setpoint;
            float max_step = RAMP_SPEED_PULSES_PER_MS * dt;
            
            if (abs(dist) <= max_step) {
                current_setpoint = final_target_pos;
            } else {
                if (dist > 0) current_setpoint += max_step;
                else current_setpoint -= max_step;
            }
        }
        target_pos = (long)current_setpoint;

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
            writePwm(M1_PWM, M1_PWM_CHANNEL, 0);
            digitalWrite(M1_BRAKE, LOW); // Or HIGH to hold? Let's use LOW for now as per original
        } else {
            digitalWrite(M1_BRAKE, HIGH);
            if (pwm_m1 > 0) digitalWrite(M1_DIR, LOW);
            else digitalWrite(M1_DIR, HIGH);
            writePwm(M1_PWM, M1_PWM_CHANNEL, pwm_m1);
        }

        // Motor 2
        if (pwm_m2 == 0 && pwm_m1 == 0) {
             writePwm(M2_PWM, M2_PWM_CHANNEL, 0);
             digitalWrite(M2_BRAKE, LOW);
        } else {
             digitalWrite(M2_BRAKE, HIGH);
             // Direction depends on sign of pwm_m2
             if (pwm_m2 > 0) digitalWrite(M2_DIR, LOW); // Assuming same wiring
             else digitalWrite(M2_DIR, HIGH);
             
             // Safety clamp
             int safe_pwm2 = constrain(abs(pwm_m2), 0, PWM_MAX);
             writePwm(M2_PWM, M2_PWM_CHANNEL, safe_pwm2);
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
        writePwm(M1_PWM, M1_PWM_CHANNEL, pwm); delay(duration); writePwm(M1_PWM, M1_PWM_CHANNEL, 0); digitalWrite(M1_BRAKE, LOW);
    } else {
        digitalWrite(M2_BRAKE, HIGH); digitalWrite(M2_DIR, up ? LOW : HIGH);
        writePwm(M2_PWM, M2_PWM_CHANNEL, pwm); delay(duration); writePwm(M2_PWM, M2_PWM_CHANNEL, 0); digitalWrite(M2_BRAKE, LOW);
    }
}

void MotorControl::setTarget(long target, bool immediate) {
    final_target_pos = target;
    if (immediate) {
        current_setpoint = target;
        target_pos = target;
    } else {
        // If we were not synced, start ramp from current position
        if (!isSynced) {
            current_setpoint = m1_pos;
        }
        // Otherwise continue ramping from current_setpoint
    }
    last_ramp_time = millis();

    isSynced = true; 
    digitalWrite(M1_BRAKE, HIGH); digitalWrite(M2_BRAKE, HIGH);
}

void MotorControl::setZero() {
    noInterrupts(); m1_pos = 0; m2_pos = 0; interrupts();
    target_pos = 0; 
    final_target_pos = 0;
    current_setpoint = 0;

    integral_error_m1 = 0; last_error_m1 = 0;
    integral_error_m2 = 0; last_error_m2 = 0;
    isSynced = true;
}

void MotorControl::emergencyHalt() {
    target_pos = m1_pos;
    final_target_pos = m1_pos;
    current_setpoint = m1_pos;
    isSynced = true;
}

// ISRs
#if defined(ESP32)
void IRAM_ATTR MotorControl::countM1() { 
    // Read Encoder B to determine direction
    // Logic inverted as per user request
    if (digitalRead(M1_ENC_B) == LOW) m1_pos--; else m1_pos++; 
}
void IRAM_ATTR MotorControl::countM2() { 
    if (digitalRead(M2_ENC_B) == LOW) m2_pos--; else m2_pos++; 
}
#else
void MotorControl::countM1() { if (digitalRead(M1_DIR) == LOW) m1_pos++; else m1_pos--; }
void MotorControl::countM2() { if (digitalRead(M2_DIR) == LOW) m2_pos++; else m2_pos--; }
#endif

// Getters
long MotorControl::getM1Pos() { return m1_pos; }
long MotorControl::getM2Pos() { return m2_pos; }
long MotorControl::getTargetPos() { return target_pos; }
int MotorControl::getPWM1() { return global_pwm1; }
int MotorControl::getPWM2() { return global_pwm2; }
void MotorControl::getMasterPID(float &kp, float &ki, float &kd) { kp = kp_m1; ki = ki_m1; kd = kd_m1; }
void MotorControl::getSlavePID(float &kp, float &ki, float &kd) { kp = kp_m2; ki = ki_m2; kd = kd_m2; }
void MotorControl::setMasterPID(float kp, float ki, float kd) { kp_m1 = kp; ki_m1 = ki; kd_m1 = kd; integral_error_m1 = 0; last_error_m1 = 0; }
void MotorControl::setSlavePID(float kp, float ki, float kd) { kp_m2 = kp; ki_m2 = ki; kd_m2 = kd; integral_error_m2 = 0; last_error_m2 = 0; }

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

