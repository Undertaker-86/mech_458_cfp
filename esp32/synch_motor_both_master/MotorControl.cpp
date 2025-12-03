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

long MotorControl::last_m1_pos = 0;
long MotorControl::last_m2_pos = 0;
float MotorControl::m1_rpm = 0;
float MotorControl::m2_rpm = 0;
unsigned long MotorControl::lastRpmCalcTime = 0;

int MotorControl::global_pwm1 = 0;
int MotorControl::global_pwm2 = 0;
// Motor 1 PID
float MotorControl::kp_m1 = KP_M1;
float MotorControl::ki_m1 = KI_M1;
float MotorControl::kd_m1 = KD_M1;
float MotorControl::integral_error_m1 = 0;
long MotorControl::last_error_m1 = 0;

// Motor 2 PID
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

    attachInterrupt(digitalPinToInterrupt(M1_ENC_A), countM1, RISING);
    attachInterrupt(digitalPinToInterrupt(M2_ENC_A), countM2, RISING);

    // Initial Stop
    digitalWrite(M1_BRAKE, LOW); digitalWrite(M2_BRAKE, LOW);
    writePwm(M1_PWM, M1_PWM_CHANNEL, 0); writePwm(M2_PWM, M2_PWM_CHANNEL, 0);
}

void MotorControl::update() {
    if (!isSynced) return;

    // Motor 1: position hold to target
    long error_m1 = target_pos - m1_pos;
    if (abs(error_m1) < POSITION_DEADBAND) {
        error_m1 = 0;
        integral_error_m1 = 0;
    }
    integral_error_m1 += error_m1;
    integral_error_m1 = constrain(integral_error_m1, -1000, 1000);
    long derivative_m1 = error_m1 - last_error_m1;
    last_error_m1 = error_m1;
    float output_m1 = (error_m1 * kp_m1) + (integral_error_m1 * ki_m1) + (derivative_m1 * kd_m1);
    int pwm_m1 = constrain(output_m1, -SPEED_CRUISE, SPEED_CRUISE);

    // Motor 2: independent position hold to same target
    long error_m2 = target_pos - m2_pos;
    if (abs(error_m2) < POSITION_DEADBAND) {
        error_m2 = 0;
        integral_error_m2 = 0;
    }
    integral_error_m2 += error_m2;
    integral_error_m2 = constrain(integral_error_m2, -1000, 1000);
    long derivative_m2 = error_m2 - last_error_m2;
    last_error_m2 = error_m2;
    float output_m2 = (error_m2 * kp_m2) + (integral_error_m2 * ki_m2) + (derivative_m2 * kd_m2);
    int pwm_m2 = constrain(output_m2, -SPEED_CRUISE, SPEED_CRUISE);

    // ---- APPLY TO HARDWARE ----
    // Motor 1
    if (pwm_m1 == 0) {
        writePwm(M1_PWM, M1_PWM_CHANNEL, 0);
        digitalWrite(M1_BRAKE, LOW);
    } else {
        digitalWrite(M1_BRAKE, HIGH);
        digitalWrite(M1_DIR, (pwm_m1 > 0) ? LOW : HIGH);
        writePwm(M1_PWM, M1_PWM_CHANNEL, pwm_m1);
    }

    // Motor 2
    if (pwm_m2 == 0) {
        writePwm(M2_PWM, M2_PWM_CHANNEL, 0);
        digitalWrite(M2_BRAKE, LOW);
    } else {
        digitalWrite(M2_BRAKE, HIGH);
        digitalWrite(M2_DIR, (pwm_m2 > 0) ? LOW : HIGH);
        writePwm(M2_PWM, M2_PWM_CHANNEL, pwm_m2);
    }

    global_pwm1 = pwm_m1;
    global_pwm2 = pwm_m2;
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
    // Hold close to current posture without commanding a large jump
    target_pos = (m1_pos + m2_pos) / 2;
    integral_error_m1 = 0; last_error_m1 = 0;
    integral_error_m2 = 0; last_error_m2 = 0;
    isSynced = true;
}

// ISRs
#if defined(ESP32)
void IRAM_ATTR MotorControl::countM1() { if (digitalRead(M1_DIR) == LOW) m1_pos++; else m1_pos--; }
void IRAM_ATTR MotorControl::countM2() { if (digitalRead(M2_DIR) == LOW) m2_pos++; else m2_pos--; }
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
void MotorControl::getM1PID(float &kp, float &ki, float &kd) { kp = kp_m1; ki = ki_m1; kd = kd_m1; }
void MotorControl::getM2PID(float &kp, float &ki, float &kd) { kp = kp_m2; ki = ki_m2; kd = kd_m2; }
void MotorControl::setM1PID(float kp, float ki, float kd) { kp_m1 = kp; ki_m1 = ki; kd_m1 = kd; integral_error_m1 = 0; last_error_m1 = 0; }
void MotorControl::setM2PID(float kp, float ki, float kd) { kp_m2 = kp; ki_m2 = ki; kd_m2 = kd; integral_error_m2 = 0; last_error_m2 = 0; }

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

