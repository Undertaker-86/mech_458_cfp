#include "AutoTuner.h"
#include <math.h>

AutoTuner::Stage AutoTuner::stage = AutoTuner::IDLE;
AutoTuner::Loop AutoTuner::loop = AutoTuner::M1;
bool AutoTuner::applyResult = false;
long AutoTuner::stepTarget = 0;
unsigned long AutoTuner::stageStart = 0;
unsigned long AutoTuner::startTime = 0;
unsigned long AutoTuner::settledAt = 0;
unsigned long AutoTuner::dwellStart = 0;
bool AutoTuner::crossed10 = false;
bool AutoTuner::crossed90 = false;
unsigned long AutoTuner::t10 = 0;
unsigned long AutoTuner::t90 = 0;
long AutoTuner::extremePos = 0;
long AutoTuner::steadyErr = 0;
float AutoTuner::baseKp = 0;
float AutoTuner::baseKi = 0;
float AutoTuner::baseKd = 0;
bool AutoTuner::chainToM2 = false;
long AutoTuner::chainTargetM2 = 0;
bool AutoTuner::chainApplyM2 = false;
AutoTuner::Sample AutoTuner::logBuf[AutoTuner::LOG_CAP];
int AutoTuner::logCount = 0;

void AutoTuner::start(long target, bool apply, Loop which) {
    if (stage != IDLE) {
        Serial.println("TUNE: already running");
        return;
    }
    if (which == M2) chainToM2 = false;
    loop = which;
    stepTarget = target == 0 ? TUNE_STEP_TARGET : target;
    if (loop == M2 && target == 0) stepTarget = TUNE_STEP_TARGET_M2;
    applyResult = apply;
    resetMetrics();
    MotorControl::setZero();
    if (loop == M1) MotorControl::getM1PID(baseKp, baseKi, baseKd);
    else MotorControl::getM2PID(baseKp, baseKi, baseKd);
    stage = PRE_SETTLE;
    stageStart = millis();
    logCount = 0;
    Serial.print("TUNE: step start ");
    Serial.print(loop == M1 ? "M1" : "M2");
    Serial.print(", target=");
    Serial.print(stepTarget);
    Serial.print(", apply=");
    Serial.println(applyResult ? "yes" : "no");
}

void AutoTuner::startBoth(long targetM1, long targetM2, bool applyM1, bool applyM2) {
    if (stage != IDLE) {
        Serial.println("TUNE: already running");
        return;
    }
    loop = M1;
    chainToM2 = true;
    chainTargetM2 = targetM2 == 0 ? TUNE_STEP_TARGET_M2 : targetM2;
    chainApplyM2 = applyM2;
    start(targetM1 == 0 ? TUNE_STEP_TARGET : targetM1, applyM1, M1);
}

void AutoTuner::abort() {
    if (stage == IDLE) return;
    stage = ABORTED;
    chainToM2 = false;
    MotorControl::emergencyHalt();
    doneMessage("TUNE: aborted");
    stage = IDLE;
}

bool AutoTuner::isRunning() {
    return stage != IDLE && stage != DONE && stage != ABORTED;
}

long AutoTuner::settleBand() {
    long band = (long)(abs(stepTarget) * 0.02f);
    if (band < 5) band = 5;
    return band;
}

void AutoTuner::resetMetrics() {
    crossed10 = false;
    crossed90 = false;
    t10 = 0;
    t90 = 0;
    dwellStart = 0;
    settledAt = 0;
    extremePos = (loop == M1) ? MotorControl::getM1Pos() : MotorControl::getM2Pos();
    steadyErr = 0;
}

void AutoTuner::tick() {
    if (stage == IDLE || stage == DONE || stage == ABORTED) return;
    unsigned long now = millis();

    switch (stage) {
    case PRE_SETTLE: {
        // Wait until holding at zero before applying step.
        long err = (loop == M1) ? MotorControl::getM1Pos() : MotorControl::getM2Pos();
        float rpm = (loop == M1) ? MotorControl::getM1RPM() : MotorControl::getM2RPM();
        if (abs(err) < settleBand() && abs(rpm) < 2.0f) {
            if (dwellStart == 0) dwellStart = now;
            if (now - dwellStart > 300) {
                stage = APPLY_STEP;
                stageStart = now;
            }
        } else {
            dwellStart = 0;
            if (now - stageStart > 1000) {
                // give up waiting; proceed anyway
                stage = APPLY_STEP;
                stageStart = now;
            }
        }
        break;
    }
    case APPLY_STEP:
        MotorControl::setTarget(stepTarget);
        startTime = now;
        stage = MEASURE;
        resetMetrics();
        break;

    case MEASURE: {
        long ref = MotorControl::getTargetPos();
        long pos = (loop == M1) ? MotorControl::getM1Pos() : MotorControl::getM2Pos();
        long err = ref - pos;
        if (stepTarget >= 0) {
            if (pos > extremePos) extremePos = pos;
        } else {
            if (pos < extremePos) extremePos = pos;
        }

        if (!crossed10 && ((stepTarget > 0 && pos >= stepTarget * 0.1f) ||
                           (stepTarget < 0 && pos <= stepTarget * 0.1f))) {
            crossed10 = true;
            t10 = now;
        }
        if (!crossed90 && ((stepTarget > 0 && pos >= stepTarget * 0.9f) ||
                           (stepTarget < 0 && pos <= stepTarget * 0.9f))) {
            crossed90 = true;
            t90 = now;
        }

        bool inBand = abs(err) <= settleBand();
        float rpm = (loop == M1) ? MotorControl::getM1RPM() : MotorControl::getM2RPM();
        if (inBand && abs(rpm) < 2.0f) {
            if (dwellStart == 0) dwellStart = now;
            if (now - dwellStart >= TUNE_SETTLE_DWELL) {
                settledAt = now;
                steadyErr = err;
                evaluate();
                if (stage == MEASURE) stage = DONE;
            }
        } else {
            dwellStart = 0;
        }

        if (now - startTime > TUNE_TIMEOUT) {
            stage = ABORTED;
            MotorControl::emergencyHalt();
            doneMessage("TUNE: timeout");
            stage = IDLE;
        }
        break;
    }

    case DONE:
    case ABORTED:
    default:
        break;
    }

    // sample logging during tune
    recordSample();
}

void AutoTuner::evaluate() {
    unsigned long rise = crossed90 ? (t90 - startTime) : 0;
    unsigned long settle = settledAt ? (settledAt - startTime) : 0;
    float overshoot = 0.0f;
    if ((stepTarget > 0 && extremePos > stepTarget) || (stepTarget < 0 && extremePos < stepTarget)) {
        overshoot = (float)(extremePos - stepTarget) / (float)stepTarget;
    }

    float kp = baseKp;
    float ki = baseKi;
    float kd = baseKd;

    // Heuristic adjustments biased against overshoot.
    if (overshoot > 0.15f) {
        kp *= 0.2f;
        ki *= 0.1f;
        kd += 0.06f * fabsf(baseKp);
    } else if (overshoot > 0.05f) {
        kp *= 0.3f;
        ki *= 0.15f;
        kd += 0.05f * fabsf(baseKp);
    } else if (overshoot < 0.02f && rise > 600) {
        kp *= 1.1f;   // gentle nudge only
        ki *= 1.02f;  // keep integral small
    }

    if (abs(steadyErr) > settleBand()) {
        ki *= 1.2f;
    }

    // Clamp to reasonable ranges.
    kp = constrain(kp, 0.05f, 5.0f);
    ki = constrain(ki, 0.0f, 1.0f);
    kd = constrain(kd, 0.0f, 1.5f);

    Serial.print("TUNE_METRICS: step ");
    Serial.println(loop == M1 ? "M1" : "M2");
    Serial.print(" rise_ms="); Serial.print(rise);
    Serial.print(" settle_ms="); Serial.print(settle);
    Serial.print(" steady_err="); Serial.print(steadyErr);
    Serial.print(" band="); Serial.println(settleBand());

    Serial.print("TUNE_SUGGEST: ");
    Serial.print(loop == M1 ? "M1" : "M2");
    Serial.print(" kp=");
    Serial.print(kp, 4);
    Serial.print(" ki=");
    Serial.print(ki, 4);
    Serial.print(" kd=");
    Serial.print(kd, 4);
    Serial.print(" (base ");
    Serial.print(baseKp, 4); Serial.print(", ");
    Serial.print(baseKi, 4); Serial.print(", ");
    Serial.print(baseKd, 4); Serial.println(")");

    if (applyResult) {
        if (loop == M1) MotorControl::setM1PID(kp, ki, kd);
        else MotorControl::setM2PID(kp, ki, kd);
        Serial.println("TUNE: applied suggested gains");
    } else {
        Serial.println("TUNE: suggested gains not applied (dry run)");
    }
    MotorControl::emergencyHalt();

    dumpLog();

    if (chainToM2 && loop == M1) {
        // launch second motor tune immediately
        chainToM2 = false;
        stage = IDLE; // allow next start
        start(chainTargetM2, chainApplyM2, M2);
        return;
    }

    stage = IDLE;
}

void AutoTuner::doneMessage(const char* msg) {
    Serial.println(msg);
}

void AutoTuner::recordSample() {
    if (logCount >= LOG_CAP) return;
    Sample &s = logBuf[logCount++];
    s.t = millis();
    s.target = MotorControl::getTargetPos();
    s.m1 = MotorControl::getM1Pos();
    s.m2 = MotorControl::getM2Pos();
    s.rpm1 = MotorControl::getM1RPM();
    s.rpm2 = MotorControl::getM2RPM();
    s.pwm1 = MotorControl::getPWM1();
    s.pwm2 = MotorControl::getPWM2();
}

void AutoTuner::dumpLog() {
    if (logCount == 0) return;
    Serial.println("TUNE_LOG_BEGIN");
    Serial.println("t_ms,target,m1,m2,rpm1,rpm2,pwm1,pwm2");
    for (int i = 0; i < logCount; i++) {
        Sample &s = logBuf[i];
        Serial.print(s.t); Serial.print(",");
        Serial.print(s.target); Serial.print(",");
        Serial.print(s.m1); Serial.print(",");
        Serial.print(s.m2); Serial.print(",");
        Serial.print(s.rpm1, 2); Serial.print(",");
        Serial.print(s.rpm2, 2); Serial.print(",");
        Serial.print(s.pwm1); Serial.print(",");
        Serial.println(s.pwm2);
    }
    Serial.println("TUNE_LOG_END");
    logCount = 0;
}
