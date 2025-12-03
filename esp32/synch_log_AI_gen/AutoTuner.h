#ifndef AUTO_TUNER_H
#define AUTO_TUNER_H

#include <Arduino.h>
#include "MotorControl.h"
#include "config.h"

class AutoTuner {
public:
    // Start a step-response tune for master motor.
    // target: encoder pulses to move (positive or negative).
    // apply: if true, write suggested gains into MotorControl.
    enum Loop { MASTER, SLAVE };

    static void start(long target, bool apply, Loop which = MASTER);
    static void startBoth(long targetMaster, long targetSlave, bool applyMaster, bool applySlave);
    static void abort();
    static void tick();
    static bool isRunning();

private:
    enum Stage { IDLE, PRE_SETTLE, APPLY_STEP, MEASURE, DONE, ABORTED };

    static Stage stage;
    static bool applyResult;
    static long stepTarget;
    static unsigned long stageStart;
    static unsigned long startTime;
    static unsigned long settledAt;
    static unsigned long dwellStart;
    static bool crossed10;
    static bool crossed90;
    static unsigned long t10;
    static unsigned long t90;
    static long extremePos;
    static long steadyErr;
    static float baseKp, baseKi, baseKd;
    static Loop loop;
    static bool chainToSlave;
    static long chainTargetSlave;
    static bool chainApplySlave;
    struct Sample {
        unsigned long t;
        long target;
        long m1;
        long m2;
        float rpm1;
        float rpm2;
        int pwm1;
        int pwm2;
    };
    static const int LOG_CAP = 300;
    static Sample logBuf[LOG_CAP];
    static int logCount;

    static long settleBand();
    static void resetMetrics();
    static void recordSample();
    static void dumpLog();
    static void evaluate();
    static void doneMessage(const char* msg);
};

#endif
