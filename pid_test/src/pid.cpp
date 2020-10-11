#include "pid.h"

float CPid::computeHerzToSeconds(float herz) {
    float secs = herz * 60;
    dt_s = secs;
}

int CPid::pidSetCoefs(int kp, int ki, int kd) {
    this->Kp = kp;
    this->Ki = ki;
    this->Kd = kd;
}

float CPid::computePid(float input, float pidSetPoint) {
    float error = pidSetPoint - input;
    float d_error = pidPrevError - error;

    pidIntegral += error * dt_s;

    pidOutput += Kp * error;
    pidOutput += Ki * pidIntegral;
    pidOutput += Kd * d_error * dt_s;

    pidOutput = abs(pidOutput);
  
    if (pidOutput < pidMin) {
        pidOutput = pidMin;
    }
    else if (pidOutput > pidMax) {
        pidOutput = pidMax;
    }

    pidPrevError = error;
    //rps = computePwmToRps();

    return pidOutput;
}