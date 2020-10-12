#include "CPid.h"

void CPid::computeHerzToSeconds(float herz) {
    this->dt_s =  herz * 60;
}

void CPid::pidSetCoefs(int kp, int ki, int kd) {
    this->Kp = kp;
    this->Ki = ki;
    this->Kd = kd;
}

void CPid::pidSetMinMax(int pid_min, int pid_max) {
    this->pidMin = pid_min;
    this->pidMax = pid_max;
}

float CPid::computePid(float input, float pidSetPoint) {
    float error = pidSetPoint - input;
    float d_error = pidPrevError - error;

    pidIntegral += error * dt_s;

    pidOutput += Kp * error;
    pidOutput += Ki * pidIntegral;
    pidOutput += Kd * d_error * dt_s;
  
    if (pidOutput < pidMin) {
        pidOutput = pidMin;
    }
    else if (pidOutput > pidMax) {
        pidOutput = pidMax;
    }

    pidPrevError = error;

    return pidOutput;
}