#include "pid.h"

float CPid::computePwmToRps() {

}

float CPid::computePid(float input, float pidSetPoint) {
    float error = pidSetPoint - input;
    float d_input = pidPrevInput - input;

    pidPrevInput = input;
    pidIntegral += error * dt_s;

    pidOutput += Kp * error;
    pidOutput += Ki * pidIntegral;
    pidOutput += Kd * d_input * dt_s;

    pidOutput = abs(pidOutput);
  
    if (pidOutput < pidMin) {
        pidOutput = pidMin;
    }
    else if (pidOutput > pidMax) {
        pidOutput = pidMax;
    }

    //rps = computePwmToRps();

    return pidOutput;
}