#pragma once

#include <Arduino.h>

class CPid {

public:

    void computeMsToSeconds(float millis);
    
    void pidSetCoefs(int kp, int ki, int kd);
    void pidSetMinMax(int pid_min, int pid_max);

    float computePid(float input, float pidSetPoint);
    
    int pidMin = 0;
    int pidMax = 0;

private:

    int pidSetPoint = 0;

    int pidInput = 0;
    int pidOutput = 0;

    // regulator coefs
    float Kp = 0;
    float Ki = 0;
    float Kd = 0;
    float dt_s = 0.0;
    float pidIntegral = 0.0;

    float pidPrevInput = 0;
    float pidPrevError = 0.0;
};