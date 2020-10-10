#pragma once

#include <Arduino.h>

class CPid {

public:

private:

    int pidSetPoint = 0;

    int pidMin = 0;
    int pidMax = 255;

    int pidInput = 0;
    int pidOutput = 0;

    // regulator coefs
    float Kp = 1;
    float Ki = 0;
    float Kd = 0;
    float dt_s = 0.2;
    float pidIntegral = 0.0;

    float rps = 0.0;

    int pidPrevInput = 0;

    float computePid(float input, float pidSetPoint);
    float computePwmToRps();

};