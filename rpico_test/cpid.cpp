/*
CPid library code is placed under the MIT License
Copyright (c) 2020 Valeriy Zubko

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

#include "cpid.h"

void CPid::computeMsToSeconds(float millis) {
    dt_s = millis / 1000;
}

void CPid::pidSetCoefs(int kp, int ki, int kd) {
    Kp = kp;
    Ki = ki;
    Kd = kd;
}

void CPid::pidSetMinMax(int pid_min, int pid_max) {
    pidMin = pid_min;
    pidMax = pid_max;
}

float CPid::computePid(float input, float pidSetPoint) {
    // Get error coefficient
    float error = pidSetPoint - input;
    // Get the difference between current and previous error coefficients
    float d_error = pidPrevError - error;

    // Calculate integral part
    pidIntegral += error * dt_s;

    // Calculate other parts of PID
    pidOutput += Kp * error;
    pidOutput += Ki * pidIntegral;
    pidOutput += Kd * d_error * dt_s;
  
    // And set restrictions for output based and working range that user wrote
    if (pidOutput < pidMin) {
        pidOutput = pidMin;
    }
    else if (pidOutput > pidMax) {
        pidOutput = pidMax;
    }

    // Set current error value as previous
    pidPrevError = error;

    return pidOutput;
}