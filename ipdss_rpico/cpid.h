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

#ifndef PID_H
#define PID_H

#pragma once

class CPid {

public:
    // Computes time value for dt_s (in seconds) coefficient
    // Accepts value in milliseconds that you need
    void computeMsToSeconds(float millis);
    
    // Set PID coefficients that user wrote
    void pidSetCoefs(int kp, int ki, int kd);

    // Set range of PID's output
    void pidSetMinMax(int pid_min, int pid_max);

    // Main computing function
    // Accepts current input and value that we need to get
    float computePid(float input, float pidSetPoint);
    
    int pidMin = 0;
    int pidMax = 0;

private:
    // Set value that we need to achieve
    int pidSetPoint = 0;

    // Vars for computePid function's result and enter values
    int pidInput = 0;
    int pidOutput = 0;

    // PID's coefficients
    float Kp = 0;
    float Ki = 0;
    float Kd = 0;
    float dt_s = 0.0;
    float pidIntegral = 0.0;

    // For compute function too
    float pidPrevInput = 0;
    float pidPrevError = 0.0;
};

#endif