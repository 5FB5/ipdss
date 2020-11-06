/*
CMotor library code is placed under the MIT License
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

#ifndef MOTOR_H
#define MOTOR_H

#pragma once

#define PWM_TO_RPS (float)1.36 // (max rps on 255 PWM / 255)

#include <Arduino.h>
#include "CPid.h"

class CMotor {

public:
    
    CMotor(uint8_t interrupt_numb, uint8_t pin_pwm, uint8_t pin_dir1, uint8_t pin_dir2);
    CPid* pidMotor = new CPid;

    float rvPerS = 0;
    
    volatile int tickCount = 0;

    void setup(float motor_reduc_coef, float speed_calc_millis, float pidKp, float pidKi, float pidKd, float pid_range_min, float pid_range_max);
    void process(uint8_t direction, float rps);  

private:
    
    volatile unsigned long int timeStamp = 0;

    float m_motor_reduc_coef = 0;
    float m_update_period = 0;
    
    uint8_t m_interrupt_numb;
    uint8_t m_pin_pwm;
    uint8_t m_pin_dir1;
    uint8_t m_pin_dir2;
};

#endif