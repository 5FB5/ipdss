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

/*
    KEEP IN MIND!
    This CMotor library developed for motors with encoder
    For example, my IPDSS robot uses GB37-3530 motor model with disc encoder
*/

#ifndef MOTOR_H
#define MOTOR_H

#pragma once

// Value for interpreting the PWM in revolutions per second
// Can be different on different motor, check this
#define PWM_TO_RPS (float)1.36 // (max rps on 255 PWM / 255)

#include <stdlib.h>
#include <stdint.h>

#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/uart.h"
#include "pisystick.h"
#include "pwm.h"
#include "cpid.h"

class CMotor {

public:
    // Constructor, accepts number of interrupt for encoder, PWM pin, 
    // digital pins that sets direction for current motor
    CMotor(uint8_t pin_interrupt, uint8_t pin_pwm, uint8_t pin_dir1, uint8_t pin_dir2);
    
    // Motor's speed in revolution per second
    float rvPerS = 0;

    // Sets default settings from motor's specifications
    // Accepts motor's reduction coefficient, period of motor processing in millis,
    // Coefficients from PID regulator and range of PID's work
    void setup(float motor_reduc_coef, float speed_calc_millis, float pidKp, float pidKi, float pidKd, float pid_range_min, float pid_range_max);
    
    // Processing of motor, accepts direction and speed like revolution per seconds
    void process(uint8_t direction, float rps);  

private:
    
    // Create new instance of PID's class
    CPid* pidMotor = new CPid;

    // Main function for increasing ticks from motor's encoder
    void increaseTickCount();
    // Callback function that returns increaseTickCount() pointer for translating it into gpio_callback_t type
    static void increaseTickCount_callback(void* context);

    // Checks time from last processing
    volatile unsigned long int timeStamp = 0;
    volatile int tickCount = 0;
    
    float m_motor_reduc_coef = 0;
    float m_update_period = 0;
    
    uint8_t m_pin_interrupt;
    uint8_t m_pin_pwm;
    uint8_t m_pin_dir1;
    uint8_t m_pin_dir2;
};

#endif