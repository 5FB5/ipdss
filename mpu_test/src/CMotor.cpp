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

#include "../include/CMotor.h"

CMotor::CMotor(uint8_t interrupt_numb, uint8_t pin_pwm, uint8_t pin_dir1,uint8_t pin_dir2) {
    // Set all values to private variables
    m_interrupt_numb = interrupt_numb;
    m_pin_pwm = pin_pwm;
    m_pin_dir1 = pin_dir1;
    m_pin_dir2 = pin_dir2;
}

void CMotor::setup(float motor_reduc_coef, float speed_calc_millis, float pidKp, float pidKi, float pidKd, float pid_range_min, float pid_range_max) {
    // Set mode for pins
    pinMode(m_pin_pwm, OUTPUT);
    pinMode(m_pin_dir1, OUTPUT);
    pinMode(m_pin_dir2, OUTPUT);
    
    // Set basic settings for PID and for other vars
    pidMotor->computeMsToSeconds(speed_calc_millis);
    pidMotor->pidSetMinMax(pid_range_min, pid_range_max);
    pidMotor->pidSetCoefs(pidKp, pidKi, pidKd);

    m_motor_reduc_coef = motor_reduc_coef;
    m_update_period = speed_calc_millis;

    // Turn off motor direction
    digitalWrite(m_pin_dir1, LOW);
    digitalWrite(m_pin_dir2, LOW);
}

void CMotor::process(uint8_t direction, float rps) {    
    // Update data from encoders and set speed to motors
    if (millis() - timeStamp >= m_update_period) {
        rvPerS = tickCount / (m_motor_reduc_coef * m_update_period);
        tickCount = 0;

        unsigned char pwm = (unsigned char)pidMotor->computePid(rvPerS, rps) / PWM_TO_RPS;
    
        pwm = abs(pwm);

        if (direction == 0) {
            digitalWrite(m_pin_dir1, HIGH);
            digitalWrite(m_pin_dir2, LOW);
        }
        else if (direction == 1) {
            digitalWrite(m_pin_dir1, LOW);
            digitalWrite(m_pin_dir2, HIGH);
        }
    
        analogWrite(m_pin_pwm, pwm);
        
        timeStamp = millis();
    }
}