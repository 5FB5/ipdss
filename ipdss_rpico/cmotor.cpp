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

#include "cmotor.h"

CMotor::CMotor(uint8_t pin_interrupt, uint8_t pin_pwm, uint8_t pin_dir1,uint8_t pin_dir2) {
    // Set all values to private variables
    m_pin_interrupt = pin_interrupt;
    m_pin_pwm = pin_pwm;
    m_pin_dir1 = pin_dir1;
    m_pin_dir2 = pin_dir2;

    gpio_init(m_pin_pwm);
    gpio_init(m_pin_dir1);
    gpio_init(m_pin_dir2);

    gpio_pull_up(m_pin_dir1);
    gpio_pull_up(m_pin_dir2);
}

void CMotor::setup(float motor_reduc_coef, float speed_calc_millis, float pidKp, float pidKi, float pidKd, float pid_range_min, float pid_range_max) {
    // Set mode for pins
    gpio_set_dir(m_pin_dir1, true);
    gpio_set_dir(m_pin_dir2, true);

    gpio_set_function(m_pin_pwm, GPIO_FUNC_PWM);
    
    // Set basic settings for PID and for other vars
    pidMotor->computeMsToSeconds(speed_calc_millis);
    pidMotor->pidSetMinMax(pid_range_min, pid_range_max);
    pidMotor->pidSetCoefs(pidKp, pidKi, pidKd);

    m_motor_reduc_coef = motor_reduc_coef;
    m_update_period = speed_calc_millis;

    // Turn off motor direction
    gpio_put(m_pin_dir1, 0);
    gpio_put(m_pin_dir2, 0);

    gpio_set_irq_enabled_with_callback(this->m_pin_interrupt, GPIO_IRQ_EDGE_RISE, true, (gpio_irq_callback_t)CMotor::increaseTickCount_callback);
}

void CMotor::process(uint8_t direction, float rps) {    
    // Update data from encoders and set speed to motors

    if (get_systick() - timeStamp >= m_update_period) {
        rvPerS = tickCount / (m_motor_reduc_coef * m_update_period);
        tickCount = 0;

        unsigned char pwm = (unsigned char)pidMotor->computePid(rvPerS, rps) / PWM_TO_RPS;
    
        pwm = abs(pwm);

        if (direction == 0) {
            gpio_put(m_pin_dir1, 1);
            gpio_put(m_pin_dir2, 0);
        }
        else if (direction == 1) {
            gpio_put(m_pin_dir1, 0);
            gpio_put(m_pin_dir2, 1);
        }
        
        pwm_set_gpio_level(m_pin_pwm, pwm);

        timeStamp = get_systick();
        
    }
}

void CMotor::increaseTickCount_callback(void* context) {
    return reinterpret_cast<CMotor*>(context)->increaseTickCount();
}

void CMotor::increaseTickCount() {
        this->tickCount++;
}