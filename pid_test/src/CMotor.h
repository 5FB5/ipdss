#pragma once

#define PWM_TO_RPS (float)1.36 // max rps on 255 PWM / 255

#include <Arduino.h>
#include "CPid.h"

class CMotor {

public:
    CMotor(uint8_t interrupt_numb, uint8_t pin_pwm, uint8_t pin_dir1, uint8_t pin_dir2);
    CPid* pidMotor = new CPid;

    float m_motor_reduc_coef = 0;
    float m_update_period = 0;
    float rvPerS = 0;
    
    volatile int tickCount = 0;

    void setup(float motor_reduc_coef, float speed_calc_herz, float pidKp, float pidKi, float pidKd, float pid_range_min, float pid_range_max);
    void motorProcess(uint8_t direction, float rps);  

private:
    volatile unsigned long int timeStamp = 0;

    uint8_t m_interrupt_numb;
    uint8_t m_pin_pwm;
    uint8_t m_pin_dir1;
    uint8_t m_pin_dir2;
};