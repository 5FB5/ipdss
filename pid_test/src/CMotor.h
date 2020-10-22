#pragma once

#define PWM_TO_RPS (float)0.012

#include <Arduino.h>
#include "CPid.h"

class CMotor {

public:
    CMotor(int hall_interrupt_numb, int pin_pwm, int pin_dir1, int pin_dir2);
    CPid* pidMotor = new CPid;

    float calculatePwmToRpsCoef();

    float m_motor_reduc_coef;
    float m_motor_update_freq;

    float rvPerS = 0;
    
    unsigned int tickCount = 0;

    void setup(float motor_reduc_coef, float update_freq, float speed_calc_herz, int pidKp, int pidKi, int pidKd, int pid_range_min, int pid_range_max);
    void motorProcess(uint8_t direction, uint8_t rps);  

private:
    volatile unsigned long int timeStamp = 0;

    int m_hall_interrupt_numb;
    int m_pin_pwm;
    int m_pin_dir1;
    int m_pin_dir2;
};