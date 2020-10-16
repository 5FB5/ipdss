#pragma once

#define PWM_TO_RPS (float)0.012

#include <Arduino.h>

#include "CPid.h"

class CMotor {

public:
    CMotor(int hall_interrupt_numb, int pin_pwm, int pin_dir1, int pin_dir2);
    CPid* pidMotor = new CPid;

    float rvPerS = 0;

    uint8_t motorPower = 0;

    void setup(float motor_reduc_coef, float update_freq, float speed_calc_herz, int pidKp, int pidKi, int pidKd, int pid_range_min, int pid_range_max);
    void motorProcess(uint8_t direction, uint8_t rps);
    void hallInterrupt_callback();

private:
    typedef void (CMotor::*hallInterrupt_callback_ptr_t)();
    hallInterrupt_callback_ptr_t callback;
    
    volatile int tickCount = 0;
    volatile unsigned long int timeStamp = 0;
    volatile unsigned long int motorTimeStamp = 0;

    int m_hall_interrupt_numb;
    int m_pin_pwm;
    int m_pin_dir1;
    int m_pin_dir2;
};