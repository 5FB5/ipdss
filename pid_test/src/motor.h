#pragma once

#define REDUCTION_COEFF (float)36.13
#define UPDATE_FREQ 5.f

#include <Arduino.h>

class CMotor {

public:
    CMotor(int hall_interrupt_numb, int pin_pwm, int pin_dir1, int pin_dir2);
    
    uint8_t motorPower = 0;

    typedef void (CMotor::*hallInterrupt_callback_ptr_t)();
    hallInterrupt_callback_ptr_t callback;
    
    float calculateRps();

    void hallInterrupt_callback();
    void startMotor(uint8_t direction, uint8_t pwm);
    void setup();

private:
    volatile int tickCount = 0;
    volatile unsigned long int timeStamp = 0;
    volatile unsigned long int motorTimeStamp = 0;

    float rvPerS = 0;

    int hall_interrupt_numb;
    int pin_pwm;
    int pin_dir1;
    int pin_dir2;
};