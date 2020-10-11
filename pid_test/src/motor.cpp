#include "motor.h"

CMotor::CMotor(int hall_interrupt_numb, int pin_pwm, int pin_dir1, int pin_dir2) {
    this->hall_interrupt_numb = hall_interrupt_numb;
    this->pin_pwm = pin_pwm;
    this->pin_dir1 = pin_dir1;
    this->pin_dir2 = pin_dir2;
}

void CMotor::startMotor(uint8_t direction, uint8_t pwm) {
    if (direction == 0) {
        digitalWrite(this->pin_dir1, HIGH);
        digitalWrite(this->pin_dir2, LOW);
    }
     else if (direction == 1) {
         digitalWrite(this->pin_dir1, LOW);
         digitalWrite(this->pin_dir2, HIGH);
     }

     analogWrite(this->pin_pwm, pwm);
}

void CMotor::hallInterrupt_callback() {
    this->tickCount++;
}

void CMotor::setup() {
    attachInterrupt(this->hall_interrupt_numb, (void(*)())this->callback, RISING);
}

float CMotor::calculateRps() {
    rvPerS = tickCount / REDUCTION_COEFF * UPDATE_FREQ;
    tickCount = 0;

    return rvPerS;
}