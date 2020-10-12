#include "CMotor.h"

CMotor::CMotor(int hall_interrupt_numb, int pin_pwm, int pin_dir1, int pin_dir2) {
    this->m_hall_interrupt_numb = hall_interrupt_numb;
    this->m_pin_pwm = pin_pwm;
    this->m_pin_dir1 = pin_dir1;
    this->m_pin_dir2 = pin_dir2;
}

void CMotor::motorProcess(uint8_t direction, uint8_t rps) {
    float pwm = this->pidMotor->computePid(rvPerS, rps) / PWM_TO_RPS;
    
    pwm = abs(pwm);

    if (pwm < pidMotor->pidMin) {
        pwm = pidMotor->pidMin;
    }
    else if (pwm > pidMotor->pidMax) {
        pwm = pidMotor->pidMax;
    }
    
    if (direction == 0) {
        digitalWrite(this->m_pin_dir1, HIGH);
        digitalWrite(this->m_pin_dir2, LOW);
    }
     else if (direction == 1) {
         digitalWrite(this->m_pin_dir1, LOW);
         digitalWrite(this->m_pin_dir2, HIGH);
     }

    analogWrite(this->m_pin_pwm, pwm);
}

void CMotor::hallInterrupt_callback() {
    this->tickCount++;
}

void CMotor::setup(int pidKp, int pidKi, int pidKd, int pid_range_min, int pid_range_max) {
    attachInterrupt(this->m_hall_interrupt_numb, (void(*)())this->callback, RISING);
    
    this->pidMotor->computeHerzToSeconds(HERZ);
    this->pidMotor->pidSetMinMax(pid_range_min, pid_range_max);
    this->pidMotor->pidSetCoefs(pidKp, pidKi, pidKd);

    rvPerS = tickCount / REDUCTION_COEFF * UPDATE_FREQ;
    tickCount = 0;
}