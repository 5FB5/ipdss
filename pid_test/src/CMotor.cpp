#include "CMotor.h"

CMotor::CMotor(int hall_interrupt_numb, int pin_pwm, int pin_dir1, int pin_dir2) {
    m_hall_interrupt_numb = hall_interrupt_numb;
    m_pin_pwm = pin_pwm;
    m_pin_dir1 = pin_dir1;
    m_pin_dir2 = pin_dir2;
}

void CMotor::motorProcess(uint8_t direction, uint8_t rps) {
    float pwm = pidMotor->computePid(rvPerS, rps) / PWM_TO_RPS;
    
    pwm = abs(pwm);

    if (pwm < pidMotor->pidMin) {
        pwm = pidMotor->pidMin;
    }
    else if (pwm > pidMotor->pidMax) {
        pwm = pidMotor->pidMax;
    }
    
    if (direction == 0) {
        digitalWrite(m_pin_dir1, HIGH);
        digitalWrite(m_pin_dir2, LOW);
    }
     else if (direction == 1) {
         digitalWrite(m_pin_dir1, LOW);
         digitalWrite(m_pin_dir2, HIGH);
     }

    analogWrite(m_pin_pwm, pwm);
}

void CMotor::hallInterrupt_callback() {
    tickCount++;
}

void CMotor::setup(float motor_reduc_coef, float update_freq, float speed_calc_herz, int pidKp, int pidKi, int pidKd, int pid_range_min, int pid_range_max) {
    attachInterrupt(m_hall_interrupt_numb, (void(*)())callback, RISING);
    
    pidMotor->computeHerzToSeconds(speed_calc_herz);
    pidMotor->pidSetMinMax(pid_range_min, pid_range_max);
    pidMotor->pidSetCoefs(pidKp, pidKi, pidKd);

    rvPerS = tickCount / motor_reduc_coef * update_freq;
    tickCount = 0;
}