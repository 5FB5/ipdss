#include "CMotor.h"

CMotor::CMotor(int hall_interrupt_numb, int pin_pwm, int pin_dir1, int pin_dir2) {
    m_hall_interrupt_numb = hall_interrupt_numb;
    m_pin_pwm = pin_pwm;
    m_pin_dir1 = pin_dir1;
    m_pin_dir2 = pin_dir2;
}

void CMotor::setup(float motor_reduc_coef, float update_freq, float speed_calc_herz, int pidKp, int pidKi, int pidKd, int pid_range_min, int pid_range_max) {
    pidMotor->computeHerzToSeconds(speed_calc_herz);
    pidMotor->pidSetMinMax(pid_range_min, pid_range_max);
    pidMotor->pidSetCoefs(pidKp, pidKi, pidKd);

    m_motor_reduc_coef = motor_reduc_coef;
    m_motor_update_freq = update_freq;

    // Turn off motor direction
    digitalWrite(m_pin_dir1, LOW);
    digitalWrite(m_pin_dir2, LOW);
}

void CMotor::motorProcess(uint8_t direction, uint8_t rps) {    
    if (millis() - timeStamp >= 10) {
        rvPerS = tickCount / m_motor_reduc_coef * m_motor_update_freq;
        tickCount = 0;

        float pwm = abs(pidMotor->computePid(rvPerS, rps));

        // if (pwm < pidMotor->pidMin) {
        //     pwm = pidMotor->pidMin;
        // }
        // else if (pwm > pidMotor->pidMax) {
        //     pwm = pidMotor->pidMax;
        // }
    
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