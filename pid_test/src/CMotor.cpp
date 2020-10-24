#include "CMotor.h"

CMotor::CMotor(uint8_t interrupt_numb, uint8_t pin_pwm, uint8_t pin_dir1,uint8_t pin_dir2) {
    m_interrupt_numb = interrupt_numb;
    m_pin_pwm = pin_pwm;
    m_pin_dir1 = pin_dir1;
    m_pin_dir2 = pin_dir2;
}

void CMotor::setup(float motor_reduc_coef, float speed_calc_millis, float pidKp, float pidKi, float pidKd, float pid_range_min, float pid_range_max) {
    pinMode(m_pin_pwm, OUTPUT);
    pinMode(m_pin_dir1, OUTPUT);
    pinMode(m_pin_dir2, OUTPUT);
    
    pidMotor->computeMsToSeconds(speed_calc_millis);
    pidMotor->pidSetMinMax(pid_range_min, pid_range_max);
    pidMotor->pidSetCoefs(pidKp, pidKi, pidKd);

    m_motor_reduc_coef = motor_reduc_coef;
    m_update_period = speed_calc_millis;

    // Turn off motor direction
    digitalWrite(m_pin_dir1, LOW);
    digitalWrite(m_pin_dir2, LOW);
}

void CMotor::motorProcess(uint8_t direction, float rps) {    
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
        
        Serial.print("Motor 1 PID PWM: ");
        Serial.println(pwm);
        
        timeStamp = millis();
    }
}