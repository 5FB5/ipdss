//TODO: check interrupt pins in Pico
// Interrupts that will accept encoder's data from motors
#define MOTOR1_INTERRUPT 0
#define MOTOR2_INTERRUPT 1

// Check your motor's specifications of reduction coefficient
#define MOTOR_REDUCTION_COEF (float)18.8f

// Time period of motor processing
#define MOTOR_PROCESS_PERIOD 10.f // milliseconds

// Motor's PID regulator coefficients
#define MOTOR_PID_KP 8
#define MOTOR_PID_KI 0
#define MOTOR_PID_KD 0
#define MOTOR_PID_MIN 0 
#define MOTOR_PID_MAX 255

//TODO: Change pins to Pico's specific
// Motor 1
// Arduino's ENA/ENB pins will transmit calculated PWM value to motor driver's ENA/ENB pins
#define PIN_MOTOR_ENA 9
#define PIN_MOTOR_IN1 8
#define PIN_MOTOR_IN2 7

// Motor 2 
#define PIN_MOTOR2_ENB 12
#define PIN_MOTOR2_IN3 11
#define PIN_MOTOR2_IN4 10

#include <stdlib.h>

#include "pico/stdlib.h"
#include "cmotor.h"
#include "cpid.h"

CMotor motor1(MOTOR1_INTERRUPT, PIN_MOTOR_ENA, PIN_MOTOR_IN1, PIN_MOTOR_IN2);
CMotor motor2(MOTOR2_INTERRUPT, PIN_MOTOR2_ENB, PIN_MOTOR2_IN3, PIN_MOTOR2_IN4);

void initDevices() {
    motor1.setup(MOTOR_REDUCTION_COEF, MOTOR_PROCESS_PERIOD, MOTOR_PID_KP, MOTOR_PID_KI, MOTOR_PID_KD, MOTOR_PID_MIN, MOTOR_PID_MAX);
    motor2.setup(MOTOR_REDUCTION_COEF, MOTOR_PROCESS_PERIOD, MOTOR_PID_KP, MOTOR_PID_KI, MOTOR_PID_KD, MOTOR_PID_MIN, MOTOR_PID_MAX);
}

int main() {
    stdio_init_all();
    initDevices();

    while (true) {
        motor1.process(1, 0.5);
    }
    
    return 0;
}