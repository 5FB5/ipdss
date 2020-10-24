/* ============
 *  DEFINES
 * ============
 */

// Motor 1
 #define PIN_MOTOR_ENA 9
 #define PIN_MOTOR_IN1 8
 #define PIN_MOTOR_IN2 7
 #define PIN_MOTOR_INTERRUPT 0

 // Motor 2
 //...

 // Motor's main data
 #define MOTOR_REDUCTION_COEFF (float)18.8f
 #define T 10.f // millis

// Motor's PID regulator
 #define PID_KP 9
 #define PID_KI 0
 #define PID_KD 0
 #define PID_MIN 0
 #define PID_MAX 255
 
 #include <Arduino.h>
 #include "CMotor.h"

CMotor* motor1 = new CMotor(PIN_MOTOR_INTERRUPT, PIN_MOTOR_ENA, PIN_MOTOR_IN1, PIN_MOTOR_IN2);

/* ============
 *  MAIN
 * ============
 */

void increaseMotor1EncoderTicks() { // 'cause it doesn't work in class
   motor1->tickCount++;
}

void setup() {  
  motor1->setup(MOTOR_REDUCTION_COEFF, T, PID_KP, PID_KI, PID_KD, PID_MIN, PID_MAX);
  attachInterrupt(0, increaseMotor1EncoderTicks, RISING);
  
  Serial.begin(115200); // ??
 }

void loop() {
   motor1->motorProcess(0, 1); // RPS
   
   Serial.print("Motor RPS: ");
   Serial.println(motor1->rvPerS);
}
