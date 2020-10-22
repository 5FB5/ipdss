/* ============
 *  DEFINES
 * ============
 */

// Motor 1
 #define PIN_ENA 9
 #define PIN_IN1 8
 #define PIN_IN2 7
 #define PIN_MOTOR_INTERRUPT 0

 // Motor 2
 //...

 // Motor's data
 #define REDUCTION_COEFF (float)18.8f
 #define UPDATE_FREQ 5.f
 #define HERZ 10

// Motor's PID regulator
 #define PID_KP 1
 #define PID_KI 0
 #define PID_KD 0
 #define PID_MIN 0
 #define PID_MAX 255
 
 #include <Arduino.h>
 #include "CMotor.h"

CMotor* motor1 = new CMotor(PIN_MOTOR_INTERRUPT, PIN_ENA, PIN_IN1, PIN_IN2);

/* ============
 *  MAIN
 * ============
 */

void increaseMotor1EncoderTicks() {
   motor1->tickCount++;
   Serial.println(motor1->tickCount);
}

void setup() {  
  motor1->setup(REDUCTION_COEFF, UPDATE_FREQ, HERZ, PID_KP, PID_KI, PID_KD, PID_MIN, PID_MAX);
  attachInterrupt(0, increaseMotor1EncoderTicks, RISING);
  
  Serial.begin(115200);
 }

void loop() {
   motor1->motorProcess(0, 0);
}
