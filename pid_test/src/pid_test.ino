/* ============
 *  DEFINES
 * ============
 */

// Motor
#define PIN_ENA 9
#define PIN_IN1 8
#define PIN_IN2 7
#define PIN_MOTOR_INTERRUPT 0 

#define REDUCTION_COEFF (float)36.13
#define UPDATE_FREQ 5.f
#define HERZ 10

#include <Arduino.h>

#include "CMotor.h"

CMotor* motor1 = new CMotor(PIN_MOTOR_INTERRUPT, PIN_ENA, PIN_IN1, PIN_IN2);

void setup() {  
  motor1->setup(REDUCTION_COEFF, UPDATE_FREQ, HERZ, 1, 0, 0, 0, 255);

  Serial.begin(115200);
 }

void loop() {
  motor1->motorProcess(0, 2);
}
