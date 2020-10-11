/* ============
 *  DEFINES
 * ============
 */

// Motor
#define PIN_ENA 9
#define PIN_IN1 8
#define PIN_IN2 7
#define PIN_ENGINE_INTERRUPT 0 
#define REDUCTION_COEFF (float)36.13
#define UPDATE_FREQ 5.f
#define PWM_TO_RPS (float)0.012

#include <Arduino.h>

#include "motor.h"
#include "pid.h"

CMotor* motor1 = new CMotor(PIN_ENGINE_INTERRUPT, PIN_ENA, PIN_IN1, PIN_IN2);
//CPid* pidMotor1 = new CPid;

void setup() {
  motor1->setup();

  Serial.begin(115200);

 }

void loop() {
  motor1->startMotor(0, 155);
  /* Work in progress
  pidMotor1->computeHerzToSeconds(10);
 r motor1->startMotor(0, pidMotor1->computePid(motor1->calculateRps(), 144));
*/
}
