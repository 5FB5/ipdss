/* ============
 *  DEFINES
 * ============
 */

// Motor
#define PIN_ENA 9
#define PIN_IN1 8
#define PIN_IN2 7
#define PIN_ENGINE_INTERRUPT 0 

#define UPDATE_FREQ 5.f

#include <Arduino.h>

#include "CMotor.h"
#include "CPid.h"

CMotor* motor1 = new CMotor(PIN_ENGINE_INTERRUPT, PIN_ENA, PIN_IN1, PIN_IN2);

void setup() {  
  motor1->setup(1, 0, 0, 0, 255);

  Serial.begin(115200);
 }

void loop() {
  motor1->motorProcess(0, 2);
}
