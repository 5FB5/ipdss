#define MPU_INTERRUPT_PIN 3

#include <Arduino.h>
#include "CMpu.h"

CMpu mpu;

void dmpDataReady() { // only for MPU
    mpu.mpuInterrupt = true;
}

void setup() {
    mpu.init();
    attachInterrupt(digitalPinToInterrupt(MPU_INTERRUPT_PIN), dmpDataReady, RISING);
}

void loop() {
    mpu.processMpu();
/* just for test

    if (mpu.mpuPitch > 15) {
        //...
    }
     else if (mpu.mpuPitch < 15) {
         //...
     }
*/

}
