#define ENCODER_COUNT 2

#include "pico/stdlib.h"
#include <stdint.h>
#include "../../pwm.h"

typedef struct {
    int32_t encoderTicks;
    uint8_t channelAGpio;
    uint8_t channelBGpio;
} rotaryEncoder_t;

void initEncoder(rotaryEncoder_t* encoder, 
                uint8_t channelAGpio, 
                uint8_t channelBGpio);