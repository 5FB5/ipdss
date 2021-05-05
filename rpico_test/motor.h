#include "pico/stdlib.h"
#include <stdint.h>

typedef struct {
    int32_t encoderTicks;
    uint8_t channelAGpio;
    uint8_t channelBGpio;
    void (*channelACallback)(uint gpio, uint32_t events);
} rotaryEncoder_t