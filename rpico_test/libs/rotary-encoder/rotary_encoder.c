#include "rotary_encoder.h"

rotaryEncoder_t* encoders[ENCODER_COUNT];
uint8_t deviceCount = 0;

static void channelA_callback(uint gpio, uint32_t events);
static void incEncoderTicks(rotaryEncoder_t* encoder, uint32_t events);

void initEncoder(rotaryEncoder_t* encoder, 
                uint8_t channelAGpio, 
                uint8_t channelBGpio) 
{
    encoders[deviceCount] = encoder;
    deviceCount++;
    encoder->channelAGpio = channelAGpio;
    encoder->channelBGpio = channelBGpio;
    encoder->encoderTicks = 0;

    gpio_set_irq_enabled_with_callback(channelAGpio, GPIO_IRQ_EDGE_RISE, true, &channelA_callback);
}

static void channelA_callback(uint gpio, uint32_t events) {
    for (uint8_t i = 0; i < ENCODER_COUNT; i++) {
        if (gpio == encoders[i]->channelAGpio) {
            incEncoderTicks(encoders[i], events);
            break;
        }
    }
}

static void incEncoderTicks(rotaryEncoder_t* encoder, uint32_t events) {
    uint channelBState = 0;

    channelBState = gpio_get(encoder->channelBGpio);

    if (events == GPIO_IRQ_EDGE_RISE) {
        if (!channelBState) {
            encoder->encoderTicks++;
        }
         else {
             encoder->encoderTicks--;
         }
    }
    else {
        if (channelBState) {
            encoder->encoderTicks++;
        }
        else {
            encoder->encoderTicks--;
        }
    }

}
