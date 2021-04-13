#define PIN_LED PICO_DEFAULT_LED_PIN


#include <stdlib.h>
#include "pico/stdlib.h"

int main() {
    stdio_init_all();

    while (true) {
        printf("Hello, world!");
        sleep_ms(250);
    }

    return 0;
}