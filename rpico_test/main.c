#include <stdlib.h>

#include "pico/stdlib.h"
#include "pisystick.h"
#include "pwm.h"
#include "rotary_encoder.h"

int32_t encoderTicks = 0;
uint8_t direction = 0;

 
rotaryEncoder_t encoder1;

int main() {
    stdio_init_all();
    init_systick();
    
    uint32_t timestamp = get_systick();
    uint32_t bTimestamp = get_systick();

    float rvPerS = 0.0f;
    float m_motor_reduc_coef = 18.8f;
    float m_update_period = 10.f;

    gpio_set_function(5, GPIO_FUNC_PWM);

    uint slice_num = pwm_gpio_to_slice_num(5);

    pwm_config cfg = pwm_get_default_config();

    // pwm_config_set_clkdiv(&cfg, 4901.f);
    pwm_config_set_clkdiv_int(&cfg, 250);
    pwm_init(slice_num, &cfg, false);

    pwm_set_wrap(slice_num, 5000);
 pwm_set_chan_level(slice_num, PWM_CHAN_B, 4999);
    pwm_set_enabled(slice_num, true);
   
    
     initEncoder(&encoder1, 2,3);

    while (true) {
        if (get_systick() - timestamp >= m_update_period) {
            rvPerS = ((float)encoder1.encoderTicks * 1000.f )/ (m_motor_reduc_coef * m_update_period) / (448.f * 1);

            encoder1.encoderTicks = 0;
            timestamp = get_systick();
        }

     

         if (get_systick() - bTimestamp >= 100) {
            printf("%d %.3f\n\r", encoder1.encoderTicks, rvPerS);

            bTimestamp = get_systick();
        }
    }


    return 0;
}
