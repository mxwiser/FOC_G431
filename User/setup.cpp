#include "setup.hpp"
#include "encoder/encoder.hpp"
#include "main.h"
#include "stdio.h"





void setup(void){
    encoder_init();
    while (1)
    {
        const float angleDeg = encoder_read_electrical_angle()
            * (180.0f / 3.14159265358979323846f);
        printf("angle:%.2f deg\r\n", angleDeg);
        HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_6);
        HAL_Delay(100);
    }
    
}

