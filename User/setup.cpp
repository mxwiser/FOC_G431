#include "setup.hpp"
#include "encoder/encoder.hpp"
#include "main.h"
#include "stdio.h"

float angle_bias;

float zero_electrical_angle(){
   float bias;
   return bias;
}


void setup(void){
    encoder_init();
    angle_bias = zero_electrical_angle();
    while (1)
    {
        const float angleDeg = encoder_read_mechanical_angle()
            * (180.0f / 3.14159265358979323846f);
        printf("mechanical:%.2f deg\r\n", angleDeg);
        HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_6);
        HAL_Delay(100);
    }
    
}

