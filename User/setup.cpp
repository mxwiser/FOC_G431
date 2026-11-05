#include "setup.hpp"
#include "foc/motor_control_impl.hpp"

// Application entry and HAL dispatch only. See foc/ and app/ for implementation.
void setup(void)
{
    motor_control::run();
}

extern "C" void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc == &hadc1) board_adc_complete();
}

extern "C" void HAL_ADC_ErrorCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc == &hadc1 || hadc == &hadc2) motor_control::adc_error();
}
