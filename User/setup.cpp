#include "setup.hpp"
#include "foc/motor_control_impl.hpp"

// Application entry and HAL dispatch only. See foc/ and app/ for implementation.
void setup(void)
{
    motor_control::run();
}

extern "C" void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    motor_control::adc_complete(hadc);
}

extern "C" void HAL_ADC_ErrorCallback(ADC_HandleTypeDef *hadc)
{
    motor_control::adc_error(hadc);
}
