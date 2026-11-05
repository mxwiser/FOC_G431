#pragma once
#include "adc.h"

namespace motor_control {
// Startup and foreground service. This function owns the main loop.
void run();
// HAL entry points; all sampling, protection and control stay in foc/.
void adc_complete(ADC_HandleTypeDef *adc);
void adc_error(ADC_HandleTypeDef *adc);
}
