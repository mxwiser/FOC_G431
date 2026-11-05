#include "setup.hpp"
#include "foc/motor_control_impl.hpp"

// Application entry and explicit physical-value input at the HAL boundary.
void setup(void)
{
    motor_control::run();
}

extern "C" void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc != &hadc1) return;
    const board_adc::QuantizedValues values = board_adc::read_quantized();
    motor_control::set_phase_currents(values.ia, values.ib);          // A
    motor_control::set_phase_voltages(values.vu, values.vv, values.vw); // V
    motor_control::set_bus_voltage(values.bus);                     // V
    motor_control::adc_complete();
    board_adc::end_frame_timing();
}

extern "C" void HAL_ADC_ErrorCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc == &hadc1 || hadc == &hadc2) motor_control::adc_error();
}
