#include "setup.hpp"
#include "foc/motor_control_impl.hpp"
#include "board/adc_port_impl.hpp"
#include "board/pwm_init.hpp"
#include "board/system_init.hpp"
#include "board/uart_init.hpp"

void setup(void)
{
    board_system::init();
    bridge_off();
    if (!board_uart::init()) data_port::receive_error();
    motor_control::initialize(board_pwm::control_period_seconds());
    if (!board_adc::init()) { motor_fail(ADC_ERROR); Error_Handler(); }
    if (!board_pwm::init()) { motor_fail(PWM_ERROR); Error_Handler(); }
    bridge_off();
    while (1) {
        board_system::enable_cycle_counter();
        motor_control::run();
        board_system::wait();
    }
}

extern "C" void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc != &hadc1) return;
    const board_adc::QuantizedValues values = board_adc::read();
    motor_control::set_phase_currents(values.ia, values.ib);           // A
    motor_control::set_phase_voltages(values.vu, values.vv, values.vw); // V
    motor_control::set_bus_voltage(values.bus);                      // V
    motor_control::adc_complete();
    board_adc::finish();
}

extern "C" void HAL_ADC_ErrorCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc == &hadc1 || hadc == &hadc2) motor_control::adc_error();
}
