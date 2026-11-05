#pragma once
// STM32 DMA and sensor calibration adapter. Only this file interprets raw ADC.
static void board_adc_complete()
{
#if defined(STM32G431xx)
    const uint32_t start = DWT->CYCCNT;
    isr_start_cycle = start;
#endif
    const volatile uint16_t *samples = adc_buffer;
    const uint16_t a = samples[motor_adc::CURRENT_A], b = samples[motor_adc::CURRENT_B];
    const uint16_t bus = samples[motor_adc::BUS];
    const uint16_t u = samples[motor_adc::VOLTAGE_U];
    const uint16_t v = samples[motor_adc::VOLTAGE_V];
    const uint16_t w = samples[motor_adc::VOLTAGE_W];
    float ia = 0.0f, ib = 0.0f;
    if (motor_control::calibrating()) {
        if (a < 256 || a > 3840 || b < 256 || b > 3840) {
            motor_control::input_error(motor_control::BAD_CURRENT_ZERO);
        } else {
            zero_sum[0] += a; zero_sum[1] += b;
            if (++zero_count == kZeroSamples) {
                zero_a = static_cast<float>(zero_sum[0]) / kZeroSamples;
                zero_b = static_cast<float>(zero_sum[1]) / kZeroSamples;
                motor_control::calibration_complete();
            }
        }
    } else {
        ia = (zero_a - a) * kAmpsPerCount;
        ib = (zero_b - b) * kAmpsPerCount;
        if ((a == 0 || a == 4095 || b == 0 || b == 4095) &&
            motor_state != IDLE && motor_state != RETRY_WAIT) {
            motor_control::input_error(motor_control::CURRENT_ADC_SATURATED);
        }
    }
    motor_control::set_phase_currents(ia, ib);
    motor_control::set_phase_voltages(motor_adc::phase_volts(u),
                                    motor_adc::phase_volts(v), motor_adc::phase_volts(w));
    motor_control::set_bus_voltage(motor_adc::bus_volts(bus));
    motor_control::adc_complete();
#if defined(STM32G431xx)
    const uint32_t cycles = DWT->CYCCNT - start;
    if (cycles > isr_max_cycles) isr_max_cycles = cycles;
#endif
}
