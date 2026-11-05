#pragma once
// STM32 acquisition and calibration. Return physical values; HAL dispatch
// forwards these by value through the public motor_control setters.
namespace board_adc {
struct QuantizedValues { float ia, ib, vu, vv, vw, bus; };

static QuantizedValues read_quantized()
{
#if defined(STM32G431xx)
    isr_start_cycle = DWT->CYCCNT;
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
    const QuantizedValues values = {ia, ib, motor_adc::phase_volts(u),
        motor_adc::phase_volts(v), motor_adc::phase_volts(w), motor_adc::bus_volts(bus)};
    return values;
}

static void end_frame_timing()
{
#if defined(STM32G431xx)
    const uint32_t cycles = DWT->CYCCNT - isr_start_cycle;
    if (cycles > isr_max_cycles) isr_max_cycles = cycles;
#endif
}
} // namespace board_adc
