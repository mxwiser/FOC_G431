#include "setup.hpp"
#include "board/adc_init.hpp"
#include "board/pwm_init.hpp"
#include "board/system_init.hpp"

// This board's dual-ADC ranks and physical-unit conversion belong here.
// ADC1 occupies the low halfword; ADC2 occupies the high halfword.
namespace motor_adc {
const unsigned dma_words = 3u, sample_count = dma_words * 2u;
enum Index { CURRENT_A, CURRENT_B, BUS, VOLTAGE_V, VOLTAGE_W, VOLTAGE_U };
const float volts_per_count = 3.3f / 4095.0f;
const float phase_divider = (33000.0f + 2200.0f) / 2200.0f;
inline float phase_volts(uint16_t raw) { return raw * volts_per_count * phase_divider; }
inline float bus_volts(uint16_t raw) { return raw * volts_per_count * (187.0f / 18.0f); }
}

namespace {
const float kAmpsPerCount = 0.02946541f;
const uint32_t kZeroSamples = 2048u;
__align(4) uint16_t adc_buffer[motor_adc::sample_count];
uint32_t zero_count = 0, zero_sum[2] = {};
float zero_a, zero_b;
}

#include "foc/motor_control_impl.hpp"
#include "board/uart_init.hpp"

void setup(void)
{
    board_system::init();
    bridge_off();
    if (!board_uart::init()) data_port::receive_error();
    motor_control::initialize(board_pwm::control_period_seconds());
    if (!board_adc::init(adc_buffer, motor_adc::dma_words)) { motor_fail(ADC_ERROR); Error_Handler(); }
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
    isr_start_cycle = board_system::cycles();
    const uint16_t a = adc_buffer[motor_adc::CURRENT_A];
    const uint16_t b = adc_buffer[motor_adc::CURRENT_B];
    const uint16_t bus = adc_buffer[motor_adc::BUS];
    const uint16_t u = adc_buffer[motor_adc::VOLTAGE_U];
    const uint16_t v = adc_buffer[motor_adc::VOLTAGE_V];
    const uint16_t w = adc_buffer[motor_adc::VOLTAGE_W];
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
    const float vu = motor_adc::phase_volts(u);
    const float vv = motor_adc::phase_volts(v);
    const float vw = motor_adc::phase_volts(w);
    const float vbus = motor_adc::bus_volts(bus);
    motor_control::set_phase_currents(ia, ib);       // A
    motor_control::set_phase_voltages(vu, vv, vw);   // V
    motor_control::set_bus_voltage(vbus);           // V
    motor_control::adc_complete();
#if defined(FOC_CURRENT_DIAG) && FOC_CURRENT_DIAG
    if (motor_state != ZERO && motor_state != FAULT) record_current_diagnostic(a, b, bus);
#endif
    const uint32_t cycles = board_system::cycles() - isr_start_cycle;
    if (cycles > isr_max_cycles) isr_max_cycles = cycles;
}

extern "C" void HAL_ADC_ErrorCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc == &hadc1 || hadc == &hadc2) motor_control::adc_error();
}
