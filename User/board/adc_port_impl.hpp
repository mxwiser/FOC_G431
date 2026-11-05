#pragma once
// Included only by setup.cpp, after the control module implementation.
#include "adc.h"
#include "opamp.h"
#include "system_init.hpp"

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
#if defined(FOC_CURRENT_DIAG) && FOC_CURRENT_DIAG
uint16_t diagnostic_raw_a, diagnostic_raw_b, diagnostic_raw_bus;
#endif
}

namespace board_adc {
struct QuantizedValues { float ia, ib, vu, vv, vw, bus; };
inline bool init()
{
    if (HAL_OPAMP_Start(&hopamp1) != HAL_OK || HAL_OPAMP_Start(&hopamp2) != HAL_OK ||
        HAL_OPAMP_Start(&hopamp3) != HAL_OK) return false;
    HAL_Delay(1);
    if (HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED) != HAL_OK ||
        HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED) != HAL_OK) return false;
    if (HAL_ADCEx_MultiModeStart_DMA(&hadc1, reinterpret_cast<uint32_t *>(adc_buffer), motor_adc::dma_words) != HAL_OK)
        return false;
    __HAL_DMA_DISABLE_IT(hadc1.DMA_Handle, DMA_IT_HT);
    return true;
}

inline QuantizedValues read()
{
    isr_start_cycle = board_system::cycles();
    const uint16_t a = adc_buffer[motor_adc::CURRENT_A];
    const uint16_t b = adc_buffer[motor_adc::CURRENT_B];
    const uint16_t bus = adc_buffer[motor_adc::BUS];
    const uint16_t u = adc_buffer[motor_adc::VOLTAGE_U];
    const uint16_t v = adc_buffer[motor_adc::VOLTAGE_V];
    const uint16_t w = adc_buffer[motor_adc::VOLTAGE_W];
#if defined(FOC_CURRENT_DIAG) && FOC_CURRENT_DIAG
    diagnostic_raw_a = a; diagnostic_raw_b = b; diagnostic_raw_bus = bus;
#endif
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
    const QuantizedValues values = {ia, ib, vu, vv, vw, vbus};
    return values;
}

inline void finish()
{
#if defined(FOC_CURRENT_DIAG) && FOC_CURRENT_DIAG
    if (motor_state != ZERO && motor_state != FAULT)
        record_current_diagnostic(diagnostic_raw_a, diagnostic_raw_b, diagnostic_raw_bus);
#endif
    const uint32_t cycles = board_system::cycles() - isr_start_cycle;
    if (cycles > isr_max_cycles) isr_max_cycles = cycles;
}
} // namespace board_adc
