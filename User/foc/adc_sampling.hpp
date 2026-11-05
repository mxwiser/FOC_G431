#pragma once
#include <stdint.h>

// Current CubeMX dual regular simultaneous sequence, one 32-bit CDR word/rank:
// rank 1: ADC1=OPAMP1/Ia, ADC2=OPAMP2/Ib
// rank 2: ADC1=IN1/Vbus,  ADC2=IN5/Vv (PC4)
// rank 3: ADC1=IN14/Vw,   ADC2=IN17/Vu (PA4)
// ADC1 is the low halfword and ADC2 the high halfword of every DMA word.
namespace motor_adc {
const unsigned dma_words = 3u;
const unsigned sample_count = dma_words * 2u;
enum Index { CURRENT_A, CURRENT_B, BUS, VOLTAGE_V, VOLTAGE_W, VOLTAGE_U };
const float volts_per_count = 3.3f / 4095.0f;
const float phase_divider = (33000.0f + 2200.0f) / 2200.0f;
inline float phase_volts(uint16_t raw) { return raw * volts_per_count * phase_divider; }
inline float bus_volts(uint16_t raw) { return raw * volts_per_count * (187.0f / 18.0f); }
}
