#pragma once
// STM32 board input storage. Algorithms consume SI data through motor_control.hpp.
const float kAmpsPerCount = 0.02946541f;
const uint32_t kZeroSamples = 2048u;
__align(4) uint16_t adc_buffer[motor_adc::sample_count];
uint32_t zero_count = 0, zero_sum[2] = {};
float zero_a, zero_b;
