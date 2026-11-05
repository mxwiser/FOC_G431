#pragma once
#include <stdint.h>

// Called by foc/adc_control_impl.hpp after decoding the current 3-rank DMA frame.
// U=ADC2 rank3, V=ADC2 rank2, W=ADC1 rank3; divider factor is 16.
// Values are physical phase-to-ground volts, not PWM voltage reconstruction.
// The publisher stamps the main current-ADC frame counter internally.
// Publish before that frame's BLDC control update; >2 frames old stops BLDC.
#ifdef __cplusplus
extern "C" {
#endif
void bldc_publish_phase_voltages(float a, float b, float c);
#ifdef __cplusplus
}
#endif
