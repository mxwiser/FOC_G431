#pragma once
#include <stdint.h>

// Board integration point, intentionally not connected to a fabricated signal.
// Call from the ADC path that captures all three phase voltages, once per PWM
// sample. Values are phase-to-ground volts after divider/offset calibration.
// The publisher stamps the main current-ADC frame counter internally.
// Publish before that frame's BLDC control update; >2 frames old stops BLDC.
// The present CubeMX configuration has no phase-voltage ADC channels.
#ifdef __cplusplus
extern "C" {
#endif
void bldc_publish_phase_voltages(float a, float b, float c);
#ifdef __cplusplus
}
#endif
