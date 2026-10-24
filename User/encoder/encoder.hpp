#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Call after MX_TIM4_Init(). TIM4 is free-running; Z pulses do not reset it. */
void encoder_init(void);
int16_t encoder_read_count(void);
/* Mechanical angle in radians, normalized to [0, 2*pi). */
float encoder_read_mechanical_angle(void);
/* Electrical angle in radians, normalized to [0, 2*pi). */
float encoder_read_electrical_angle(void);

#ifdef __cplusplus
}
#endif
