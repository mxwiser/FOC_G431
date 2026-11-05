#pragma once
#include "tim.h"

namespace board_pwm {
inline float control_period_seconds()
{
    if (htim1.Init.CounterMode != TIM_COUNTERMODE_CENTERALIGNED2) return 0.0f;
    RCC_ClkInitTypeDef clocks = {}; uint32_t latency = 0;
    HAL_RCC_GetClockConfig(&clocks, &latency);
    const float timer_clock = static_cast<float>(HAL_RCC_GetPCLK2Freq())
        * (clocks.APB2CLKDivider == RCC_HCLK_DIV1 ? 1.0f : 2.0f);
    return 2.0f * __HAL_TIM_GET_AUTORELOAD(&htim1) * (htim1.Init.Prescaler + 1.0f) / timer_clock;
}

inline bool init()
{
    const bool started = HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4) == HAL_OK;
    // HAL starts the sampling channel with MOE set; keep the bridge disabled.
    __HAL_TIM_MOE_DISABLE_UNCONDITIONALLY(&htim1);
    return started;
}
}
