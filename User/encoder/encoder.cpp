#include "encoder.hpp"
#include "tim.h"

namespace {
const int kPolePairs = 7;
const int kCountsPerRevolution = 2048;
const float kPi = 3.14159265358979323846f;
}

void encoder_init(void)
{
    if (HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL) != HAL_OK) {
        Error_Handler();
    }
}

int16_t encoder_read_count(void)
{
    return static_cast<int16_t>(__HAL_TIM_GET_COUNTER(&htim4));
}

float encoder_read_mechanical_angle(void)
{
    int32_t mechanicalCount = encoder_read_count() % kCountsPerRevolution;
    if (mechanicalCount < 0) {
        mechanicalCount += kCountsPerRevolution;
    }
    return static_cast<float>(mechanicalCount)
        * (2.0f * kPi / static_cast<float>(kCountsPerRevolution));
}

float encoder_read_electrical_angle(void)
{
    const int32_t count = encoder_read_count();
    // Multiply before wrapping: counts per electrical turn need not be an integer.
    int32_t electricalCount = (count * kPolePairs) % kCountsPerRevolution;
    if (electricalCount < 0) {
        electricalCount += kCountsPerRevolution;
    }
    return static_cast<float>(electricalCount)
        * (2.0f * kPi / static_cast<float>(kCountsPerRevolution));
}

extern "C" void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    // Z must not change the counter reference used by electrical_angle_bias.
    // The HAL EXTI handler already acknowledges the interrupt pending flag.
    (void)GPIO_Pin;
}
