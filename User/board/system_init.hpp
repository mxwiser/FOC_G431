#pragma once
#include "main.h"

namespace board_system {
inline void enable_cycle_counter()
{
#if defined(STM32G431xx)
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
#endif
}

inline void init()
{
    enable_cycle_counter();
#if defined(STM32G431xx)
    DWT->CYCCNT = 0;
#endif
    CLEAR_BIT(EXTI->IMR1, ENCODE_ZERO_Pin);
    __HAL_GPIO_EXTI_CLEAR_IT(ENCODE_ZERO_Pin);
}

inline uint32_t cycles()
{
#if defined(STM32G431xx)
    return DWT->CYCCNT;
#else
    return 0u;
#endif
}

inline void wait() { __WFI(); }
}
