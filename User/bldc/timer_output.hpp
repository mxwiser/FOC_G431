#pragma once
// Included by setup.cpp after the HAL timer declarations.
// Reconfigure only while MOE is off, or latch a BLDC step through COM preload.
// CH4 (ADC trigger), counter frequency, deadtime and polarity stay untouched.
inline void motor_timer_foc() {
#if defined(STM32G431xx)
    TIM_TypeDef *t = htim1.Instance;
    t->CR2 |= TIM_CR2_CCPC;
    t->CCMR1 = (t->CCMR1 & ~(TIM_CCMR1_OC1M | TIM_CCMR1_OC2M)) |
        (6u << TIM_CCMR1_OC1M_Pos) | (6u << TIM_CCMR1_OC2M_Pos);
    t->CCMR2 = (t->CCMR2 & ~TIM_CCMR2_OC3M) | (6u << TIM_CCMR2_OC3M_Pos);
    t->CCER |= TIM_CCER_CC1E | TIM_CCER_CC1NE | TIM_CCER_CC2E |
        TIM_CCER_CC2NE | TIM_CCER_CC3E | TIM_CCER_CC3NE;
    t->EGR = TIM_EGR_COMG;
    t->CR2 &= ~TIM_CR2_CCPC;
#endif
}
inline void motor_timer_bldc(unsigned sector, float duty) {
    const bldc::Step phase = bldc::step(sector);
    const uint32_t ticks = static_cast<uint32_t>(duty * __HAL_TIM_GET_AUTORELOAD(&htim1));
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, phase.source == 0u ? ticks : 0u);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, phase.source == 1u ? ticks : 0u);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, phase.source == 2u ? ticks : 0u);
#if defined(STM32G431xx)
    TIM_TypeDef *t = htim1.Instance;
    t->CR2 |= TIM_CR2_CCPC;
    const uint32_t mode_a = phase.source == 0u ? 6u : 4u;
    const uint32_t mode_b = phase.source == 1u ? 6u : 4u;
    const uint32_t mode_c = phase.source == 2u ? 6u : 4u;
    t->CCMR1 = (t->CCMR1 & ~(TIM_CCMR1_OC1M | TIM_CCMR1_OC2M)) |
        (mode_a << TIM_CCMR1_OC1M_Pos) | (mode_b << TIM_CCMR1_OC2M_Pos);
    t->CCMR2 = (t->CCMR2 & ~TIM_CCMR2_OC3M) | (mode_c << TIM_CCMR2_OC3M_Pos);
    uint32_t enable = 0u;
    for (unsigned i = 0; i < 3u; ++i) {
        // Source: complementary PWM; sink: forced inactive -> low on;
        // floating: both outputs disabled (true high impedance).
        if (i != phase.floating) enable |= (TIM_CCER_CC1E | TIM_CCER_CC1NE) << (i * 4u);
    }
    const uint32_t all = TIM_CCER_CC1E | TIM_CCER_CC1NE | TIM_CCER_CC2E |
        TIM_CCER_CC2NE | TIM_CCER_CC3E | TIM_CCER_CC3NE;
    t->CCER = (t->CCER & ~all) | enable;
    t->EGR = TIM_EGR_COMG;
#endif
}
