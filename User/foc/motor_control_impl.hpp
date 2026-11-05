#pragma once
// The existing setup.cpp translation unit owns this module; no IDE list edits.
#include "motor_control.hpp"
#include "main.h"
#include "adc.h"
#include "opamp.h"
#include "tim.h"
#include "usart.h"
#include "foc_math.hpp"
#include "svpwm.h"
#include "vesc_observer.hpp"
#include "vesc_startup.hpp"
#include "adc_sampling.hpp"
#include "../app/serial_command.hpp"
#include "../app/data_port.hpp"
#include "../motor_mode.hpp"
#include "../bldc/bldc.hpp"
#include "../bldc/phase_feedback.hpp"
#include "../bldc/timer_output.hpp"
#include <cmath>
#include <stdio.h>
#if defined(__CC_ARM)
#pragma Otime
#endif


namespace {
#include "../board/adc_state.hpp"
#include "motor_state.hpp"
#include "../app/communication_state.hpp"
#include "state_machine_impl.hpp"
#include "../board/uart_port_impl.hpp"
#include "../app/host_protocol_impl.hpp"
#include "diagnostics_impl.hpp"
} // namespace

#include "../bldc/phase_feedback_impl.hpp"
#include "adc_control_impl.hpp"
#include "../board/adc_port_impl.hpp"
#include "../board/uart_irq_impl.hpp"

void motor_control::run()
{
    // 零点采集/待机时也准确报告所选类型；控制器其余参数仍在每次启动时初始化。
    observer_config.type = motor_observer_type();
#if defined(STM32G431xx)
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
#endif
    // No TIM4 reads or encoder startup. Ignore an unplugged Z input.
    CLEAR_BIT(EXTI->IMR1, ENCODE_ZERO_Pin); __HAL_GPIO_EXTI_CLEAR_IT(ENCODE_ZERO_Pin);
    bridge_off();
    if (start_uart_receive() != HAL_OK) data_port::receive_error();
    RCC_ClkInitTypeDef clocks = {}; uint32_t latency = 0;
    HAL_RCC_GetClockConfig(&clocks, &latency);
    const float timer_clock = static_cast<float>(HAL_RCC_GetPCLK2Freq())
        * (clocks.APB2CLKDivider == RCC_HCLK_DIV1 ? 1.0f : 2.0f);
    sample_dt = 2.0f * __HAL_TIM_GET_AUTORELOAD(&htim1) * (htim1.Init.Prescaler + 1.0f) / timer_clock;
    if (!limits_valid() || sample_dt <= 0.0f || sample_dt > 0.001f ||
        htim1.Init.CounterMode != TIM_COUNTERMODE_CENTERALIGNED2) motor_fail(BAD_CONFIG);
    parameters_ready = model_valid();
    if (HAL_OPAMP_Start(&hopamp1) != HAL_OK || HAL_OPAMP_Start(&hopamp2) != HAL_OK ||
        HAL_OPAMP_Start(&hopamp3) != HAL_OK) { motor_fail(ADC_ERROR); Error_Handler(); }
    HAL_Delay(1);
    if (HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED) != HAL_OK ||
        HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED) != HAL_OK) { motor_fail(ADC_ERROR); Error_Handler(); }
    if (HAL_ADCEx_MultiModeStart_DMA(&hadc1, reinterpret_cast<uint32_t *>(adc_buffer), motor_adc::dma_words) != HAL_OK) {
        motor_fail(ADC_ERROR); Error_Handler();
    }
    __HAL_DMA_DISABLE_IT(hadc1.DMA_Handle, DMA_IT_HT);
    if (HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4) != HAL_OK) { motor_fail(PWM_ERROR); Error_Handler(); }
    bridge_off(); // Sampling-channel HAL start enables MOE too.
    uint32_t last_frame = sample_frames, last_adc = HAL_GetTick(), last_status = HAL_GetTick();
    while (1) {
#if defined(STM32G431xx)
        // Debug-probe detach may restore DEMCR and clear TRCENA.
        CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
        DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
#endif
        service_serial();
        const uint32_t now = HAL_GetTick(), frame = sample_frames;
        if (frame != last_frame) { last_frame = frame; last_adc = now; }
        if (now - last_adc > 10u && motor_state != FAULT) motor_fail(ADC_TIMEOUT);
        service_restart();
        if (now - last_status >= 20u) {
            // Give state/command acknowledgements priority over waveform data.
            if (send_status()) last_status = now;
        } else send_phase_samples();
        __WFI();
    }
}
