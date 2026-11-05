#pragma once
// Private implementation, included only by foc/motor_control_impl.hpp.

static void adc_sample(ADC_HandleTypeDef *hadc)
{
    if (hadc != &hadc1) return;
    ++sample_frames;
    const volatile uint16_t *samples = adc_buffer;
    // 两相电流、母线和三相电压；完整布局见 adc_sampling.hpp。
    const uint16_t a = samples[motor_adc::CURRENT_A], b = samples[motor_adc::CURRENT_B];
    const uint16_t bus = samples[motor_adc::BUS];
    const uint16_t voltage_u = samples[motor_adc::VOLTAGE_U];
    const uint16_t voltage_v = samples[motor_adc::VOLTAGE_V];
    const uint16_t voltage_w = samples[motor_adc::VOLTAGE_W];
    Udc_quant = motor_adc::bus_volts(bus);
    bldc_publish_phase_voltages(motor_adc::phase_volts(voltage_u),
        motor_adc::phase_volts(voltage_v), motor_adc::phase_volts(voltage_w));
    if (motor_state == FAULT) { bridge_off(); return; }
    if (motor_state == ZERO) {
        if (a < 256 || a > 3840 || b < 256 || b > 3840) {
            motor_fail(BAD_ZERO); return;
        }
        zero_sum[0] += a; zero_sum[1] += b;
        if (++zero_count == kZeroSamples) {
            zero_a = static_cast<float>(zero_sum[0]) / kZeroSamples;
            zero_b = static_cast<float>(zero_sum[1]) / kZeroSamples;
            enter_state(IDLE);
        }
        return;
    }
    I_a = (zero_a - a) * kAmpsPerCount; I_b = (zero_b - b) * kAmpsPerCount;
    I_c = -I_a - I_b; // 同一时刻的两相电流重建 C 相，不再读取第三路电流 ADC。
    capture_phase_sample();
    last_I_alpha = I_alpha; last_I_beta = I_beta;
    // FOC 电流环和观测器保持使用同步 A/B 的两相 Clarke。
    clark_two_phase_ab(&I_a, &I_b, &I_alpha, &I_beta);
    if (motor_state == IDLE || motor_state == RETRY_WAIT) {
#if defined(FOC_CURRENT_DIAG) && FOC_CURRENT_DIAG
        record_current_diagnostic();
#endif
        return;
    }
    if (a == 0 || a == 4095 || b == 0 || b == 4095 ||
        foc_abs(I_a) > cfg::overcurrent || foc_abs(I_b) > cfg::overcurrent ||
        foc_abs(I_c) > cfg::overcurrent) { motor_fail(OVERCURRENT); return; }
    if (bus == 4095 || Udc_quant < cfg::min_bus || Udc_quant > cfg::max_bus) { motor_fail(BAD_BUS); return; }
    // Peak sampling / valley CCR preload: half of each of the two last commands.
    applied_alpha = Udc_quant * 0.5f * (prior_alpha + older_alpha);
    applied_beta = Udc_quant * 0.5f * (prior_beta + older_beta);
    const bool update_observer = motor_mode != MOTOR_BLDC && !detecting &&
        (motor_state == OPEN_LOOP || motor_state == RUN);
    if (update_observer && motor_state != FAULT) {
#if defined(STM32G431xx)
        const uint32_t start_cycles = DWT->CYCCNT;
#endif
        // 对齐 VESC 的增益调度。prior_alpha/beta 为上一条实际 PWM 电压与母线电压之比，
        // |duty| = sqrt(3) * |Ualpha_beta| / Udc（对应 VESC 的 overmod_factor = 1）。
        const float duty = 1.7320508075688772f *
            foc_sqrt(prior_alpha * prior_alpha + prior_beta * prior_beta);
        observer_config.gain = vesc_observer_gain(observer_gain_base, cfg::observer_gain_slow,
                                                   duty, Udc_quant);
        // 重建电压和同步采样的两相 Clarke 电流直接送入观测器，不增加低通或滤波补偿。
        // 每次执行观测器均更新 lambda_est，与 VESC 的 ORTEGA_LAMBDA_COMP 分支一致。
        if (!vesc_observer_update(observer_config, observer, applied_alpha, applied_beta,
                                  I_alpha, I_beta, sample_dt, true)) motor_fail(NUMERIC_ERROR);
        observer_phase = observer.angle;
#if defined(STM32G431xx)
        const uint32_t cycles = DWT->CYCCNT - start_cycles;
        if (cycles > observer_max_cycles) observer_max_cycles = cycles;
#endif
    }
    if (motor_mode == MOTOR_BLDC && !detecting && !bldc_feedback_ready()) motor_fail(BLDC_FEEDBACK_ERROR);
    if (motor_state != FAULT) control_sample();
#if defined(FOC_CURRENT_DIAG) && FOC_CURRENT_DIAG
    record_current_diagnostic();
#endif
}

void motor_control::adc_complete(ADC_HandleTypeDef *hadc)
{
#if defined(STM32G431xx)
    const uint32_t start = DWT->CYCCNT;
    isr_start_cycle = start;
#endif
    adc_sample(hadc);
#if defined(STM32G431xx)
    const uint32_t cycles = DWT->CYCCNT - start;
    if (cycles > isr_max_cycles) isr_max_cycles = cycles;
#endif
}

void motor_control::adc_error(ADC_HandleTypeDef *hadc)
{
    if (hadc == &hadc1 || hadc == &hadc2) motor_fail(ADC_ERROR);
}
