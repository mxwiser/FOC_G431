// Included only by setup.cpp inside its private namespace.
// FOC voltage/current loops and sensorless startup; both FOC observers use this path.

// SI volts -> VESC modulation -> timer ticks. Save quantized voltage / Udc.
void output_voltage(float U_alpha, float U_beta, bool vector_limited = false)
{
#if defined(STM32G431xx)
    const uint32_t start_cycles = DWT->CYCCNT;
#endif
    if (!foc_finite(U_alpha) || !foc_finite(U_beta) || !(Udc_quant > 0.0f)) {
        motor_fail(NUMERIC_ERROR); return;
    }
    const float maximum = Udc_quant * cfg::max_modulation * ONE_BY_SQRT3;
    if (!vector_limited) {
        const float magnitude = foc_sqrt(U_alpha * U_alpha + U_beta * U_beta);
        if (magnitude > maximum) { U_alpha *= maximum / magnitude; U_beta *= maximum / magnitude; }
    }
    uint32_t tA, tB, tC;
    const uint32_t period = __HAL_TIM_GET_AUTORELOAD(&htim1);
    svpwm_fast(1.5f * U_alpha / Udc_quant, 1.5f * U_beta / Udc_quant,
            cfg::max_modulation, period, &tA, &tB, &tC);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, tA);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, tB);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, tC);
#if defined(STM32G431xx)
    const uint32_t write_cycles = DWT->CYCCNT - isr_start_cycle;
    if (write_cycles > pwm_write_max_cycles) pwm_write_max_cycles = write_cycles;
#endif
    older_alpha = prior_alpha; older_beta = prior_beta;
    prior_alpha = (2.0f * tA - tB - tC) / (3.0f * period);
    prior_beta = (static_cast<float>(tB) - tC) * ONE_BY_SQRT3 / period;
#if defined(STM32G431xx)
    const uint32_t cycles = DWT->CYCCNT - start_cycles;
    if (cycles > pwm_max_cycles) pwm_max_cycles = cycles;
#endif
}

void current_control(float angle, float target_d, float target_q)
{
#if defined(STM32G431xx)
    const uint32_t start_cycles = DWT->CYCCNT;
#endif
    electrical_angle = angle;
    iq_reference = target_q;
    float sn, cs;
    foc_fast_sincos(angle, sn, cs);
    I_d = I_alpha * cs + I_beta * sn;
    I_q = -I_alpha * sn + I_beta * cs;
    // 与 VESC 的 iq_filter 一致，仅用于低速辅助管理，观测器仍使用未滤波的 alpha/beta 电流。
    startup_iq_filtered += 0.1f * (I_q - startup_iq_filtered);
    const float maximum = Udc_quant * cfg::max_modulation * ONE_BY_SQRT3;
    pi_d.output_limit = pi_q.output_limit = maximum;
    pi_d.input = target_d - I_d; pi_q.input = target_q - I_q;
    calculate_PI(&pi_d); calculate_PI(&pi_q);
    float U_d = pi_d.output, U_q = pi_q.output;
    if (!detecting && motor_state != ALIGN && motor_flux > 0.0f) {
        // VESC-style cross-coupling and back-EMF feedforward. Important when
        // R and L are small: PI alone reacts too late during acceleration.
        float speed = motor_state == OPEN_LOOP && !detecting && !startup.phase_override
            ? observer.speed : openloop_speed;
        if (motor_state == RUN) speed = observer.speed;
        const float difference = motor_state == OPEN_LOOP ? 0.0f :
            vesc_observer_wrap(observer_phase + observer.speed * sample_dt * 0.5f - angle);
        float difference_sin, difference_cos;
        foc_fast_sincos(difference, difference_sin, difference_cos);
        U_d += -speed * motor_inductance * I_q - speed * motor_flux * difference_sin;
        U_q += speed * motor_inductance * I_d + speed * motor_flux * difference_cos;
    }
    // Preserve d-axis regulation as back EMF consumes the voltage budget.
    // Uniform vector scaling also weakens Ud, allowing uncontrolled Id.
    const float limited_d = clamp(U_d, maximum);
    const float q_budget_sq = maximum * maximum - limited_d * limited_d;
    const float limited_q = clamp(U_q, foc_sqrt(q_budget_sq > 0.0f ? q_budget_sq : 0.0f));
    pi_d.integral += limited_d - U_d;
    pi_q.integral += limited_q - U_q;
    U_d = limited_d; U_q = limited_q;
    output_voltage(U_d * cs - U_q * sn, U_d * sn + U_q * cs, true);
#if defined(STM32G431xx)
    const uint32_t cycles = DWT->CYCCNT - start_cycles;
    if (cycles > current_max_cycles) current_max_cycles = cycles;
#endif
}

void sensorless_drive_sample()
{
    // VESC 的低速辅助按 1 kHz 运行；电流环与观测器仍按 ADC 采样频率执行。
    startup_elapsed += sample_dt;
    if (startup_elapsed >= cfg::startup_manager_seconds) {
        float sn, cs;
        foc_fast_sincos(electrical_angle, sn, cs);
        const float q_voltage = prior_beta * cs - prior_alpha * sn;
        const float duty_magnitude = 1.7320508075688772f *
            foc_sqrt(prior_alpha * prior_alpha + prior_beta * prior_beta);
        const float duty = q_voltage < 0.0f ? -duty_magnitude : duty_magnitude;
        const float maximum_current = current_mode ? cfg::max_iq : speed_current_limit;
        vesc_startup_update(startup_config, startup, observer_phase, observer.speed,
                            startup_iq_filtered, maximum_current, duty, true, cfg::startup_manager_seconds);
        startup_elapsed -= cfg::startup_manager_seconds;
    }

    float phase = vesc_observer_wrap(observer_phase + observer.speed * sample_dt * 0.5f);
    if (startup.phase_override) {
        // 与 VESC 的 FOC_SENSOR_MODE_SENSORLESS 分支一致：强制相位驱动，同时预置观测器磁链。
        phase = startup.phase;
        observer.x1 = startup.x1;
        observer.x2 = startup.x2;
    }
    openloop_angle = startup.phase;
    openloop_speed = startup.forced_speed;
    update_iq_voltage_limit();
    float reference;
    if (current_mode) {
        const float allowed = iq_setpoint < iq_voltage_limit ? iq_setpoint : iq_voltage_limit;
        reference = iq_reference + clamp(cfg::direction * allowed - iq_reference,
                                            cfg::iq_slew_per_second * sample_dt);
        if (cfg::direction * reference > iq_voltage_limit) reference = cfg::direction * iq_voltage_limit;
    } else reference = speed_control_reference();
    // VESC 无感启动和运行都使用 Iq 产生转矩，Id 目标保持为零。
    // 开环辅助定时结束便直接使用观测器相位；速度环不再等待原先连续 0.2 秒的接管门槛。
    current_control(phase, 0.0f, reference);

    const bool quality = !startup.phase_override && observer.ready &&
        cfg::direction * observer.speed > cfg::minimum_rad_s && foc_abs(observer.speed) * sample_dt < 0.8f;
    if (motor_state == OPEN_LOOP) {
        // 以下仅确认启动成功并保留 G431 原有超时/冷却保护，不阻塞上述 Iq/速度闭环控制。
        // 使用累加/衰减的滞回，单个采样抖动不会清空已经取得的稳定时间。
        if (quality) stable_time += sample_dt;
        else if (stable_time > 0.0f) stable_time -= sample_dt;
        if (stable_time >= cfg::lock_seconds) { enter_state(RUN); return; }
        if (state_time >= cfg::startup_timeout) retry_motor(STARTUP_TIMEOUT);
    } else {
        if (quality) lost_time = 0.0f; else lost_time += sample_dt;
        if (lost_time >= cfg::lost_seconds) retry_motor(LOST_LOCK);
    }
}

