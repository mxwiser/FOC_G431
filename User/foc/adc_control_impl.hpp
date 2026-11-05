#pragma once
// Private implementation, included only by foc/motor_control_impl.hpp.

// All values here are supplied as physical SI quantities. No ADC/HAL handles.
static void control_input_frame()
{
    ++sample_frames;
    const ControlInputFrame frame = input_frame;
    input_frame.supplied = 0u;
    if (frame.supplied != 7u) { motor_fail(ADC_ERROR); return; }
    if (!foc_finite(frame.ia) || !foc_finite(frame.ib) || !foc_finite(frame.bus)) {
        motor_fail(NUMERIC_ERROR); return;
    }
    Udc_quant = frame.bus;
    bldc_publish_phase_voltages(frame.vu, frame.vv, frame.vw);
    if (motor_state == FAULT) { bridge_off(); return; }
    if (motor_state == ZERO) return;
    I_a = frame.ia; I_b = frame.ib; I_c = -I_a - I_b;
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
    if (foc_abs(I_a) > cfg::overcurrent || foc_abs(I_b) > cfg::overcurrent ||
        foc_abs(I_c) > cfg::overcurrent) { motor_fail(OVERCURRENT); return; }
    if (Udc_quant < cfg::min_bus || Udc_quant > cfg::max_bus) { motor_fail(BAD_BUS); return; }
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

void motor_control::set_phase_currents(float ia, float ib)
{
    input_frame.ia = ia; input_frame.ib = ib; input_frame.supplied |= 1u;
}

void motor_control::set_phase_voltages(float vu, float vv, float vw)
{
    input_frame.vu = vu; input_frame.vv = vv; input_frame.vw = vw;
    input_frame.supplied |= 2u;
}

void motor_control::set_bus_voltage(float volts)
{
    input_frame.bus = volts; input_frame.supplied |= 4u;
}

void motor_control::set_encoder_angle(float radians)
{
    encoder_angle_available = foc_finite(radians);
    if (encoder_angle_available) encoder_electrical_angle = vesc_observer_wrap(radians);
}

void motor_control::adc_complete() { control_input_frame(); }
void motor_control::adc_error() { motor_fail(ADC_ERROR); }
bool motor_control::calibrating() { return motor_state == ZERO; }
void motor_control::calibration_complete()
{
    if (motor_state == ZERO) enter_state(IDLE);
}
void motor_control::input_error(InputError error)
{
    motor_fail(error == BAD_CURRENT_ZERO ? BAD_ZERO : OVERCURRENT);
}
