#pragma once
// Private implementation, included only by foc/motor_control_impl.hpp.

float clamp(float value, float limit)
{
    if (value > limit) return limit;
    if (value < -limit) return -limit;
    return value;
}

void bridge_off()
{
    __HAL_TIM_MOE_DISABLE_UNCONDITIONALLY(&htim1);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0u);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0u);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0u);
    prior_alpha = prior_beta = older_alpha = older_beta = 0.0f;
    iq_reference = 0.0f;
    iq_voltage_limit = requested_current_limit();
    speed_reference = speed_elapsed = 0.0f;
}

void motor_fail(MotorFault fault)
{
    run_requested = false;
    bridge_off();
    if (motor_fault == NO_FAULT) motor_fault = fault;
    motor_state = FAULT;
}

void enter_state(MotorState state)
{
    motor_state = state;
    state_time = stable_time = lost_time = 0.0f;
    state_samples = 0;
    if (state == ALIGN) { align_d = I_alpha; align_q = I_beta; }
    if (state == RUN && current_mode) {
        const float torque_current = cfg::direction * I_q;
        pi_speed.integral = torque_current < 0.0f ? 0.0f :
            (torque_current > cfg::current ? cfg::current : torque_current);
    }
    sum_i = sum_v = lockin_ic = lockin_is = lockin_vc = lockin_vs = 0.0f;
    emf_d = emf_q = emf_abs = 0.0f;
    filtered_emf_a = filtered_emf_b = 0.0f;
    detect_phase = 0.0f;
    detect_samples = 0;
}

void retry_motor(MotorFault reason)
{
    if (detecting || !run_requested) { motor_fail(reason); return; }
    bridge_off();
    restart_reason = reason;
    coast_until = HAL_GetTick() + cfg::retry_delay_ms;
    enter_state(RETRY_WAIT);
}

bool limits_valid()
{
    return foc_finite(cfg::current) && cfg::current >= 0.1f &&
        foc_finite(cfg::startup_current_scale) && cfg::startup_current_scale > 0.0f &&
        foc_finite(cfg::overcurrent) && cfg::overcurrent > cfg::current * 1.5f &&
        foc_finite(cfg::min_bus) && cfg::min_bus > 0.0f &&
        foc_finite(cfg::max_bus) && cfg::max_bus > cfg::min_bus &&
        (cfg::direction == 1.0f || cfg::direction == -1.0f);
}

bool model_valid()
{
    return foc_finite(motor_resistance) && motor_resistance > 0.001f && motor_resistance < 100.0f &&
        foc_finite(motor_inductance) && motor_inductance > 0.000001f && motor_inductance < 0.1f &&
        foc_finite(motor_flux) && motor_flux > 0.00001f && motor_flux < 1.0f;
}


#include "controllers_impl.hpp"
#include "drive_impl.hpp"
#include "parameter_detection_impl.hpp"
#include "../bldc/drive_impl.hpp"

void control_sample()
{
    state_time = ++state_samples * sample_dt;
    if (motor_state == BOOTSTRAP) {
        if (state_time >= 0.005f) {
            if (detecting) { detect_voltage = 0.0f; detect_stage = 0; enter_state(DETECT_R); }
            else {
                configure_controllers(); openloop_angle = openloop_speed = 0.0f;
                bldc::reset(bldc_state);
                enter_state(OPEN_LOOP);
            }
        }
        return;
    }
    if (motor_state == DETECT_R) { update_resistance(); return; }
    if (motor_state == DETECT_L) { update_inductance(); return; }
    if (motor_state == DETECT_FLUX) { update_flux(); return; }
    if (!detecting && (motor_state == OPEN_LOOP || motor_state == RUN)) {
        if (motor_mode == MOTOR_BLDC) bldc_drive_sample();
        else sensorless_drive_sample();
        return;
    }
    if (motor_state == ALIGN) {
        // 仅参数辨识流程使用 Id 对齐；正常启动已在 BOOTSTRAP 后直接进入 Iq 驱动。
        const float fraction = state_time < 0.3f ? state_time / 0.3f : 1.0f;
        current_control(0.0f, cfg::current * fraction, 0.0f);
        const float filter = sample_dt / (0.01f + sample_dt);
        align_d += filter * (I_d - align_d); align_q += filter * (I_q - align_q);
        if (state_time > 0.5f && foc_abs(align_d - cfg::current) < cfg::current * 0.3f &&
            foc_abs(align_q) < cfg::current * 0.3f) stable_time += sample_dt;
        else stable_time = 0.0f;
        if (state_time >= cfg::align_seconds) {
            if (stable_time < 0.15f) {
                if (state_time >= cfg::align_seconds + cfg::align_settle_grace) retry_motor(ALIGN_FAILED);
                return;
            }
            vesc_observer_reset(observer_config, observer, 0.0f);
            if (observer_config.type == VESC_OBSERVER_ORTEGA_LAMBDA_COMP) {
                observer.x1 += motor_inductance * I_alpha; observer.x2 += motor_inductance * I_beta;
            }
            observer_phase = 0.0f;
            enter_state(OPEN_LOOP);
        }
        return;
    }
    if (motor_state == OPEN_LOOP) {
        const float destination = cfg::direction * cfg::openloop_rad_s *
            (detecting && detect_stage == 0 ? 0.7f : 1.0f);
        const float step = cfg::openloop_rad_s * sample_dt / cfg::ramp_seconds;
        openloop_speed += clamp(destination - openloop_speed, step);
        openloop_angle = vesc_observer_wrap(openloop_angle + openloop_speed * sample_dt);
        current_control(openloop_angle, cfg::current, 0.0f);
        if (foc_abs(openloop_speed - destination) < 0.01f) enter_state(DETECT_FLUX);
        return;
    }
    motor_fail(BAD_CONFIG);
}

bool start_motor(bool detect)
{
    if (motor_state != IDLE || !limits_valid() || (!detect && !parameters_ready)) return false;
    if (!detect && current_mode && iq_setpoint <= 0.0f) return false;
    if (!detect && !current_mode && speed_setpoint <= 0.0f) return false;
    if (motor_mode == MOTOR_BLDC && (detect || !bldc_feedback_ready())) return false;
    if (static_cast<int32_t>(HAL_GetTick() - coast_until) < 0) return false;
    if (Udc_quant < cfg::min_bus || Udc_quant > cfg::max_bus) { motor_fail(BAD_BUS); return false; }
    const uint32_t mask = __get_PRIMASK(); __disable_irq();
    detecting = detect;
    run_requested = !detect;
    if (detect) parameters_ready = false;
    bridge_off();
    motor_timer_foc(); // Bootstrap all three low sides before either drive starts.
    __HAL_TIM_SET_COUNTER(&htim1, 0u);
    if (HAL_TIM_GenerateEvent(&htim1, TIM_EVENTSOURCE_UPDATE) != HAL_OK) {
        motor_fail(PWM_ERROR); __set_PRIMASK(mask); return false;
    }
    enter_state(BOOTSTRAP);
    if (!bridge_started) {
        if (HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1) != HAL_OK ||
            HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1) != HAL_OK ||
            HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2) != HAL_OK ||
            HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2) != HAL_OK ||
            HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3) != HAL_OK ||
            HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3) != HAL_OK) motor_fail(PWM_ERROR);
        else bridge_started = true;
    } else __HAL_TIM_MOE_ENABLE(&htim1);
    __set_PRIMASK(mask);
    return motor_state == BOOTSTRAP;
}

void service_restart()
{
    if (motor_state != RETRY_WAIT || !run_requested || motor_fault != NO_FAULT ||
        static_cast<int32_t>(HAL_GetTick() - coast_until) < 0) return;
    const uint32_t mask = __get_PRIMASK(); __disable_irq();
    enter_state(IDLE);
    if (start_motor(false)) ++retry_attempts;
    __set_PRIMASK(mask);
}
