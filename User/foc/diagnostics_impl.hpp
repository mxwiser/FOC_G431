#pragma once
// Private implementation, included only by foc/motor_control_impl.hpp.

#if defined(FOC_CURRENT_DIAG) && FOC_CURRENT_DIAG
void record_current_diagnostic(uint16_t raw_a, uint16_t raw_b, uint16_t raw_bus)
{
    const bool timeout = diag_frame_deadline &&
        static_cast<int32_t>(sample_frames - diag_frame_deadline) >= 0;
    if (diag_mode && !diag_frozen && motor_state == diag_state &&
        state_samples >= diag_min_samples && foc_abs(observer.speed) >= diag_min_speed &&
        diag_stride && sample_frames % diag_stride == 0) {
        CurrentDiagnosticSample &r = current_diag[diag_count % 192u];
        r.frame = sample_frames; r.state = motor_state;
        r.raw_a = raw_a; r.raw_b = raw_b;
        r.raw_c = 0xffffu; // No C-phase ADC sample; r.ic contains the A/B reconstruction.
        r.raw_bus = raw_bus;
        r.ia = I_a; r.ib = I_b; r.ic = I_c; r.id = I_d; r.iq = I_q;
        r.iqref = iq_reference; r.phase = electrical_angle;
        r.observer_phase = observer_phase; r.speed = observer.speed;
        r.flux = observer.flux_magnitude; r.bus = Udc_quant;
        r.va = applied_alpha; r.vb = applied_beta;
        r.command_va = prior_alpha * Udc_quant; r.command_vb = prior_beta * Udc_quant;
        r.integral_d = pi_d.integral; r.integral_q = pi_q.integral;
        r.observer_ia = I_alpha; r.observer_ib = I_beta;
        r.phase_error = observer.phase_error;
        r.raw_angle = observer.angle; r.pll_angle = observer.pll_angle;
        r.ccr_a = __HAL_TIM_GET_COMPARE(&htim1, TIM_CHANNEL_1);
        r.ccr_b = __HAL_TIM_GET_COMPARE(&htim1, TIM_CHANNEL_2);
        r.ccr_c = __HAL_TIM_GET_COMPARE(&htim1, TIM_CHANNEL_3);
        ++diag_count;
        if ((diag_mode == 1u && diag_count >= 192u) ||
            (diag_mode == 2u && (foc_abs(I_a) > diag_trigger_current ||
             foc_abs(I_b) > diag_trigger_current || foc_abs(I_c) > diag_trigger_current))) {
            diag_frozen = 1;
            if (diag_mode == 2u) {
                run_requested = false; bridge_off();
                coast_until = HAL_GetTick() + 2000u; enter_state(IDLE);
            }
            diag_mode = 0;
        }
    }
    if (timeout) {
        diag_frame_deadline = 0; diag_mode = 0; diag_frozen = 1;
        run_requested = false; bridge_off();
        coast_until = HAL_GetTick() + 2000u; enter_state(IDLE);
    }
}
#endif
