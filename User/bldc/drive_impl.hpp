// Included only by foc/motor_control_impl.hpp inside its private namespace.
// Shared protection and speed PI live in foc/state_machine_impl.hpp; all six-step work is here.
void bldc_drive_sample()
{
    if (!bldc_feedback_ready()) { motor_fail(BLDC_FEEDBACK_ERROR); return; }
    iq_voltage_limit = requested_current_limit();
    if (current_mode) {
        iq_reference += clamp(cfg::direction * iq_setpoint - iq_reference,
                              cfg::iq_slew_per_second * sample_dt);
    } else iq_reference = speed_control_reference();
    const bldc::Config config = {motor_resistance, motor_inductance, motor_flux,
        cfg::max_modulation, cfg::startup_timeout, cfg::direction > 0.0f ? 1 : -1};
    const float currents[3] = {I_a, I_b, I_c};
    if (!bldc::update(config, bldc_state, bldc_phase_voltage, currents, Udc_quant,
                      cfg::direction * iq_reference, sample_dt)) {
        retry_motor(bldc_state.locked ? LOST_LOCK : STARTUP_TIMEOUT); return;
    }
    I_d = 0.0f; // No d-axis exists in this mode; iq is the signed line current.
    I_q = cfg::direction * bldc_state.current;
    electrical_angle = bldc_state.sector * bldc::sector_radians;
    motor_timer_bldc(bldc_state.sector, bldc_state.duty);
    if (bldc_state.locked && motor_state == OPEN_LOOP) enter_state(RUN);
}
