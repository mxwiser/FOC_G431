#pragma once
// Private implementation, included only by foc/motor_control_impl.hpp.

void configure_controllers()
{
    PID cleared = {};
    pi_d = pi_q = cleared;
    pi_speed = cleared;
    pi_speed.dt = sample_dt;
    pi_speed.kp = cfg::speed_kp;
    pi_speed.ki = cfg::speed_ki;
    pi_speed.output_limit = speed_current_limit;
    pi_d.dt = pi_q.dt = sample_dt;
    const float bandwidth = detecting ? cfg::current_bandwidth : cfg::run_current_bandwidth;
    pi_d.kp = pi_q.kp = motor_inductance * bandwidth;
    pi_d.ki = pi_q.ki = motor_resistance * bandwidth;
    observer_config.resistance = motor_resistance;
    observer_config.inductance = motor_inductance;
    observer_config.flux_linkage = motor_flux;
    // VESC detection uses gamma = 500 / lambda^2.
    observer_gain_base = motor_flux > 0.0f ? 500.0f / (motor_flux * motor_flux) : 0.0f;
    observer_config.gain = observer_gain_base * cfg::observer_gain_slow * 4.0f;
    observer_config.pll_kp = 1000.0f;
    observer_config.pll_ki = 250000.0f;
    observer_config.minimum_speed = cfg::minimum_rad_s;
    observer_config.type = motor_observer_type();
    vesc_observer_reset(observer_config, observer, 0.0f);
    observer_phase = 0.0f;
    VescStartupState cleared_startup = {};
    startup = cleared_startup;
    startup_elapsed = startup_iq_filtered = 0.0f;
    startup_config.openloop_speed = cfg::startup_openloop_rad_s;
    startup_config.openloop_low = cfg::startup_openloop_low;
    startup_config.hysteresis_seconds = cfg::startup_hysteresis_seconds;
    startup_config.lock_seconds = cfg::startup_lock_seconds;
    startup_config.ramp_seconds = cfg::startup_ramp_seconds;
    startup_config.hold_seconds = cfg::startup_hold_seconds;
    startup_config.flux_linkage = motor_flux;
    speed_reference = speed_feedback = speed_elapsed = 0.0f;
    speed_voltage_limited = false;
}

void track_speed_integral()
{
    // Preserve delivered load torque, rather than an unattainable current
    // request. Never turn a negative braking integral into positive torque.
    const float limit = speed_current_limit < iq_voltage_limit ? speed_current_limit : iq_voltage_limit;
    float delivered = clamp(cfg::direction * I_q, limit);
    if (delivered < 0.0f) delivered = 0.0f;
    if (pi_speed.integral > delivered) pi_speed.integral = delivered;
}

void update_iq_voltage_limit()
{
    const float maximum = Udc_quant * cfg::max_modulation * ONE_BY_SQRT3;
    const float emf = foc_abs(observer.speed) * motor_flux;
    const float range = maximum * (cfg::iq_voltage_derate_end - cfg::iq_voltage_derate_start);
    float fraction = (maximum * cfg::iq_voltage_derate_end - emf) / range;
    if (fraction < 0.0f) fraction = 0.0f;
    const float requested = requested_current_limit();
    const float scale = requested > cfg::iq_voltage_derate_scale ? requested : cfg::iq_voltage_derate_scale;
    // Follow the requested current ceiling without a fixed ampere cap. Keep
    // voltage-headroom derating; bound the fraction before multiplying.
    const float full_request = requested / scale;
    if (fraction > full_request) fraction = full_request;
    iq_voltage_limit = fraction * scale;
}

float speed_control_reference()
{
    const float filter = sample_dt / (cfg::speed_filter_seconds + sample_dt);
    speed_feedback += filter * (cfg::direction * motor_speed() - speed_feedback);
    speed_elapsed += sample_dt;
    const float limit = speed_current_limit < iq_voltage_limit ? speed_current_limit : iq_voltage_limit;
    if (speed_elapsed >= cfg::speed_loop_seconds) {
        const float elapsed = speed_elapsed;
        speed_elapsed = 0.0f;
        const bool voltage_limited = speed_voltage_limited;
        speed_voltage_limited = false;
        const float rpm_to_electrical = 2.0f * kPi * FOC_POLE_PAIRS / 60.0f;
        // Use the slower end of the move, so a step to or from low speed
        // cannot bypass the low-speed ramp limit.
        const float reference_rpm = speed_reference / rpm_to_electrical;
        const float ramp_speed_rpm = reference_rpm < speed_setpoint ? reference_rpm : speed_setpoint;
        const float ramp_fraction = clamp((ramp_speed_rpm - cfg::speed_ramp_fast_start_rpm) /
            (cfg::speed_ramp_fast_full_rpm - cfg::speed_ramp_fast_start_rpm), 1.0f);
        const float ramp = cfg::speed_ramp_rpm_per_second +
            (cfg::speed_ramp_fast_rpm_per_second - cfg::speed_ramp_rpm_per_second) *
            (ramp_fraction > 0.0f ? ramp_fraction : 0.0f);
        speed_reference += clamp(speed_setpoint * rpm_to_electrical - speed_reference,
            ramp * rpm_to_electrical * elapsed);
        pi_speed.dt = elapsed;
        // ILIM bounds both motoring and braking. The back-EMF headroom
        // derating limits positive torque; it must not suppress braking.
        pi_speed.output_limit = speed_current_limit;
        pi_speed.input = speed_reference - speed_feedback;
        if (pi_speed.input > 0.0f && (voltage_limited || pi_speed.integral > limit))
            track_speed_integral();
        const float previous_integral = clamp(pi_speed.integral, speed_current_limit);
        calculate_PI(&pi_speed);
        // Keep PI anti-windup at the asymmetric positive voltage limit,
        // while retaining negative integral/output for regenerative torque.
        if ((pi_speed.output > limit || voltage_limited) && pi_speed.input > 0.0f) {
            pi_speed.integral = previous_integral;
            pi_speed.output = clamp(pi_speed.kp * pi_speed.input + pi_speed.integral,
                                    speed_current_limit);
        }
        if (pi_speed.output > limit) pi_speed.output = limit;
    }
    // The current loop remains at the ADC rate. Clamp rapidly if headroom shrinks.
    float reference = cfg::direction * iq_reference;
    const float target = pi_speed.output < limit ? pi_speed.output : limit;
    reference += clamp(target-reference,cfg::speed_iq_slew_per_second * sample_dt);
    if (reference > limit) reference = limit;
    if (reference < -speed_current_limit) reference = -speed_current_limit;
    return cfg::direction * reference;
}
