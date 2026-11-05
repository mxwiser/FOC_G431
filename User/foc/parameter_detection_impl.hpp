// Included only by setup.cpp inside its private namespace.
// FOC R/L/flux identification. BLDC never enters this identification flow.

void update_resistance()
{
    const float target = cfg::current * (detect_stage == 0 ? 0.5f : 1.0f);
    detect_voltage = clamp(detect_voltage + 20.0f * (target - I_alpha) * sample_dt, Udc_quant * 0.35f);
    output_voltage(detect_voltage, 0.0f);
    if (state_time > 2.0f) { sum_i += I_alpha; sum_v += applied_alpha; ++detect_samples; }
    if (state_time < 3.0f) return;
    const float mean_i = sum_i / detect_samples, mean_v = sum_v / detect_samples;
    if (detect_stage == 0) {
        low_i = mean_i; low_v = mean_v; detect_stage = 1; enter_state(DETECT_R);
    } else {
        // Two positive plateaus remove most of the constant bridge drop.
        motor_resistance = (mean_v - low_v) / (mean_i - low_i);
        detect_bias = mean_v - motor_resistance * mean_i;
        if (!foc_finite(motor_resistance) || motor_resistance <= 0.001f || motor_resistance >= 100.0f ||
            mean_i < cfg::current * 0.7f || mean_i - low_i < cfg::current * 0.25f) {
            motor_fail(DETECT_FAILED); return;
        }
        detect_stage = 0; enter_state(DETECT_L);
    }
}

void update_inductance()
{
    const float frequency = detect_stage == 0 ? 200.0f : 700.0f;
    const float omega = 2.0f * kPi * frequency;
    detect_phase = vesc_observer_wrap(detect_phase + omega * sample_dt);
    const float phase = detect_phase;
    if (state_time > 0.2f) {
        const float cs = std::cos(phase), sn = std::sin(phase);
        lockin_ic += I_alpha * cs; lockin_is += I_alpha * sn;
        lockin_vc += applied_alpha * cs; lockin_vs += applied_alpha * sn; ++detect_samples;
    }
    const float dc = motor_resistance * cfg::current * 0.75f + detect_bias;
    const float ac = motor_resistance * cfg::current * 0.375f;
    output_voltage(dc + ac * std::cos(phase), 0.0f);
    if (state_time < 1.2f) return;
    // Interval-average voltage -> current sample instant (sinusoidal steady state).
    const float half = omega * sample_dt * 0.5f, cs = std::cos(half), sn = std::sin(half);
    const float sinc = sn / half;
    const float vc = (lockin_vc * cs + lockin_vs * sn) / sinc;
    const float vs = (lockin_vs * cs - lockin_vc * sn) / sinc;
    const float current_sq = lockin_ic * lockin_ic + lockin_is * lockin_is;
    const float amplitude = 2.0f * foc_sqrt(current_sq) / detect_samples;
    const float inductance = (vc * lockin_is - vs * lockin_ic) / (current_sq * omega);
    const float ac_resistance = (vc * lockin_ic + vs * lockin_is) / current_sq;
    if (!foc_finite(inductance) || inductance <= 0.000001f || inductance >= 0.1f ||
        amplitude < 0.015f || foc_abs(ac_resistance / motor_resistance - 1.0f) > 0.35f) {
        motor_fail(DETECT_FAILED); return;
    }
    if (detect_stage == 0) {
        first_inductance = inductance; detect_stage = 1; enter_state(DETECT_L);
    } else {
        if (foc_abs(inductance / first_inductance - 1.0f) > 0.35f) { motor_fail(DETECT_FAILED); return; }
        motor_inductance = 0.5f * (inductance + first_inductance);
        configure_controllers(); openloop_angle = openloop_speed = 0.0f;
        detect_stage = 0; enter_state(ALIGN);
    }
}

void update_flux()
{
    openloop_angle = vesc_observer_wrap(openloop_angle + openloop_speed * sample_dt);
    const float phase_mid = openloop_angle - openloop_speed * sample_dt * 0.5f;
    const float ea = applied_alpha - motor_resistance * 0.5f * (I_alpha + last_I_alpha)
        - motor_inductance * (I_alpha - last_I_alpha) / sample_dt;
    const float eb = applied_beta - motor_resistance * 0.5f * (I_beta + last_I_beta)
        - motor_inductance * (I_beta - last_I_beta) / sample_dt;
    const float filter = sample_dt / (0.0005f + sample_dt);
    filtered_emf_a += filter * (ea - filtered_emf_a);
    filtered_emf_b += filter * (eb - filtered_emf_b);
    if (state_time > 0.5f) {
        const float cs = std::cos(phase_mid), sn = std::sin(phase_mid);
        emf_d += filtered_emf_a * cs + filtered_emf_b * sn;
        emf_q += -filtered_emf_a * sn + filtered_emf_b * cs;
        emf_abs += foc_sqrt(filtered_emf_a * filtered_emf_a + filtered_emf_b * filtered_emf_b);
        ++detect_samples;
    }
    current_control(openloop_angle, cfg::current, 0.0f);
    if (state_time < 1.5f) return;
    const float magnitude = foc_sqrt(emf_d * emf_d + emf_q * emf_q);
    const float lag = openloop_speed * 0.0005f;
    const float flux = magnitude * foc_sqrt(1.0f + lag * lag) / (detect_samples * foc_abs(openloop_speed));
    if (!foc_finite(flux) || flux <= 0.00001f || flux >= 1.0f ||
        magnitude < 0.7f * emf_abs || magnitude / detect_samples < 0.03f) {
        motor_fail(DETECT_FAILED); return;
    }
    if (detect_stage == 0) { first_flux = flux; detect_stage = 1; enter_state(OPEN_LOOP); }
    else {
        if (foc_abs(flux / first_flux - 1.0f) > 0.2f) { motor_fail(DETECT_FAILED); return; }
        motor_flux = 0.5f * (flux + first_flux); parameters_ready = model_valid();
        bridge_off(); coast_until = HAL_GetTick() + 2000u; enter_state(IDLE);
    }
}

