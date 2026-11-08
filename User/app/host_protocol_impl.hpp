#pragma once
// Private implementation, included only by foc/motor_control_impl.hpp.

int16_t phase_milliamps(float value)
{
    value *= 1000.0f;
    if (value > 32767.0f) return 32767;
    if (value < -32768.0f) return -32768;
    return static_cast<int16_t>(value);
}

void capture_phase_sample()
{
    const uint8_t selected = phase_telemetry_mask;
    if (!selected || sample_frames % kPhaseStride) return;
    const uint8_t write = phase_write;
    const uint8_t next = (write + 1u) & 127u;
    if (next == phase_read) { ++phase_dropped; return; }
    PhaseSample &sample = phase_samples[write];
    sample.frame = sample_frames;
    sample.a = (selected & 1u) ? phase_milliamps(I_a) : 0;
    sample.b = (selected & 2u) ? phase_milliamps(I_b) : 0;
    sample.c = (selected & 4u) ? phase_milliamps(I_c) : 0;
    data_port::memory_barrier();
    phase_write = next;
}
void service_command(uint8_t command)
{
    if (command == 's' || command == 'S') {
        const uint32_t mask = data_port::enter_critical();
        run_requested = false;
        restart_reason = NO_FAULT;
        bridge_off();
        // Stop does not clear latched faults. Reset is required for that.
        if (motor_fault == NO_FAULT && motor_state != ZERO) {
            coast_until = data_port::milliseconds() + 2000u; enter_state(IDLE);
        }
        data_port::leave_critical(mask);
        command_result = COMMAND_OK;
    } else if (command == 'd' || command == 'D' || command == 'r' || command == 'R') {
        if (motor_mode == MOTOR_BLDC && (command == 'd' || command == 'D')) {
            command_result = COMMAND_UNSUPPORTED; return;
        }
        if (motor_mode == MOTOR_BLDC && !bldc_feedback_ready()) {
            command_result = COMMAND_FEEDBACK; return;
        }
        if (start_motor(command == 'd' || command == 'D')) {
            retry_attempts = 0;
            restart_reason = NO_FAULT;
            command_result = COMMAND_OK;
        } else command_result = COMMAND_BUSY;
    }
}

void service_serial()
{
    data_port::ReceiveState &rx = data_port::receive_state();
recover_uart:
    if (rx.overflow || rx.error) {
        const uint32_t mask = data_port::enter_critical();
        rx.read = rx.write;
        rx.overflow = rx.error = false;
        command_parser.length = 0;
        command_parser.discarding = true;
        data_port::leave_critical(mask);
        service_command('S');
        command_result = COMMAND_RX;
        rx.read = rx.write;
        if (!data_port::recover_receive()) { rx.error = true; return; }
    }
    while (!rx.overflow && !rx.error && rx.read != rx.write) {
        const uint8_t byte = rx.data[rx.read];
        rx.read = static_cast<uint8_t>(rx.read + 1u);
        SerialCommand command;
        if (!serial_command_feed(command_parser, byte, command)) continue;
        // Do not execute an already-queued command after an RX fault.  ADC
        // remains protected by the short bounded command critical section.
        const uint32_t command_mask = data_port::enter_critical();
        if (rx.overflow || rx.error) { data_port::leave_critical(command_mask); break; }
        if (command.type == SERIAL_STOP) service_command('S');
        else if (command.type == SERIAL_START) service_command('R');
        else if (command.type == SERIAL_DETECT) service_command('D');
        else if (command.type == SERIAL_MUSIC_STATUS) {
            music_status_pending = true;
            if (music_requested) music_last_command_ms = data_port::milliseconds();
        }
        else if (command.type == SERIAL_MUSIC_NOTE) {
            music_tone::Note note;
            if (!music_tone::make_note(command.value, command.amplitude, command.duration_ms, sample_dt, note))
                command_result = COMMAND_RANGE;
            else if (motor_mode == MOTOR_BLDC) command_result = COMMAND_UNSUPPORTED;
            else {
                music_player.live = false;
                if ((!music_requested && !start_motor(false, true)) ||
                    !music_tone::enqueue(music_player, note)) command_result = COMMAND_BUSY;
                else { command_result = COMMAND_OK; music_last_command_ms = data_port::milliseconds(); }
            }
            music_command_result = command_result;
            music_status_pending = true;
        }
        else if (command.type == SERIAL_MUSIC_LIVE) {
            if (!music_tone::valid_live(command.value, command.amplitude, sample_dt))
                command_result = COMMAND_RANGE;
            else if (motor_mode == MOTOR_BLDC) command_result = COMMAND_UNSUPPORTED;
            else if (!music_requested && command.amplitude <= 0.0f) command_result = COMMAND_OK;
            else if (!music_requested && !start_motor(false, true)) command_result = COMMAND_BUSY;
            else {
                music_tone::set_live(music_player, command.value, command.amplitude);
                command_result = COMMAND_OK;
                music_last_command_ms = data_port::milliseconds();
            }
            music_command_result = command_result;
            music_status_pending = true;
        }
        else if (command.type == SERIAL_SET_MOTOR_MODE) {
            if (!foc_finite(command.value) || command.value < 0.0f || command.value > 2.0f ||
                command.value != static_cast<float>(static_cast<unsigned>(command.value))) {
                command_result = COMMAND_RANGE;
            } else if (motor_state != IDLE || run_requested || motor_fault != NO_FAULT) {
                command_result = COMMAND_BUSY;
            } else {
                bridge_off();
                motor_timer_foc();
                motor_mode = static_cast<MotorMode>(static_cast<unsigned>(command.value));
                detecting = false;
                bldc::reset(bldc_state);
                configure_controllers();
                command_result = COMMAND_OK;
            }
        }
        else if (command.type == SERIAL_SET_PHASES) {
            if (foc_finite(command.value) && command.value >= 0.0f && command.value <= 7.0f) {
                const uint8_t phases = static_cast<uint8_t>(command.value);
                if (command.value == static_cast<float>(phases) && phases != phase_telemetry_mask) {
                    const uint32_t mask = data_port::enter_critical();
                    phase_read = phase_write; // discard samples from the previous selection
                    phase_telemetry_mask = phases;
                    data_port::leave_critical(mask);
                }
            }
        }
        else if (command.type == SERIAL_SET_IQ || command.type == SERIAL_SET_SPEED ||
                 command.type == SERIAL_SET_SPEED_LIMIT) {
            const bool speed = command.type == SERIAL_SET_SPEED;
            const bool limit = command.type == SERIAL_SET_SPEED_LIMIT;
            const bool range_ok = speed ? (command.value == 0.0f ||
                command.value >= cfg::min_speed_rpm) : command.value >= (limit ? 0.1f : 0.0f);
            if (!foc_finite(command.value) || !range_ok) {
                command_result = COMMAND_RANGE;
            } else if (motor_state == FAULT || (detecting && motor_state != IDLE && motor_state != ZERO) ||
                       (music_requested && !((command.type == SERIAL_SET_IQ || speed) && command.value == 0.0f))) {
                command_result = COMMAND_BUSY;
            } else {
                const uint32_t mask = data_port::enter_critical();
                if (limit) speed_current_limit = command.value;
                else if (speed) {
                    if (current_mode) {
                        speed_reference = speed_feedback = cfg::direction * motor_speed();
                        speed_elapsed = 0.0f;
                        pi_speed.integral = pi_speed.output = cfg::direction * iq_reference;
                        speed_voltage_limited = false;
                    } else if (command.value < speed_setpoint) {
                        const float measured = cfg::direction * motor_speed();
                        const float target = command.value * (2.0f * kPi * FOC_POLE_PAIRS / 60.0f);
                        const float ceiling = measured > target ? measured : target;
                        // An unreachable old command must not keep the ramp
                        // above both the real speed and the new lower target.
                        if (speed_reference > ceiling) {
                            speed_reference = ceiling;
                            speed_feedback = measured;
                            speed_elapsed = 0.0f;
                            track_speed_integral();
                            pi_speed.input = speed_reference - speed_feedback;
                            pi_speed.output = clamp(pi_speed.kp * pi_speed.input + pi_speed.integral,
                                                    speed_current_limit);
                            if (pi_speed.output > iq_voltage_limit) pi_speed.output = iq_voltage_limit;
                        }
                    }
                    speed_setpoint = command.value;
                    current_mode = false;
                    if (speed_setpoint == 0.0f) service_command('S');
                } else {
                    iq_setpoint = command.value;
                    current_mode = true;
                    if (iq_setpoint == 0.0f) service_command('S');
                }
                data_port::leave_critical(mask);
                command_result = COMMAND_OK;
            }
        } else command_result = COMMAND_FORMAT;
        data_port::leave_critical(command_mask);
    }
    if (rx.overflow || rx.error) goto recover_uart;
}

bool send_music_status(uint32_t now)
{
    if ((!music_status_pending && (!music_requested || now - service_last_music < 20u)) ||
        !data_port::tx_ready()) return false;
    static TelemetryText text;
    const uint32_t mask = data_port::enter_critical();
    const uint32_t ack = music_player.accepted, done = music_player.completed;
    const unsigned free = music_tone::capacity - music_player.count;
    const bool active = music_requested;
    const MotorState state = motor_state;
    const MotorFault fault = motor_fault;
    const CommandResult result = music_command_result;
    music_status_pending = false;
    data_port::leave_critical(mask);
    text.reset("MUS"); text.integer("v", 1u); text.integer("ack", ack);
    text.integer("done", done); text.integer("free", free); text.integer("active", active ? 1u : 0u);
    text.integer("cmd", result); text.integer("s", state); text.integer("f", fault);
    text.finish();
    if (!text.valid || !data_port::write(reinterpret_cast<const uint8_t *>(text.data),
                                       static_cast<uint16_t>(text.length))) {
        music_status_pending = true; return false;
    }
    service_last_music = now;
    return true;
}

bool send_status()
{
    if (!data_port::tx_ready()) return false;
    static TelemetryText text; // Persistent until the interrupt-driven TX completes.
    const uint32_t mask = data_port::enter_critical();
    const MotorState state = motor_state; const MotorFault fault = motor_fault;
    const bool inactive = state == IDLE || state == ZERO || state == RETRY_WAIT;
    const float bus = Udc_quant, id = inactive ? 0.0f : I_d, iq = inactive ? 0.0f : I_q;
    const float speed = inactive || music_requested ? 0.0f : motor_speed();
    const float r = motor_resistance, l = motor_inductance, flux = motor_flux, ia = I_alpha;
    // Same ADC frame, before Clarke/Park or observer filtering. Keep the
    // legacy ia=Ialpha field for existing serial clients; i_a/i_b/i_c are phases.
    const uint8_t phases = phase_telemetry_mask;
    const float phase_a = (phases & 1u) ? I_a : 0.0f;
    const float phase_b = (phases & 2u) ? I_b : 0.0f;
    const float phase_c = (phases & 4u) ? I_c : 0.0f;
    const float angle = electrical_angle;
    const uint32_t cycles = isr_max_cycles, write_cycles = pwm_write_max_cycles;
    const bool ready = parameters_ready;
    const bool locked = !inactive && !music_requested && motor_fault == NO_FAULT &&
        (motor_mode == MOTOR_BLDC ? bldc_state.locked : observer.ready);
    const bool forced_phase = music_requested || (!inactive && !detecting &&
        (motor_mode == MOTOR_BLDC ? !bldc_state.locked : startup.phase_override));
    const char observer_type = motor_mode == MOTOR_BLDC ? 'B' :
        (observer_config.type == VESC_OBSERVER_MXLEMMING_LAMBDA_COMP ? 'M' : 'O');
    const unsigned drive = static_cast<unsigned>(motor_mode);
    const bool feedback = bldc_feedback_ready();
    const uint32_t frames = sample_frames;
    const float target_iq = music_requested ? music_player.current.amplitude : iq_setpoint;
    const float reference_iq = iq_reference, iq_cap = music_requested ? music_tone::max_amplitude : iq_voltage_limit;
    const bool mode = current_mode || music_requested, armed = run_requested;
    const uint32_t retries = retry_attempts;
    const MotorFault reason = restart_reason;
    const CommandResult result = command_result;
    const float target_rpm = cfg::direction * speed_setpoint;
    const float reference_rpm = inactive || current_mode ? 0.0f :
        cfg::direction * speed_reference * 60.0f / (2.0f * kPi * FOC_POLE_PAIRS);
    const float speed_limit = speed_current_limit;
    data_port::leave_critical(mask);
    text.reset("OBS");
    text.integer("s", state); text.integer("f", fault);
    text.real("udc", bus, 3); text.real("ia", ia, 4);
    text.real("id", id, 4); text.real("iq", iq, 4);
    text.real("rpm", speed * 60.0f / (2.0f * kPi * FOC_POLE_PAIRS), 2);
    text.integer("ready", locked ? 1u : 0u); text.integer("params", ready ? 1u : 0u);
    text.real("R", r, 6); text.real("L", l, 9); text.real("flux", flux, 8);
    text.integer("n", frames); text.real("angle", angle, 3);
    text.integer("isr", cycles); text.integer("pwm", write_cycles);
    text.letter("mode", mode ? 'I' : 'S'); text.significant("iqset", target_iq);
    text.real("iqref", reference_iq, 3); text.integer("auto", armed ? 1u : 0u);
    text.integer("retry", retries); text.integer("why", reason); text.integer("cmd", result);
    text.significant("iqcap", iq_cap); text.significant("vset", target_rpm);
    text.significant("vref", reference_rpm); text.significant("ilim", speed_limit);
    text.integer("ph", phases); text.integer("ol", forced_phase ? 1u : 0u);
    text.letter("obs", observer_type); text.integer("drv", drive);
    text.integer("bfb", feedback ? 1u : 0u);
    const char *names[] = {"i_a", "i_b", "i_c"};
    const float values[] = {phase_a, phase_b, phase_c};
    for (unsigned i = 0; i < 3; ++i) {
        if (!(phases & (1u << i))) continue;
        text.real(names[i], values[i], 4);
    }
    text.finish();
    return text.valid && data_port::write(reinterpret_cast<const uint8_t *>(text.data),
                                         static_cast<uint16_t>(text.length));
}

void send_phase_samples()
{
    if (!data_port::tx_ready()) return;
    const uint8_t selected = phase_telemetry_mask;
    uint8_t read = phase_read;
    const uint8_t available = (phase_write - read) & 127u;
    if (!selected || available < 16u) return;
    // Worst case: 16 * 3 * 7 bytes plus header, < 448 bytes.
    static TelemetryText text;
    const uint32_t first_frame = phase_samples[read].frame;
    text.reset("CUR,"); text.number(first_frame); text.character(','); text.number(selected);
    text.character(','); text.number(phase_dropped); text.character(',');
    text.number(static_cast<uint32_t>(sample_dt * 1000000000.0f + 0.5f));
    for (unsigned i = 0; i < 16u; ++i) {
        const PhaseSample &sample = phase_samples[read];
        // A full ring causes missed frames: terminate the batch before the gap.
        if (sample.frame != first_frame + i * kPhaseStride) break;
        if (selected & 1u) { text.character(','); text.signed_number(sample.a); }
        if (selected & 2u) { text.character(','); text.signed_number(sample.b); }
        if (selected & 4u) { text.character(','); text.signed_number(sample.c); }
        read = (read + 1u) & 127u;
    }
    text.finish();
    if (text.valid && data_port::write(reinterpret_cast<const uint8_t *>(text.data),
                                      static_cast<uint16_t>(text.length))) phase_read = read;
}
