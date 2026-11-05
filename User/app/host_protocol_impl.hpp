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
                (command.value >= cfg::min_speed_rpm && command.value <= cfg::max_speed_rpm)) :
                (command.value >= (limit ? 0.1f : 0.0f) && command.value <= cfg::max_iq);
            if (!foc_finite(command.value) || !range_ok) {
                command_result = COMMAND_RANGE;
            } else if (motor_state == FAULT || (detecting && motor_state != IDLE && motor_state != ZERO)) {
                command_result = COMMAND_BUSY;
            } else {
                const uint32_t mask = data_port::enter_critical();
                if (limit) speed_current_limit = command.value;
                else if (speed) {
                    speed_setpoint = command.value;
                    if (current_mode) {
                        speed_reference = speed_feedback = cfg::direction * motor_speed();
                        speed_elapsed = 0.0f;
                        pi_speed.integral = pi_speed.output = cfg::direction * iq_reference;
                    }
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

bool send_status()
{
    if (!data_port::tx_ready()) return false;
    static char text[448]; // Persistent until the interrupt-driven TX completes.
    const uint32_t mask = data_port::enter_critical();
    const MotorState state = motor_state; const MotorFault fault = motor_fault;
    const bool inactive = state == IDLE || state == ZERO || state == RETRY_WAIT;
    const float bus = Udc_quant, id = inactive ? 0.0f : I_d, iq = inactive ? 0.0f : I_q;
    const float speed = inactive ? 0.0f : motor_speed();
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
    const bool locked = !inactive && motor_fault == NO_FAULT &&
        (motor_mode == MOTOR_BLDC ? bldc_state.locked : observer.ready);
    const bool forced_phase = !inactive && !detecting &&
        (motor_mode == MOTOR_BLDC ? !bldc_state.locked : startup.phase_override);
    const char observer_type = motor_mode == MOTOR_BLDC ? 'B' :
        (observer_config.type == VESC_OBSERVER_MXLEMMING_LAMBDA_COMP ? 'M' : 'O');
    const unsigned drive = static_cast<unsigned>(motor_mode);
    const bool feedback = bldc_feedback_ready();
    const uint32_t frames = sample_frames;
    const float target_iq = iq_setpoint, reference_iq = iq_reference, iq_cap = iq_voltage_limit;
    const bool mode = current_mode, armed = run_requested;
    const uint32_t retries = retry_attempts;
    const MotorFault reason = restart_reason;
    const CommandResult result = command_result;
    const float target_rpm = cfg::direction * speed_setpoint;
    const float reference_rpm = inactive || current_mode ? 0.0f :
        cfg::direction * speed_reference * 60.0f / (2.0f * kPi * FOC_POLE_PAIRS);
    const float speed_limit = speed_current_limit;
    data_port::leave_critical(mask);
    int length = snprintf(text, sizeof(text),
        "OBS,s=%d,f=%d,udc=%.3f,ia=%.4f,id=%.4f,iq=%.4f,rpm=%.2f,ready=%d,params=%d,R=%.6f,L=%.9f,flux=%.8f,n=%lu,angle=%.3f,isr=%lu,pwm=%lu,mode=%c,iqset=%.3f,iqref=%.3f,auto=%d,retry=%lu,why=%d,cmd=%d,iqcap=%.3f,vset=%.1f,vref=%.1f,ilim=%.3f,ph=%u,ol=%d,obs=%c,drv=%u,bfb=%d",
        static_cast<int>(state), static_cast<int>(fault), bus, ia, id, iq,
        speed * 60.0f / (2.0f * kPi * FOC_POLE_PAIRS), locked ? 1 : 0, ready ? 1 : 0,
        r, l, flux, static_cast<unsigned long>(frames), angle,
        static_cast<unsigned long>(cycles), static_cast<unsigned long>(write_cycles),
        mode ? 'I' : 'S', target_iq, reference_iq, armed ? 1 : 0,
        static_cast<unsigned long>(retries), static_cast<int>(reason), static_cast<int>(result), iq_cap,
        target_rpm, reference_rpm, speed_limit, static_cast<unsigned>(phases), forced_phase ? 1 : 0,
        observer_type, drive, feedback ? 1 : 0);
    if (length <= 0 || length >= static_cast<int>(sizeof(text))) return false;
    const char *names[] = {",i_a=%.4f", ",i_b=%.4f", ",i_c=%.4f"};
    const float values[] = {phase_a, phase_b, phase_c};
    for (unsigned i = 0; i < 3; ++i) {
        if (!(phases & (1u << i))) continue;
        const int added = snprintf(text + length, sizeof(text) - length, names[i], values[i]);
        if (added < 0 || added >= static_cast<int>(sizeof(text) - length)) return false;
        length += added;
    }
    if (length + 2 >= static_cast<int>(sizeof(text))) return false;
    text[length++] = '\r'; text[length++] = '\n';
    return data_port::write(reinterpret_cast<const uint8_t *>(text), static_cast<uint16_t>(length));
}

// Append a signed decimal field to a bounded, fixed-size packet.
void append_phase_number(char *text, unsigned &length, int32_t number)
{
    text[length++] = ',';
    uint32_t magnitude;
    if (number < 0) { text[length++] = '-'; magnitude = static_cast<uint32_t>(-number); }
    else magnitude = static_cast<uint32_t>(number);
    char digits[10]; unsigned count = 0;
    do { digits[count++] = static_cast<char>('0' + magnitude % 10u); magnitude /= 10u; } while (magnitude);
    while (count) text[length++] = digits[--count];
}

void send_phase_samples()
{
    if (!data_port::tx_ready()) return;
    const uint8_t selected = phase_telemetry_mask;
    uint8_t read = phase_read;
    const uint8_t available = (phase_write - read) & 127u;
    if (!selected || available < 16u) return;
    // Worst case: 16 * 3 * 7 bytes plus header, < 448 bytes.
    static char text[448];
    unsigned length = 3; text[0] = 'C'; text[1] = 'U'; text[2] = 'R';
    const uint32_t first_frame = phase_samples[read].frame;
    // Frame/drop are encoded as unsigned decimals (cast avoids signed rollover).
    const int header = snprintf(text, sizeof(text), "CUR,%lu,%u,%lu,%lu",
        static_cast<unsigned long>(first_frame), static_cast<unsigned>(selected),
        static_cast<unsigned long>(phase_dropped),
        static_cast<unsigned long>(sample_dt * 1000000000.0f + 0.5f));
    if (header <= 0 || header >= static_cast<int>(sizeof(text))) return;
    length = static_cast<unsigned>(header);
    for (unsigned i = 0; i < 16u; ++i) {
        const PhaseSample &sample = phase_samples[read];
        // A full ring causes missed frames: terminate the batch before the gap.
        if (sample.frame != first_frame + i * kPhaseStride) break;
        if (selected & 1u) append_phase_number(text, length, sample.a);
        if (selected & 2u) append_phase_number(text, length, sample.b);
        if (selected & 4u) append_phase_number(text, length, sample.c);
        read = (read + 1u) & 127u;
    }
    text[length++] = '\r'; text[length++] = '\n';
    if (data_port::write(reinterpret_cast<const uint8_t *>(text),
                         static_cast<uint16_t>(length))) phase_read = read;
}
