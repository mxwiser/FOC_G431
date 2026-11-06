#pragma once
// The existing setup.cpp translation unit owns this module; no IDE list edits.
#include "motor_control.hpp"
#include "main.h"
#include "tim.h"
#include "usart.h"
#include "foc_math.hpp"
#include "svpwm.h"
#include "vesc_observer.hpp"
#include "vesc_startup.hpp"
#include "music_tone.hpp"
#include "../app/serial_command.hpp"
#include "../app/data_port.hpp"
#include "../app/telemetry_text.hpp"
#include "../motor_mode.hpp"
#include "../bldc/bldc.hpp"
#include "../bldc/phase_feedback.hpp"
#include "../bldc/timer_output.hpp"
#include <cmath>
#include <stdio.h>
#if defined(__CC_ARM)
#pragma Otime
#endif


namespace {
#include "motor_state.hpp"
#include "../app/communication_state.hpp"
#include "state_machine_impl.hpp"
#include "../board/uart_port_impl.hpp"
#include "../app/host_protocol_impl.hpp"
#include "diagnostics_impl.hpp"
} // namespace

#include "../bldc/phase_feedback_impl.hpp"
#include "adc_control_impl.hpp"
#include "../board/uart_irq_impl.hpp"

void motor_control::initialize(float sample_period_seconds)
{
    observer_config.type = motor_observer_type();
    sample_dt = sample_period_seconds;
    if (!limits_valid() || !foc_finite(sample_dt) || sample_dt <= 0.0f || sample_dt > 0.001f)
        motor_fail(BAD_CONFIG);
    parameters_ready = model_valid();
    service_last_frame = sample_frames;
    service_last_adc = service_last_status = data_port::milliseconds();
}

void motor_control::run()
{
    service_serial();
    const uint32_t now = data_port::milliseconds(), frame = sample_frames;
    if (frame != service_last_frame) { service_last_frame = frame; service_last_adc = now; }
    if (now - service_last_adc > 10u && motor_state != FAULT) motor_fail(ADC_TIMEOUT);
    service_restart();
    if (now - service_last_status >= 20u) {
        if (send_status()) service_last_status = now;
    } else if (!send_music_status(now)) send_phase_samples();
}
