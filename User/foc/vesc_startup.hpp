#pragma once
#include "vesc_observer.hpp"

/* Low-speed phase override ported from bldc-master/motor/mcpwm_foc.c.
 * Copyright 2016-2022 Benjamin Vedder. GPL-3.0-or-later; see VESC_LICENSE.txt.
 * No allocation, RTOS or hardware dependency. All speeds are electrical rad/s.
 */
struct VescStartupConfig {
    float openloop_speed;
    float openloop_low;
    float hysteresis_seconds;
    float lock_seconds;
    float ramp_seconds;
    float hold_seconds;
    float flux_linkage;
};

struct VescStartupState {
    float phase;
    float x1;
    float x2;
    float forced_speed;
    float remaining;
    float low_speed_time;
    bool phase_override;
};

// 与 VESC 的 1 kHz 启动管理流程一致：低速滞回触发定时开环，结束后交还观测器。
inline void vesc_startup_update(const VescStartupConfig &config, VescStartupState &state,
                                float observer_phase, float speed, float iq_filtered,
                                float maximum_current, float duty, bool running, float dt)
{
    const float pi = 3.14159265358979323846f;
    float maximum_speed = config.openloop_speed *
        (config.openloop_low + foc_abs(iq_filtered) / maximum_current * (1.0f - config.openloop_low));
    if (maximum_speed > config.openloop_speed) maximum_speed = config.openloop_speed;
    float forced_speed = maximum_speed;
    const float elapsed = config.lock_seconds + config.ramp_seconds + config.hold_seconds - state.remaining;
    if (elapsed < config.lock_seconds) forced_speed = 0.0f;
    else if (elapsed < config.lock_seconds + config.ramp_seconds)
        forced_speed *= (elapsed - config.lock_seconds) / config.ramp_seconds;
    if (forced_speed > maximum_speed) forced_speed = maximum_speed;

    if (foc_abs(speed) < maximum_speed && state.low_speed_time < config.hysteresis_seconds)
        state.low_speed_time += dt;
    else if (state.low_speed_time > 0.0f) state.low_speed_time -= dt;

    if (running && foc_abs(duty) < 0.001f) {
        state.low_speed_time = state.remaining = 0.0f;
        state.phase_override = false;
    }
    bool started_now = false;
    if (state.low_speed_time >= config.hysteresis_seconds && state.remaining <= 0.0001f) {
        state.remaining = config.lock_seconds + config.ramp_seconds + config.hold_seconds;
        started_now = true;
    }
    if (!running) state.remaining = 0.0f;
    if (state.remaining > 0.0f) {
        const float direction = duty > 0.0f ? 1.0f : -1.0f;
        state.forced_speed = direction * forced_speed;
        state.phase += state.forced_speed * dt;
        // VESC：重新触发开环时修正 60°；按强制相位沿运行方向超前 45°预置磁链状态。
        if (started_now) state.phase += direction * pi / 3.0f;
        state.phase = vesc_observer_wrap(state.phase);
        state.phase_override = true;
        state.remaining -= dt;
        state.low_speed_time = 0.0f;
        float sn, cs;
        foc_fast_sincos(state.phase + direction * pi / 4.0f, sn, cs);
        state.x1 = cs * config.flux_linkage;
        state.x2 = sn * config.flux_linkage;
    } else {
        state.phase = observer_phase;
        state.forced_speed = 0.0f;
        state.phase_override = false;
    }
}
