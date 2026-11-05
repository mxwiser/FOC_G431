#pragma once
#include <cmath>
#include <float.h>
#include <stdint.h>

// Six-step table and BEMF delay commutation adapted from VESC motor/mcpwm.c
// (Benjamin Vedder, GPL-3.0-or-later; see ../foc/VESC_LICENSE.txt).
// Fixed storage only. No FOC observer, Park transform or SVPWM in this module.
namespace bldc {
const float sector_radians = 1.0471975511965977f;
struct Step { uint8_t source, sink, floating; int8_t crossing_sign; };
inline Step step(unsigned index) {
    // VESC's vectors ordered for positive A/B Clarke rotation, matching FOC.
    const Step table[6] = {
        {1, 2, 0, -1}, {1, 0, 2, 1}, {2, 0, 1, -1},
        {2, 1, 0, 1}, {0, 1, 2, -1}, {0, 2, 1, 1}
    };
    return table[index % 6u];
}
inline bool finite(float x) { return x == x && x <= FLT_MAX && x >= -FLT_MAX; }
inline float bound(float x, float low, float high) { return x < low ? low : (x > high ? high : x); }
struct Config {
    float resistance, inductance, flux, max_duty, startup_timeout;
    int direction;
};
struct State {
    unsigned sector, crossings;
    float elapsed, sector_time, period, positive_time, integral, current, duty, speed;
    bool armed, crossed, locked, failed;
};
inline void reset(State &s) {
    State cleared = {};
    s = cleared;
    s.period = 0.02f;
}
inline float signed_bemf(const State &s, const float voltage[3], int direction) {
    const Step phase = step(s.sector);
    const float neutral = (voltage[0] + voltage[1] + voltage[2]) / 3.0f;
    return (voltage[phase.floating] - neutral) * phase.crossing_sign * direction;
}
inline void commutate(State &s, int direction, bool measured) {
    if (measured) {
        // One commutation spans 60 electrical degrees.
        const float measured_period = s.sector_time;
        s.period += 0.25f * (measured_period - s.period);
        s.speed = direction * sector_radians / s.period;
        ++s.crossings;
        if (s.crossings >= 12u) s.locked = true;
    } else s.crossings = 0;
    s.sector = (s.sector + (direction > 0 ? 1u : 5u)) % 6u;
    s.sector_time = s.positive_time = 0.0f;
    s.armed = s.crossed = false;
}

// voltage[] must be physical phase-to-ground volts captured at a valid PWM
// sampling point. Fabricated phase voltages from duty cycles are not feedback.
inline bool update(const Config &c, State &s, const float voltage[3],
                   const float current[3], float bus, float reference, float dt) {
    if (!finite(bus) || bus <= 0.0f || !finite(dt) || dt <= 0.0f || dt > 0.001f ||
        !finite(reference) || reference < 0.0f ||
        !finite(c.resistance) || c.resistance <= 0.0f ||
        !finite(c.inductance) || c.inductance <= 0.0f ||
        !finite(c.flux) || c.flux <= 0.0f || !finite(c.max_duty) ||
        c.max_duty <= 0.0f || c.max_duty > 0.9f ||
        !finite(c.startup_timeout) || c.startup_timeout <= 0.0f ||
        (c.direction != 1 && c.direction != -1)) { s.failed = true; return false; }
    for (unsigned i = 0; i < 3; ++i) {
        if (!finite(voltage[i]) || voltage[i] < -0.5f || voltage[i] > bus + 0.5f ||
            !finite(current[i])) { s.failed = true; return false; }
    }
    if (s.failed) return false;
    const Step sampled_phase = step(s.sector);
    s.elapsed += dt;
    s.sector_time += dt;
    // Hold the first vector for 200 ms, then force commutation until BEMF
    // qualifies. Missing BEMF can never be reported as a locked rotor.
    const bool aligning = s.elapsed < 0.2f;
    if (aligning) s.sector_time = 0.0f;
    if (!aligning) {
        const float threshold = bus * 0.002f + 0.02f;
        const float emf = signed_bemf(s, voltage, c.direction);
        const float blank = s.period * 0.1f > 2.0f * dt ? s.period * 0.1f : 2.0f * dt;
        if (s.sector_time > blank) {
            if (emf < -threshold && !s.crossed) s.armed = true;
            if (s.armed && emf > threshold && s.sector_time > s.period * 0.2f &&
                s.sector_time < s.period * 0.85f) s.crossed = true;
            // VESC COMM_MODE_DELAY: positive BEMF for half of a 60-degree
            // period gives the 30-degree delay after the zero crossing.
            if (s.crossed && emf > 0.0f) s.positive_time += dt;
            else if (s.crossed) { s.positive_time = 0.0f; s.crossed = false; }
        }
        if (s.crossed && s.positive_time >= 0.5f * s.period) commutate(s, c.direction, true);
        else if (!s.locked && s.sector_time >= s.period * 1.5f) {
            commutate(s, c.direction, false);
            const float ramp = bound((s.elapsed - 0.2f) / 3.0f, 0.0f, 1.0f);
            s.period = 0.02f + (0.007f - 0.02f) * ramp;
        } else if (s.locked && s.sector_time > (s.period * 3.0f + 0.01f)) s.failed = true;
        if (!s.locked && s.elapsed > c.startup_timeout) s.failed = true;
    }
    // A scalar line-current PI. The negative leg conducts at the ADC trigger;
    // unlike FOC, there is no d/q current here. R_line=2R, L_line=2L.
    const float measured = -current[sampled_phase.sink];
    s.current += dt / (0.001f + dt) * (measured - s.current);
    const float error = reference - s.current;
    const float maximum = bus * c.max_duty;
    s.integral += 2.0f * c.resistance * 500.0f * error * dt;
    const float feedforward = s.locked ? 1.7320508f * c.flux * std::fabs(s.speed) : 0.0f;
    const float requested = 2.0f * c.inductance * 500.0f * error + s.integral + feedforward;
    const float output = bound(requested, 0.0f, maximum);
    s.integral += output - requested;
    s.duty = s.failed ? 0.0f : output / bus;
    return !s.failed;
}
}
