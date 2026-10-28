#include "foc_math.hpp"
#include <cmath>
#include "svpwm.h"
#include "vesc_observer.hpp"
#include <float.h>
#if defined(__CC_ARM)
#pragma Otime
#endif
void clark(float *Ia,float *Ib,float *Ic,float *alpha,float *beta){
    // Amplitude invariant: alpha/beta remain in physical amperes.
    *alpha = (2.0f * *Ia - *Ib - *Ic) / 3.0f;
    *beta = (*Ib - *Ic) * ONE_BY_SQRT3;
}
void park(float *x1,float *y1,float *theta,float *x2,float *y2){
    float sn, cs;
    foc_fast_sincos(*theta, sn, cs);
    const float d = *x1 * cs + *y1 * sn;
    const float q = -*x1 * sn + *y1 * cs;
    *x2 = d; *y2 = q;
}

void calculate_Idq(float I_a, float I_b, float I_c,float theta,float &Id,float &Iq){
    float I_alpha,I_beta;
    clark(&I_a,&I_b,&I_c,&I_alpha,&I_beta);
    park(&I_alpha,&I_beta,&theta,&Id,&Iq);
}

void calculate_foc(float I_a, float I_b, float I_c,float theta, uint32_t &tAout, uint32_t &tBout, uint32_t &tCout)
{       
    float I_q,I_d;
    calculate_Idq(I_a,I_b,I_c,theta,I_d,I_q);
    float temp = -theta;
    float U_alpha,U_beta;
    I_q = 1;
    I_d = 0;
    park(&I_d,&I_q,&temp,&U_alpha,&U_beta);
    uint32_t sector;
    foc_svm_normalized(U_alpha,U_beta,tAout,tBout,tCout,sector);
}

void foc_svm_normalized(float U_alpha,float U_beta, uint32_t &tA, uint32_t &tB, uint32_t &tC,uint32_t &sector){
    const float alpha = U_alpha * 1.5f * ONE_BY_SQRT3;
    const float beta = U_beta * 1.5f * ONE_BY_SQRT3;
    foc_svm(alpha, beta, 1.0f, FOC_PWM_PERIOD_TICKS,
            &tA, &tB, &tC, &sector);
}

void calculate_PI(PID *pid){
    if (pid == 0) {
        return;
    }
    const float limit = pid->output_limit;
    if (!(pid->dt > 0.0f) || !(limit > 0.0f)) {
        pid->integral = 0.0f;
        pid->last_input = 0.0f;
        pid->output = 0.0f;
        return;
    }

    // Re-clamp saved state if the voltage/output limit has been reduced.
    if (pid->integral > limit) pid->integral = limit;
    if (pid->integral < -limit) pid->integral = -limit;

    const float proportional = pid->kp * pid->input;
    float candidate = pid->integral + pid->ki * pid->input * pid->dt;
    if (candidate > limit) candidate = limit;
    if (candidate < -limit) candidate = -limit;

    // Do not accumulate an error that would push further into saturation.
    // Opposite-sign error can still unwind the integral immediately.
    const float requested = proportional + candidate;
    if (!((requested > limit && pid->input > 0.0f) ||
          (requested < -limit && pid->input < 0.0f))) {
        pid->integral = candidate;
    }

    pid->output = proportional + pid->integral;
    if (pid->output > limit) pid->output = limit;
    if (pid->output < -limit) pid->output = -limit;
    pid->last_input = pid->input;
}

void Foc_motor::init()
{

}


void Foc_motor::updateSvpwm(uint32_t &tA, uint32_t &tB, uint32_t &tC)
{
}

/*
 * Copyright 2016 - 2022 Benjamin Vedder <benjamin@vedder.se>
 * Adapted from bldc-master/motor/foc_math.c:
 * FOC_OBSERVER_ORTEGA_ORIGINAL and foc_pll_run, plus fast sine/cosine/atan2
 * from bldc-master/util/utils_math.c (same copyright and GPL-3.0-or-later).
 *
 * This part is free software: you can redistribute it and/or modify it under
 * the GNU General Public License as published by the Free Software Foundation,
 * either version 3 of the License, or (at your option) any later version.
 * It is distributed WITHOUT ANY WARRANTY; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See User/foc/VESC_LICENSE.txt for the GNU General Public License.
 *
 * Port: explicit SI-unit parameters, finite checks, bounded sample interval,
 * and PLL/flux quality reporting. No RTOS, allocation, or encoder dependency.
 */
float vesc_observer_wrap(float angle)
{
    if (!foc_finite(angle)) return 0.0f;
    const float pi = 3.14159265358979323846f;
    const float two_pi = 2.0f * pi;
    if (angle > pi) angle -= two_pi;
    if (angle < -pi) angle += two_pi;
    if (angle > pi || angle < -pi) {
        angle = std::fmod(angle, two_pi);
        if (angle > pi) angle -= two_pi;
        if (angle < -pi) angle += two_pi;
    }
    return angle;
}

static float vesc_fast_sin_wrapped(float angle)
{
    float result = 1.27323954f * angle - 0.405284735f * angle * foc_abs(angle);
    return 0.225f * (result * foc_abs(result) - result) + result;
}

void foc_fast_sincos(float angle, float &sn, float &cs)
{
    angle = vesc_observer_wrap(angle);
    sn = vesc_fast_sin_wrapped(angle);
    float shifted = angle + 1.5707963267948966f;
    if (shifted > 3.1415926535897932f) shifted -= 6.2831853071795864f;
    cs = vesc_fast_sin_wrapped(shifted);
}

float foc_fast_atan2(float y, float x)
{
    const float abs_y = foc_abs(y) + 1.0e-20f;
    const float r = x >= 0.0f ? (x - abs_y) / (x + abs_y) : (x + abs_y) / (abs_y - x);
    const float offset = x >= 0.0f ? 0.7853981633974483f : 2.356194490192345f;
    const float angle = (0.1963f * r * r - 0.9817f) * r + offset;
    return y < 0.0f ? -angle : angle;
}

void vesc_observer_reset(const VescObserverConfig &config,
                         VescObserverState &state, float angle)
{
    VescObserverState cleared = {};
    state = cleared;
    state.angle = state.pll_angle = vesc_observer_wrap(angle);
    if (foc_finite(config.flux_linkage) && config.flux_linkage > 0.0f) {
        float sn, cs;
        foc_fast_sincos(state.angle, sn, cs);
        state.x1 = config.flux_linkage * cs;
        state.x2 = config.flux_linkage * sn;
        state.flux_magnitude = config.flux_linkage;
    }
}

bool vesc_observer_update(const VescObserverConfig &config,
                          VescObserverState &state, float U_alpha, float U_beta,
                          float I_alpha, float I_beta, float dt, bool config_validated)
{
    if (!foc_finite(dt) || dt <= 0.0f || dt > 0.001f ||
        (!config_validated && (!foc_finite(config.resistance) || config.resistance <= 0.0f ||
        !foc_finite(config.inductance) || config.inductance <= 0.0f ||
        !foc_finite(config.flux_linkage) || config.flux_linkage <= 0.0f ||
        !foc_finite(config.gain) || config.gain <= 0.0f ||
        !foc_finite(config.pll_kp) || config.pll_kp <= 0.0f ||
        !foc_finite(config.pll_ki) || config.pll_ki <= 0.0f ||
        !foc_finite(config.minimum_speed) || config.minimum_speed <= 0.0f)) ||
        !foc_finite(U_alpha) || !foc_finite(U_beta) ||
        !foc_finite(I_alpha) || !foc_finite(I_beta) ||
        !foc_finite(state.x1) || !foc_finite(state.x2) ||
        !foc_finite(state.pll_angle) || !foc_finite(state.speed)) {
        VescObserverState cleared = {};
        state = cleared;
        return false;
    }

    const float L_ia = config.inductance * I_alpha;
    const float L_ib = config.inductance * I_beta;
    const float flux_a = state.x1 - L_ia;
    const float flux_b = state.x2 - L_ib;
    float error = config.flux_linkage * config.flux_linkage
        - flux_a * flux_a - flux_b * flux_b;
    // VESC Ortega correction: positive error is deliberately suppressed.
    if (error > 0.0f) error = 0.0f;
    const float gamma_half = config.gain * 0.5f;
    state.x1 += (U_alpha - config.resistance * I_alpha
        + gamma_half * flux_a * error) * dt;
    state.x2 += (U_beta - config.resistance * I_beta
        + gamma_half * flux_b * error) * dt;

    const float stator_magnitude = foc_sqrt(state.x1 * state.x1 + state.x2 * state.x2);
    if (stator_magnitude < config.flux_linkage * 0.5f) {
        state.x1 *= 1.1f;
        state.x2 *= 1.1f;
    }
    const float pm_a = state.x1 - L_ia;
    const float pm_b = state.x2 - L_ib;
    state.flux_magnitude = foc_sqrt(pm_a * pm_a + pm_b * pm_b);
    state.angle = foc_fast_atan2(pm_b, pm_a);
    const float delta = vesc_observer_wrap(state.angle - state.pll_angle);
    state.pll_angle = vesc_observer_wrap(state.pll_angle
        + (state.speed + config.pll_kp * delta) * dt);
    state.speed += config.pll_ki * delta * dt;
    const float filter = dt / (0.02f + dt);
    state.phase_error += filter * (foc_abs(delta) - state.phase_error);
    if (!foc_finite(state.flux_magnitude) || !foc_finite(state.speed) ||
        !foc_finite(state.x1) || !foc_finite(state.x2)) {
        VescObserverState cleared = {};
        state = cleared;
        return false;
    }
    const float ratio = state.flux_magnitude / config.flux_linkage;
    state.ready = ratio > 0.7f && ratio < 1.3f
        && foc_abs(state.speed) >= config.minimum_speed
        && state.phase_error < 0.25f;
    return true;
}
