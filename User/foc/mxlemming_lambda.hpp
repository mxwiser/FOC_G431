#pragma once
// VESC LAMBDA_COMP observer branch; see VESC_LICENSE.txt.
#include "vesc_observer.hpp"

inline void mxlemming_lambda_update(const VescObserverConfig &config,
    VescObserverState &state, float U_alpha, float U_beta,
    float I_alpha, float I_beta, float dt)
{
    const float gamma_half = config.gain * 0.5f;
    const float lambda_min = config.flux_linkage * 0.3f;
    const float lambda_max = config.flux_linkage * 2.5f;

    /* LICENCE NOTE (from the original VESC/MESC implementation):
     * This function deviates slightly from the BSD 3 clause licence.
     * The work here is entirely original to the MESC FOC project, and not based
     * on any appnotes, or borrowed from another project. This work is free to
     * use, as granted in BSD 3 clause, with the exception that this note must
     * be included in where this code is implemented/modified to use your
     * variable names, structures containing variables or other minor
     * rearrangements in place of the original names I have chosen, and credit
     * to David Molony as the original author must be noted.
     */
    // 对齐本地 VESC 默认 MXLEMMING_LAMBDA_COMP：电压积分减去 L*delta_i。
    state.x1 += (U_alpha - config.resistance * I_alpha) * dt
        - config.inductance * (I_alpha - state.i_alpha_last);
    state.x2 += (U_beta - config.resistance * I_beta) * dt
        - config.inductance * (I_beta - state.i_beta_last);
    const float error = state.lambda_est * state.lambda_est
        - state.x1 * state.x1 - state.x2 * state.x2;
    state.lambda_est += 0.1f * gamma_half * state.lambda_est * -error * dt;
    if (state.lambda_est < lambda_min) state.lambda_est = lambda_min;
    if (state.lambda_est > lambda_max) state.lambda_est = lambda_max;
    if (state.x1 > state.lambda_est) state.x1 = state.lambda_est;
    if (state.x1 < -state.lambda_est) state.x1 = -state.lambda_est;
    if (state.x2 > state.lambda_est) state.x2 = state.lambda_est;
    if (state.x2 < -state.lambda_est) state.x2 = -state.lambda_est;
    // 此分支的 x1/x2 已是永磁磁链，统一求角度时不能再次减去 L*i。

}
