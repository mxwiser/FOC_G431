#pragma once
// VESC LAMBDA_COMP observer branch; see VESC_LICENSE.txt.
#include "vesc_observer.hpp"

inline void ortega_lambda_update(const VescObserverConfig &config,
    VescObserverState &state, float U_alpha, float U_beta,
    float I_alpha, float I_beta, float dt)
{
    const float L_ia = config.inductance * I_alpha;
    const float L_ib = config.inductance * I_beta;
    const float gamma_half = config.gain * 0.5f;
    const float lambda_min = config.flux_linkage * 0.3f;
    const float lambda_max = config.flux_linkage * 2.5f;

    const float flux_a = state.x1 - L_ia;
    const float flux_b = state.x2 - L_ib;
    // 与 VESC ORTEGA_LAMBDA_COMP 一致，直接使用物理磁链估计值计算完整误差。
    float error = state.lambda_est * state.lambda_est
        - flux_a * flux_a - flux_b * flux_b;
    // 先用有符号误差更新 lambda_est，使其可增可减，再按标称磁链的 0.3~2.5 倍限幅。
    state.lambda_est += 0.2f * gamma_half * state.lambda_est * -error * dt;
    if (state.lambda_est < lambda_min) state.lambda_est = lambda_min;
    if (state.lambda_est > lambda_max) state.lambda_est = lambda_max;
    // 与 VESC 一致，仅磁链矢量的非线性校正项抑制正误差。
    if (error > 0.0f) error = 0.0f;
    state.x1 += (U_alpha - config.resistance * I_alpha
        + gamma_half * flux_a * error) * dt;
    state.x2 += (U_beta - config.resistance * I_beta
        + gamma_half * flux_b * error) * dt;

}
