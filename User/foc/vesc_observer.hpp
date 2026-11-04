#pragma once
#include <float.h>
#include <cmath>

// LAMBDA_COMP observers and PLL adapted from bldc-master/motor/foc_math.c.
// Implementation is in the already registered foc_math.cpp; no IDE edits needed.
enum VescObserverType {
    VESC_OBSERVER_ORTEGA_LAMBDA_COMP,
    VESC_OBSERVER_MXLEMMING_LAMBDA_COMP
};
struct VescObserverConfig {
    float resistance;
    float inductance;
    float flux_linkage;   // 配置/辨识得到的标称物理磁链，Wb。
    float gain;           // VESC gamma; correction uses gain / 2.
    float pll_kp;
    float pll_ki;
    float minimum_speed;  // Electrical rad/s.
    VescObserverType type;
};

struct VescObserverState {
    float x1;
    float x2;
    float angle;
    float pll_angle;
    float speed;          // Signed electrical rad/s.
    float flux_magnitude;
    float lambda_est;     // 与 VESC 一致，直接保存在线估计的物理磁链，Wb。
    float i_alpha_last;   // MXLEMMING 的电感电流差分项，保存上一拍实测电流。
    float i_beta_last;
    float phase_error;
    bool ready;           // Quality indication, not proof of rotor motion.
};

#if defined(__CC_ARM)
#define FOC_INLINE __forceinline
#else
#define FOC_INLINE inline
#endif

FOC_INLINE bool foc_finite(float value)
{
    return value == value && value <= FLT_MAX && value >= -FLT_MAX;
}

FOC_INLINE float foc_abs(float value)
{
    return value < 0.0f ? -value : value;
}

FOC_INLINE float foc_sqrt(float value)
{
#if defined(__CC_ARM)
    return __sqrtf(value);
#else
    return std::sqrt(value);
#endif
}
void foc_fast_sincos(float angle, float &sn, float &cs);
float foc_fast_atan2(float y, float x);
float vesc_observer_wrap(float angle);
void vesc_observer_reset(const VescObserverConfig &config,
                         VescObserverState &state, float angle);
// VESC 的母线电压/占空比增益调度；返回值对应 m_gamma_now。
float vesc_observer_gain(float configured_gain, float slow_gain, float duty, float bus_voltage);
// 首次使用前先 reset；每次调用都按所选 VESC 分支更新自适应磁链。
bool vesc_observer_update(const VescObserverConfig &config,
                          VescObserverState &state, float U_alpha, float U_beta,
                          float I_alpha, float I_beta, float dt,
                          bool config_validated = false);
