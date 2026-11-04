#pragma once
#include <float.h>
#include <cmath>

// Ortega LAMBDA_COMP observer and PLL adapted from bldc-master/motor/foc_math.c.
// Implementation is in the already registered foc_math.cpp; no IDE edits needed.
struct VescObserverConfig {
    float resistance;
    float inductance;
    float flux_linkage;   // 标称磁链，Wb；应与观测器输入具有相同的滤波幅值尺度。
    float gain;           // VESC gamma; correction uses gain / 2.
    float pll_kp;
    float pll_ki;
    float minimum_speed;  // Electrical rad/s.
};

struct VescObserverState {
    float x1;
    float x2;
    float angle;
    float pll_angle;
    float speed;          // Signed electrical rad/s.
    float flux_magnitude;
    float lambda_est;     // 自适应磁链，Wb；与观测器的滤波输入处于同一尺度。
    float lambda_ratio;   // 自适应磁链/标称磁链，范围 0.3~2.5；跨滤波幅值补偿保存。
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
// 首次使用前先 reset；adapt_flux=false 冻结自适应比例，仍更新磁链矢量、角度和速度。
bool vesc_observer_update(const VescObserverConfig &config,
                          VescObserverState &state, float U_alpha, float U_beta,
                          float I_alpha, float I_beta, float dt,
                          bool config_validated = false, bool adapt_flux = true);
