#pragma once
#include <stdint.h>

// 20 magnetic poles / 18 slots. Slots do not enter the electrical angle ratio.
#define FOC_POLE_PAIRS 10

// 2026-10-28: measured on the connected 20-pole motor at ~12.1 V / 1 A.
// Trial estimates, including the present board/ADC/PWM measurement errors.
// R: two DC plateaus; L: 200/700 Hz; flux: 105/150 electrical rad/s.
// D repeats detection in RAM. Overrides are used by the host simulation.
#ifndef FOC_MOTOR_R
#define FOC_MOTOR_R 0.093664f
#endif
#ifndef FOC_MOTOR_L
#define FOC_MOTOR_L 0.000017074f
#endif
#ifndef FOC_MOTOR_FLUX
#define FOC_MOTOR_FLUX 0.00182327f
#endif
// 参数辨识电流及默认速度环电流上限（A）；正常启动使用速度环或用户指令给出的 Iq。
#ifndef FOC_TEST_CURRENT
#define FOC_TEST_CURRENT 1.0f
#endif
// Current normalization for the sensorless startup helper, not a command limit.
#ifndef FOC_STARTUP_CURRENT_SCALE
#define FOC_STARTUP_CURRENT_SCALE 5.0f
#endif
#ifndef FOC_TRIP_CURRENT
#define FOC_TRIP_CURRENT 30.0f
#endif
#ifndef FOC_MIN_BUS
#define FOC_MIN_BUS 8.0f
#endif
#ifndef FOC_MAX_BUS
#define FOC_MAX_BUS 30.0f
#endif
#ifndef FOC_SENSORLESS_DIRECTION
#define FOC_SENSORLESS_DIRECTION 1.0f
#endif
#ifndef FOC_RUN_CURRENT_BANDWIDTH
// 20 V / 17 uH motor: 5000 rad/s permits growing observer/current-loop
// oscillation around electrical frequency. 9000 rad/s was cycle-captured
// through ~4000 mechanical rpm; Ts*bandwidth = 0.72 at 12.5 kHz.
#define FOC_RUN_CURRENT_BANDWIDTH 9000.0f
#endif
#ifndef FOC_USE_MXLEMMING
#define FOC_USE_MXLEMMING 1 // 与本地 VESC 默认配置一致；设为 0 可对照 Ortega LAMBDA_COMP。
#endif

namespace sensorless_config {
const float resistance = FOC_MOTOR_R;        // Ohms, phase resistance.
const float inductance = FOC_MOTOR_L;        // H, phase inductance (Ld = Lq).
const float flux_linkage = FOC_MOTOR_FLUX;   // Wb, phase PM flux linkage.
const float current = FOC_TEST_CURRENT;     // A, parameter detection and default speed-mode limit.
const float startup_current_scale = FOC_STARTUP_CURRENT_SCALE; // A, startup speed normalization only.
const float overcurrent = FOC_TRIP_CURRENT; // A, absolute phase-current trip.
const float min_bus = FOC_MIN_BUS;
const float max_bus = FOC_MAX_BUS;
const float direction = FOC_SENSORLESS_DIRECTION;
const float max_modulation = 0.8f;          // Preserve low-side sampling time.
const float observer_gain_slow = 0.05f;     // VESC 默认的低占空比观测器增益比例。
const bool use_mxlemming = FOC_USE_MXLEMMING != 0;
// 以下启动参数与本地 VESC mcconf_default.h 的无感开环辅助默认值一致。
const float startup_openloop_rad_s = 157.07963267948966f; // 1500 ERPM，10 对极时为 150 机械 rpm。
const float startup_openloop_low = 0.0f;
const float startup_hysteresis_seconds = 0.1f;
const float startup_lock_seconds = 0.0f;
const float startup_ramp_seconds = 0.1f;
const float startup_hold_seconds = 0.05f;
const float startup_manager_seconds = 0.001f; // 与 VESC 一样按 1 kHz 管理开环辅助。
const float current_bandwidth = 500.0f;     // rad/s; alignment, open loop, detection.
const float run_current_bandwidth = FOC_RUN_CURRENT_BANDWIDTH; // rad/s; closed-loop current tracking.
// Reserve voltage for d-axis regulation before back EMF saturates the bridge.
const float iq_voltage_derate_start = 0.75f;
const float iq_voltage_derate_end = 0.85f;
const float iq_voltage_derate_scale = 1.0f; // A, minimum current scale for voltage-headroom derating.
const float align_seconds = 1.0f;           // 仅参数辨识的 Id 对齐使用。
const float align_settle_grace = 0.5f;      // Release during alignment may need settling.
const float ramp_seconds = 3.0f;
const float startup_timeout = 8.0f;
const float openloop_rad_s = 150.0f;        // 仅参数辨识使用的电角速度；不是正常运行的转速上限。
const float minimum_rad_s = 80.0f;
const float lock_seconds = 0.2f;            // 启动成功确认时间；不阻塞 Iq/速度控制和相位交接。
const float lost_seconds = 0.1f;
const uint32_t retry_delay_ms = 2000u;       // Bridge off between stall retries.
const float iq_slew_per_second = 2.0f;      // A/s for live current-mode changes.
// Positive speed magnitude, in the configured direction. Zero requests stop.
const float default_speed_rpm = 150.0f;
const float min_speed_rpm = 150.0f;         // Keep above the 76 rpm observer floor.
// VESC default: fixed 25000 ERPM/s for both acceleration and deceleration.
const float speed_ramp_rpm_per_second = 25000.0f / FOC_POLE_PAIRS;
const float bldc_startup_speed_ramp_rpm_per_second = 500.0f; // Before BEMF lock only.
const float speed_loop_seconds = 0.001f;   // ~1 kHz; uses actual elapsed sample time.
const float speed_filter_seconds = 0.01f;
// Fast torque sign changes keep the signed speed loop from oscillating under
// load release/braking. Independent of the manual IQ command ramp.
const float speed_iq_slew_per_second = 100.0f;
// PI input: electrical rad/s error; output: signed torque current in amperes.
const float speed_kp = current * 0.3f / openloop_rad_s;
const float speed_ki = speed_kp * 5.0f;
}
