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
#ifndef FOC_TEST_CURRENT
#define FOC_TEST_CURRENT 1.0f
#endif
#ifndef FOC_TRIP_CURRENT
#define FOC_TRIP_CURRENT 3.0f
#endif
#ifndef FOC_MIN_BUS
#define FOC_MIN_BUS 8.0f
#endif
#ifndef FOC_MAX_BUS
#define FOC_MAX_BUS 14.0f
#endif
#ifndef FOC_SENSORLESS_DIRECTION
#define FOC_SENSORLESS_DIRECTION 1.0f
#endif

namespace sensorless_config {
const float resistance = FOC_MOTOR_R;        // Ohms, phase resistance.
const float inductance = FOC_MOTOR_L;        // H, phase inductance (Ld = Lq).
const float flux_linkage = FOC_MOTOR_FLUX;   // Wb, phase PM flux linkage.
const float current = FOC_TEST_CURRENT;     // A, positive dq test current.
const float overcurrent = FOC_TRIP_CURRENT; // A, absolute phase-current trip.
const float min_bus = FOC_MIN_BUS;
const float max_bus = FOC_MAX_BUS;
const float direction = FOC_SENSORLESS_DIRECTION;
const float max_modulation = 0.8f;          // Preserve low-side sampling time.
const float current_bandwidth = 500.0f;     // rad/s; alignment, open loop, detection.
const float run_current_bandwidth = 5000.0f; // rad/s; closed-loop current tracking.
// Reserve voltage for d-axis regulation before back EMF saturates the bridge.
const float iq_voltage_derate_start = 0.75f;
const float iq_voltage_derate_end = 0.85f;
const float align_seconds = 1.0f;
const float align_settle_grace = 0.5f;      // Release during alignment may need settling.
const float ramp_seconds = 3.0f;
const float startup_timeout = 8.0f;
const float openloop_rad_s = 150.0f;        // Electrical rad/s, about 143 mech rpm.
const float minimum_rad_s = 80.0f;
const float lock_seconds = 0.2f;
const float blend_seconds = 0.05f;
const float lost_seconds = 0.1f;
const uint32_t retry_delay_ms = 2000u;       // Bridge off between stall retries.
const float iq_slew_per_second = 2.0f;      // A/s for live current-mode changes.
}
