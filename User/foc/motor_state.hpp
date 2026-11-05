#pragma once
// Private implementation, included only by foc/motor_control_impl.hpp.

namespace cfg = sensorless_config;
MotorMode motor_mode = cfg::use_mxlemming ? MOTOR_FOC_MXLEMMING : MOTOR_FOC_ORTEGA;
VescObserverType motor_observer_type() {
    return motor_mode == MOTOR_FOC_MXLEMMING ? VESC_OBSERVER_MXLEMMING_LAMBDA_COMP
        : VESC_OBSERVER_ORTEGA_LAMBDA_COMP;
}
const float kPi = 3.14159265358979323846f;
const float kAmpsPerCount = 0.02946541f;
const uint32_t kZeroSamples = 2048u;
enum MotorState { ZERO, IDLE, BOOTSTRAP, DETECT_R, DETECT_L,
    ALIGN, OPEN_LOOP, DETECT_FLUX, BLEND /* 旧协议保留值，正常启动不再经过此状态 */, RUN, FAULT, RETRY_WAIT };
enum MotorFault { NO_FAULT, BAD_ZERO, OVERCURRENT, ADC_ERROR, ADC_TIMEOUT,
    BAD_BUS, BAD_CONFIG, ALIGN_FAILED, STARTUP_TIMEOUT,
    LOST_LOCK, DETECT_FAILED, NUMERIC_ERROR, PWM_ERROR, BLDC_FEEDBACK_ERROR };

__align(4) uint16_t adc_buffer[motor_adc::sample_count];
volatile MotorState motor_state = ZERO;
volatile MotorFault motor_fault = NO_FAULT;
volatile uint32_t sample_frames = 0;
volatile uint32_t isr_max_cycles = 0;
volatile uint32_t observer_max_cycles = 0, current_max_cycles = 0, pwm_max_cycles = 0;
volatile uint32_t pwm_write_max_cycles = 0;
uint32_t isr_start_cycle;
uint32_t zero_count = 0, zero_sum[2] = {};
float zero_a, zero_b;
float I_a, I_b, I_c, I_alpha, I_beta, I_d, I_q;
float Udc_quant, electrical_angle, openloop_angle, openloop_speed;
float sample_dt, state_time, stable_time, lost_time;
uint32_t state_samples;
float align_d, align_q;
float observer_phase;
float observer_gain_base;
float motor_resistance = cfg::resistance;
float motor_inductance = cfg::inductance;
float motor_flux = cfg::flux_linkage;
bool parameters_ready = false, detecting = false, bridge_started = false;
volatile bool run_requested = false;
bool current_mode = false;
float iq_setpoint = cfg::current, iq_reference = 0.0f;
float iq_voltage_limit = cfg::max_iq;
float speed_setpoint = cfg::default_speed_rpm, speed_current_limit = cfg::current;
float speed_reference = 0.0f, speed_feedback = 0.0f, speed_elapsed = 0.0f;
uint32_t retry_attempts = 0;
MotorFault restart_reason = NO_FAULT;
uint32_t coast_until = 0;
float prior_alpha, prior_beta, older_alpha, older_beta;
float applied_alpha, applied_beta, last_I_alpha, last_I_beta;
PID pi_d = {}, pi_q = {}, pi_speed = {};
VescObserverConfig observer_config = {};
VescObserverState observer = {};
VescStartupConfig startup_config = {};
VescStartupState startup = {};
float startup_elapsed, startup_iq_filtered;
bldc::State bldc_state = {};
float bldc_phase_voltage[3] = {};
uint32_t bldc_feedback_frame = 0;
bool bldc_feedback_published = false;
bool bldc_feedback_ready() {
    return bldc_feedback_published && sample_frames - bldc_feedback_frame <= 2u;
}
float motor_speed() { return motor_mode == MOTOR_BLDC ? bldc_state.speed : observer.speed; }

#if defined(FOC_CURRENT_DIAG) && FOC_CURRENT_DIAG
// Optional SWD capture; omitted entirely from normal firmware.
struct CurrentDiagnosticSample {
    uint32_t frame, state;
    uint16_t raw_a, raw_b, raw_c, raw_bus;
    float ia, ib, ic, id, iq, iqref, phase, observer_phase, speed, flux,
        bus, va, vb, command_va, command_vb, integral_d, integral_q,
        observer_ia, observer_ib, phase_error, raw_angle, pll_angle;
    uint32_t ccr_a, ccr_b, ccr_c;
};
CurrentDiagnosticSample current_diag[192];
volatile uint32_t diag_mode = 0, diag_count = 0, diag_frozen = 0;
volatile uint32_t diag_state = RUN, diag_min_samples = 0, diag_stride = 1;
volatile uint32_t diag_frame_deadline = 0;
volatile float diag_min_speed = 0.0f, diag_trigger_current = 4.5f;
#endif

// Parameter identification: fixed accumulators, no allocation in the ISR.
unsigned detect_stage = 0;
float detect_voltage, detect_bias, sum_i, sum_v, low_i, low_v;
float lockin_ic, lockin_is, lockin_vc, lockin_vs;
float first_inductance, first_flux, emf_d, emf_q, emf_abs;
float filtered_emf_a, filtered_emf_b;
float detect_phase;
uint32_t detect_samples;
