#pragma once
// Private implementation, included only by foc/motor_control_impl.hpp.

namespace cfg = sensorless_config;
MotorMode motor_mode = cfg::use_mxlemming ? MOTOR_FOC_MXLEMMING : MOTOR_FOC_ORTEGA;
VescObserverType motor_observer_type() {
    return motor_mode == MOTOR_FOC_MXLEMMING ? VESC_OBSERVER_MXLEMMING_LAMBDA_COMP
        : VESC_OBSERVER_ORTEGA_LAMBDA_COMP;
}
const float kPi = 3.14159265358979323846f;
enum MotorState { ZERO, IDLE, BOOTSTRAP, DETECT_R, DETECT_L,
    ALIGN, OPEN_LOOP, DETECT_FLUX, BLEND /* 旧协议保留值，正常启动不再经过此状态 */, RUN, FAULT, RETRY_WAIT,
    MUSIC }; // 12: fixed-coordinate AC-current test; never auto-restarts.
enum MotorFault { NO_FAULT, BAD_ZERO, OVERCURRENT, ADC_ERROR, ADC_TIMEOUT,
    BAD_BUS, BAD_CONFIG, ALIGN_FAILED, STARTUP_TIMEOUT,
    LOST_LOCK, DETECT_FAILED, NUMERIC_ERROR, PWM_ERROR, BLDC_FEEDBACK_ERROR };

volatile MotorState motor_state = ZERO;
volatile MotorFault motor_fault = NO_FAULT;
volatile uint32_t sample_frames = 0;
uint32_t service_last_frame = 0, service_last_adc = 0, service_last_status = 0;
volatile uint32_t isr_max_cycles = 0;
volatile uint32_t observer_max_cycles = 0, current_max_cycles = 0, pwm_max_cycles = 0;
volatile uint32_t pwm_write_max_cycles = 0;
uint32_t isr_start_cycle;
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
music_tone::Player music_player = {};
volatile bool music_requested = false, music_status_pending = false;
uint32_t service_last_music = 0u;
volatile uint32_t music_last_command_ms = 0u;
volatile bool run_requested = false;
bool current_mode = false;
float iq_setpoint = cfg::current, iq_reference = 0.0f;
float iq_voltage_limit = cfg::current;
float speed_setpoint = cfg::default_speed_rpm, speed_current_limit = cfg::current;
float requested_current_limit() { return current_mode ? iq_setpoint : speed_current_limit; }
float speed_reference = 0.0f, speed_feedback = 0.0f, speed_elapsed = 0.0f;
// Latched by the current loop until the next speed-loop update.
bool speed_voltage_limited = false;
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
// Preserve the first rejected active BLDC frame; later idle samples keep arriving.
struct BldcFeedbackFaultCapture { float phase[3], bus; uint32_t frame; };
BldcFeedbackFaultCapture bldc_feedback_fault_capture = {};
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

// Transport-independent, coherent SI input frame. The board adapter owns raw ADC.
struct ControlInputFrame {
    float ia, ib, vu, vv, vw, bus;
    uint8_t supplied;
};
ControlInputFrame input_frame = {};
float encoder_electrical_angle = 0.0f;
bool encoder_angle_available = false;
