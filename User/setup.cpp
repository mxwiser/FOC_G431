#include "setup.hpp"
#include "main.h"
#include "adc.h"
#include "opamp.h"
#include "tim.h"
#include "usart.h"
#include "foc/foc_math.hpp"
#include "foc/svpwm.h"
#include "foc/vesc_observer.hpp"
#include "foc/vesc_startup.hpp"
#include "serial_command.hpp"
#include "motor_mode.hpp"
#include "bldc/bldc.hpp"
#include "bldc/phase_feedback.hpp"
#include "bldc/timer_output.hpp"
#include <cmath>
#include <stdio.h>
#if defined(__CC_ARM)
#pragma Otime
#endif

namespace {
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

__align(4) uint16_t adc_buffer[4];
volatile MotorState motor_state = ZERO;
volatile MotorFault motor_fault = NO_FAULT;
volatile uint32_t sample_frames = 0;
volatile uint32_t isr_max_cycles = 0;
volatile uint32_t observer_max_cycles = 0, current_max_cycles = 0, pwm_max_cycles = 0;
volatile uint32_t pwm_write_max_cycles = 0;
uint32_t isr_start_cycle;
uint32_t zero_count = 0, zero_sum[3] = {};
float zero_a, zero_b, zero_c;
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
enum CommandResult { COMMAND_NONE, COMMAND_OK, COMMAND_FORMAT, COMMAND_RANGE, COMMAND_BUSY, COMMAND_RX,
    COMMAND_FEEDBACK, COMMAND_UNSUPPORTED };
CommandResult command_result = COMMAND_NONE;
uint8_t uart_rx_byte;
volatile uint8_t uart_rx_data[256];
volatile uint8_t uart_rx_write = 0, uart_rx_read = 0;
volatile bool uart_rx_overflow = false, uart_rx_error = false;
// UART and RX DMA IRQs must use the same preemption priority.  Both invoke
// the receive-to-idle callback and therefore share this DMA cursor.
volatile uint8_t uart_rx_dma_data[128];
uint16_t uart_rx_dma_read = 0;
SerialCommandParser command_parser = {};
// Telemetry selection only; ADC sampling remains active for FOC and protection.
volatile uint8_t phase_telemetry_mask = 0u;
// ADC is the sole producer; foreground is the sole consumer. The producer
// never overwrites unread slots, including while DMA is transmitting.
struct PhaseSample { uint32_t frame; int16_t a, b, c; };
PhaseSample phase_samples[128];
volatile uint8_t phase_write = 0u, phase_read = 0u;
volatile uint32_t phase_dropped = 0u;
const uint32_t kPhaseStride = 12u;

int16_t phase_milliamps(float value)
{
    value *= 1000.0f;
    if (value > 32767.0f) return 32767;
    if (value < -32768.0f) return -32768;
    return static_cast<int16_t>(value);
}

void capture_phase_sample()
{
    const uint8_t selected = phase_telemetry_mask;
    if (!selected || sample_frames % kPhaseStride) return;
    const uint8_t write = phase_write;
    const uint8_t next = (write + 1u) & 127u;
    if (next == phase_read) { ++phase_dropped; return; }
    PhaseSample &sample = phase_samples[write];
    sample.frame = sample_frames;
    sample.a = (selected & 1u) ? phase_milliamps(I_a) : 0;
    sample.b = (selected & 2u) ? phase_milliamps(I_b) : 0;
    sample.c = (selected & 4u) ? phase_milliamps(I_c) : 0;
    __DMB();
    phase_write = next;
}
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

float clamp(float value, float limit)
{
    if (value > limit) return limit;
    if (value < -limit) return -limit;
    return value;
}

void bridge_off()
{
    __HAL_TIM_MOE_DISABLE_UNCONDITIONALLY(&htim1);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0u);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0u);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0u);
    prior_alpha = prior_beta = older_alpha = older_beta = 0.0f;
    iq_reference = 0.0f;
    iq_voltage_limit = cfg::max_iq;
    speed_reference = speed_elapsed = 0.0f;
}

void motor_fail(MotorFault fault)
{
    run_requested = false;
    bridge_off();
    if (motor_fault == NO_FAULT) motor_fault = fault;
    motor_state = FAULT;
}

void enter_state(MotorState state)
{
    motor_state = state;
    state_time = stable_time = lost_time = 0.0f;
    state_samples = 0;
    if (state == ALIGN) { align_d = I_alpha; align_q = I_beta; }
    if (state == RUN && current_mode) {
        const float torque_current = cfg::direction * I_q;
        pi_speed.integral = torque_current < 0.0f ? 0.0f :
            (torque_current > cfg::current ? cfg::current : torque_current);
    }
    sum_i = sum_v = lockin_ic = lockin_is = lockin_vc = lockin_vs = 0.0f;
    emf_d = emf_q = emf_abs = 0.0f;
    filtered_emf_a = filtered_emf_b = 0.0f;
    detect_phase = 0.0f;
    detect_samples = 0;
}

void retry_motor(MotorFault reason)
{
    if (detecting || !run_requested) { motor_fail(reason); return; }
    bridge_off();
    restart_reason = reason;
    coast_until = HAL_GetTick() + cfg::retry_delay_ms;
    enter_state(RETRY_WAIT);
}

bool limits_valid()
{
    return foc_finite(cfg::current) && cfg::current >= 0.1f &&
        foc_finite(cfg::max_iq) && cfg::max_iq > 0.0f &&
        foc_finite(cfg::overcurrent) && cfg::overcurrent > cfg::current * 1.5f &&
        cfg::overcurrent > cfg::max_iq * 1.5f &&
        foc_finite(cfg::min_bus) && cfg::min_bus > 0.0f &&
        foc_finite(cfg::max_bus) && cfg::max_bus > cfg::min_bus &&
        (cfg::direction == 1.0f || cfg::direction == -1.0f);
}

bool model_valid()
{
    return foc_finite(motor_resistance) && motor_resistance > 0.001f && motor_resistance < 100.0f &&
        foc_finite(motor_inductance) && motor_inductance > 0.000001f && motor_inductance < 0.1f &&
        foc_finite(motor_flux) && motor_flux > 0.00001f && motor_flux < 1.0f;
}

void configure_controllers()
{
    PID cleared = {};
    pi_d = pi_q = cleared;
    pi_speed = cleared;
    pi_speed.dt = sample_dt;
    pi_speed.kp = cfg::speed_kp;
    pi_speed.ki = cfg::speed_ki;
    pi_speed.output_limit = speed_current_limit;
    pi_d.dt = pi_q.dt = sample_dt;
    const float bandwidth = detecting ? cfg::current_bandwidth : cfg::run_current_bandwidth;
    pi_d.kp = pi_q.kp = motor_inductance * bandwidth;
    pi_d.ki = pi_q.ki = motor_resistance * bandwidth;
    observer_config.resistance = motor_resistance;
    observer_config.inductance = motor_inductance;
    observer_config.flux_linkage = motor_flux;
    // VESC detection uses gamma = 500 / lambda^2.
    observer_gain_base = motor_flux > 0.0f ? 500.0f / (motor_flux * motor_flux) : 0.0f;
    observer_config.gain = observer_gain_base * cfg::observer_gain_slow * 4.0f;
    observer_config.pll_kp = 1000.0f;
    observer_config.pll_ki = 250000.0f;
    observer_config.minimum_speed = cfg::minimum_rad_s;
    observer_config.type = motor_observer_type();
    vesc_observer_reset(observer_config, observer, 0.0f);
    observer_phase = 0.0f;
    VescStartupState cleared_startup = {};
    startup = cleared_startup;
    startup_elapsed = startup_iq_filtered = 0.0f;
    startup_config.openloop_speed = cfg::startup_openloop_rad_s;
    startup_config.openloop_low = cfg::startup_openloop_low;
    startup_config.hysteresis_seconds = cfg::startup_hysteresis_seconds;
    startup_config.lock_seconds = cfg::startup_lock_seconds;
    startup_config.ramp_seconds = cfg::startup_ramp_seconds;
    startup_config.hold_seconds = cfg::startup_hold_seconds;
    startup_config.flux_linkage = motor_flux;
    speed_reference = speed_feedback = speed_elapsed = 0.0f;
}

void update_iq_voltage_limit()
{
    const float maximum = Udc_quant * cfg::max_modulation * ONE_BY_SQRT3;
    const float emf = foc_abs(observer.speed) * motor_flux;
    const float range = maximum * (cfg::iq_voltage_derate_end - cfg::iq_voltage_derate_start);
    float fraction = (maximum * cfg::iq_voltage_derate_end - emf) / range;
    if (fraction < 0.0f) fraction = 0.0f;
    iq_voltage_limit = fraction * cfg::iq_voltage_derate_scale;
    if (iq_voltage_limit > cfg::max_iq) iq_voltage_limit = cfg::max_iq;
}

float speed_control_reference()
{
    const float filter = sample_dt / (cfg::speed_filter_seconds + sample_dt);
    speed_feedback += filter * (cfg::direction * motor_speed() - speed_feedback);
    speed_elapsed += sample_dt;
    const float limit = speed_current_limit < iq_voltage_limit ? speed_current_limit : iq_voltage_limit;
    if (speed_elapsed >= cfg::speed_loop_seconds) {
        const float elapsed = speed_elapsed;
        speed_elapsed = 0.0f;
        const float rpm_to_electrical = 2.0f * kPi * FOC_POLE_PAIRS / 60.0f;
        // Use the slower end of the move, so a step to or from low speed
        // cannot bypass the low-speed ramp limit.
        const float reference_rpm = speed_reference / rpm_to_electrical;
        const float ramp_speed_rpm = reference_rpm < speed_setpoint ? reference_rpm : speed_setpoint;
        const float ramp_fraction = clamp((ramp_speed_rpm - cfg::speed_ramp_fast_start_rpm) /
            (cfg::speed_ramp_fast_full_rpm - cfg::speed_ramp_fast_start_rpm), 1.0f);
        const float ramp = cfg::speed_ramp_rpm_per_second +
            (cfg::speed_ramp_fast_rpm_per_second - cfg::speed_ramp_rpm_per_second) *
            (ramp_fraction > 0.0f ? ramp_fraction : 0.0f);
        speed_reference += clamp(speed_setpoint * rpm_to_electrical - speed_reference,
            ramp * rpm_to_electrical * elapsed);
        pi_speed.dt = elapsed;
        pi_speed.output_limit = limit;
        pi_speed.input = speed_reference - speed_feedback;
        calculate_PI(&pi_speed);
        // Forward torque only: deceleration coasts, avoiding regenerative bus rise.
        if (pi_speed.integral < 0.0f) pi_speed.integral = 0.0f;
        if (pi_speed.output < 0.0f) pi_speed.output = 0.0f;
    }
    // The current loop remains at the ADC rate. Clamp rapidly if headroom shrinks.
    float reference = cfg::direction * iq_reference;
    const float target = pi_speed.output < limit ? pi_speed.output : limit;
    reference += clamp(target-reference,cfg::speed_iq_slew_per_second * sample_dt);
    if (reference > limit) reference = limit;
    if (reference < 0.0f) reference = 0.0f;
    return cfg::direction * reference;
}

#include "foc/drive_impl.hpp"
#include "foc/parameter_detection_impl.hpp"
#include "bldc/drive_impl.hpp"

void control_sample()
{
    state_time = ++state_samples * sample_dt;
    if (motor_state == BOOTSTRAP) {
        if (state_time >= 0.005f) {
            if (detecting) { detect_voltage = 0.0f; detect_stage = 0; enter_state(DETECT_R); }
            else {
                configure_controllers(); openloop_angle = openloop_speed = 0.0f;
                bldc::reset(bldc_state);
                enter_state(OPEN_LOOP);
            }
        }
        return;
    }
    if (motor_state == DETECT_R) { update_resistance(); return; }
    if (motor_state == DETECT_L) { update_inductance(); return; }
    if (motor_state == DETECT_FLUX) { update_flux(); return; }
    if (!detecting && (motor_state == OPEN_LOOP || motor_state == RUN)) {
        if (motor_mode == MOTOR_BLDC) bldc_drive_sample();
        else sensorless_drive_sample();
        return;
    }
    if (motor_state == ALIGN) {
        // 仅参数辨识流程使用 Id 对齐；正常启动已在 BOOTSTRAP 后直接进入 Iq 驱动。
        const float fraction = state_time < 0.3f ? state_time / 0.3f : 1.0f;
        current_control(0.0f, cfg::current * fraction, 0.0f);
        const float filter = sample_dt / (0.01f + sample_dt);
        align_d += filter * (I_d - align_d); align_q += filter * (I_q - align_q);
        if (state_time > 0.5f && foc_abs(align_d - cfg::current) < cfg::current * 0.3f &&
            foc_abs(align_q) < cfg::current * 0.3f) stable_time += sample_dt;
        else stable_time = 0.0f;
        if (state_time >= cfg::align_seconds) {
            if (stable_time < 0.15f) {
                if (state_time >= cfg::align_seconds + cfg::align_settle_grace) retry_motor(ALIGN_FAILED);
                return;
            }
            vesc_observer_reset(observer_config, observer, 0.0f);
            if (observer_config.type == VESC_OBSERVER_ORTEGA_LAMBDA_COMP) {
                observer.x1 += motor_inductance * I_alpha; observer.x2 += motor_inductance * I_beta;
            }
            observer_phase = 0.0f;
            enter_state(OPEN_LOOP);
        }
        return;
    }
    if (motor_state == OPEN_LOOP) {
        const float destination = cfg::direction * cfg::openloop_rad_s *
            (detecting && detect_stage == 0 ? 0.7f : 1.0f);
        const float step = cfg::openloop_rad_s * sample_dt / cfg::ramp_seconds;
        openloop_speed += clamp(destination - openloop_speed, step);
        openloop_angle = vesc_observer_wrap(openloop_angle + openloop_speed * sample_dt);
        current_control(openloop_angle, cfg::current, 0.0f);
        if (foc_abs(openloop_speed - destination) < 0.01f) enter_state(DETECT_FLUX);
        return;
    }
    motor_fail(BAD_CONFIG);
}

bool start_motor(bool detect)
{
    if (motor_state != IDLE || !limits_valid() || (!detect && !parameters_ready)) return false;
    if (!detect && current_mode && iq_setpoint <= 0.0f) return false;
    if (!detect && !current_mode && speed_setpoint <= 0.0f) return false;
    if (motor_mode == MOTOR_BLDC && (detect || !bldc_feedback_ready())) return false;
    if (static_cast<int32_t>(HAL_GetTick() - coast_until) < 0) return false;
    if (Udc_quant < cfg::min_bus || Udc_quant > cfg::max_bus) { motor_fail(BAD_BUS); return false; }
    const uint32_t mask = __get_PRIMASK(); __disable_irq();
    detecting = detect;
    run_requested = !detect;
    if (detect) parameters_ready = false;
    bridge_off();
    motor_timer_foc(); // Bootstrap all three low sides before either drive starts.
    __HAL_TIM_SET_COUNTER(&htim1, 0u);
    if (HAL_TIM_GenerateEvent(&htim1, TIM_EVENTSOURCE_UPDATE) != HAL_OK) {
        motor_fail(PWM_ERROR); __set_PRIMASK(mask); return false;
    }
    enter_state(BOOTSTRAP);
    if (!bridge_started) {
        if (HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1) != HAL_OK ||
            HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1) != HAL_OK ||
            HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2) != HAL_OK ||
            HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2) != HAL_OK ||
            HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3) != HAL_OK ||
            HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3) != HAL_OK) motor_fail(PWM_ERROR);
        else bridge_started = true;
    } else __HAL_TIM_MOE_ENABLE(&htim1);
    __set_PRIMASK(mask);
    return motor_state == BOOTSTRAP;
}

void service_command(uint8_t command)
{
    if (command == 's' || command == 'S') {
        const uint32_t mask = __get_PRIMASK(); __disable_irq();
        run_requested = false;
        restart_reason = NO_FAULT;
        bridge_off();
        // Stop does not clear latched faults. Reset is required for that.
        if (motor_fault == NO_FAULT && motor_state != ZERO) {
            coast_until = HAL_GetTick() + 2000u; enter_state(IDLE);
        }
        __set_PRIMASK(mask);
        command_result = COMMAND_OK;
    } else if (command == 'd' || command == 'D' || command == 'r' || command == 'R') {
        if (motor_mode == MOTOR_BLDC && (command == 'd' || command == 'D')) {
            command_result = COMMAND_UNSUPPORTED; return;
        }
        if (motor_mode == MOTOR_BLDC && !bldc_feedback_ready()) {
            command_result = COMMAND_FEEDBACK; return;
        }
        if (start_motor(command == 'd' || command == 'D')) {
            retry_attempts = 0;
            restart_reason = NO_FAULT;
            command_result = COMMAND_OK;
        } else command_result = COMMAND_BUSY;
    }
}

void service_restart()
{
    if (motor_state != RETRY_WAIT || !run_requested || motor_fault != NO_FAULT ||
        static_cast<int32_t>(HAL_GetTick() - coast_until) < 0) return;
    const uint32_t mask = __get_PRIMASK(); __disable_irq();
    enter_state(IDLE);
    if (start_motor(false)) ++retry_attempts;
    __set_PRIMASK(mask);
}

HAL_StatusTypeDef start_uart_receive()
{
    if (huart2.hdmarx) {
        if (huart2.hdmarx->Init.Mode != DMA_CIRCULAR) return HAL_ERROR;
        uart_rx_dma_read = 0;
        return HAL_UARTEx_ReceiveToIdle_DMA(&huart2, const_cast<uint8_t *>(uart_rx_dma_data), sizeof(uart_rx_dma_data));
    }
    return HAL_UART_Receive_IT(&huart2, &uart_rx_byte, 1);
}

void service_serial()
{
recover_uart:
    if (uart_rx_overflow || uart_rx_error) {
        const uint32_t mask = __get_PRIMASK(); __disable_irq();
        uart_rx_read = uart_rx_write;
        uart_rx_overflow = uart_rx_error = false;
        command_parser.length = 0;
        command_parser.discarding = true;
        __set_PRIMASK(mask);
        service_command('S');
        command_result = COMMAND_RX;
        HAL_UART_AbortReceive(&huart2);
        uart_rx_read = uart_rx_write;
        if (start_uart_receive() != HAL_OK) { uart_rx_error = true; return; }
    }
    while (!uart_rx_overflow && !uart_rx_error && uart_rx_read != uart_rx_write) {
        const uint8_t byte = uart_rx_data[uart_rx_read];
        uart_rx_read = static_cast<uint8_t>(uart_rx_read + 1u);
        SerialCommand command;
        if (!serial_command_feed(command_parser, byte, command)) continue;
        // Do not execute an already-queued command after an RX fault.  ADC
        // remains protected by the short bounded command critical section.
        const uint32_t command_mask = __get_PRIMASK(); __disable_irq();
        if (uart_rx_overflow || uart_rx_error) { __set_PRIMASK(command_mask); break; }
        if (command.type == SERIAL_STOP) service_command('S');
        else if (command.type == SERIAL_START) service_command('R');
        else if (command.type == SERIAL_DETECT) service_command('D');
        else if (command.type == SERIAL_SET_MOTOR_MODE) {
            if (!foc_finite(command.value) || command.value < 0.0f || command.value > 2.0f ||
                command.value != static_cast<float>(static_cast<unsigned>(command.value))) {
                command_result = COMMAND_RANGE;
            } else if (motor_state != IDLE || run_requested || motor_fault != NO_FAULT) {
                command_result = COMMAND_BUSY;
            } else {
                bridge_off();
                motor_timer_foc();
                motor_mode = static_cast<MotorMode>(static_cast<unsigned>(command.value));
                detecting = false;
                bldc::reset(bldc_state);
                configure_controllers();
                command_result = COMMAND_OK;
            }
        }
        else if (command.type == SERIAL_SET_PHASES) {
            if (foc_finite(command.value) && command.value >= 0.0f && command.value <= 7.0f) {
                const uint8_t phases = static_cast<uint8_t>(command.value);
                if (command.value == static_cast<float>(phases) && phases != phase_telemetry_mask) {
                    const uint32_t mask = __get_PRIMASK(); __disable_irq();
                    phase_read = phase_write; // discard samples from the previous selection
                    phase_telemetry_mask = phases;
                    __set_PRIMASK(mask);
                }
            }
        }
        else if (command.type == SERIAL_SET_IQ || command.type == SERIAL_SET_SPEED ||
                 command.type == SERIAL_SET_SPEED_LIMIT) {
            const bool speed = command.type == SERIAL_SET_SPEED;
            const bool limit = command.type == SERIAL_SET_SPEED_LIMIT;
            const bool range_ok = speed ? (command.value == 0.0f ||
                (command.value >= cfg::min_speed_rpm && command.value <= cfg::max_speed_rpm)) :
                (command.value >= (limit ? 0.1f : 0.0f) && command.value <= cfg::max_iq);
            if (!foc_finite(command.value) || !range_ok) {
                command_result = COMMAND_RANGE;
            } else if (motor_state == FAULT || (detecting && motor_state != IDLE && motor_state != ZERO)) {
                command_result = COMMAND_BUSY;
            } else {
                const uint32_t mask = __get_PRIMASK(); __disable_irq();
                if (limit) speed_current_limit = command.value;
                else if (speed) {
                    speed_setpoint = command.value;
                    if (current_mode) {
                        speed_reference = speed_feedback = cfg::direction * motor_speed();
                        speed_elapsed = 0.0f;
                        pi_speed.integral = pi_speed.output = cfg::direction * iq_reference;
                    }
                    current_mode = false;
                    if (speed_setpoint == 0.0f) service_command('S');
                } else {
                    iq_setpoint = command.value;
                    current_mode = true;
                    if (iq_setpoint == 0.0f) service_command('S');
                }
                __set_PRIMASK(mask);
                command_result = COMMAND_OK;
            }
        } else command_result = COMMAND_FORMAT;
        __set_PRIMASK(command_mask);
    }
    if (uart_rx_overflow || uart_rx_error) goto recover_uart;
}

bool send_status()
{
    if (huart2.gState != HAL_UART_STATE_READY) return false;
    static char text[448]; // Persistent until the interrupt-driven TX completes.
    const uint32_t mask = __get_PRIMASK(); __disable_irq();
    const MotorState state = motor_state; const MotorFault fault = motor_fault;
    const bool inactive = state == IDLE || state == ZERO || state == RETRY_WAIT;
    const float bus = Udc_quant, id = inactive ? 0.0f : I_d, iq = inactive ? 0.0f : I_q;
    const float speed = inactive ? 0.0f : motor_speed();
    const float r = motor_resistance, l = motor_inductance, flux = motor_flux, ia = I_alpha;
    // Same ADC frame, before Clarke/Park or observer filtering. Keep the
    // legacy ia=Ialpha field for existing serial clients; i_a/i_b/i_c are phases.
    const uint8_t phases = phase_telemetry_mask;
    const float phase_a = (phases & 1u) ? I_a : 0.0f;
    const float phase_b = (phases & 2u) ? I_b : 0.0f;
    const float phase_c = (phases & 4u) ? I_c : 0.0f;
    const float angle = electrical_angle;
    const uint32_t cycles = isr_max_cycles, write_cycles = pwm_write_max_cycles;
    const bool ready = parameters_ready;
    const bool locked = !inactive && motor_fault == NO_FAULT &&
        (motor_mode == MOTOR_BLDC ? bldc_state.locked : observer.ready);
    const bool forced_phase = !inactive && !detecting &&
        (motor_mode == MOTOR_BLDC ? !bldc_state.locked : startup.phase_override);
    const char observer_type = motor_mode == MOTOR_BLDC ? 'B' :
        (observer_config.type == VESC_OBSERVER_MXLEMMING_LAMBDA_COMP ? 'M' : 'O');
    const unsigned drive = static_cast<unsigned>(motor_mode);
    const bool feedback = bldc_feedback_ready();
    const uint32_t frames = sample_frames;
    const float target_iq = iq_setpoint, reference_iq = iq_reference, iq_cap = iq_voltage_limit;
    const bool mode = current_mode, armed = run_requested;
    const uint32_t retries = retry_attempts;
    const MotorFault reason = restart_reason;
    const CommandResult result = command_result;
    const float target_rpm = cfg::direction * speed_setpoint;
    const float reference_rpm = inactive || current_mode ? 0.0f :
        cfg::direction * speed_reference * 60.0f / (2.0f * kPi * FOC_POLE_PAIRS);
    const float speed_limit = speed_current_limit;
    __set_PRIMASK(mask);
    int length = snprintf(text, sizeof(text),
        "OBS,s=%d,f=%d,udc=%.3f,ia=%.4f,id=%.4f,iq=%.4f,rpm=%.2f,ready=%d,params=%d,R=%.6f,L=%.9f,flux=%.8f,n=%lu,angle=%.3f,isr=%lu,pwm=%lu,mode=%c,iqset=%.3f,iqref=%.3f,auto=%d,retry=%lu,why=%d,cmd=%d,iqcap=%.3f,vset=%.1f,vref=%.1f,ilim=%.3f,ph=%u,ol=%d,obs=%c,drv=%u,bfb=%d",
        static_cast<int>(state), static_cast<int>(fault), bus, ia, id, iq,
        speed * 60.0f / (2.0f * kPi * FOC_POLE_PAIRS), locked ? 1 : 0, ready ? 1 : 0,
        r, l, flux, static_cast<unsigned long>(frames), angle,
        static_cast<unsigned long>(cycles), static_cast<unsigned long>(write_cycles),
        mode ? 'I' : 'S', target_iq, reference_iq, armed ? 1 : 0,
        static_cast<unsigned long>(retries), static_cast<int>(reason), static_cast<int>(result), iq_cap,
        target_rpm, reference_rpm, speed_limit, static_cast<unsigned>(phases), forced_phase ? 1 : 0,
        observer_type, drive, feedback ? 1 : 0);
    if (length <= 0 || length >= static_cast<int>(sizeof(text))) return false;
    const char *names[] = {",i_a=%.4f", ",i_b=%.4f", ",i_c=%.4f"};
    const float values[] = {phase_a, phase_b, phase_c};
    for (unsigned i = 0; i < 3; ++i) {
        if (!(phases & (1u << i))) continue;
        const int added = snprintf(text + length, sizeof(text) - length, names[i], values[i]);
        if (added < 0 || added >= static_cast<int>(sizeof(text) - length)) return false;
        length += added;
    }
    if (length + 2 >= static_cast<int>(sizeof(text))) return false;
    text[length++] = '\r'; text[length++] = '\n';
    return HAL_UART_Transmit_DMA(&huart2, reinterpret_cast<uint8_t *>(text), static_cast<uint16_t>(length)) == HAL_OK;
}

// Append a signed decimal field to a bounded, fixed-size packet.
void append_phase_number(char *text, unsigned &length, int32_t number)
{
    text[length++] = ',';
    uint32_t magnitude;
    if (number < 0) { text[length++] = '-'; magnitude = static_cast<uint32_t>(-number); }
    else magnitude = static_cast<uint32_t>(number);
    char digits[10]; unsigned count = 0;
    do { digits[count++] = static_cast<char>('0' + magnitude % 10u); magnitude /= 10u; } while (magnitude);
    while (count) text[length++] = digits[--count];
}

void send_phase_samples()
{
    if (huart2.gState != HAL_UART_STATE_READY) return;
    const uint8_t selected = phase_telemetry_mask;
    uint8_t read = phase_read;
    const uint8_t available = (phase_write - read) & 127u;
    if (!selected || available < 16u) return;
    // Worst case: 16 * 3 * 7 bytes plus header, < 448 bytes.
    static char text[448];
    unsigned length = 3; text[0] = 'C'; text[1] = 'U'; text[2] = 'R';
    const uint32_t first_frame = phase_samples[read].frame;
    // Frame/drop are encoded as unsigned decimals (cast avoids signed rollover).
    const int header = snprintf(text, sizeof(text), "CUR,%lu,%u,%lu,%lu",
        static_cast<unsigned long>(first_frame), static_cast<unsigned>(selected),
        static_cast<unsigned long>(phase_dropped),
        static_cast<unsigned long>(sample_dt * 1000000000.0f + 0.5f));
    if (header <= 0 || header >= static_cast<int>(sizeof(text))) return;
    length = static_cast<unsigned>(header);
    for (unsigned i = 0; i < 16u; ++i) {
        const PhaseSample &sample = phase_samples[read];
        // A full ring causes missed frames: terminate the batch before the gap.
        if (sample.frame != first_frame + i * kPhaseStride) break;
        if (selected & 1u) append_phase_number(text, length, sample.a);
        if (selected & 2u) append_phase_number(text, length, sample.b);
        if (selected & 4u) append_phase_number(text, length, sample.c);
        read = (read + 1u) & 127u;
    }
    text[length++] = '\r'; text[length++] = '\n';
    if (HAL_UART_Transmit_DMA(&huart2, reinterpret_cast<uint8_t *>(text),
                              static_cast<uint16_t>(length)) == HAL_OK) phase_read = read;
}

#if defined(FOC_CURRENT_DIAG) && FOC_CURRENT_DIAG
void record_current_diagnostic()
{
    const bool timeout = diag_frame_deadline &&
        static_cast<int32_t>(sample_frames - diag_frame_deadline) >= 0;
    if (diag_mode && !diag_frozen && motor_state == diag_state &&
        state_samples >= diag_min_samples && foc_abs(observer.speed) >= diag_min_speed &&
        diag_stride && sample_frames % diag_stride == 0) {
        CurrentDiagnosticSample &r = current_diag[diag_count % 192u];
        r.frame = sample_frames; r.state = motor_state;
        r.raw_a = adc_buffer[0]; r.raw_b = adc_buffer[1];
        r.raw_c = adc_buffer[3]; r.raw_bus = adc_buffer[2];
        r.ia = I_a; r.ib = I_b; r.ic = I_c; r.id = I_d; r.iq = I_q;
        r.iqref = iq_reference; r.phase = electrical_angle;
        r.observer_phase = observer_phase; r.speed = observer.speed;
        r.flux = observer.flux_magnitude; r.bus = Udc_quant;
        r.va = applied_alpha; r.vb = applied_beta;
        r.command_va = prior_alpha * Udc_quant; r.command_vb = prior_beta * Udc_quant;
        r.integral_d = pi_d.integral; r.integral_q = pi_q.integral;
        r.observer_ia = I_alpha; r.observer_ib = I_beta;
        r.phase_error = observer.phase_error;
        r.raw_angle = observer.angle; r.pll_angle = observer.pll_angle;
        r.ccr_a = __HAL_TIM_GET_COMPARE(&htim1, TIM_CHANNEL_1);
        r.ccr_b = __HAL_TIM_GET_COMPARE(&htim1, TIM_CHANNEL_2);
        r.ccr_c = __HAL_TIM_GET_COMPARE(&htim1, TIM_CHANNEL_3);
        ++diag_count;
        if ((diag_mode == 1u && diag_count >= 192u) ||
            (diag_mode == 2u && (foc_abs(I_a) > diag_trigger_current ||
             foc_abs(I_b) > diag_trigger_current || foc_abs(I_c) > diag_trigger_current))) {
            diag_frozen = 1;
            if (diag_mode == 2u) {
                run_requested = false; bridge_off();
                coast_until = HAL_GetTick() + 2000u; enter_state(IDLE);
            }
            diag_mode = 0;
        }
    }
    if (timeout) {
        diag_frame_deadline = 0; diag_mode = 0; diag_frozen = 1;
        run_requested = false; bridge_off();
        coast_until = HAL_GetTick() + 2000u; enter_state(IDLE);
    }
}
#endif
} // namespace

extern "C" void bldc_publish_phase_voltages(float a, float b, float c)
{
    const uint32_t mask = __get_PRIMASK(); __disable_irq();
    bldc_feedback_published = foc_finite(a) && foc_finite(b) && foc_finite(c) &&
        a >= -0.5f && b >= -0.5f && c >= -0.5f &&
        a <= Udc_quant + 0.5f && b <= Udc_quant + 0.5f && c <= Udc_quant + 0.5f;
    bldc_phase_voltage[0] = a; bldc_phase_voltage[1] = b; bldc_phase_voltage[2] = c;
    bldc_feedback_frame = sample_frames;
    __DMB();
    __set_PRIMASK(mask);
}

static void adc_sample(ADC_HandleTypeDef *hadc)
{
    if (hadc != &hadc1) return;
    ++sample_frames;
    const volatile uint16_t *samples = adc_buffer;
    // 双 ADC 规则同步采样的 DMA 排列：Rank 1 为 ADC1/A、ADC2/B，Rank 2 为 ADC1/母线、ADC2/C。
    // A、B 相采样时刻一致；C 相晚一个转换序列步，不能与 A、B 当作同一时刻的三相电流。
    const uint16_t a = samples[0], b = samples[1], bus = samples[2], c = samples[3];
    Udc_quant = bus * (3.3f / 4095.0f) * (187.0f / 18.0f);
    if (motor_state == FAULT) { bridge_off(); return; }
    if (motor_state == ZERO) {
        if (a < 256 || a > 3840 || b < 256 || b > 3840 || c < 256 || c > 3840) {
            motor_fail(BAD_ZERO); return;
        }
        zero_sum[0] += a; zero_sum[1] += b; zero_sum[2] += c;
        if (++zero_count == kZeroSamples) {
            zero_a = static_cast<float>(zero_sum[0]) / kZeroSamples;
            zero_b = static_cast<float>(zero_sum[1]) / kZeroSamples;
            zero_c = static_cast<float>(zero_sum[2]) / kZeroSamples;
            enter_state(IDLE);
        }
        return;
    }
    I_a = (zero_a - a) * kAmpsPerCount; I_b = (zero_b - b) * kAmpsPerCount;
    I_c = (zero_c - c) * kAmpsPerCount;
    capture_phase_sample();
    last_I_alpha = I_alpha; last_I_beta = I_beta;
    // 电流环和观测器仅使用同步采样的 A、B 相做两相 Clarke；C 相实测值保留用于保护和遥测。
    clark_two_phase_ab(&I_a, &I_b, &I_alpha, &I_beta);
    if (motor_state == IDLE || motor_state == RETRY_WAIT) {
#if defined(FOC_CURRENT_DIAG) && FOC_CURRENT_DIAG
        record_current_diagnostic();
#endif
        return;
    }
    if (a == 0 || a == 4095 || b == 0 || b == 4095 || c == 0 || c == 4095 ||
        foc_abs(I_a) > cfg::overcurrent || foc_abs(I_b) > cfg::overcurrent ||
        foc_abs(I_c) > cfg::overcurrent) { motor_fail(OVERCURRENT); return; }
    if (bus == 4095 || Udc_quant < cfg::min_bus || Udc_quant > cfg::max_bus) { motor_fail(BAD_BUS); return; }
    // Peak sampling / valley CCR preload: half of each of the two last commands.
    applied_alpha = Udc_quant * 0.5f * (prior_alpha + older_alpha);
    applied_beta = Udc_quant * 0.5f * (prior_beta + older_beta);
    const bool update_observer = motor_mode != MOTOR_BLDC && !detecting &&
        (motor_state == OPEN_LOOP || motor_state == RUN);
    if (update_observer && motor_state != FAULT) {
#if defined(STM32G431xx)
        const uint32_t start_cycles = DWT->CYCCNT;
#endif
        // 对齐 VESC 的增益调度。prior_alpha/beta 为上一条实际 PWM 电压与母线电压之比，
        // |duty| = sqrt(3) * |Ualpha_beta| / Udc（对应 VESC 的 overmod_factor = 1）。
        const float duty = 1.7320508075688772f *
            foc_sqrt(prior_alpha * prior_alpha + prior_beta * prior_beta);
        observer_config.gain = vesc_observer_gain(observer_gain_base, cfg::observer_gain_slow,
                                                   duty, Udc_quant);
        // 重建电压和同步采样的两相 Clarke 电流直接送入观测器，不增加低通或滤波补偿。
        // 每次执行观测器均更新 lambda_est，与 VESC 的 ORTEGA_LAMBDA_COMP 分支一致。
        if (!vesc_observer_update(observer_config, observer, applied_alpha, applied_beta,
                                  I_alpha, I_beta, sample_dt, true)) motor_fail(NUMERIC_ERROR);
        observer_phase = observer.angle;
#if defined(STM32G431xx)
        const uint32_t cycles = DWT->CYCCNT - start_cycles;
        if (cycles > observer_max_cycles) observer_max_cycles = cycles;
#endif
    }
    if (motor_mode == MOTOR_BLDC && !detecting && !bldc_feedback_ready()) motor_fail(BLDC_FEEDBACK_ERROR);
    if (motor_state != FAULT) control_sample();
#if defined(FOC_CURRENT_DIAG) && FOC_CURRENT_DIAG
    record_current_diagnostic();
#endif
}

extern "C" void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
#if defined(STM32G431xx)
    const uint32_t start = DWT->CYCCNT;
    isr_start_cycle = start;
#endif
    adc_sample(hadc);
#if defined(STM32G431xx)
    const uint32_t cycles = DWT->CYCCNT - start;
    if (cycles > isr_max_cycles) isr_max_cycles = cycles;
#endif
}

extern "C" void HAL_ADC_ErrorCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc == &hadc1 || hadc == &hadc2) motor_fail(ADC_ERROR);
}

extern "C" void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart != &huart2 || huart->hdmarx) return;
    const uint8_t next = static_cast<uint8_t>(uart_rx_write + 1u);
    if (next == uart_rx_read) uart_rx_overflow = true;
    else { uart_rx_data[uart_rx_write] = uart_rx_byte; uart_rx_write = next; }
    if (HAL_UART_Receive_IT(&huart2, &uart_rx_byte, 1) != HAL_OK) uart_rx_error = true;
}

extern "C" void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t)
{
    if (huart != &huart2 || !huart->hdmarx) return;
    // Use the current hardware position rather than the event's Size. A
    // delayed half/full interrupt may be delivered after a newer idle event.
    // Both IRQs must run within half a DMA buffer time: NDTR cannot count
    // complete unseen laps if interrupts are blocked for an entire buffer.
    const uint16_t remaining = __HAL_DMA_GET_COUNTER(huart->hdmarx);
    if (remaining > sizeof(uart_rx_dma_data)) { uart_rx_error = true; return; }
    const uint16_t end = (sizeof(uart_rx_dma_data) - remaining) % sizeof(uart_rx_dma_data);
    while (uart_rx_dma_read != end) {
        const uint8_t next = static_cast<uint8_t>(uart_rx_write + 1u);
        if (next == uart_rx_read) { uart_rx_overflow = true; break; }
        uart_rx_data[uart_rx_write] = uart_rx_dma_data[uart_rx_dma_read];
        uart_rx_write = next;
        uart_rx_dma_read = (uart_rx_dma_read + 1u) % sizeof(uart_rx_dma_data);
    }
}

extern "C" void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart == &huart2) uart_rx_error = true;
}

void setup(void)
{
    // 零点采集/待机时也准确报告所选类型；控制器其余参数仍在每次启动时初始化。
    observer_config.type = motor_observer_type();
#if defined(STM32G431xx)
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
#endif
    // No TIM4 reads or encoder startup. Ignore an unplugged Z input.
    CLEAR_BIT(EXTI->IMR1, ENCODE_ZERO_Pin); __HAL_GPIO_EXTI_CLEAR_IT(ENCODE_ZERO_Pin);
    bridge_off();
    if (start_uart_receive() != HAL_OK) uart_rx_error = true;
    RCC_ClkInitTypeDef clocks = {}; uint32_t latency = 0;
    HAL_RCC_GetClockConfig(&clocks, &latency);
    const float timer_clock = static_cast<float>(HAL_RCC_GetPCLK2Freq())
        * (clocks.APB2CLKDivider == RCC_HCLK_DIV1 ? 1.0f : 2.0f);
    sample_dt = 2.0f * __HAL_TIM_GET_AUTORELOAD(&htim1) * (htim1.Init.Prescaler + 1.0f) / timer_clock;
    if (!limits_valid() || sample_dt <= 0.0f || sample_dt > 0.001f ||
        htim1.Init.CounterMode != TIM_COUNTERMODE_CENTERALIGNED2) motor_fail(BAD_CONFIG);
    parameters_ready = model_valid();
    if (HAL_OPAMP_Start(&hopamp1) != HAL_OK || HAL_OPAMP_Start(&hopamp2) != HAL_OK ||
        HAL_OPAMP_Start(&hopamp3) != HAL_OK) { motor_fail(ADC_ERROR); Error_Handler(); }
    HAL_Delay(1);
    if (HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED) != HAL_OK ||
        HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED) != HAL_OK) { motor_fail(ADC_ERROR); Error_Handler(); }
    if (HAL_ADCEx_MultiModeStart_DMA(&hadc1, reinterpret_cast<uint32_t *>(adc_buffer), 2) != HAL_OK) {
        motor_fail(ADC_ERROR); Error_Handler();
    }
    __HAL_DMA_DISABLE_IT(hadc1.DMA_Handle, DMA_IT_HT);
    if (HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4) != HAL_OK) { motor_fail(PWM_ERROR); Error_Handler(); }
    bridge_off(); // Sampling-channel HAL start enables MOE too.
    uint32_t last_frame = sample_frames, last_adc = HAL_GetTick(), last_status = HAL_GetTick();
    while (1) {
#if defined(STM32G431xx)
        // Debug-probe detach may restore DEMCR and clear TRCENA.
        CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
        DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
#endif
        service_serial();
        const uint32_t now = HAL_GetTick(), frame = sample_frames;
        if (frame != last_frame) { last_frame = frame; last_adc = now; }
        if (now - last_adc > 10u && motor_state != FAULT) motor_fail(ADC_TIMEOUT);
        service_restart();
        if (now - last_status >= 20u) {
            // Give state/command acknowledgements priority over waveform data.
            if (send_status()) last_status = now;
        } else send_phase_samples();
        __WFI();
    }
}
