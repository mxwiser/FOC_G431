#include "setup.hpp"
#include "main.h"
#include "adc.h"
#include "opamp.h"
#include "tim.h"
#include "usart.h"
#include "foc/foc_math.hpp"
#include "foc/svpwm.h"
#include "foc/vesc_observer.hpp"
#include "serial_command.hpp"
#include <cmath>
#include <stdio.h>
#if defined(__CC_ARM)
#pragma Otime
#endif

namespace {
namespace cfg = sensorless_config;
const float kPi = 3.14159265358979323846f;
const float kAmpsPerCount = 0.02946541f;
const uint32_t kZeroSamples = 2048u;
enum MotorState { ZERO, IDLE, BOOTSTRAP, DETECT_R, DETECT_L,
    ALIGN, OPEN_LOOP, DETECT_FLUX, BLEND, RUN, FAULT, RETRY_WAIT };
enum MotorFault { NO_FAULT, BAD_ZERO, OVERCURRENT, ADC_ERROR, ADC_TIMEOUT,
    BAD_BUS, BAD_CONFIG, ALIGN_FAILED, STARTUP_TIMEOUT,
    LOST_LOCK, DETECT_FAILED, NUMERIC_ERROR, PWM_ERROR };

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
float blend_offset;
float observer_phase;
float filtered_U_alpha, filtered_U_beta, filtered_I_alpha, filtered_I_beta;
const float kObserverFilterSeconds = 0.0003f;
float motor_resistance = cfg::resistance;
float motor_inductance = cfg::inductance;
float motor_flux = cfg::flux_linkage;
bool parameters_ready = false, detecting = false, bridge_started = false;
volatile bool run_requested = false;
bool current_mode = false;
float iq_setpoint = cfg::current, iq_reference = 0.0f;
uint32_t retry_attempts = 0;
MotorFault restart_reason = NO_FAULT;
enum CommandResult { COMMAND_NONE, COMMAND_OK, COMMAND_FORMAT, COMMAND_RANGE, COMMAND_BUSY, COMMAND_RX };
CommandResult command_result = COMMAND_NONE;
uint8_t uart_rx_byte;
volatile uint8_t uart_rx_data[64];
volatile uint8_t uart_rx_write = 0, uart_rx_read = 0;
volatile bool uart_rx_overflow = false, uart_rx_error = false;
SerialCommandParser command_parser = {};
uint32_t coast_until = 0;
float prior_alpha, prior_beta, older_alpha, older_beta;
float applied_alpha, applied_beta, last_I_alpha, last_I_beta;
PID pi_d = {}, pi_q = {}, pi_speed = {};
VescObserverConfig observer_config = {};
VescObserverState observer = {};
struct IqTraceSample {
    uint32_t frame;
    float id, iq, reference, phase, speed, flux, va, vb, ia, ib,
        filtered_va, filtered_vb, filtered_ia, filtered_ib, ud, uq;
};
IqTraceSample iq_trace[256];
uint32_t iq_trace_index = 0;

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
    if (state == BLEND) {
        pi_d.kp = pi_q.kp = motor_inductance * cfg::run_current_bandwidth;
        pi_d.ki = pi_q.ki = motor_resistance * cfg::run_current_bandwidth;
        blend_offset = vesc_observer_wrap(openloop_angle
            - observer_phase - observer.speed * sample_dt * 0.5f);
        if (current_mode) {
            // Preserve the torque already produced during open-loop tracking.
            // Starting from zero can drop below the sensorless minimum speed.
            float sn, cs;
            foc_fast_sincos(blend_offset, sn, cs);
            const float torque = cfg::direction * (I_d * sn + I_q * cs);
            iq_reference = cfg::direction * (torque < 0.0f ? 0.0f :
                (torque > iq_setpoint ? iq_setpoint : torque));
        }
    }
    if (state == RUN) {
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
        foc_finite(cfg::overcurrent) && cfg::overcurrent > cfg::current * 1.5f &&
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
    pi_speed.kp = cfg::current * 0.1f / cfg::openloop_rad_s;
    pi_speed.ki = pi_speed.kp * 5.0f;
    pi_speed.output_limit = cfg::current;
    pi_d.dt = pi_q.dt = sample_dt;
    pi_d.kp = pi_q.kp = motor_inductance * cfg::current_bandwidth;
    pi_d.ki = pi_q.ki = motor_resistance * cfg::current_bandwidth;
    observer_config.resistance = motor_resistance;
    observer_config.inductance = motor_inductance;
    observer_config.flux_linkage = motor_flux;
    // VESC detection uses gamma = 500 / lambda^2.
    observer_config.gain = motor_flux > 0.0f ? 500.0f / (motor_flux * motor_flux) : 0.0f;
    observer_config.pll_kp = 1000.0f;
    observer_config.pll_ki = 250000.0f;
    observer_config.minimum_speed = cfg::minimum_rad_s;
    vesc_observer_reset(observer_config, observer, 0.0f);
    observer_phase = 0.0f;
    filtered_U_alpha = filtered_U_beta = filtered_I_alpha = filtered_I_beta = 0.0f;
}

// SI volts -> VESC modulation -> timer ticks. Save quantized voltage / Udc.
void output_voltage(float U_alpha, float U_beta, bool vector_limited = false)
{
#if defined(STM32G431xx)
    const uint32_t start_cycles = DWT->CYCCNT;
#endif
    if (!foc_finite(U_alpha) || !foc_finite(U_beta) || !(Udc_quant > 0.0f)) {
        motor_fail(NUMERIC_ERROR); return;
    }
    const float maximum = Udc_quant * cfg::max_modulation * ONE_BY_SQRT3;
    if (!vector_limited) {
        const float magnitude = foc_sqrt(U_alpha * U_alpha + U_beta * U_beta);
        if (magnitude > maximum) { U_alpha *= maximum / magnitude; U_beta *= maximum / magnitude; }
    }
    uint32_t tA, tB, tC;
    const uint32_t period = __HAL_TIM_GET_AUTORELOAD(&htim1);
    svpwm_fast(1.5f * U_alpha / Udc_quant, 1.5f * U_beta / Udc_quant,
            cfg::max_modulation, period, &tA, &tB, &tC);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, tA);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, tB);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, tC);
#if defined(STM32G431xx)
    const uint32_t write_cycles = DWT->CYCCNT - isr_start_cycle;
    if (write_cycles > pwm_write_max_cycles) pwm_write_max_cycles = write_cycles;
#endif
    older_alpha = prior_alpha; older_beta = prior_beta;
    prior_alpha = (2.0f * tA - tB - tC) / (3.0f * period);
    prior_beta = (static_cast<float>(tB) - tC) * ONE_BY_SQRT3 / period;
#if defined(STM32G431xx)
    const uint32_t cycles = DWT->CYCCNT - start_cycles;
    if (cycles > pwm_max_cycles) pwm_max_cycles = cycles;
#endif
}

void current_control(float angle, float target_d, float target_q)
{
#if defined(STM32G431xx)
    const uint32_t start_cycles = DWT->CYCCNT;
#endif
    electrical_angle = angle;
    iq_reference = target_q;
    float sn, cs;
    foc_fast_sincos(angle, sn, cs);
    I_d = I_alpha * cs + I_beta * sn;
    I_q = -I_alpha * sn + I_beta * cs;
    const float maximum = Udc_quant * cfg::max_modulation * ONE_BY_SQRT3;
    pi_d.output_limit = pi_q.output_limit = maximum;
    pi_d.input = target_d - I_d; pi_q.input = target_q - I_q;
    calculate_PI(&pi_d); calculate_PI(&pi_q);
    float U_d = pi_d.output, U_q = pi_q.output;
    if (!detecting && motor_state != ALIGN && motor_flux > 0.0f) {
        // VESC-style cross-coupling and back-EMF feedforward. Important when
        // R and L are small: PI alone reacts too late during acceleration.
        float speed = openloop_speed;
        if (motor_state == BLEND) {
            const float fraction = state_time < cfg::blend_seconds ? state_time / cfg::blend_seconds : 1.0f;
            speed += fraction * (observer.speed - speed);
        } else if (motor_state == RUN) speed = observer.speed;
        const float difference = motor_state == OPEN_LOOP ? 0.0f :
            vesc_observer_wrap(observer_phase + observer.speed * sample_dt * 0.5f - angle);
        float difference_sin, difference_cos;
        foc_fast_sincos(difference, difference_sin, difference_cos);
        U_d += -speed * motor_inductance * I_q - speed * motor_flux * difference_sin;
        U_q += speed * motor_inductance * I_d + speed * motor_flux * difference_cos;
    }
    // Preserve d-axis regulation as back EMF consumes the voltage budget.
    // Uniform vector scaling also weakens Ud, allowing uncontrolled Id.
    const float limited_d = clamp(U_d, maximum);
    const float q_budget_sq = maximum * maximum - limited_d * limited_d;
    const float limited_q = clamp(U_q, foc_sqrt(q_budget_sq > 0.0f ? q_budget_sq : 0.0f));
    pi_d.integral += limited_d - U_d;
    pi_q.integral += limited_q - U_q;
    U_d = limited_d; U_q = limited_q;
    output_voltage(U_d * cs - U_q * sn, U_d * sn + U_q * cs, true);
    if ((motor_state == BLEND || motor_state == RUN) && sample_frames % 6u == 0u) {
        IqTraceSample &row = iq_trace[iq_trace_index++ & 255u];
        row.frame = sample_frames; row.id = I_d; row.iq = I_q;
        row.reference = target_q; row.phase = angle; row.speed = observer.speed;
        row.flux = observer.flux_magnitude; row.va = applied_alpha; row.vb = applied_beta;
        row.ia = I_alpha; row.ib = I_beta;
        row.filtered_va = filtered_U_alpha; row.filtered_vb = filtered_U_beta;
        row.filtered_ia = filtered_I_alpha; row.filtered_ib = filtered_I_beta;
        row.ud = U_d; row.uq = U_q;
    }
#if defined(STM32G431xx)
    const uint32_t cycles = DWT->CYCCNT - start_cycles;
    if (cycles > current_max_cycles) current_max_cycles = cycles;
#endif
}

void update_resistance()
{
    const float target = cfg::current * (detect_stage == 0 ? 0.5f : 1.0f);
    detect_voltage = clamp(detect_voltage + 20.0f * (target - I_alpha) * sample_dt, Udc_quant * 0.35f);
    output_voltage(detect_voltage, 0.0f);
    if (state_time > 2.0f) { sum_i += I_alpha; sum_v += applied_alpha; ++detect_samples; }
    if (state_time < 3.0f) return;
    const float mean_i = sum_i / detect_samples, mean_v = sum_v / detect_samples;
    if (detect_stage == 0) {
        low_i = mean_i; low_v = mean_v; detect_stage = 1; enter_state(DETECT_R);
    } else {
        // Two positive plateaus remove most of the constant bridge drop.
        motor_resistance = (mean_v - low_v) / (mean_i - low_i);
        detect_bias = mean_v - motor_resistance * mean_i;
        if (!foc_finite(motor_resistance) || motor_resistance <= 0.001f || motor_resistance >= 100.0f ||
            mean_i < cfg::current * 0.7f || mean_i - low_i < cfg::current * 0.25f) {
            motor_fail(DETECT_FAILED); return;
        }
        detect_stage = 0; enter_state(DETECT_L);
    }
}

void update_inductance()
{
    const float frequency = detect_stage == 0 ? 200.0f : 700.0f;
    const float omega = 2.0f * kPi * frequency;
    detect_phase = vesc_observer_wrap(detect_phase + omega * sample_dt);
    const float phase = detect_phase;
    if (state_time > 0.2f) {
        const float cs = std::cos(phase), sn = std::sin(phase);
        lockin_ic += I_alpha * cs; lockin_is += I_alpha * sn;
        lockin_vc += applied_alpha * cs; lockin_vs += applied_alpha * sn; ++detect_samples;
    }
    const float dc = motor_resistance * cfg::current * 0.75f + detect_bias;
    const float ac = motor_resistance * cfg::current * 0.375f;
    output_voltage(dc + ac * std::cos(phase), 0.0f);
    if (state_time < 1.2f) return;
    // Interval-average voltage -> current sample instant (sinusoidal steady state).
    const float half = omega * sample_dt * 0.5f, cs = std::cos(half), sn = std::sin(half);
    const float sinc = sn / half;
    const float vc = (lockin_vc * cs + lockin_vs * sn) / sinc;
    const float vs = (lockin_vs * cs - lockin_vc * sn) / sinc;
    const float current_sq = lockin_ic * lockin_ic + lockin_is * lockin_is;
    const float amplitude = 2.0f * foc_sqrt(current_sq) / detect_samples;
    const float inductance = (vc * lockin_is - vs * lockin_ic) / (current_sq * omega);
    const float ac_resistance = (vc * lockin_ic + vs * lockin_is) / current_sq;
    if (!foc_finite(inductance) || inductance <= 0.000001f || inductance >= 0.1f ||
        amplitude < 0.015f || foc_abs(ac_resistance / motor_resistance - 1.0f) > 0.35f) {
        motor_fail(DETECT_FAILED); return;
    }
    if (detect_stage == 0) {
        first_inductance = inductance; detect_stage = 1; enter_state(DETECT_L);
    } else {
        if (foc_abs(inductance / first_inductance - 1.0f) > 0.35f) { motor_fail(DETECT_FAILED); return; }
        motor_inductance = 0.5f * (inductance + first_inductance);
        configure_controllers(); openloop_angle = openloop_speed = 0.0f;
        detect_stage = 0; enter_state(ALIGN);
    }
}

void update_flux()
{
    openloop_angle = vesc_observer_wrap(openloop_angle + openloop_speed * sample_dt);
    const float phase_mid = openloop_angle - openloop_speed * sample_dt * 0.5f;
    const float ea = applied_alpha - motor_resistance * 0.5f * (I_alpha + last_I_alpha)
        - motor_inductance * (I_alpha - last_I_alpha) / sample_dt;
    const float eb = applied_beta - motor_resistance * 0.5f * (I_beta + last_I_beta)
        - motor_inductance * (I_beta - last_I_beta) / sample_dt;
    const float filter = sample_dt / (0.0005f + sample_dt);
    filtered_emf_a += filter * (ea - filtered_emf_a);
    filtered_emf_b += filter * (eb - filtered_emf_b);
    if (state_time > 0.5f) {
        const float cs = std::cos(phase_mid), sn = std::sin(phase_mid);
        emf_d += filtered_emf_a * cs + filtered_emf_b * sn;
        emf_q += -filtered_emf_a * sn + filtered_emf_b * cs;
        emf_abs += foc_sqrt(filtered_emf_a * filtered_emf_a + filtered_emf_b * filtered_emf_b);
        ++detect_samples;
    }
    current_control(openloop_angle, cfg::current, 0.0f);
    if (state_time < 1.5f) return;
    const float magnitude = foc_sqrt(emf_d * emf_d + emf_q * emf_q);
    const float lag = openloop_speed * 0.0005f;
    const float flux = magnitude * foc_sqrt(1.0f + lag * lag) / (detect_samples * foc_abs(openloop_speed));
    if (!foc_finite(flux) || flux <= 0.00001f || flux >= 1.0f ||
        magnitude < 0.7f * emf_abs || magnitude / detect_samples < 0.03f) {
        motor_fail(DETECT_FAILED); return;
    }
    if (detect_stage == 0) { first_flux = flux; detect_stage = 1; enter_state(OPEN_LOOP); }
    else {
        if (foc_abs(flux / first_flux - 1.0f) > 0.2f) { motor_fail(DETECT_FAILED); return; }
        motor_flux = 0.5f * (flux + first_flux); parameters_ready = model_valid();
        bridge_off(); coast_until = HAL_GetTick() + 2000u; enter_state(IDLE);
    }
}

void control_sample()
{
    state_time = ++state_samples * sample_dt;
    if (motor_state == BOOTSTRAP) {
        if (state_time >= 0.005f) {
            if (detecting) { detect_voltage = 0.0f; detect_stage = 0; enter_state(DETECT_R); }
            else { configure_controllers(); openloop_angle = openloop_speed = 0.0f; enter_state(ALIGN); }
        }
        return;
    }
    if (motor_state == DETECT_R) { update_resistance(); return; }
    if (motor_state == DETECT_L) { update_inductance(); return; }
    if (motor_state == DETECT_FLUX) { update_flux(); return; }
    if (motor_state == ALIGN) {
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
            observer.x1 += motor_inductance * I_alpha; observer.x2 += motor_inductance * I_beta;
            observer_phase = 0.0f;
            filtered_I_alpha = I_alpha; filtered_I_beta = I_beta;
            filtered_U_alpha = applied_alpha; filtered_U_beta = applied_beta;
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
        if (detecting) {
            if (foc_abs(openloop_speed - destination) < 0.01f) enter_state(DETECT_FLUX);
            return;
        }
        if (observer.ready && cfg::direction * observer.speed > cfg::minimum_rad_s &&
            foc_abs(observer.speed - openloop_speed) < 0.2f * cfg::openloop_rad_s &&
            foc_abs(vesc_observer_wrap(observer_phase - openloop_angle)) < 0.7f) stable_time += sample_dt;
        else stable_time = 0.0f;
        if (stable_time >= cfg::lock_seconds) { enter_state(BLEND); return; }
        if (state_time >= cfg::startup_timeout) retry_motor(STARTUP_TIMEOUT);
        return;
    }
    const bool quality = observer.ready && cfg::direction * observer.speed > cfg::minimum_rad_s &&
        foc_abs(observer.speed) * sample_dt < 0.8f;
    if (quality) lost_time = 0.0f; else lost_time += sample_dt;
    if (lost_time >= cfg::lost_seconds) { retry_motor(LOST_LOCK); return; }
    // Current-sample observer phase, then sample-to-valley compensation.
    const float phase = vesc_observer_wrap(observer_phase + observer.speed * sample_dt * 0.5f);
    if (motor_state == BLEND) {
        const float fraction = state_time < cfg::blend_seconds ? state_time / cfg::blend_seconds : 1.0f;
        // Follow observer speed immediately; fade only the initial angle offset.
        // Interpolating two independently moving angles can jump by 2*pi.
        const float angle = vesc_observer_wrap(phase + (1.0f - fraction) * blend_offset);
        float reference = cfg::direction * cfg::current * fraction;
        if (current_mode) {
            // Start the commanded torque ramp during handover. A fixed 1 A
            // pulse here bypasses IQ= and can accelerate a light rotor before
            // RUN has a chance to slew down to the user's requested current.
            const float target = cfg::direction * iq_setpoint;
            reference = iq_reference + clamp(target - iq_reference,
                cfg::iq_slew_per_second * sample_dt);
        }
        current_control(angle, cfg::current * (1.0f - fraction), reference);
        if (fraction >= 1.0f) enter_state(RUN);
    } else if (motor_state == RUN) {
        if (current_mode) {
            const float target = cfg::direction * iq_setpoint;
            const float reference = iq_reference + clamp(target - iq_reference,
                cfg::iq_slew_per_second * sample_dt);
            current_control(phase, 0.0f, reference);
        } else {
            pi_speed.input = cfg::openloop_rad_s - cfg::direction * observer.speed;
            calculate_PI(&pi_speed);
            // Coast above the target; do not command reverse torque during trial runs.
            if (pi_speed.integral < 0.0f) pi_speed.integral = 0.0f;
            if (pi_speed.output < 0.0f) pi_speed.output = 0.0f;
            current_control(phase, 0.0f, cfg::direction * pi_speed.output);
        }
    }
}

bool start_motor(bool detect)
{
    if (motor_state != IDLE || !limits_valid() || (!detect && !parameters_ready)) return false;
    if (!detect && current_mode && iq_setpoint <= 0.0f) return false;
    if (static_cast<int32_t>(HAL_GetTick() - coast_until) < 0) return false;
    if (Udc_quant < cfg::min_bus || Udc_quant > cfg::max_bus) { motor_fail(BAD_BUS); return false; }
    const uint32_t mask = __get_PRIMASK(); __disable_irq();
    detecting = detect;
    run_requested = !detect;
    if (detect) parameters_ready = false;
    bridge_off(); __HAL_TIM_SET_COUNTER(&htim1, 0u);
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

void service_serial()
{
    if (uart_rx_overflow || uart_rx_error) {
        const uint32_t mask = __get_PRIMASK(); __disable_irq();
        const bool error = uart_rx_error;
        uart_rx_read = uart_rx_write;
        uart_rx_overflow = uart_rx_error = false;
        command_parser.length = 0;
        command_parser.discarding = true;
        __set_PRIMASK(mask);
        service_command('S');
        command_result = COMMAND_RX;
        if (error) {
            HAL_UART_AbortReceive(&huart2);
            if (HAL_UART_Receive_IT(&huart2, &uart_rx_byte, 1) != HAL_OK) uart_rx_error = true;
        }
    }
    while (uart_rx_read != uart_rx_write) {
        const uint8_t byte = uart_rx_data[uart_rx_read];
        uart_rx_read = (uart_rx_read + 1u) & 63u;
        SerialCommand command;
        if (!serial_command_feed(command_parser, byte, command)) continue;
        if (command.type == SERIAL_STOP) service_command('S');
        else if (command.type == SERIAL_START) service_command('R');
        else if (command.type == SERIAL_DETECT) service_command('D');
        else if (command.type == SERIAL_SET_IQ) {
            if (!foc_finite(command.value) || command.value < 0.0f || command.value > cfg::current) {
                command_result = COMMAND_RANGE;
            } else if (motor_state == FAULT || (detecting && motor_state != IDLE && motor_state != ZERO)) {
                command_result = COMMAND_BUSY;
            } else {
                const uint32_t mask = __get_PRIMASK(); __disable_irq();
                iq_setpoint = command.value;
                current_mode = true;
                if (iq_setpoint == 0.0f) service_command('S');
                __set_PRIMASK(mask);
                command_result = COMMAND_OK;
            }
        } else command_result = COMMAND_FORMAT;
    }
}

void send_status()
{
    if (huart2.gState != HAL_UART_STATE_READY) return;
    static char text[320]; // Persistent until the interrupt-driven TX completes.
    const uint32_t mask = __get_PRIMASK(); __disable_irq();
    const MotorState state = motor_state; const MotorFault fault = motor_fault;
    const bool inactive = state == IDLE || state == ZERO || state == RETRY_WAIT;
    const float bus = Udc_quant, id = inactive ? 0.0f : I_d, iq = inactive ? 0.0f : I_q;
    const float speed = inactive ? 0.0f : observer.speed;
    const float r = motor_resistance, l = motor_inductance, flux = motor_flux, ia = I_alpha;
    const float angle = electrical_angle;
    const uint32_t cycles = isr_max_cycles, write_cycles = pwm_write_max_cycles;
    const bool ready = parameters_ready, locked = observer.ready && !inactive;
    const uint32_t frames = sample_frames;
    const float target_iq = iq_setpoint, reference_iq = iq_reference;
    const bool mode = current_mode, armed = run_requested;
    const uint32_t retries = retry_attempts;
    const MotorFault reason = restart_reason;
    const CommandResult result = command_result;
    __set_PRIMASK(mask);
    const int length = snprintf(text, sizeof(text),
        "OBS,s=%d,f=%d,udc=%.3f,ia=%.4f,id=%.4f,iq=%.4f,rpm=%.2f,ready=%d,params=%d,R=%.6f,L=%.9f,flux=%.8f,n=%lu,angle=%.3f,isr=%lu,pwm=%lu,mode=%c,iqset=%.3f,iqref=%.3f,auto=%d,retry=%lu,why=%d,cmd=%d\r\n",
        static_cast<int>(state), static_cast<int>(fault), bus, ia, id, iq,
        speed * 60.0f / (2.0f * kPi * FOC_POLE_PAIRS), locked ? 1 : 0, ready ? 1 : 0,
        r, l, flux, static_cast<unsigned long>(frames), angle,
        static_cast<unsigned long>(cycles), static_cast<unsigned long>(write_cycles),
        mode ? 'I' : 'S', target_iq, reference_iq, armed ? 1 : 0,
        static_cast<unsigned long>(retries), static_cast<int>(reason), static_cast<int>(result));
    if (length > 0 && length < static_cast<int>(sizeof(text)))
        HAL_UART_Transmit_IT(&huart2, reinterpret_cast<uint8_t *>(text), static_cast<uint16_t>(length));
}
} // namespace

static void adc_sample(ADC_HandleTypeDef *hadc)
{
    if (hadc != &hadc1) return;
    ++sample_frames;
    const volatile uint16_t *samples = adc_buffer;
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
    last_I_alpha = I_alpha; last_I_beta = I_beta;
    clark(&I_a, &I_b, &I_c, &I_alpha, &I_beta);
    if (motor_state == IDLE || motor_state == RETRY_WAIT) return;
    if (a == 0 || a == 4095 || b == 0 || b == 4095 || c == 0 || c == 4095 ||
        foc_abs(I_a) > cfg::overcurrent || foc_abs(I_b) > cfg::overcurrent ||
        foc_abs(I_c) > cfg::overcurrent) { motor_fail(OVERCURRENT); return; }
    if (bus == 4095 || Udc_quant < cfg::min_bus || Udc_quant > cfg::max_bus) { motor_fail(BAD_BUS); return; }
    // Peak sampling / valley CCR preload: half of each of the two last commands.
    applied_alpha = Udc_quant * 0.5f * (prior_alpha + older_alpha);
    applied_beta = Udc_quant * 0.5f * (prior_beta + older_beta);
    const bool update_observer = !detecting &&
        (motor_state == OPEN_LOOP || motor_state == BLEND || motor_state == RUN);
    if (update_observer && motor_state != FAULT) {
#if defined(STM32G431xx)
        const uint32_t start_cycles = DWT->CYCCNT;
#endif
        // Matched filtering keeps v = R*i + L*di/dt + emf consistent while
        // rejecting sample ripple. Compensate its phase lag on the raw angle.
        const float filter = sample_dt / (kObserverFilterSeconds + sample_dt);
        filtered_U_alpha += filter * (applied_alpha - filtered_U_alpha);
        filtered_U_beta += filter * (applied_beta - filtered_U_beta);
        filtered_I_alpha += filter * (I_alpha - filtered_I_alpha);
        filtered_I_beta += filter * (I_beta - filtered_I_beta);
        // Matched filtering also attenuates the rotating PM flux. The Ortega
        // radius must match that filtered flux, otherwise its one-sided
        // correction leaves a DC flux offset as electrical speed increases.
        // Use the discrete one-pole response, including its phase lag.
        float filter_sn, filter_cs;
        foc_fast_sincos(observer.speed * sample_dt, filter_sn, filter_cs);
        const float retained = 1.0f - filter;
        const float filter_real = 1.0f - retained * filter_cs;
        const float filter_imag = retained * filter_sn;
        observer_config.flux_linkage = motor_flux * filter /
            foc_sqrt(filter_real * filter_real + filter_imag * filter_imag);
        observer_config.gain = 500.0f /
            (observer_config.flux_linkage * observer_config.flux_linkage);
        if (!vesc_observer_update(observer_config, observer, filtered_U_alpha, filtered_U_beta,
                                  filtered_I_alpha, filtered_I_beta, sample_dt, true)) motor_fail(NUMERIC_ERROR);
        observer_phase = vesc_observer_wrap(observer.angle +
            foc_fast_atan2(filter_imag, filter_real));
#if defined(STM32G431xx)
        const uint32_t cycles = DWT->CYCCNT - start_cycles;
        if (cycles > observer_max_cycles) observer_max_cycles = cycles;
#endif
    }
    if (motor_state != FAULT) control_sample();
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
    if (huart != &huart2) return;
    const uint8_t next = (uart_rx_write + 1u) & 63u;
    if (next == uart_rx_read) uart_rx_overflow = true;
    else { uart_rx_data[uart_rx_write] = uart_rx_byte; uart_rx_write = next; }
    if (HAL_UART_Receive_IT(&huart2, &uart_rx_byte, 1) != HAL_OK) uart_rx_error = true;
}

extern "C" void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart == &huart2) uart_rx_error = true;
}

void setup(void)
{
#if defined(STM32G431xx)
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
#endif
    // No TIM4 reads or encoder startup. Ignore an unplugged Z input.
    CLEAR_BIT(EXTI->IMR1, ENCODE_ZERO_Pin); __HAL_GPIO_EXTI_CLEAR_IT(ENCODE_ZERO_Pin);
    bridge_off();
    if (HAL_UART_Receive_IT(&huart2, &uart_rx_byte, 1) != HAL_OK) uart_rx_error = true;
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
        if (now - last_status >= 100u) { last_status = now; send_status(); }
        __WFI();
    }
}
