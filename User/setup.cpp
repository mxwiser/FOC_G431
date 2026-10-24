#include "setup.hpp"
#include "encoder/encoder.hpp"
#include "main.h"
#include "adc.h"
#include "opamp.h"
#include "tim.h"
#include "usart.h"
#include "foc/svpwm.h"
#include <math.h>
#include <stdio.h>

namespace {
const float kTwoPi = 6.28318530717958647692f;
const float kPhaseVoltagePeak = 3.0f; // Phase-to-neutral fundamental peak, volts.
const float kElectricalFrequency = 20.0f;
const float kMaxModulation = 0.8f;
const float kMinimumBusVoltage = 8.0f;
const uint32_t kZeroSamples = 1024u;
const uint32_t kTelemetryHz = 200u;

struct TelemetryFrame {
    uint32_t frame;
    float ia;
    float ib;
    float ic;
    float udc;
    uint8_t running;
};

volatile TelemetryFrame telemetry_latest = {};
volatile uint32_t telemetry_sequence = 0;
uint32_t telemetry_decimation = 125u;
float control_sample_hz = 25000.0f;

uint32_t zero_sum_a = 0;
uint32_t zero_sum_b = 0;
uint32_t zero_sum_c = 0;
uint32_t zero_count = 0;
float zero_a = 0.0f;
float zero_b = 0.0f;
float zero_c = 0.0f;
volatile bool current_zero_ready = false;

float electrical_angle = 0.0f;
float angle_step = 0.0f;
volatile bool pwm_ready = false;
volatile bool output_active = false;
volatile uint32_t sample_frames = 0;
}

__align(4) static uint16_t adc_buffer[4];
static volatile float I_a = 0;
static volatile float I_b = 0;
static volatile float I_c = 0;
volatile float Udc_quant = 0.0f;


void samples_cov(){
    const volatile uint16_t *samples = adc_buffer;
    const float amps_per_count = 0.02946541f;
    const uint16_t raw_a = samples[0];
    const uint16_t raw_b = samples[1];
    const uint16_t raw_udc = samples[2];
    const uint16_t raw_c = samples[3];
    Udc_quant = static_cast<float>(raw_udc)
        * (3.3f / 4095.0f) * (187.0f / 18.0f);

    // TIM1 CH4 samples with all six bridge outputs disabled during this stage.
    if (!current_zero_ready) {
        zero_sum_a += raw_a;
        zero_sum_b += raw_b;
        zero_sum_c += raw_c;
        if (++zero_count == kZeroSamples) {
            zero_a = static_cast<float>(zero_sum_a) / kZeroSamples;
            zero_b = static_cast<float>(zero_sum_b) / kZeroSamples;
            zero_c = static_cast<float>(zero_sum_c) / kZeroSamples;
            current_zero_ready = true;
        }
    }
    if (current_zero_ready) {
        I_a = (zero_a - raw_a) * amps_per_count;
        I_b = (zero_b - raw_b) * amps_per_count;
        I_c = (zero_c - raw_c) * amps_per_count;
    }
}


void mc_foc(float mod_alpha, float mod_beta, float max_mod)
{
    uint32_t tA, tB, tC, sector;
    foc_svm(mod_alpha, mod_beta, max_mod,
            __HAL_TIM_GET_AUTORELOAD(&htim1),
            &tA, &tB, &tC, &sector);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, tA);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, tB);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, tC);
}


//中断内执行foc算法。实时性。
extern "C" void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc != &hadc1) {
        return;
    }
    samples_cov();
    ++sample_frames;

    // Publish a decimated snapshot; formatting and UART transmission stay outside ISR.
    if (current_zero_ready && sample_frames % telemetry_decimation == 0u) {
        telemetry_latest.frame = sample_frames;
        telemetry_latest.ia = I_a;
        telemetry_latest.ib = I_b;
        telemetry_latest.ic = I_c;
        telemetry_latest.udc = Udc_quant;
        telemetry_latest.running = output_active ? 1u : 0u;
        ++telemetry_sequence;
    }

    // Keep CCR1/2/3 at zero while the bootstrap capacitors charge.
    if (!pwm_ready) {
        return;
    }

    const float bus_voltage = Udc_quant;
    if (!(bus_voltage >= kMinimumBusVoltage) || adc_buffer[2] == 4095u) {
        electrical_angle = 0.0f;
        output_active = false;
        mc_foc(0.0f, 0.0f, kMaxModulation);
        return;
    }

    // This SVM uses V_alpha_beta = modulation * (2/3) * Udc.
    const float modulation = 1.5f * kPhaseVoltagePeak / bus_voltage;
    mc_foc(modulation * cosf(electrical_angle),
           modulation * sinf(electrical_angle), kMaxModulation);
    output_active = true;

    electrical_angle += angle_step;
    if (electrical_angle >= kTwoPi) {
        electrical_angle -= kTwoPi;
    }
}





void setup(void){
    encoder_init();

    // OC4REF rises once per full center-aligned PWM cycle.
    RCC_ClkInitTypeDef clocks;
    uint32_t flash_latency;
    HAL_RCC_GetClockConfig(&clocks, &flash_latency);
    uint32_t timer_clock = HAL_RCC_GetPCLK2Freq();
    if (clocks.APB2CLKDivider != RCC_HCLK_DIV1) {
        timer_clock *= 2u;
    }
    const uint32_t period = __HAL_TIM_GET_AUTORELOAD(&htim1);
    if (period == 0u || htim1.Init.CounterMode != TIM_COUNTERMODE_CENTERALIGNED2) {
        Error_Handler();
    }
    const float sample_hz = static_cast<float>(timer_clock)
        / (2.0f * (htim1.Init.Prescaler + 1u) * period);
    control_sample_hz = sample_hz;
    telemetry_decimation = static_cast<uint32_t>(sample_hz / kTelemetryHz + 0.5f);
    if (telemetry_decimation == 0u) {
        telemetry_decimation = 1u;
    }
    angle_step = kTwoPi * kElectricalFrequency / sample_hz;
    // ① 启动三路运放。
    // MX_OPAMPx_Init只是配置，并没有让运放开始工作。
    if (HAL_OPAMP_Start(&hopamp1) != HAL_OK ||
        HAL_OPAMP_Start(&hopamp2) != HAL_OK ||
        HAL_OPAMP_Start(&hopamp3) != HAL_OK) {
        Error_Handler();
    }
    HAL_Delay(1);

    // ② 校准两个ADC，必须在启动转换之前执行。
    if (HAL_ADCEx_Calibration_Start(
            &hadc1, ADC_SINGLE_ENDED) != HAL_OK ||
        HAL_ADCEx_Calibration_Start(
            &hadc2, ADC_SINGLE_ENDED) != HAL_OK) {
        Error_Handler();
    }
    // Load zero high-side duty before enabling any bridge output.
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0u);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0u);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0u);
    __HAL_TIM_SET_COUNTER(&htim1, 0u);
    if (HAL_TIM_GenerateEvent(&htim1, TIM_EVENTSOURCE_UPDATE) != HAL_OK) {
        Error_Handler();
    }

    // Arm DMA and disable HT before allowing the timer to run.
    if (HAL_ADCEx_MultiModeStart_DMA(
        &hadc1,
        reinterpret_cast<uint32_t *>(adc_buffer),
        2
    ) != HAL_OK) {
        Error_Handler();
    }
    __HAL_DMA_DISABLE_IT(hadc1.DMA_Handle, DMA_IT_HT);

    // 启动CH4，先进行采样；旋转电压等自举充电后再输出。
    if (HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4) != HAL_OK) {
        Error_Handler();
    }

    // Measure zero offsets before enabling any of the six bridge outputs.
    const uint32_t zero_start = HAL_GetTick();
    while (!current_zero_ready) {
        if (HAL_GetTick() - zero_start > 200u) {
            Error_Handler();
        }
        HAL_Delay(1);
    }

    // PWM1 CCR=0 keeps high sides off and complementary low sides on.
    if (HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1) != HAL_OK ||
        HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1) != HAL_OK ||
        HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2) != HAL_OK ||
        HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2) != HAL_OK ||
        HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3) != HAL_OK ||
        HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3) != HAL_OK) {
        __HAL_TIM_MOE_DISABLE_UNCONDITIONALLY(&htim1);
        Error_Handler();
    }
    HAL_Delay(5);
    pwm_ready = true;


    static char uart_line[96];
    uint32_t last_sequence = 0;
    uint32_t last_metadata_ms = HAL_GetTick() - 1000u;
    uint32_t last_led_ms = HAL_GetTick();
    while (1)
    {
        const uint32_t now = HAL_GetTick();
        if (huart2.gState == HAL_UART_STATE_READY) {
            int length = 0;
            const bool metadata_due = now - last_metadata_ms >= 1000u;
            uint32_t sequence = last_sequence;
            if (metadata_due) {
                length = snprintf(uart_line, sizeof(uart_line),
                    "#FOC,1,%.0f,%.0f,%.1f,%.1f\r\n",
                    static_cast<double>(control_sample_hz),
                    static_cast<double>(control_sample_hz / telemetry_decimation),
                    static_cast<double>(kElectricalFrequency),
                    static_cast<double>(kPhaseVoltagePeak));
            } else if (telemetry_sequence != last_sequence) {
                TelemetryFrame snapshot;
                const uint32_t irq_state = __get_PRIMASK();
                __disable_irq();
                snapshot.frame = telemetry_latest.frame;
                snapshot.ia = telemetry_latest.ia;
                snapshot.ib = telemetry_latest.ib;
                snapshot.ic = telemetry_latest.ic;
                snapshot.udc = telemetry_latest.udc;
                snapshot.running = telemetry_latest.running;
                sequence = telemetry_sequence;
                __set_PRIMASK(irq_state);
                length = snprintf(uart_line, sizeof(uart_line),
                    "I,%lu,%.3f,%.3f,%.3f,%.2f,%u\r\n",
                    static_cast<unsigned long>(snapshot.frame),
                    static_cast<double>(snapshot.ia),
                    static_cast<double>(snapshot.ib),
                    static_cast<double>(snapshot.ic),
                    static_cast<double>(snapshot.udc),
                    static_cast<unsigned>(snapshot.running));
            }
            if (length > 0 && static_cast<unsigned>(length) < sizeof(uart_line)
                && HAL_UART_Transmit_IT(&huart2, reinterpret_cast<uint8_t *>(uart_line),
                    static_cast<uint16_t>(length)) == HAL_OK) {
                if (metadata_due) {
                    last_metadata_ms = now;
                } else {
                    last_sequence = sequence;
                }
            }
        }
        if (now - last_led_ms >= 100u) {
            HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_6);
            last_led_ms = now;
        }
        HAL_Delay(1);
    }
    
}

