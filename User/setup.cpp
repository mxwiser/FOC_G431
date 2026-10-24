#include "setup.hpp"
#include "encoder/encoder.hpp"
#include "main.h"
#include "adc.h"
#include "opamp.h"
#include "tim.h"
#include "foc/svpwm.h"
#include <math.h>
#include <stdio.h>

namespace {
const float kTwoPi = 6.28318530717958647692f;
const float kPhaseVoltagePeak = 3.0f; // Phase-to-neutral fundamental peak, volts.
const float kElectricalFrequency = 50.0f;
const float kMaxModulation = 0.8f;
const float kMinimumBusVoltage = 8.0f;

float electrical_angle = 0.0f;
float angle_step = 0.0f;
volatile bool pwm_ready = false;
volatile bool output_active = false;
volatile uint32_t sample_frames = 0;
}

__align(4) static uint16_t adc_buffer[4];
static volatile uint16_t adc_a = 0;
static volatile uint16_t adc_b = 0;
static volatile uint16_t adc_c = 0;
volatile float Udc_quant = 0.0f;


void samples_cov(){
    const volatile uint16_t *samples = adc_buffer;
    adc_a = samples[0];
    adc_b = samples[1];
    Udc_quant =static_cast<float>(samples[2])
                * (3.3f / 4095.0f)
                * (187.0f / 18.0f); // PB10 low / Q48 off divider.
    adc_c = samples[3];
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


    while (1)
    {
        const float angleDeg = encoder_read_mechanical_angle()
            * (180.0f / 3.14159265358979323846f);
        printf("Udc:%.2f V mechanical:%.2f deg SVPWM:3Vpk/50Hz run:%u frames:%lu\r\n",
               static_cast<double>(Udc_quant), static_cast<double>(angleDeg),
               static_cast<unsigned>(output_active),
               static_cast<unsigned long>(sample_frames));
        HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_6);
        HAL_Delay(100);
    }
    
}

