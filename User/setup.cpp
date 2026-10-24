#include "setup.hpp"
#include "encoder/encoder.hpp"
#include "main.h"
#include "adc.h"
#include "opamp.h"
#include "tim.h"
#include "usart.h"
#include "foc/svpwm.h"
#include "foc/foc_math.h"
#include <math.h>
#include <stdio.h>

namespace {
const float kMaxModulation = 1.0f;
float electrical_angle = 0.0f;
float electrical_angle_bias = 0.0f;
}

__align(4) static uint16_t adc_buffer[4];
static  float I_a = 0;
static  float I_b = 0;
static  float I_c = 0;
volatile float Udc_quant = 0.0f;
volatile int mesure = 0;

void samples_cov(){
    const volatile uint16_t *samples = adc_buffer;
    const float amps_per_count = 0.02946541f;
    const uint16_t raw_a = samples[0];
    const uint16_t raw_b = samples[1];
    const uint16_t raw_udc = samples[2];
    const uint16_t raw_c = samples[3];
    Udc_quant = static_cast<float>(raw_udc)
        * (3.3f / 4095.0f) * (187.0f / 18.0f);
    I_a = (raw_a) * amps_per_count;
    I_b = (raw_b) * amps_per_count;
    I_c = (raw_c) * amps_per_count;
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

float I_alpha,I_beta;
float I_q,I_d;
//中断内执行foc算法。实时性。
extern "C" void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc != &hadc1) {
        return;
    }
    samples_cov();

    if (mesure==0)
    {
       return;
    }
    

    

    electrical_angle =
    encoder_read_electrical_angle() - electrical_angle_bias;
    if (electrical_angle < 0.0f) {
        electrical_angle += 2.0f * static_cast<float>(PI);
    }


    clark(&I_a,&I_b,&I_c,&I_alpha,&I_beta);
    park(&I_alpha,&I_beta,&electrical_angle,&I_d,&I_q);
    float temp = -electrical_angle;
    float U_alpha,U_beta;
    I_q = 1;
    I_d = 0;
    park(&I_d,&I_q,&temp,&U_alpha,&U_beta);

    mc_foc(U_alpha,
        U_beta,
        kMaxModulation);

}





void setup(void){
    encoder_init();


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

  
    if (HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1) != HAL_OK ||
        HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1) != HAL_OK ||
        HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2) != HAL_OK ||
        HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2) != HAL_OK ||
        HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3) != HAL_OK ||
        HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3) != HAL_OK) {
        __HAL_TIM_MOE_DISABLE_UNCONDITIONALLY(&htim1);
        Error_Handler();
    }
    
    //校准电角度，给D轴电流。
    if (mesure==0)
    {
        mc_foc(0.5,0,
        kMaxModulation);
        HAL_Delay(200);
        electrical_angle_bias = encoder_read_electrical_angle();
        mc_foc(0,
        0,
        kMaxModulation);
        mesure =1;
    }
    


    while (1)
    {
        HAL_Delay(1);
    }
    
}

