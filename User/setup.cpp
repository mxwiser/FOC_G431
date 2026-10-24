#include "setup.hpp"
#include "encoder/encoder.hpp"
#include "main.h"
#include "stdio.h"
extern  ADC_HandleTypeDef hadc1;
extern  OPAMP_HandleTypeDef hopamp1;
extern  OPAMP_HandleTypeDef hopamp2;
extern  OPAMP_HandleTypeDef hopamp3;
extern  ADC_HandleTypeDef hadc1;
extern  ADC_HandleTypeDef hadc2;
extern  TIM_HandleTypeDef htim1;
__align(4) static uint16_t adc_buffer[4];
static volatile uint16_t adc_a = 0;
static volatile uint16_t adc_b = 0;
static volatile uint16_t adc_c = 0;

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
    //先配置DMA
    HAL_ADCEx_MultiModeStart_DMA(
        &hadc1,
        reinterpret_cast<uint32_t *>(adc_buffer),
        2
    );
    __HAL_DMA_DISABLE_IT(hadc1.DMA_Handle, DMA_IT_HT);
    
    // 最后启动CH4，开始产生TRGO2采样触发。
    if (HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4) != HAL_OK) {
        Error_Handler();
    }


    while (1)
    {
        const float angleDeg = encoder_read_mechanical_angle()
            * (180.0f / 3.14159265358979323846f);
        printf("mechanical:%.2f deg\r\n", angleDeg);
        HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_6);
        HAL_Delay(100);
    }
    
}

