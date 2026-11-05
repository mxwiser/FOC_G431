#pragma once
#include "adc.h"
#include "opamp.h"

namespace board_adc {
inline bool init(uint16_t *samples, uint32_t dual_word_count)
{
    if (HAL_OPAMP_Start(&hopamp1) != HAL_OK || HAL_OPAMP_Start(&hopamp2) != HAL_OK ||
        HAL_OPAMP_Start(&hopamp3) != HAL_OK) return false;
    HAL_Delay(1);
    if (HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED) != HAL_OK ||
        HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED) != HAL_OK) return false;
    if (HAL_ADCEx_MultiModeStart_DMA(&hadc1, reinterpret_cast<uint32_t *>(samples), dual_word_count) != HAL_OK)
        return false;
    __HAL_DMA_DISABLE_IT(hadc1.DMA_Handle, DMA_IT_HT);
    return true;
}
}
