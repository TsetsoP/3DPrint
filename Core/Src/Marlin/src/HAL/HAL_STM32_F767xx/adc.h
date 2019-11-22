
#pragma once

extern ADC_HandleTypeDef hadc1;

void HAL_adc_init(void);
void HAL_adc_start_conversion(const uint8_t adc_pin);
uint16_t HAL_adc_get_result(void);
