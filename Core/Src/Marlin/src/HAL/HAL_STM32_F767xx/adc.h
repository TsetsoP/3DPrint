#pragma once

#define TEMP_0_VAL_INDEX 0
#define LASER_VAL_INDEX 1

void HAL_adc_init(void);
void HAL_adc_start_conversion(const uint8_t adc_pin);
uint16_t HAL_adc_get_result(void);
uint16_t HAL_adc_get_result(uint8_t valIndex);
