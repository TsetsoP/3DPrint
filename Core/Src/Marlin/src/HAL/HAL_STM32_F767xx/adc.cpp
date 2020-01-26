#include "HAL.h"
static const char *MODULE_NAME = "ADC";
ADC_HandleTypeDef hadc1;
uint16_t HAL_adc_result;

void HAL_adc_init(void)
{
	ADC_ChannelConfTypeDef sConfig =
	{ 0 };

	hadc1.Instance = ADC1;
	hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
	hadc1.Init.Resolution = ADC_RESOLUTION_10B;
	hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
	hadc1.Init.ContinuousConvMode = DISABLE;
	hadc1.Init.DiscontinuousConvMode = DISABLE;
	hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
	hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc1.Init.NbrOfConversion = 1;
	hadc1.Init.DMAContinuousRequests = DISABLE;
	hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;

	HAL_StatusTypeDef status = HAL_ADC_Init(&hadc1);
	if (status != HAL_OK)
	{
		Error_Handler(MODULE_NAME, &status);
	}
	/** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
	 */
	sConfig.Channel = ADC_CHANNEL_6;
	sConfig.Rank = ADC_REGULAR_RANK_1;
	sConfig.SamplingTime = ADC_SAMPLETIME_15CYCLES;
	status = HAL_ADC_ConfigChannel(&hadc1, &sConfig);
	if (status != HAL_OK)
	{
		Error_Handler(MODULE_NAME, &status);
	}
	HAL_ADC_Start(&hadc1);
}

void HAL_adc_start_conversion(const uint8_t adc_pin)
{
	HAL_ADC_Start(&hadc1);
	HAL_StatusTypeDef status = HAL_ADC_PollForConversion(&hadc1, 30);

	if (status == HAL_OK)
	{
		HAL_adc_result = HAL_ADC_GetValue(&hadc1);
		__HAL_ADC_CLEAR_FLAG(&hadc1, ADC_FLAG_EOC | ADC_FLAG_OVR);
	}
}

uint16_t HAL_adc_get_result(void)
{
	return HAL_adc_result;
}
