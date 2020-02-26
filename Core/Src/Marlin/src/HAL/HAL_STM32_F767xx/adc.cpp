#include "HAL.h"
#include "../../pins/pins.h"

#define ADC_BUFFER_SIZE 2

static const char *MODULE_NAME = "ADC";
static ADC_HandleTypeDef hadc1;
static uint16_t HAL_adc_result;
static uint32_t adc_result[ADC_BUFFER_SIZE];

DMA_HandleTypeDef hdma_adc1;
static void MX_DMA_Init(void)
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
}

void HAL_adc_init(void)
{
	ADC_ChannelConfTypeDef sConfig =
	{ 0 };
	MX_DMA_Init();
	hadc1.Instance = ADC1;
	hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
	hadc1.Init.Resolution = ADC_RESOLUTION_10B;
	hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
	hadc1.Init.ContinuousConvMode = ENABLE;
	hadc1.Init.DiscontinuousConvMode = DISABLE;
	hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
	hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc1.Init.NbrOfConversion = 2;
	hadc1.Init.DMAContinuousRequests = ENABLE;
	hadc1.Init.EOCSelection = ADC_EOC_SEQ_CONV;

	HAL_StatusTypeDef status = HAL_ADC_Init(&hadc1);
	if (status != HAL_OK)
	{
		Error_Handler(MODULE_NAME, &status);
	}
	/** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
	 */
	sConfig.Channel = ADC_CHANNEL_6;
	sConfig.Rank = ADC_REGULAR_RANK_1;
	sConfig.SamplingTime = ADC_SAMPLETIME_28CYCLES;
	status = HAL_ADC_ConfigChannel(&hadc1, &sConfig);
	if (status != HAL_OK)
	{
		Error_Handler(MODULE_NAME, &status);
	}

	sConfig.Channel = ADC_CHANNEL_5;
	sConfig.Rank = ADC_REGULAR_RANK_2;
	sConfig.SamplingTime = ADC_SAMPLETIME_28CYCLES;
	status = HAL_ADC_ConfigChannel(&hadc1, &sConfig);
	if (status != HAL_OK)
	{
		Error_Handler(MODULE_NAME, &status);
	}

	SET_BIT(hadc1.Instance->CR2, ADC_CR2_DMA);
	HAL_ADC_Start(&hadc1);
	HAL_DMA_Start(&hdma_adc1, (uint32_t)&(hadc1.Instance->DR), (uint32_t)adc_result, 2);
}
void ADC_IRQHandler(void)
{
	MYSERIAL0.write("ADC_IRQHandler");
}
void HAL_adc_start_conversion(const uint8_t adc_pin)
{
	if (TEMP_0_PIN == adc_pin)
	{
		HAL_adc_result = adc_result[TEMP_0_VAL_INDEX];
	}
}

uint16_t HAL_adc_get_result(void)
{
	return HAL_adc_result;
}

uint16_t HAL_adc_get_result(uint8_t valIndex)
{
	if (valIndex < ADC_BUFFER_SIZE) {
		return adc_result[valIndex];
	}
	return 0;
}
