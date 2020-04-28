#include "../../inc/MarlinConfig.h"
#include "../../pins/pins.h"
//#include "../../sd/cardreader.h"

#define ADC_BUFFER_SIZE 2
#define ADC2_MAX_VALUE 4095

static const char *MODULE_NAME = "ADC";
static ADC_HandleTypeDef hadc1;
static ADC_HandleTypeDef hadc2;
static uint16_t HAL_adc_result;
static uint32_t adc_result[ADC_BUFFER_SIZE];

static void log_status(HAL_StatusTypeDef status);
static void MX_DMA_Init(void);

DMA_HandleTypeDef hdma_adc1;

typedef struct
{
	uint16_t *stateBuffer;
	uint16_t numTaps;
	float_t coef;
	uint16_t *start;
	uint16_t *end;
	uint16_t acc;
} lmsState;

extern "C" {
#define NUM_TAPS 25
#define FILTER_COEF 0.04f
static uint16_t filterBuffer[NUM_TAPS];
static lmsState filterState;

static void lmsFilterInit( lmsState *state, uint16_t *stateBuffer, uint16_t numTaps, float_t coef)
{
	memset(stateBuffer, 0,  numTaps * sizeof(uint16_t));
	state->stateBuffer = stateBuffer;
	state->numTaps = numTaps;
	state->coef = coef;
	state->end = stateBuffer + numTaps;
	state->acc = 0;
}

static uint16_t lmsFilterValue(lmsState *state, uint16_t newValue)
{
	if (state->end == state->stateBuffer + state->numTaps)
	{
		state->end = state->stateBuffer;
	}
	else
	{
		state->end++;
	}

	state->acc -= *state->end;
	*state->end = ((uint16_t)newValue) * state->coef;
	state->acc +=  *state->end;
	return state->acc;
}

void ADC_IRQHandler(void)
{
	uint32_t eoc = __HAL_ADC_GET_FLAG(&hadc2, ADC_FLAG_EOC);
	uint32_t it_eoc = __HAL_ADC_GET_IT_SOURCE(&hadc2, ADC_IT_EOC);
	if (eoc && it_eoc)
	{
		//adc_result[LASER_VAL_INDEX] = (uint32_t) hadc2.Instance->DR;
		adc_result[LASER_VAL_INDEX] = (uint32_t)lmsFilterValue(&filterState, hadc2.Instance->DR);
		//char text_num[6];
		//itoa(adc_result[LASER_VAL_INDEX], text_num, 10);
		//int len = strlen(text_num);
		//text_num[len] = ',';
		//card.write(text_num,  len + 1);
	}

	uint32_t ovr = __HAL_ADC_GET_FLAG(&hadc2, ADC_FLAG_OVR);
	uint32_t it_ovr = __HAL_ADC_GET_IT_SOURCE(&hadc2, ADC_IT_OVR);
	if (ovr && it_ovr)
	{
		__HAL_ADC_CLEAR_FLAG(&hadc2, ADC_FLAG_OVR);
	}
}
}

void HAL_adc_start(adc_index index, bool isStarted)
{
	switch(index)
	{
	case TEMP_ADC:
		if (isStarted)
		{
			HAL_ADC_Start(&hadc1);
			HAL_DMA_Start(&hdma_adc1, (uint32_t)&(hadc1.Instance->DR), (uint32_t)adc_result, 1);
		}
		else
		{
			HAL_ADC_Stop(&hadc1);
			adc_result[TEMP_0_VAL_INDEX] = 0;
		}
		break;

	case LASER_ADC:
		if (isStarted)
		{
			lmsFilterInit(&filterState, filterBuffer, NUM_TAPS, FILTER_COEF);
		    __HAL_ADC_ENABLE(&hadc2);
			__HAL_ADC_CLEAR_FLAG(&hadc2, ADC_FLAG_EOC | ADC_FLAG_OVR);
			__HAL_ADC_ENABLE_IT(&hadc2, (ADC_IT_EOC | ADC_IT_OVR));
            hadc2.Instance->CR2 |= (uint32_t)ADC_CR2_SWSTART;
			//card.mount();
			//card.openFileWrite("rawdata.txt");
		}
		else
		{
			HAL_ADC_Stop_IT(&hadc2);
			//card.release();
			//card.closefile();
		}
		break;
	}
}

void HAL_adc_init(void)
{
	ADC_ChannelConfTypeDef sConfig = { 0 };
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
	hadc1.Init.NbrOfConversion = 1;
	hadc1.Init.DMAContinuousRequests = ENABLE;
	hadc1.Init.EOCSelection = ADC_EOC_SEQ_CONV;
	log_status(HAL_ADC_Init(&hadc1));

	SET_BIT(hadc1.Instance->CR2, ADC_CR2_DMA);
	sConfig.Channel = ADC_CHANNEL_6;
	sConfig.Rank = ADC_REGULAR_RANK_1;
	sConfig.SamplingTime = ADC_SAMPLETIME_28CYCLES;
	log_status(HAL_ADC_ConfigChannel(&hadc1, &sConfig));
	HAL_adc_start(TEMP_ADC, true);

	hadc2.Instance = ADC2;
	hadc2.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV8;
	hadc2.Init.Resolution = ADC_RESOLUTION_12B;
	hadc2.Init.ScanConvMode = ADC_SCAN_DISABLE;
	hadc2.Init.ContinuousConvMode = ENABLE;
	hadc2.Init.DiscontinuousConvMode = DISABLE;
	hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
	hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc2.Init.NbrOfConversion = 1;
	hadc2.Init.DMAContinuousRequests = DISABLE;
	hadc2.Init.EOCSelection = ADC_EOC_SEQ_CONV;
	log_status(HAL_ADC_Init(&hadc2));

	sConfig = { 0 };
	sConfig.Channel = ADC_CHANNEL_5;
	sConfig.Rank = ADC_REGULAR_RANK_1;
	sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
	log_status(HAL_ADC_ConfigChannel(&hadc2, &sConfig));

	adc_result[TEMP_0_VAL_INDEX] = 1020;
	adc_result[LASER_VAL_INDEX] = ADC2_MAX_VALUE;
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

void MX_DMA_Init(void)
{
  __HAL_RCC_DMA2_CLK_ENABLE();
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
}

void log_status(HAL_StatusTypeDef status)
{
	if (status != HAL_OK)
	{
		Error_Handler(MODULE_NAME, &status);
	}
}
