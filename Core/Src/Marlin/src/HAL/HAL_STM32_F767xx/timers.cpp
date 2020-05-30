/**
 * Marlin 3D Printer Firmware
 *
 * Copyright (c) 2019 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
 * Copyright (c) 2016 Bob Cousins bobcousins42@googlemail.com
 * Copyright (c) 2015-2016 Nico Tonnhofer wurstnase.reprap@gmail.com
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

#if defined(STM32GENERIC) && defined(STM32F767xx)

#include "../HAL.h"
#include "timers.h"

// ------------------------
// Local defines
// ------------------------

#define NUM_HARDWARE_TIMERS 5
#define FAN_TIMER_NAME "TIM2"
#define LASER_TIMER_NAME "TIM3"
// ------------------------
// Private Variables
// ------------------------

tTimerConfig timerConfig[NUM_HARDWARE_TIMERS];
extern "C" void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

static void set_timer_ferq(const uint8_t timer_num, const uint32_t frequency)
{
	timerConfig[timer_num].timerdef.Init.Period = (((HAL_TIMER_RATE)
			/ timerConfig[timer_num].timerdef.Init.Prescaler) / frequency) - 1;

	if (HAL_TIM_Base_Init(&timerConfig[timer_num].timerdef) == HAL_OK)
		HAL_TIM_Base_Start_IT(&timerConfig[timer_num].timerdef);
}

static void start_steppers_timer(const uint32_t frequency)
{
	//STEPPER TIMER TIM5 //use a 32bit timer
	__HAL_RCC_TIM5_CLK_ENABLE();
	timerConfig[0].timerdef.Instance = STEPPER_TIMER;
	timerConfig[0].timerdef.Init.Prescaler = (STEPPER_TIMER_PRESCALE);
	timerConfig[0].timerdef.Init.CounterMode = TIM_COUNTERMODE_UP;
	timerConfig[0].timerdef.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	timerConfig[0].IRQ_Id = TIM5_IRQn;
	timerConfig[0].callback = (uint32_t) TC5_Handler;
	HAL_NVIC_SetPriority(timerConfig[0].IRQ_Id, 1, 0);
	set_timer_ferq(STEP_TIMER_NUM, frequency);
}

static void start_temp_timer(const uint32_t frequency)
{
	__HAL_RCC_TIM7_CLK_ENABLE();
	timerConfig[1].timerdef.Instance = TEMP_TIMER;
	timerConfig[1].timerdef.Init.Prescaler = (TEMP_TIMER_PRESCALE);
	timerConfig[1].timerdef.Init.CounterMode = TIM_COUNTERMODE_UP;
	timerConfig[1].timerdef.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	timerConfig[1].IRQ_Id = TEMP_TIMER_IRQ_N;
	timerConfig[1].callback = (uint32_t) TEMP_TIMER_HANDLER;
	HAL_NVIC_SetPriority(timerConfig[1].IRQ_Id, 2, 0);
	HAL_NVIC_EnableIRQ(TEMP_TIMER_IRQ_N);
	set_timer_ferq(TEMP_TIMER_NUM, frequency);
}

static void start_fan_timer()
{
	TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };
	TIM_OC_InitTypeDef sConfigOC = { 0 };
	TIM_HandleTypeDef *htim = &timerConfig[FAN_TIMER_NUM].timerdef;
	__HAL_RCC_TIM2_CLK_ENABLE();

	htim->Instance = FAN_TIMER;
	htim->Init.Prescaler = FAN_TIMER_PRESCALE;
	htim->Init.CounterMode = TIM_COUNTERMODE_UP;
	htim->Init.Period = FAN_TIMER_PERIOD;
	htim->Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim->Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	log_status(HAL_TIM_OC_Init(htim), FAN_TIMER_NAME);

	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	log_status(HAL_TIM_ConfigClockSource(htim, &sClockSourceConfig),
			FAN_TIMER_NAME);

	log_status(HAL_TIM_PWM_Init(htim), FAN_TIMER_NAME);

	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	log_status(HAL_TIMEx_MasterConfigSynchronization(htim, &sMasterConfig),
			FAN_TIMER_NAME);

	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_LOW;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	log_status(HAL_TIM_PWM_ConfigChannel(htim, &sConfigOC, TIM_CHANNEL_1),
			FAN_TIMER_NAME);
	log_status(HAL_TIM_PWM_ConfigChannel(htim, &sConfigOC, TIM_CHANNEL_3),
			FAN_TIMER_NAME);
	log_status(HAL_TIM_PWM_ConfigChannel(htim, &sConfigOC, TIM_CHANNEL_4),
			FAN_TIMER_NAME);

	HAL_TIM_MspPostInit(htim);
	HAL_TIM_PWM_Start(htim, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(htim, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(htim, TIM_CHANNEL_4);
}

static void start_laser_timer()
{
	TIM_ClockConfigTypeDef sClockSourceConfig = {0};
	TIM_MasterConfigTypeDef sMasterConfig = {0};
	TIM_OC_InitTypeDef sConfigOC = {0};
	TIM_HandleTypeDef *htim = &timerConfig[LASER_TIMER_NUM].timerdef;
	__HAL_RCC_TIM3_CLK_ENABLE();

	htim->Instance = LASER_TIMER;
	htim->Init.Prescaler = LASER_TIMER_PRESCALE;
	htim->Init.CounterMode = TIM_COUNTERMODE_UP;
	htim->Init.Period = LASER_TIMER_PERIOD;
	htim->Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim->Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	log_status(HAL_TIM_OC_Init(htim), LASER_TIMER_NAME);

	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	log_status(HAL_TIM_ConfigClockSource(htim, &sClockSourceConfig),
			LASER_TIMER_NAME);

	log_status(HAL_TIM_PWM_Init(htim), LASER_TIMER_NAME);

	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	log_status(HAL_TIMEx_MasterConfigSynchronization(htim, &sMasterConfig),
			LASER_TIMER_NAME);

	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_LOW;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	log_status(HAL_TIM_PWM_ConfigChannel(htim, &sConfigOC, TIM_CHANNEL_1),
			LASER_TIMER_NAME);

	HAL_TIM_MspPostInit(htim);
	HAL_TIM_PWM_Start(htim, TIM_CHANNEL_1);
}

static void start_servo_timer()
{

	TIM_OC_InitTypeDef sConfigOC = { 0 };
	TIM_HandleTypeDef *htim = &timerConfig[SERVO_TIMER_NUM].timerdef;
	htim->Instance = TIM10;
	htim->Init.Prescaler = SERVO_TIMER_PRESCALE;
	htim->Init.CounterMode = TIM_COUNTERMODE_UP;
	htim->Init.Period = SERVO_TIMER_PERIOD;
	htim->Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim->Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	HAL_TIM_Base_MspInit(htim);
	log_status(HAL_TIM_Base_Init(htim), "TIM10");
	log_status(HAL_TIM_PWM_Init(htim), "TIM10");

	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	log_status(HAL_TIM_PWM_ConfigChannel(htim, &sConfigOC, TIM_CHANNEL_1), "TIM10");
	HAL_TIM_MspPostInit (htim);
	HAL_TIM_PWM_Start(htim, TIM_CHANNEL_1);
}

bool timers_initialized[NUM_HARDWARE_TIMERS] =
{ false };
// ------------------------
// Public functions
// ------------------------
void HAL_timer_start(const uint8_t timer_num, const uint32_t frequency)
{
	if (!timers_initialized[timer_num])
	{
		switch (timer_num)
		{
		case STEP_TIMER_NUM:
			start_steppers_timer(frequency);
#if PIN_EXISTS(STEPPER_ENABLE)
            OUT_WRITE(STEPPER_ENABLE_PIN, HIGH);
#endif
			break;
		case TEMP_TIMER_NUM:
			start_temp_timer(frequency);
			break;

		case FAN_TIMER_NUM:
			start_fan_timer();
			break;

		case LASER_TIMER_NUM:
			start_laser_timer();
			break;
		case SERVO_TIMER_NUM:
			start_servo_timer();
			break;

		}
		timers_initialized[timer_num] = true;
	}
}

//forward the interrupt
extern "C" void TIM5_IRQHandler()
{
	((void (*)(void)) timerConfig[0].callback)();
}
extern "C" void TIM7_IRQHandler(void)
{
	((void (*)(void)) timerConfig[TEMP_TIMER_NUM].callback)();
}

void HAL_timer_set_PWM(const uint8_t timer_num, uint32_t chanel, const uint32_t compare)
{
	__HAL_TIM_SetCompare(&timerConfig[timer_num].timerdef, chanel, compare);
}

void HAL_timer_set_PWM_uS(const uint8_t timer_num, uint32_t chanel, const uint32_t tau)
{
	double timerRate = HAL_TIMER_RATE / timerConfig[timer_num].timerdef.Init.Prescaler ;
	double compare =  tau * (timerRate / 1000000.0) * 2;
	__HAL_TIM_SetCompare(&timerConfig[timer_num].timerdef, chanel, (int)compare);
}

uint32_t HAL_timer_get_PWM(const uint8_t timer_num, uint32_t chanel)
{
	return __HAL_TIM_GetCompare(&timerConfig[timer_num].timerdef, chanel);
}

void HAL_timer_set_compare(const uint8_t timer_num, const uint32_t compare)
{
	__HAL_TIM_SetAutoreload(&timerConfig[timer_num].timerdef, compare);
}

void HAL_timer_enable_interrupt(const uint8_t timer_num)
{
	HAL_NVIC_EnableIRQ(timerConfig[timer_num].IRQ_Id);
}

void HAL_timer_disable_interrupt(const uint8_t timer_num)
{
	HAL_NVIC_DisableIRQ(timerConfig[timer_num].IRQ_Id);

	// We NEED memory barriers to ensure Interrupts are actually disabled!
	// ( https://dzone.com/articles/nvic-disabling-interrupts-on-arm-cortex-m-and-the )
	__DSB();
	__ISB();
}

hal_timer_t HAL_timer_get_compare(const uint8_t timer_num)
{
	return __HAL_TIM_GetAutoreload(&timerConfig[timer_num].timerdef);
}

uint32_t HAL_timer_get_count(const uint8_t timer_num)
{
	return __HAL_TIM_GetCounter(&timerConfig[timer_num].timerdef);
}

void HAL_timer_isr_prologue(const uint8_t timer_num)
{
	if (__HAL_TIM_GET_FLAG(&timerConfig[timer_num].timerdef, TIM_FLAG_UPDATE) == SET)
	{
		__HAL_TIM_CLEAR_FLAG(&timerConfig[timer_num].timerdef, TIM_FLAG_UPDATE);
	}
}

bool HAL_timer_interrupt_enabled(const uint8_t timer_num)
{
	const uint32_t IRQ_Id = uint32_t(timerConfig[timer_num].IRQ_Id);
	return NVIC->ISER[IRQ_Id >> 5] & _BV32(IRQ_Id & 0x1F);
}

#endif // STM32GENERIC && STM32F7
