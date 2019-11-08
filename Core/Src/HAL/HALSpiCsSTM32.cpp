/*
 * HALSpiCsSTM32.cpp
 *
 *  Created on: 1.09.2019 г.
 *      Author: ceco
 */

#include "HAL/HALSpiCsSTM32.h"
#define CS_LOW()      HAL_GPIO_WritePin(LCD_CS_GPIO_PORT, LCD_CS_PIN, GPIO_PIN_RESET)
#define CS_HIGH()     HAL_GPIO_WritePin(LCD_CS_GPIO_PORT, LCD_CS_PIN, GPIO_PIN_SET)

HAL_SpiCsSTM32::HAL_SpiCsSTM32(GPIO_TypeDef *csPort, uint16_t csPin)
: csPort(csPort), csPin(csPin)
{


}

HAL_SpiCsSTM32::~HAL_SpiCsSTM32()
{

}

void HAL_SpiCsSTM32::switchCSpin(bool isSet)
{
	GPIO_PinState pinState = GPIO_PIN_RESET;
	if (isSet)
	{
		pinState = GPIO_PIN_SET;
	}
	HAL_GPIO_WritePin(csPort, csPin, pinState);
}
