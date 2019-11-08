/*
 * HAL.c
 *
 *  Created on: Sep 1, 2019
 *      Author: ceco
 */
#include "HAL/HAL_core.h"
#include "stm32f7xx_hal.h"

uint32_t millis()
{
	return HAL_GetTick();
}

void delay(uint32_t timeMS)
{
	HAL_Delay(timeMS);
}

HAL_SpiCS::HAL_SpiCS()
{
}

HAL_SpiCS::~HAL_SpiCS()
{
}

HAL_SPI::HAL_SPI()
{


}

HAL_SPI::~HAL_SPI()
{

}
void HAL_SPI::init()
{

}
void HAL_SPI::begin()
{

}
void HAL_SPI::endTransaction()
{

}
Stream::Stream()
{
	// TODO Auto-generated constructor stub

}

Stream::~Stream()
{
	// TODO Auto-generated destructor stub
}

uint16_t Stream::write(uint8_t unsignedChar)
{
	return 0;
}

uint16_t Stream::read()
{
	return 0;
}

bool Stream::available()
{
	return false;
}
