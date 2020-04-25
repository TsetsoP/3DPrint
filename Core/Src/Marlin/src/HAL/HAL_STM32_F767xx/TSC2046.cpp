/*
 * TSC2046.cpp
 *
 *  Created on: Nov 8, 2019
 *      Author: ceco
 */
#include "../../inc/MarlinConfigPre.h"

#if ENABLED(TOUCH_BUTTONS)
#include "../../inc/MarlinConfig.h"
#include "TSC2046.h"
#include "../../lcd/ultralcd.h" // For EN_C bit mask
#define TSC2046_DFR_MODE 0x00
#define TSC2046_SER_MODE 0x04
#define TSC2046_CONTROL  0x80

TSC2046::TSC2046(HAL_SPI *spi):
spi(spi)
{

}

void TSC2046::init()
{
	spi->init();
	WRITE(TOUCH_CS_PIN, HIGH);
}

uint8_t TSC2046::read_buttons()
{
	if (!isTouched())
	{
		return 0;
	}

	const uint16_t x =  uint16_t( (uint32_t(readAxis(TSC2046_X) * XPT2046_X_CALIBRATION)) >> 16) + XPT2046_X_OFFSET;
	const uint16_t y = uint16_t( (uint32_t(readAxis(TSC2046_Y) * XPT2046_Y_CALIBRATION)) >> 16) + XPT2046_Y_OFFSET;

	if (!isTouched())
	{
		return 0; // Fingers must still be on the TS for a valid read.
	}

	if (y < 25 || y > 55)
	{
		return 0;
	}

	return WITHIN(x,  25,  75) ? EN_C
	   : WITHIN(x,  100, 153) ? EN_B
	   : WITHIN(x, 170, 229) ? EN_A
	   : WITHIN(x, 247, 305) ? EN_D
	   : 0;
}

bool TSC2046::isTouched()
{
	return READ(TOUCH_INT_PIN) != HIGH;
}

uint16_t TSC2046::readAxis(const TSCCoordinate coordinate)
{

	const uint8_t coord = uint8_t(
			coordinate) | TSC2046_CONTROL | TSC2046_DFR_MODE;

	uint16_t pin = TOUCH_CS_PIN;
	OUT_WRITE(pin, LOW);
	spi->transfer(coord);
	uint16_t axisRawData = spi->transfer(0);

	OUT_WRITE(TOUCH_CS_PIN, HIGH);
	return axisRawData;
}

#endif // TOUCH_BUTTONS

