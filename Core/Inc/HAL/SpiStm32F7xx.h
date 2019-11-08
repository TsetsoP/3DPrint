/*
 * SpiStm32.h
 *
 *  Created on: 1.09.2019 г.
 *      Author: ceco
 */

#ifndef INC_TMCDRIVER_SPISTM32F7XX_H_
#define INC_TMCDRIVER_SPISTM32F7XX_H_
#include "HAL_core.h"
#include "stm32f7xx_hal.h"

class SpiStm32F7xx: public HAL_SPI
{
public:
	SpiStm32F7xx(SPI_TypeDef *stmSpi);
	virtual ~SpiStm32F7xx();
	virtual void init();
	virtual uint8_t transfer(uint8_t lVal);
	virtual uint16_t transfer16(uint16_t data);

private:
	SPI_TypeDef  *stmSpi;
	SPI_HandleTypeDef hspi;
};

#endif /* INC_TMCDRIVER_SPISTM32F7XX_H_ */
