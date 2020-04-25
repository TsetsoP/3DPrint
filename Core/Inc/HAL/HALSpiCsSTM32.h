/*
 * HALSpiCsSTM32.h
 *
 *  Created on: 1.09.2019 г.
 *      Author: ceco
 */

#ifndef INC_TMCDRIVER_HALSPICSSTM32_H_
#define INC_TMCDRIVER_HALSPICSSTM32_H_

#include "HAL_core.h"
#include "stm32f7xx_hal.h"

class HAL_SpiCsSTM32 : public HAL_SpiCS
{
public:
	HAL_SpiCsSTM32(GPIO_TypeDef *csPort, uint16_t csPin);
	virtual ~HAL_SpiCsSTM32();
	virtual void switchCSpin(bool isSet);

private:
	GPIO_TypeDef *csPort;
	uint16_t csPin;
};

#endif /* INC_TMCDRIVER_HALSPICSSTM32_H_ */
