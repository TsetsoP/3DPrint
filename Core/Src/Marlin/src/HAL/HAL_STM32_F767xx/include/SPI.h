/*
 * SPI.h
 *
 *  Created on: 24.09.2019 г.
 *      Author: ceco
 */

#pragma once


#include "HAL/SpiStm32F7xx.h"

#if ENABLED(TOUCH_BUTTONS)

#define HAL_TOUCH_SPI  SPI6

#endif //TOUCH_BUTTONS

