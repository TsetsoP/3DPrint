/*
 * SPI.h
 *
 *  Created on: 24.09.2019 г.
 *      Author: ceco
 */

#pragma once

#include <stddef.h>
#include "stm32f7xx_hal.h"
#include "../../../inc/MarlinConfigPre.h"

#ifdef TOUCH_BUTTONS
#define HAL_TOUCH_SPI  SPI1
#define HAL_TOUCH_SPI_INDEX 1
void Spi_touch_init();
#endif //TOUCH_BUTTONS


#ifdef EEPROM_SETTINGS
#define HAL_FLASH_SPI  SPI6
#define FLASH_SPI_INDEX  0
void Spi_flash_init();
#endif //EEPROM_SETTINGS

SPI_HandleTypeDef* HAL_Spi_get(uint8_t index);
