/*
 * TSC2046.h
 *
 *  Created on: Nov 8, 2019
 *      Author: ceco
 */

#pragma once
#include "HAL.h"

enum TSCCoordinate : uint8_t {
  TSC2046_X  = 0x10,
  TSC2046_Y  = 0x50,
  TSC2046_Z1 = 0x30,
  TSC2046_Z2 = 0x40
};

#ifndef TSC2046_Z1_THRESHOLD
  #define TSC2046_Z1_THRESHOLD 10
#endif

class TSC2046 {
public:
	TSC2046(HAL_SPI *stmSpi);

	void init();
	uint8_t read_buttons();
	bool isTouched();

private:
	uint16_t readAxis(const TSCCoordinate coordinate);
	HAL_SPI *spi;
};

extern TSC2046 touch;
