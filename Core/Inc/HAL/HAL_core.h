/*
 * HAL.h
 *
 *  Created on: Sep 1, 2019
 *      Author: ceco
 */

#pragma once

#include "stdint.h"

uint32_t millis();
void delay(uint32_t timeMS);

class HAL_SpiCS {
public:

	HAL_SpiCS();
	virtual ~HAL_SpiCS();
	virtual void switchCSpin(bool isSet) = 0;
};

class HAL_SPI
{
public:
	HAL_SPI();
	virtual ~HAL_SPI();
	virtual void init();
	virtual void begin();
	virtual uint8_t transfer(uint8_t lVal) = 0;
	virtual uint16_t transfer16(uint16_t data) = 0;
	virtual void endTransaction();
};

class Stream
{
public:
	Stream();
	virtual ~Stream();
	uint16_t write(uint8_t);
	uint16_t read();
	bool available();
};

