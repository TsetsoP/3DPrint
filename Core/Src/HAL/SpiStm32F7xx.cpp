/*
 * SpiStm32.cpp
 *
 *  Created on: 1.09.2019 г.
 *      Author: ceco
 */

#include "HAL/SpiStm32F7xx.h"
#define SPI_TIMEOUT 1000

SpiStm32F7xx::SpiStm32F7xx(SPI_TypeDef *stmSpi)
:stmSpi(stmSpi)
{
	// TODO Auto-generated constructor stub

}

SpiStm32F7xx::~SpiStm32F7xx()
{
	// TODO Auto-generated destructor stub
}

void SpiStm32F7xx::init()
{

	hspi.Instance = stmSpi;
	hspi.Init.Mode = SPI_MODE_MASTER;
	hspi.Init.Direction = SPI_DIRECTION_2LINES;
	hspi.Init.DataSize = SPI_DATASIZE_8BIT;
	hspi.Init.CLKPolarity = SPI_POLARITY_LOW;
	hspi.Init.CLKPhase = SPI_PHASE_1EDGE;
	hspi.Init.NSS = SPI_NSS_SOFT;
	hspi.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
	hspi.Init.FirstBit = SPI_FIRSTBIT_MSB;
	hspi.Init.TIMode = SPI_TIMODE_DISABLE;
	hspi.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
	hspi.Init.CRCPolynomial = 7;
	hspi.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
	hspi.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
	if (HAL_SPI_Init(&hspi) != HAL_OK)
	{

	}
}

uint8_t SpiStm32F7xx::transfer(uint8_t txData)
{
	HAL_StatusTypeDef status = HAL_OK;
	uint8_t rxData = 0;
	status = HAL_SPI_TransmitReceive(&hspi, (uint8_t*) &txData, &rxData, 1,
			SPI_TIMEOUT);

	/* Check the communication status */
	if (status != HAL_OK)
	{

	}

 	return rxData;
}

uint16_t SpiStm32F7xx::transfer16(uint16_t txData)
{
	HAL_StatusTypeDef status = HAL_OK;
	uint8_t rxBuffer[2];
	uint8_t txBuffer[2];

	txBuffer[1] = txData & 0xFF;
    txBuffer[0] = (txData >> 8) & 0xFF;

	status = HAL_SPI_TransmitReceive(&hspi, txBuffer, rxBuffer, 2,
	SPI_TIMEOUT);

	/* Check the communication status */
	if (status != HAL_OK)
	{

	}

	return rxBuffer[0] << 8 | rxBuffer[1];
}
