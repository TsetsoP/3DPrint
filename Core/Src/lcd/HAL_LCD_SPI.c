#include "HAL_LCD_SPI.h"
#include "stm32f7xx_hal.h"

static SPI_HandleTypeDef lcd_Spi;
uint32_t SpixTimeout = SPIx_TIMEOUT_MAX;
static void       SPIx_Error(void);


void SPIx_Init(void)
{

	lcd_Spi.Instance = LCD_SPI;
	lcd_Spi.Init.Mode = SPI_MODE_MASTER;
	lcd_Spi.Init.Direction = SPI_DIRECTION_2LINES;
	lcd_Spi.Init.DataSize = SPI_DATASIZE_8BIT;
	lcd_Spi.Init.CLKPolarity = SPI_POLARITY_LOW;
	lcd_Spi.Init.CLKPhase = SPI_PHASE_1EDGE;
	lcd_Spi.Init.NSS = SPI_NSS_SOFT;
	lcd_Spi.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
	lcd_Spi.Init.FirstBit = SPI_FIRSTBIT_MSB;
	lcd_Spi.Init.TIMode = SPI_TIMODE_DISABLE;
	lcd_Spi.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
	lcd_Spi.Init.CRCPolynomial = 7;
	lcd_Spi.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
	lcd_Spi.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
	if (HAL_SPI_Init(&lcd_Spi) != HAL_OK)
	{
		SPIx_Error();
	}
}

/**
  * @brief  SPI Read a byte from device.
  * @param  Value: read value
  * @retval None
  */
void SPIx_Read(uint8_t *pData)
{
	HAL_StatusTypeDef status = HAL_OK;

	status = HAL_SPI_Receive(&lcd_Spi, pData, 1, SpixTimeout);

	/* Check the communication status */
	if (status != HAL_OK)
	{
		/* Execute user timeout callback */
		SPIx_Error();
	}
}

void SPIx_ReadMultipleData(uint8_t *pData, uint16_t size)
{
	HAL_StatusTypeDef status = HAL_OK;

	status = HAL_SPI_Receive(&lcd_Spi, pData, size, SpixTimeout);

	/* Check the communication status */
	if (status != HAL_OK)
	{
		/* Execute user timeout callback */
		SPIx_Error();
	}
}


void SPIx_WriteMultipleData(uint8_t *pData, uint32_t Size)
{
	__IO uint32_t data = 0;
	if (Size == 1)
	{
		/* Only 1 byte to be sent to LCD - general interface can be used */
		/* Send Data */
		SPIx_Write(*pData);
	} else
	{
		HAL_SPI_Transmit(&lcd_Spi, pData, Size, SpixTimeout);
	}

	/* Empty the Rx fifo */
	data = *(&lcd_Spi.Instance->DR);
	UNUSED(data); /* Remove GNU warning */

}

/**
  * @brief  SPI Write a byte to device
  * @param  Value: value to be written
  * @retval None
  */
void SPIx_WriteReadData(const uint8_t *DataIn, uint8_t *DataOut, uint16_t DataLegnth)
{
  HAL_StatusTypeDef status = HAL_OK;

  status = HAL_SPI_TransmitReceive(&lcd_Spi, (uint8_t*) DataIn, DataOut, DataLegnth, SpixTimeout);

  /* Check the communication status */
  if(status != HAL_OK)
  {
    /* Execute user timeout callback */
    SPIx_Error();
  }
}

//=========================================SPI=========================================
/**
  * @brief  SPI Write a byte to device.
  * @param  Value: value to be written
  * @retval None
  */
void SPIx_Write(uint8_t Value)
{
  HAL_StatusTypeDef status = HAL_OK;
  uint8_t data;

  status = HAL_SPI_TransmitReceive(&lcd_Spi, (uint8_t*) &Value, &data, 1, SpixTimeout);

  /* Check the communication status */
  if(status != HAL_OK)
  {
    /* Execute user timeout callback */
    SPIx_Error();
  }
}

/**
  * @brief  SPI error treatment function
  * @param  None
  * @retval None
  */
void SPIx_Error (void)
{
  /* De-initialize the SPI communication BUS */
  HAL_SPI_DeInit(&lcd_Spi);

  /* Re-Initiaize the SPI communication BUS */
  SPIx_Init();
}
