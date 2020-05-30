#include "SPI.h"
#include "HAL.h"

#define FLASH_SPI_MOD "FLASH SPI"
#define TOUCH_SPI_MOD "TOUCH SPI"
#define NUM_SPI 1
static SPI_HandleTypeDef spiConfig[NUM_SPI];

#if ENABLED(EEPROM_SETTINGS)
void Spi_flash_init()
{
	SPI_HandleTypeDef &spi = spiConfig[FLASH_SPI_INDEX];
	spi.Instance = HAL_FLASH_SPI;
	spi.Init.Mode = SPI_MODE_MASTER;
	spi.Init.Direction = SPI_DIRECTION_2LINES;
	spi.Init.DataSize = SPI_DATASIZE_8BIT;
	spi.Init.CLKPolarity = SPI_POLARITY_LOW;
	spi.Init.CLKPhase = SPI_PHASE_1EDGE;
	spi.Init.NSS = SPI_NSS_SOFT;
	spi.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
	spi.Init.FirstBit = SPI_FIRSTBIT_MSB;
	spi.Init.TIMode = SPI_TIMODE_DISABLE;
	spi.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
	spi.Init.CRCPolynomial = 7;
	spi.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
	spi.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;

	log_status(HAL_SPI_Init(&spi), FLASH_SPI_MOD);

}
#endif //EEPROM_SETTINGS

#if ENABLED(TOUCH_BUTTONS)
void Spi_touch_init()
{
	SPI_HandleTypeDef spi = spiConfig[HAL_TOUCH_SPI_INDEX];
	spi.Instance = HAL_TOUCH_SPI;
	spi.Init.Mode = SPI_MODE_MASTER;
	spi.Init.Direction = SPI_DIRECTION_2LINES;
	spi.Init.DataSize = SPI_DATASIZE_8BIT;
	spi.Init.CLKPolarity = SPI_POLARITY_LOW;
	spi.Init.CLKPhase = SPI_PHASE_1EDGE;
	spi.Init.NSS = SPI_NSS_SOFT;
	spi.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
	spi.Init.FirstBit = SPI_FIRSTBIT_MSB;
	spi.Init.TIMode = SPI_TIMODE_DISABLE;
	spi.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
	spi.Init.CRCPolynomial = 7;
	spi.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
	spi.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;

	log_status(HAL_SPI_Init(&lcd_Spi), TOUCH_SPI_MOD);

}
#endif //TOUCH_BUTTONS

SPI_HandleTypeDef* HAL_Spi_get(uint8_t index)
{
	return &spiConfig[index];
}
