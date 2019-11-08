#ifndef HAL_LCD_SPI_H
#define HAL_LCD_SPI_H

#ifdef __cplusplus
 extern "C" {
#endif

#include <stdint.h>

#include "LCD.h"

#define SPIx_TIMEOUT_MAX 1000

#define LCD_CS_LOW()      HAL_GPIO_WritePin(LCD_CS_GPIO_PORT, LCD_CS_PIN, GPIO_PIN_RESET)
#define LCD_CS_HIGH()     HAL_GPIO_WritePin(LCD_CS_GPIO_PORT, LCD_CS_PIN, GPIO_PIN_SET)
#define LCD_DC_LOW()      HAL_GPIO_WritePin(LCD_DC_GPIO_PORT, LCD_DC_PIN, GPIO_PIN_RESET)
#define LCD_DC_HIGH()     HAL_GPIO_WritePin(LCD_DC_GPIO_PORT, LCD_DC_PIN, GPIO_PIN_SET)
#define LCD_RESET_LOW()      HAL_GPIO_WritePin(LCD_RESET_GPIO_PORT, LCD_RST_PIN, GPIO_PIN_RESET)
#define LCD_RESET_HIGH()     HAL_GPIO_WritePin(LCD_RESET_GPIO_PORT, LCD_RST_PIN, GPIO_PIN_SET)

void SPIx_Init(void);
void SPIx_WriteMultipleData(uint8_t *pData, uint32_t Size);
void SPIx_WriteReadData(const uint8_t *DataIn, uint8_t *DataOut, uint16_t DataLegnth);
void SPIx_Write(uint8_t Value);
void SPIx_ReadMultipleData(uint8_t *pData, uint16_t size);
void SPIx_Read(uint8_t *pData);

#ifdef __cplusplus
}
#endif

#endif /* HAL_LCD_SPI_H */
