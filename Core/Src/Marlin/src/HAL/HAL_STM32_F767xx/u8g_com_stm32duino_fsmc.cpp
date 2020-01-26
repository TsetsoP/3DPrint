/**
 * Marlin 3D Printer Firmware
 * Copyright (c) 2019 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
 *
 * Based on Sprinter and grbl.
 * Copyright (c) 2011 Camiel Gubbels / Erik van der Zalm
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

/**
 * u8g_com_stm32duino_fsmc.cpp
 *
 * Communication interface for FSMC
 */

#include "../../../inc/MarlinConfig.h"
#include "HAL_LCD_SPI.h"
#include "ili9341.h"

#if PIN_EXISTS(FSMC_CS) // FSMC on 100/144 pins SoCs

#if HAS_GRAPHICAL_LCD

#include <U8glib.h>

#include "../../core/boards.h"

#ifndef LCD_READ_ID
  #define LCD_READ_ID 0x04   // Read display identification information (0xD3 on ILI9341)
#endif

/* Timing configuration */
#define FSMC_ADDRESS_SETUP_TIME   15  // AddressSetupTime
#define FSMC_DATA_SETUP_TIME      15  // DataSetupTime

void LCD_IO_Init(uint8_t cs, uint8_t rs);
void LCD_IO_WriteData(uint16_t RegValue);
void LCD_IO_WriteReg(uint16_t Reg);
uint32_t LCD_IO_ReadData(uint16_t RegValue, uint8_t ReadSize);
#ifdef LCD_USE_DMA_FSMC
  void LCD_IO_WriteMultiple(uint16_t data, uint32_t count);
  void LCD_IO_WriteSequence(uint16_t *data, uint16_t length);
#endif

static uint8_t msgInitCount = 2; // Ignore all messages until 2nd U8G_COM_MSG_INIT

uint8_t u8g_com_stm32duino_fsmc_fn(u8g_t *u8g, uint8_t msg, uint8_t arg_val, void *arg_ptr) {
  if (msgInitCount) {
    if (msg == U8G_COM_MSG_INIT) msgInitCount--;
    if (msgInitCount) return -1;
  }

  static uint8_t isCommand;

  switch (msg) {
    case U8G_COM_MSG_STOP: break;
    case U8G_COM_MSG_INIT:
      u8g_SetPIOutput(u8g, U8G_PI_RESET);

      #ifdef LCD_USE_DMA_FSMC
        dma_init(FSMC_DMA_DEV);
        dma_disable(FSMC_DMA_DEV, FSMC_DMA_CHANNEL);
        dma_set_priority(FSMC_DMA_DEV, FSMC_DMA_CHANNEL, DMA_PRIORITY_MEDIUM);
      #endif

      LCD_IO_Init(u8g->pin_list[U8G_PI_CS], u8g->pin_list[U8G_PI_A0]);
      u8g_Delay(50);

      if (arg_ptr)
        *((uint32_t *) arg_ptr) =  0x9341;//LCD_IO_ReadData(LCD_READ_ID, 3);
      isCommand = 0;
      break;

    case U8G_COM_MSG_ADDRESS: // define cmd (arg_val = 0) or data mode (arg_val = 1)
      isCommand = arg_val == 0 ? 1 : 0;
      break;

    case U8G_COM_MSG_RESET:
      u8g_SetPILevel(u8g, U8G_PI_RESET, arg_val);
      break;

    case U8G_COM_MSG_WRITE_BYTE:
      if (isCommand)
    	ili9341_WriteReg(arg_val);
      else
    	ili9341_WriteData(arg_val);
      break;

    case U8G_COM_MSG_WRITE_SEQ:

		ili9341_WriteMultipleData((uint8_t *)arg_ptr, arg_val);

      break;
    case U8G_COM_MSG_CHIP_SELECT:
      break;
  }
  return 1;
}


#define FSMC_CS_NE1   PD7

#if ENABLED(STM32_XL_DENSITY)
  #define FSMC_CS_NE2 PG9
  #define FSMC_CS_NE3 PG10
  #define FSMC_CS_NE4 PG12

  #define FSMC_RS_A0  PF0
  #define FSMC_RS_A1  PF1
  #define FSMC_RS_A2  PF2
  #define FSMC_RS_A3  PF3
  #define FSMC_RS_A4  PF4
  #define FSMC_RS_A5  PF5
  #define FSMC_RS_A6  PF12
  #define FSMC_RS_A7  PF13
  #define FSMC_RS_A8  PF14
  #define FSMC_RS_A9  PF15
  #define FSMC_RS_A10 PG0
  #define FSMC_RS_A11 PG1
  #define FSMC_RS_A12 PG2
  #define FSMC_RS_A13 PG3
  #define FSMC_RS_A14 PG4
  #define FSMC_RS_A15 PG5
#endif

#define FSMC_RS_A16   PD11
#define FSMC_RS_A17   PD12
#define FSMC_RS_A18   PD13
#define FSMC_RS_A19   PE3
#define FSMC_RS_A20   PE4
#define FSMC_RS_A21   PE5
#define FSMC_RS_A22   PE6
#define FSMC_RS_A23   PE2

#if ENABLED(STM32_XL_DENSITY)
  #define FSMC_RS_A24 PG13
  #define FSMC_RS_A25 PG14
#endif



void LCD_IO_Init(uint8_t cs, uint8_t rs) {
	SPIx_Init();
    LCD_CS_HIGH();
	LCD_RESET_HIGH();
	//ili9341_Init();

}

uint32_t LCD_IO_ReadData(uint16_t RegValue, uint8_t ReadSize) {

  return ili9341_ReadData(RegValue, 4);
}

#if ENABLED(LCD_USE_DMA_FSMC)

void LCD_IO_WriteMultiple(uint16_t color, uint32_t count) {
  while (count > 0) {
    dma_setup_transfer(FSMC_DMA_DEV, FSMC_DMA_CHANNEL, &color, DMA_SIZE_16BITS, &LCD->RAM, DMA_SIZE_16BITS, DMA_MEM_2_MEM);
    dma_set_num_transfers(FSMC_DMA_DEV, FSMC_DMA_CHANNEL, count > 65535 ? 65535 : count);
    dma_clear_isr_bits(FSMC_DMA_DEV, FSMC_DMA_CHANNEL);
    dma_enable(FSMC_DMA_DEV, FSMC_DMA_CHANNEL);

    while ((dma_get_isr_bits(FSMC_DMA_DEV, FSMC_DMA_CHANNEL) & 0x0A) == 0) {};
    dma_disable(FSMC_DMA_DEV, FSMC_DMA_CHANNEL);

    count = count > 65535 ? count - 65535 : 0;
  }
}

void LCD_IO_WriteSequence(uint16_t *data, uint16_t length) {
  dma_setup_transfer(FSMC_DMA_DEV, FSMC_DMA_CHANNEL, data, DMA_SIZE_16BITS, &LCD->RAM, DMA_SIZE_16BITS, DMA_MEM_2_MEM | DMA_PINC_MODE);
  dma_set_num_transfers(FSMC_DMA_DEV, FSMC_DMA_CHANNEL, length);
  dma_clear_isr_bits(FSMC_DMA_DEV, FSMC_DMA_CHANNEL);
  dma_enable(FSMC_DMA_DEV, FSMC_DMA_CHANNEL);

  while ((dma_get_isr_bits(FSMC_DMA_DEV, FSMC_DMA_CHANNEL) & 0x0A) == 0) {};
  dma_disable(FSMC_DMA_DEV, FSMC_DMA_CHANNEL);
}

void LCD_IO_WriteSequence_Async(uint16_t *data, uint16_t length) {
  dma_setup_transfer(FSMC_DMA_DEV, FSMC_DMA_CHANNEL, data, DMA_SIZE_16BITS, &LCD->RAM, DMA_SIZE_16BITS, DMA_MEM_2_MEM | DMA_PINC_MODE);
  dma_set_num_transfers(FSMC_DMA_DEV, FSMC_DMA_CHANNEL, length);
  dma_clear_isr_bits(FSMC_DMA_DEV, FSMC_DMA_CHANNEL);
  dma_enable(FSMC_DMA_DEV, FSMC_DMA_CHANNEL);
}

void LCD_IO_WaitSequence_Async() {
  while ((dma_get_isr_bits(FSMC_DMA_DEV, FSMC_DMA_CHANNEL) & 0x0A) == 0) {};
  dma_disable(FSMC_DMA_DEV, FSMC_DMA_CHANNEL);
}

#endif // LCD_USE_DMA_FSMC

#endif // HAS_GRAPHICAL_LCD
#endif
