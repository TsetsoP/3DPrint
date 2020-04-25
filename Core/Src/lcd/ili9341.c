/**
  ******************************************************************************
  * @file    ili9341.c
  * @author  MCD Application Team
  * @version V1.0.2
  * @date    02-December-2014
  * @brief   This file includes the LCD driver for ILI9341 LCD.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2014 STMicroelectronics</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */ 

/* Includes ------------------------------------------------------------------*/
#include "ili9341.h"
#include "HAL_LCD_SPI.h"

void ili9341_Init(void)
{

   SPIx_Init();
   LCD_CS_HIGH();

   LCD_RESET_HIGH();

   ili9341_WriteReg (LCD_SWRESET); // software reset comand
   HAL_Delay(100);
   ili9341_WriteReg (LCD_DISPLAY_OFF); // display off
   //------------power control------------------------------
   ili9341_WriteReg (LCD_POWER1); // power control
   ili9341_WriteData   (0x26); // GVDD = 4.75v
   ili9341_WriteReg (LCD_POWER2); // power control
   ili9341_WriteData   (0x11); // AVDD=VCIx2, VGH=VCIx7, VGL=-VCIx3
   //--------------VCOM-------------------------------------
   ili9341_WriteReg (LCD_VCOM1); // vcom control
   ili9341_WriteData   (0x35); // Set the VCOMH voltage (0x35 = 4.025v)
   ili9341_WriteData   (0x3e); // Set the VCOML voltage (0x3E = -0.950v)
   ili9341_WriteReg (LCD_VCOM2); // vcom control
   ili9341_WriteData   (0xbe);

   //------------memory access control------------------------
   ili9341_WriteReg (LCD_MAC); // memory access control
   ili9341_WriteData(0x48);

   ili9341_WriteReg (LCD_PIXEL_FORMAT); // pixel format set
   //ili9341_WriteData16(0x0E05);


   ili9341_WriteData   (0x55); // 16bit /pixel

   ili9341_WriteReg(LCD_FRMCTR1);
   ili9341_WriteData(0);
   ili9341_WriteData(0x1F);
   //-------------ddram ----------------------------
   ili9341_WriteReg (LCD_COLUMN_ADDR); // column set

   ili9341_WriteData   (0x00); // x0_HIGH---0
   ili9341_WriteData   (0x00); // x0_LOW----0
   ili9341_WriteData   (0x00); // x1_HIGH---240
   ili9341_WriteData   (0xEF); // x1_LOW----240

   ili9341_WriteReg (LCD_PAGE_ADDR); // page address set

   ili9341_WriteData   (0x00); // y0_HIGH---0
   ili9341_WriteData   (0x00); // y0_LOW----0
   ili9341_WriteData   (0x01); // y1_HIGH---320
   ili9341_WriteData   (0x3F); // y1_LOW----320


   ili9341_WriteReg (LCD_TEOFF); // tearing effect off
   //LCD_write_cmd(ILI9341_TEARING_ON); // tearing effect on
   //LCD_write_cmd(ILI9341_DISPLAY_INVERSION); // display inversion
   ili9341_WriteReg (LCD_ETMOD); // entry mode set
   // Deep Standby Mode: OFF
   // Set the output level of gate driver G1-G320: Normal display
   // Low voltage detection: Disable
   ili9341_WriteData   (0x07);
   //-----------------display------------------------
   ili9341_WriteReg (LCD_DFC); // display function control
   //Set the scan mode in non-display area
   //Determine source/VCOM output in a non-display area in the partial display mode
   ili9341_WriteData   (0x0a);
   //Select whether the liquid crystal type is normally white type or normally black type
   //Sets the direction of scan by the gate driver in the range determined by SCN and NL
   //Select the shift direction of outputs from the source driver
   //Sets the gate driver pin arrangement in combination with the GS bit to select the optimal scan mode for the module
   //Specify the scan cycle interval of gate driver in non-display area when PTG to select interval scan
   ili9341_WriteData   (0x82);
   // Sets the number of lines to drive the LCD at an interval of 8 lines
   ili9341_WriteData   (0x27);
   ili9341_WriteData   (0x00); // clock divisor

   ili9341_WriteReg (LCD_SLEEP_OUT); // sleep out
   HAL_Delay(100);
   ili9341_WriteReg (LCD_DISPLAY_ON); // display on
   HAL_Delay(100);
   ili9341_WriteReg (LCD_GRAM); // memory write
   HAL_Delay(5);

   TOUCH_X_Init();
}

/**
  * @brief  Power on the LCD.
  * @param  None
  * @retval None
  */
void ili9341_Init_old(void)
{
  /* Initialize ILI9341 low level bus layer ----------------------------------*/
  //LCD_IO_Init();
  HAL_Delay(100);
  /* Configure LCD */
  ili9341_WriteReg(0xCA);
  ili9341_WriteData(0xC3);
  ili9341_WriteData(0x08);
  ili9341_WriteData(0x50);
  ili9341_WriteReg(LCD_POWERB);
  ili9341_WriteData(0x00);
  ili9341_WriteData(0xC1);
  ili9341_WriteData(0x30);
  ili9341_WriteReg(LCD_POWER_SEQ);
  ili9341_WriteData(0x64);
  ili9341_WriteData(0x03);
  ili9341_WriteData(0x12);
  ili9341_WriteData(0x81);
  ili9341_WriteReg(LCD_DTCA);
  ili9341_WriteData(0x85);
  ili9341_WriteData(0x00);
  ili9341_WriteData(0x78);
  ili9341_WriteReg(LCD_POWERA);
  ili9341_WriteData(0x39);
  ili9341_WriteData(0x2C);
  ili9341_WriteData(0x00);
  ili9341_WriteData(0x34);
  ili9341_WriteData(0x02);
  ili9341_WriteReg(LCD_PRC);
  ili9341_WriteData(0x20);
  ili9341_WriteReg(LCD_DTCB);
  ili9341_WriteData(0x00);
  ili9341_WriteData(0x00);
  ili9341_WriteReg(LCD_FRMCTR1);
  ili9341_WriteData(0x00);
  ili9341_WriteData(0x1B);
  ili9341_WriteReg(LCD_DFC);
  ili9341_WriteData(0x0A);
  ili9341_WriteData(0xA2);
  ili9341_WriteReg(LCD_POWER1);
  ili9341_WriteData(0x10);
  ili9341_WriteReg(LCD_POWER2);
  ili9341_WriteData(0x10);
  ili9341_WriteReg(LCD_VCOM1);
  ili9341_WriteData(0x45);
  ili9341_WriteData(0x15);
  ili9341_WriteReg(LCD_VCOM2);
  ili9341_WriteData(0x90);
  ili9341_WriteReg(LCD_MAC);
  ili9341_WriteData(0xC8);
  ili9341_WriteReg(LCD_3GAMMA_EN);
  ili9341_WriteData(0x00);
  ili9341_WriteReg(LCD_RGB_INTERFACE);
  ili9341_WriteData(0xC2);
  ili9341_WriteReg(LCD_DFC);
  ili9341_WriteData(0x0A);
  ili9341_WriteData(0xA7);
  ili9341_WriteData(0x27);
  ili9341_WriteData(0x04);
  
  /* Colomn address set */
  ili9341_WriteReg(LCD_COLUMN_ADDR);
  ili9341_WriteData(0x00);
  ili9341_WriteData(0x00);
  ili9341_WriteData(0x00);
  ili9341_WriteData(0xEF);
  /* Page address set */
  ili9341_WriteReg(LCD_PAGE_ADDR);
  ili9341_WriteData(0x00);
  ili9341_WriteData(0x00);
  ili9341_WriteData(0x01);
  ili9341_WriteData(0x3F);
  ili9341_WriteReg(LCD_INTERFACE);
  ili9341_WriteData(0x01);
  ili9341_WriteData(0x00);
  ili9341_WriteData(0x06);
  
  ili9341_WriteReg(LCD_GRAM);
  HAL_Delay(200);
  
  ili9341_WriteReg(LCD_GAMMA);
  ili9341_WriteData(0x01);
  
  ili9341_WriteReg(LCD_PGAMMA);
  ili9341_WriteData(0x0F);
  ili9341_WriteData(0x29);
  ili9341_WriteData(0x24);
  ili9341_WriteData(0x0C);
  ili9341_WriteData(0x0E);
  ili9341_WriteData(0x09);
  ili9341_WriteData(0x4E);
  ili9341_WriteData(0x78);
  ili9341_WriteData(0x3C);
  ili9341_WriteData(0x09);
  ili9341_WriteData(0x13);
  ili9341_WriteData(0x05);
  ili9341_WriteData(0x17);
  ili9341_WriteData(0x11);
  ili9341_WriteData(0x00);
  ili9341_WriteReg(LCD_NGAMMA);
  ili9341_WriteData(0x00);
  ili9341_WriteData(0x16);
  ili9341_WriteData(0x1B);
  ili9341_WriteData(0x04);
  ili9341_WriteData(0x11);
  ili9341_WriteData(0x07);
  ili9341_WriteData(0x31);
  ili9341_WriteData(0x33);
  ili9341_WriteData(0x42);
  ili9341_WriteData(0x05);
  ili9341_WriteData(0x0C);
  ili9341_WriteData(0x0A);
  ili9341_WriteData(0x28);
  ili9341_WriteData(0x2F);
  ili9341_WriteData(0x0F);
  
  ili9341_WriteReg(LCD_SLEEP_OUT);
  HAL_Delay(200);
  ili9341_WriteReg(LCD_DISPLAY_ON);
  /* GRAM start writing */
  ili9341_WriteReg(LCD_GRAM);
}

/**
  * @brief  Disables the Display.
  * @param  None
  * @retval LCD Register Value.
  */
uint16_t ili9341_ReadID(void)
{
  //LCD_IO_Init();
  return ((uint16_t)ili9341_ReadData(LCD_READ_ID4, LCD_READ_ID4_SIZE));
}

/**
  * @brief  Enables the Display.
  * @param  None
  * @retval None
  */
void ili9341_DisplayOn(void)
{
  /* Display On */
  ili9341_WriteReg(LCD_DISPLAY_ON);
}

/**
  * @brief  Disables the Display.
  * @param  None
  * @retval None
  */
void ili9341_DisplayOff(void)
{
  /* Display Off */
  ili9341_WriteReg(LCD_DISPLAY_OFF);
}

uint32_t ili9341_ReadData(uint16_t RegValue, uint8_t ReadSize)
{
	if (ReadSize > 4) {
		return 0;
	}

	uint8_t txData[4] = {0, 0, 0, 0};

	LCD_CS_LOW();
	LCD_DC_LOW();
	SPIx_Write(RegValue);

	LCD_DC_HIGH();
	for (int idx = 0; idx < ReadSize; ++idx) {
		SPIx_Read(&txData[idx]);
	}


	LCD_CS_HIGH();
	uint32_t reg = (uint32_t) (txData[0] << 24 |  txData[1] << 16 | txData[2] << 8 | txData[3]);
	return reg;
}

/**
  * @brief  Writes  to the selected LCD register.
  * @param  LCD_Reg: address of the selected register.
  * @retval None
  */
void ili9341_WriteReg(uint8_t regValue)
{
	LCD_CS_LOW();
	LCD_DC_LOW();
	SPIx_Write(regValue);
	LCD_CS_HIGH();
}

/**
  * @brief  Writes data to the selected LCD register.
  * @param  LCD_Reg: address of the selected register.
  * @retval None
  */
void ili9341_WriteData(uint8_t regValue)
{
	LCD_CS_LOW();
	LCD_DC_HIGH();
	SPIx_Write(regValue);
	LCD_CS_HIGH();
}

void ili9341_WriteData16(uint16_t regValue)
{
	/* Reset LCD control line CS */
	LCD_CS_LOW();

	/* Set LCD data/command line DC to High */
	LCD_DC_HIGH();

	/* Send Data */
	SPIx_WriteMultipleData((uint8_t *) &regValue, 2);

	/* Deselect : Chip Select high */
	LCD_CS_HIGH();
}

void ili9341_WriteMultipleData16(uint16_t *pData, uint16_t size)
{

	/* Reset LCD control line CS */
	LCD_CS_LOW();

	/* Set LCD data/command line DC to High */
	LCD_DC_HIGH();

	SPIx_WriteMultipleData((uint8_t *) pData, size * 2);

	/* Deselect : Chip Select high */
	LCD_CS_HIGH();

}

void ili9341_WriteMultipleData(uint8_t *pData, uint16_t size)
{
	/* Reset LCD control line CS */
	LCD_CS_LOW();

	/* Set LCD data/command line DC to High */
	LCD_DC_HIGH();

	SPIx_WriteMultipleData(pData, size);

	/* Deselect : Chip Select high */
	LCD_CS_HIGH();
}

/**
  * @brief  Get LCD PIXEL WIDTH.
  * @param  None
  * @retval LCD PIXEL WIDTH.
  */
uint16_t ili9341_GetLcdPixelWidth(void)
{
  /* Return LCD PIXEL WIDTH */
  return ILI9341_LCD_PIXEL_WIDTH;
}

/**
  * @brief  Get LCD PIXEL HEIGHT.
  * @param  None
  * @retval LCD PIXEL HEIGHT.
  */
uint16_t ili9341_GetLcdPixelHeight(void)
{
  /* Return LCD PIXEL HEIGHT */
  return ILI9341_LCD_PIXEL_HEIGHT;
}

/**
  * @}
  */ 

/**
  * @}
  */ 
  
/**
  * @}
  */ 

/**
  * @}
  */
  
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
