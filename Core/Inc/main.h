/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f7xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define DRV_Z1_CS_Pin GPIO_PIN_3
#define DRV_Z1_CS_GPIO_Port GPIOF
#define BED_PROBE_Pin GPIO_PIN_10
#define BED_PROBE_GPIO_Port GPIOF
#define DRV_Y_DIR_Pin GPIO_PIN_11
#define DRV_Y_DIR_GPIO_Port GPIOF
#define DRV_X_DIR_Pin GPIO_PIN_12
#define DRV_X_DIR_GPIO_Port GPIOF
#define DRV_Z2_EN_Pin GPIO_PIN_13
#define DRV_Z2_EN_GPIO_Port GPIOF
#define DRV_E0_EN_Pin GPIO_PIN_14
#define DRV_E0_EN_GPIO_Port GPIOF
#define DRV_Z1_EN_Pin GPIO_PIN_15
#define DRV_Z1_EN_GPIO_Port GPIOF
#define LCD_DC_Pin GPIO_PIN_0
#define LCD_DC_GPIO_Port GPIOG
#define DRV_E0_DIR_Pin GPIO_PIN_7
#define DRV_E0_DIR_GPIO_Port GPIOE
#define DRV_Z1_DIR_Pin GPIO_PIN_9
#define DRV_Z1_DIR_GPIO_Port GPIOE
#define DRV_Z1_STEP_Pin GPIO_PIN_11
#define DRV_Z1_STEP_GPIO_Port GPIOE
#define HEAT0_Pin GPIO_PIN_12
#define HEAT0_GPIO_Port GPIOE
#define DRV_Z2_CS_Pin GPIO_PIN_13
#define DRV_Z2_CS_GPIO_Port GPIOE
#define DRV_Z2_DIR_Pin GPIO_PIN_14
#define DRV_Z2_DIR_GPIO_Port GPIOE
#define DRV_Z2_STEP_Pin GPIO_PIN_15
#define DRV_Z2_STEP_GPIO_Port GPIOE
#define DRV_X_CS_Pin GPIO_PIN_10
#define DRV_X_CS_GPIO_Port GPIOD
#define DRV_E0_STEP_Pin GPIO_PIN_14
#define DRV_E0_STEP_GPIO_Port GPIOD
#define DRV_E0_CS_Pin GPIO_PIN_15
#define DRV_E0_CS_GPIO_Port GPIOD
#define DRV_X_EN_Pin GPIO_PIN_4
#define DRV_X_EN_GPIO_Port GPIOG
#define DRV_Y_EN_Pin GPIO_PIN_5
#define DRV_Y_EN_GPIO_Port GPIOG
#define USB_VBUS_Pin GPIO_PIN_6
#define USB_VBUS_GPIO_Port GPIOG
#define DRV_Y_CS_Pin GPIO_PIN_8
#define DRV_Y_CS_GPIO_Port GPIOG
#define LCD_CS_Pin GPIO_PIN_0
#define LCD_CS_GPIO_Port GPIOD
#define LCD_RESET_Pin GPIO_PIN_1
#define LCD_RESET_GPIO_Port GPIOD
#define TOUCH_CS_Pin GPIO_PIN_10
#define TOUCH_CS_GPIO_Port GPIOG
#define TOUCH_MISO_Pin GPIO_PIN_12
#define TOUCH_MISO_GPIO_Port GPIOG
#define TOUCH_CLK_Pin GPIO_PIN_13
#define TOUCH_CLK_GPIO_Port GPIOG
#define DRV_X_STEP_Pin GPIO_PIN_14
#define DRV_X_STEP_GPIO_Port GPIOG
#define TOUCH_MOSI_Pin GPIO_PIN_5
#define TOUCH_MOSI_GPIO_Port GPIOB
#define LED_BLUE_Pin GPIO_PIN_7
#define LED_BLUE_GPIO_Port GPIOB
#define DRV_Y_STEP_Pin GPIO_PIN_0
#define DRV_Y_STEP_GPIO_Port GPIOE
#define TOUCH_IRQ_Pin GPIO_PIN_1
#define TOUCH_IRQ_GPIO_Port GPIOE
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
