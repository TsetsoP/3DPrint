#pragma once
#include "HAL.h"

#ifdef __cplusplus
 extern "C" {
#endif

extern UART_HandleTypeDef huart5;
//extern UART_HandleTypeDef huart3;
extern HalSerialUART uart_serial0;

void UART5_IRQHandler(void);
void HAL_init_serial();

#ifdef __cplusplus
}
#endif
