#include "serial.h"


HalSerialUART uart_serial0(UART5);
/**
  * @brief This function handles UART5 global interrupt.
  */
extern "C"
void UART5_IRQHandler(void)
{
  HAL_UART_IRQHandler(uart_serial0.getUART_Handle());
}

//void UART_RxCpltCallback(UART_HandleTypeDef *huart);

void HAL_init_serial(void)
{
	//MX_USART1_UART_Init();
	//MX_USART3_UART_Init();
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart->Instance == UART5)
	{
		uart_serial0.UART_RxCpltCallback();
	}
}


void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
	if (huart->Instance == UART5)
	{
		uart_serial0.errroHandler();
	}
}
