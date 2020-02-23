#include "HAL.h"
#if defined(STM32GENERIC) && defined(STM32F767xx)

#include<stdio.h>

#define TIME_OUT  1000

HalSerialUART::HalSerialUART(USART_TypeDef *USART) :
		USART(USART), rxBufferPos(0), rxDataSize(0), hasRxData(false)
{

}

HalSerialUART::~HalSerialUART()
{

}
void HalSerialUART::UART_RxCpltCallback()
{
	rxBuffer[rxBufferPos] = (uint16_t) READ_REG(huart.Instance->RDR);
	if (rxBuffer[rxBufferPos] == '\n' || rxBuffer[rxBufferPos] == '\r'
			|| rxBufferPos == UART_BUFFER_SIZE - 1)
	{
		finishRxTransffer();
	}
	else
	{
		rxBufferPos++;
	}

}

void HalSerialUART::begin(int32_t baud)
{
	huart.Instance = USART;
	huart.Init.BaudRate = 115200;
	huart.Init.WordLength = UART_WORDLENGTH_8B;
	huart.Init.StopBits = UART_STOPBITS_1;
	huart.Init.Parity = UART_PARITY_NONE;
	huart.Init.Mode = UART_MODE_TX_RX;
	huart.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart.Init.OverSampling = UART_OVERSAMPLING_16;
	huart.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
	huart.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;

	HAL_StatusTypeDef status = HAL_UART_Init(&huart);

	if (status != HAL_OK)
	{
		Error_Handler("USART3", &status);
	}

	restartRxTransffer();
}
void HalSerialUART::end()
{
}
int HalSerialUART::peek()
{
	return 0;
}
int HalSerialUART::read()
{
	uint8_t data = this->rxBuffer[rxBufferPos];
	rxBufferPos++;
	if (rxBufferPos == rxDataSize)
	{
		restartRxTransffer();
	}

	return data;
}

void HalSerialUART::write(const char c)
{
	HAL_UART_Transmit(&huart, (uint8_t*) &c, 1, TIME_OUT);
}

void HalSerialUART::write(const char *str)
{
	size_t size = strlen(str);
	HAL_UART_Transmit(&huart, (uint8_t*) str, size, TIME_OUT);
}

void HalSerialUART::write(const uint8_t *buffer, size_t size)
{
	HAL_UART_Transmit(&huart, (uint8_t*) buffer, size, TIME_OUT);
}

uint16_t HalSerialUART::available()
{
	return this->hasRxData;
}
void HalSerialUART::flush()
{
}
uint8_t HalSerialUART::availableForWrite(void)
{
	return 0;
}
void HalSerialUART::flushTX(void)
{
}
void HalSerialUART::print_bin(uint32_t value, uint8_t num_digits)
{
}

void HalSerialUART::print(const char value[])
{
	int lenght = strlen(value);
	HAL_UART_Transmit(&huart, (uint8_t*) value, lenght, TIME_OUT);
}
void HalSerialUART::print(char value, int nbase)
{
}
void HalSerialUART::print(unsigned char value, int nbase)
{
}
void HalSerialUART::print(int value, int nbase)
{
	int lenght = 0;
	switch(nbase) {

	case 10:
	case 0:
		lenght = snprintf(numberBuffer, NUMBER_BUFFER_SIZE, "%d", value);
		break;

	case 8:
		lenght = snprintf(numberBuffer, NUMBER_BUFFER_SIZE, "%o", value);
		break;

	case 16:
		lenght = snprintf(numberBuffer, NUMBER_BUFFER_SIZE, "%x", value);
		break;
	}

	if (lenght > 0)
	{
		HAL_UART_Transmit(&huart, (uint8_t*)numberBuffer, lenght, TIME_OUT);
	}

}
void HalSerialUART::print(unsigned int value, int nbase)
{
}
void HalSerialUART::print(long value, int nbase)
{
}
void HalSerialUART::print(unsigned long value, int nbase)
{
}
void HalSerialUART::print(float value, int round)
{
	snprintf(formatBuffer, FORMAT_BUFFER_SIZE,"%s%df", "%.", round);
	int lenght = snprintf(numberBuffer, NUMBER_BUFFER_SIZE, formatBuffer, value);
	HAL_UART_Transmit(&huart, (uint8_t*)numberBuffer, lenght, TIME_OUT);
}
void HalSerialUART::print(double value, int round)
{
}
void HalSerialUART::println(const char value[])
{
}
void HalSerialUART::println(char value, int nbase)
{
}
void HalSerialUART::println(unsigned char value, int nbase)
{
}
void HalSerialUART::println(int value, int nbase)
{
}
void HalSerialUART::println(unsigned int value, int nbase)
{
}
void HalSerialUART::println(long value, int nbase)
{
}
void HalSerialUART::println(unsigned long value, int nbase)
{
}
void HalSerialUART::println(float value, int round)
{
}
void HalSerialUART::println(double value, int round)
{
}
void HalSerialUART::println(void)
{
}

UART_HandleTypeDef* HalSerialUART::getUART_Handle()
{
	return &this->huart;
}

void HalSerialUART::errroHandler()
{
	if (huart.ErrorCode & HAL_UART_ERROR_PE) {
		write("Serial: Parity ERROR!");
	}

	if (huart.ErrorCode & HAL_UART_ERROR_FE) {
		write("Serial: Frame ERROR!");
	}

	if (huart.ErrorCode & HAL_UART_ERROR_NE) {
		write("Serial: Noise ERROR!");
	}

	if (huart.ErrorCode & HAL_UART_ERROR_ORE) {
		write("Serial: Overrun ERROR!");
		restartRxTransffer();
	}
}

void HalSerialUART::restartRxTransffer()
{
	hasRxData = false;
	rxBufferPos = 0;
	rxDataSize = 0;
	huart.RxISR = HAL_UART_RxCpltCallback;
	SET_BIT(huart.Instance->CR3, USART_CR3_EIE);
	SET_BIT(huart.Instance->CR1, USART_CR1_PEIE | USART_CR1_RXNEIE);
	huart.ErrorCode = HAL_UART_ERROR_NONE;
}

void HalSerialUART::finishRxTransffer()
{
	hasRxData = true;
	rxDataSize = rxBufferPos + 1;
	rxBufferPos = 0;
	/* Disable the UART Parity Error Interrupt and RXNE interrupts */
	CLEAR_BIT(huart.Instance->CR1, (USART_CR1_RXNEIE | USART_CR1_PEIE));

	/* Disable the UART Error Interrupt: (Frame error, noise error, overrun error) */
	CLEAR_BIT(huart.Instance->CR3, USART_CR3_EIE);
}

#endif
