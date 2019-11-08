#if defined(STM32GENERIC) && defined(STM32F767xx)
#include "HAL.h"
#include "serial.h"
#include "usbd_cdc_if.h"
#define TIME_OUT  1000


static bool hasRxData = false;


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	hasRxData = true;
}

HalSerialUART::HalSerialUART()
:bufferpos(0)
{

}

HalSerialUART::~HalSerialUART()
{

}
void HalSerialUART::begin(int32_t baud)
{
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
	uint8_t data;
	if (HAL_UART_Receive(&huart3, &data, 1, TIME_OUT) != HAL_OK)
	{
		// log some error
	}
	return data;
}

void HalSerialUART::write(const char c)
{
	HAL_UART_Transmit(&huart3, (uint8_t*)&c, 1, TIME_OUT);
}

void HalSerialUART::write(const char* str)
{
	size_t size = strlen(str);
	HAL_UART_Transmit(&huart3, (uint8_t *)str, size, TIME_OUT);
}

void HalSerialUART::write(const uint8_t* buffer, size_t size)
{
	HAL_UART_Transmit(&huart3, (uint8_t *)buffer, size, TIME_OUT);
}

uint16_t HalSerialUART::available()
{
	//HAL_UART_GetState(huart)

	//
	return huart3.Instance->ISR & USART_ISR_RXNE;;
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
void HalSerialUART::printf(const char *format, ...)
{
}
void HalSerialUART::print_bin(uint32_t value, uint8_t num_digits)
{
}

void HalSerialUART::print(const char value[])
{
}
void HalSerialUART::print(char value, int nbase)
{
}
void HalSerialUART::print(unsigned char value, int nbase)
{
}
void HalSerialUART::print(int value, int nbase)
{
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

#endif
