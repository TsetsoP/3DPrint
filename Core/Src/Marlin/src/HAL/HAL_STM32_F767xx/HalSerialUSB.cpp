#if defined(STM32GENERIC) && defined(STM32F767xx)
#include "serial.h"
#include "usbd_cdc_if.h"

HalSerialUSB::~HalSerialUSB()
{

}
void HalSerialUSB::begin(int32_t baud)
{
}
void HalSerialUSB::end()
{
}
int HalSerialUSB::peek()
{
	return 0;
}
int HalSerialUSB::read()
{
	return 0;
}

void HalSerialUSB::write(const char c)
{
	CDC_Transmit_FS((uint8_t*)&c, 1);
}

void HalSerialUSB::write(const char* str)
{
	size_t size = strlen(str);
	CDC_Transmit_FS((uint8_t*)str, size);
}

void HalSerialUSB::write(const uint8_t* buffer, size_t size)
{
	CDC_Transmit_FS((uint8_t*)buffer, size);
}

uint16_t HalSerialUSB::available()
{
	return 0;
}
void HalSerialUSB::flush()
{
}
uint8_t HalSerialUSB::availableForWrite(void)
{
	return 0;
}
void HalSerialUSB::flushTX(void)
{
}
void HalSerialUSB::printf(const char *format, ...)
{
}
void HalSerialUSB::print_bin(uint32_t value, uint8_t num_digits)
{
}

void HalSerialUSB::print(const char value[])
{
}
void HalSerialUSB::print(char value, int nbase)
{
}
void HalSerialUSB::print(unsigned char value, int nbase)
{
}
void HalSerialUSB::print(int value, int nbase)
{
}
void HalSerialUSB::print(unsigned int value, int nbase)
{
}
void HalSerialUSB::print(long value, int nbase)
{
}
void HalSerialUSB::print(unsigned long value, int nbase)
{
}
void HalSerialUSB::print(float value, int round)
{
}
void HalSerialUSB::print(double value, int round)
{
}

void HalSerialUSB::println(const char value[])
{
}
void HalSerialUSB::println(char value, int nbase)
{
}
void HalSerialUSB::println(unsigned char value, int nbase)
{
}
void HalSerialUSB::println(int value, int nbase)
{
}
void HalSerialUSB::println(unsigned int value, int nbase)
{
}
void HalSerialUSB::println(long value, int nbase)
{
}
void HalSerialUSB::println(unsigned long value, int nbase)
{
}
void HalSerialUSB::println(float value, int round)
{
}
void HalSerialUSB::println(double value, int round)
{
}
void HalSerialUSB::println(void)
{
}

#endif
