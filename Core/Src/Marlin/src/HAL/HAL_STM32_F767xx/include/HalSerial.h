/*
 * SerialUSB.h
 *
 *  Created on: Sep 14, 2019
 *      Author: ceco
 */
#pragma once

#include "../../../inc/MarlinConfigPre.h"
#include <stddef.h>

#define DEC 10
#define HEX 16
#define OCT 8
#define BIN 2

#if HAS_USB_SERIAL
class HalSerialUSB
{

public:

  #if ENABLED(EMERGENCY_PARSER)
    EmergencyParser::State emergency_state;
  #endif

	HalSerialUSB()
	{
		host_connected = true;
	}
	virtual ~HalSerialUSB();
	void begin(int32_t baud);
	void end();
	int peek();
	int read();

	void write(const char c);
	void write(const char* str);
	void write(const uint8_t* buffer, size_t size);
	//void print(const String& s) { for (int i = 0; i < (int)s.length(); i++) write(s[i]); }

	operator bool()
	{
		return host_connected;
	}
	uint16_t available();
	void flush();
	uint8_t availableForWrite(void);
	void flushTX(void);
	void printf(const char *format, ...);
	void print_bin(uint32_t value, uint8_t num_digits);

	void print(const char value[]);
	void print(char value, int nbase = 0);
	void print(unsigned char value, int nbase = 0);
	void print(int value, int nbase = 0);
	void print(unsigned int value, int nbase = 0);
	void print(long value, int nbase = 0);
	void print(unsigned long value, int nbase = 0);
	void print(float value, int round = 6);
	void print(double value, int round = 6);

	void println(const char value[]);
	void println(char value, int nbase = 0);
	void println(unsigned char value, int nbase = 0);
	void println(int value, int nbase = 0);
	void println(unsigned int value, int nbase = 0);
	void println(long value, int nbase = 0);
	void println(unsigned long value, int nbase = 0);
	void println(float value, int round = 6);
	void println(double value, int round = 6);
	void println(void);

	//volatile RingBuffer<uint8_t, 128> receive_buffer;
	//volatile RingBuffer<uint8_t, 128> transmit_buffer;
	volatile bool host_connected;

};
#endif //HAS_USB_SERIAL

class HalSerialUART
{
#define UART_BUFFER_SIZE 64
#define NUMBER_BUFFER_SIZE 18
#define FORMAT_BUFFER_SIZE 6
public:

  #if ENABLED(EMERGENCY_PARSER)
    EmergencyParser::State emergency_state;
  #endif

    HalSerialUART(USART_TypeDef *USART);
	virtual ~HalSerialUART();
	void begin(int32_t baud);
	void end();
	int peek();
	int read();

	void write(const char c);
	void write(const char* str);
	void write(const uint8_t* buffer, size_t size);
	//void print(const String& s) { for (int i = 0; i < (int)s.length(); i++) write(s[i]); }

	operator bool()
	{
		return true;
	}
	uint16_t available();
	void flush();
	uint8_t availableForWrite(void);
	void flushTX(void);
	void print_bin(uint32_t value, uint8_t num_digits);

	void print(const char value[]);
	void print(char value, int nbase = 0);
	void print(unsigned char value, int nbase = 0);
	void print(int value, int nbase = 0);
	void print(unsigned int value, int nbase = 0);
	void print(long value, int nbase = 0);
	void print(unsigned long value, int nbase = 0);
	void print(float value, int round = 6);
	void print(double value, int round = 6);

	void println(const char value[]);
	void println(char value, int nbase = 0);
	void println(unsigned char value, int nbase = 0);
	void println(int value, int nbase = 0);
	void println(unsigned int value, int nbase = 0);
	void println(long value, int nbase = 0);
	void println(unsigned long value, int nbase = 0);
	void println(float value, int round = 6);
	void println(double value, int round = 6);
	void println(void);

	void UART_RxCpltCallback();
	void errroHandler();
	UART_HandleTypeDef *getUART_Handle();
	//volatile RingBuffer<uint8_t, 128> receive_buffer;
	//volatile RingBuffer<uint8_t, 128> transmit_buffer;
	//volatile bool host_connected;
private:
	void restartRxTransffer();
	void finishRxTransffer();
	USART_TypeDef *USART;
	UART_HandleTypeDef huart;
	uint8_t rxBuffer[UART_BUFFER_SIZE];
	uint8_t rxBufferPos;
	uint16_t rxDataSize;
	bool hasRxData;

	char numberBuffer[NUMBER_BUFFER_SIZE];
	char formatBuffer[FORMAT_BUFFER_SIZE];

};

