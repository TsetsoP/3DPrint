#pragma once

#define CPU_32_BIT

#undef UNUSED  //fix redefinition waning
#include "stm32f7xx_hal.h"

// ------------------------
// Types
// ------------------------



#define F_CPU 216000000
#define SystemCoreClock F_CPU

#include "../shared/Marduino.h"
#include "../shared/math_32bit.h"
#include "../shared/HAL_SPI.h"

#include "../../inc/MarlinConfigPre.h"
#include "usb_host.h"

#include "fastio.h"
#include "watchdog.h"
#include "timers.h"
#include "HalSerial.h"
#include "serial.h"
#include "adc.h"

#include <stdint.h>
#include <stdlib.h>
#include "SPI.h"

#if ENABLED(TOUCH_BUTTONS)
	#include "TSC2046.h"
#endif //TOUCH_BUTTONS

// ------------------------
// Defines
// ------------------------

//Serial override
//extern HalSerialUSB usb_serial;
extern SpiStm32F7xx SPI;

#if !WITHIN(SERIAL_PORT, -1, 6)
  #error "SERIAL_PORT must be from -1 to 6"
#endif
#if SERIAL_PORT == -1
  #define MYSERIAL0 uart_serial0
#elif SERIAL_PORT == 1
  #define MYSERIAL0 SerialUART1
#elif SERIAL_PORT == 2
  #define MYSERIAL0 SerialUART2
#elif SERIAL_PORT == 3
  #define MYSERIAL0 SerialUART3
#elif SERIAL_PORT == 4
  #define MYSERIAL0 SerialUART4
#elif SERIAL_PORT == 5
  #define MYSERIAL0 SerialUART5
#elif SERIAL_PORT == 6
  #define MYSERIAL0 SerialUART6
#endif

#ifdef SERIAL_PORT_2
  #if !WITHIN(SERIAL_PORT_2, -1, 6)
    #error "SERIAL_PORT_2 must be from -1 to 6"
  #elif SERIAL_PORT_2 == SERIAL_PORT
    #error "SERIAL_PORT_2 must be different than SERIAL_PORT"
  #endif
  #define NUM_SERIAL 2
  #if SERIAL_PORT_2 == -1
    #define MYSERIAL1 SerialUSB
  #elif SERIAL_PORT_2 == 1
    #define MYSERIAL1 SerialUART1
  #elif SERIAL_PORT_2 == 2
    #define MYSERIAL1 SerialUART2
  #elif SERIAL_PORT_2 == 3
    #define MYSERIAL1 SerialUART3
  #elif SERIAL_PORT_2 == 4
    #define MYSERIAL1 SerialUART4
  #elif SERIAL_PORT_2 == 5
    #define MYSERIAL1 SerialUART5
  #elif SERIAL_PORT_2 == 6
    #define MYSERIAL1 SerialUART6
  #endif
#else
  #define NUM_SERIAL 1
#endif

#if  ENABLED(USB_MASS_STORAGE_SUPPORT)
	#define USB_HOST_PROCESS MX_USB_HOST_Process();
#endif

#define CRITICAL_SECTION_START  uint32_t primask = __get_PRIMASK(); __disable_irq()
#define CRITICAL_SECTION_END    if (!primask) __enable_irq()
#define ISRS_ENABLED() (!__get_PRIMASK())
#define ENABLE_ISRS()  __enable_irq()
#define DISABLE_ISRS() __disable_irq()
#define cli() __disable_irq()
#define sei() __enable_irq()

// On AVR this is in math.h?
//#define square(x) ((x)*(x))

#ifndef strncpy_P
  #define strncpy_P(dest, src, num) strncpy((dest), (src), (num))
#endif

// Fix bug in pgm_read_ptr
#undef pgm_read_ptr
#define pgm_read_ptr(addr) (*(addr))



// ------------------------
// Public Variables
// ------------------------

// Result of last ADC conversion
extern uint16_t HAL_adc_result;

// ------------------------
// Public functions
// ------------------------

// Memory related
#define __bss_end __bss_end__

void HAL_init(void);

// Clear reset reason
void HAL_clear_reset_source (void);

// Reset reason
uint8_t HAL_get_reset_source(void);

void _delay_ms(const int delay);

/*
extern "C" {
  int freeMemory(void);
}
*/

extern "C" char* _sbrk(int incr);

/*
int freeMemory() {
  volatile int top;
  top = (int)((char*)&top - reinterpret_cast<char*>(_sbrk(0)));
  return top;
}
*/

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-function"

static inline int freeMemory(void) {
  volatile char top;
  return &top - reinterpret_cast<char*>(_sbrk(0));
}

#pragma GCC diagnostic pop

//
// EEPROM
//

/**
 * TODO: Write all this EEPROM stuff. Can emulate EEPROM in flash as last resort.
 * Wire library should work for i2c EEPROMs.
 */
void eeprom_write_byte(uint8_t *pos, unsigned char value);
uint8_t eeprom_read_byte(uint8_t *pos);
void eeprom_read_block (void *__dst, const void *__src, size_t __n);
void eeprom_update_block (const void *__src, void *__dst, size_t __n);

//
// ADC
//
#define HAL_ADC_RESOLUTION 10
#define HAL_ANALOG_SELECT(pin) pinMode(pin, INPUT)

#define HAL_START_ADC(pin)  HAL_adc_start_conversion(pin)
#define HAL_READ_ADC()      HAL_adc_result
#define HAL_ADC_READY()     true

#define GET_PIN_MAP_PIN(index) index
#define GET_PIN_MAP_INDEX(pin) pin
#define PARSED_PIN_INDEX(code, dval) parser.intval(code, dval)

void Error_Handler(const char *module, HAL_StatusTypeDef *hal_error);
