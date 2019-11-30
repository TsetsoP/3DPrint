
#if defined(STM32GENERIC) && defined(STM32F767xx)
#include "../../inc/MarlinConfig.h"
#include "../shared/Delay.h"
#include "HAL.h"

#if HAS_USB_SERIAL
HalSerialUSB usb_serial;
#endif //HAS_USB_SERIAL

HalSerialUART uart_serial;

SpiStm32F7xx SPI(SPI1);

#if ENABLED(TOUCH_BUTTONS)

	SpiStm32F7xx TOUCH_SPI(HAL_TOUCH_SPI);
	TSC2046 touch(&TOUCH_SPI);

#endif //TOUCH_BUTTONS

// Needed for DELAY_NS() / DELAY_US() on CORTEX-M7
#if (defined(__arm__) || defined(__thumb__)) && __CORTEX_M == 7
  // HAL pre-initialization task
  // Force the preinit function to run between the premain() and main() function
  // of the STM32 arduino core
  __attribute__((constructor (102)))
  void HAL_preinit() {
    enableCycleCounter();
  }
#endif


void HAL_init(void)
{
	HAL_IOinit();
}


void HAL_clear_reset_source(void)
{
	__HAL_RCC_CLEAR_RESET_FLAGS();
}

void _delay_ms(const int delay_ms)
{
	delay(delay_ms);
}

uint8_t HAL_get_reset_source(void) {
  if (__HAL_RCC_GET_FLAG(RCC_FLAG_IWDGRST) != RESET) return RST_WATCHDOG;
  if (__HAL_RCC_GET_FLAG(RCC_FLAG_SFTRST)  != RESET) return RST_SOFTWARE;
  if (__HAL_RCC_GET_FLAG(RCC_FLAG_PINRST)  != RESET) return RST_EXTERNAL;
  if (__HAL_RCC_GET_FLAG(RCC_FLAG_PORRST)  != RESET) return RST_POWER_ON;
  return 0;
}

void Error_Handler(const char *module, HAL_StatusTypeDef *hal_error)
{
#if NUM_SERIAL > 0
  MYSERIAL0.write(module);
  MYSERIAL0.write(':');

  switch(*hal_error)
  {
  case HAL_ERROR:
	  MYSERIAL0.write("Module error!");
	  break;
  case HAL_BUSY:
	  MYSERIAL0.write("The module is busy!");
	  break;
  case HAL_TIMEOUT:
	  MYSERIAL0.write("Module timeout!");
	  break;
  case HAL_OK:
	  MYSERIAL0.write("Module OK.");
	  break;
  }

#endif

}
#endif
