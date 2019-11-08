
#if defined(STM32GENERIC) && defined(STM32F767xx)
#include "../../inc/MarlinConfig.h"
#include "../shared/Delay.h"
#include "HAL.h"

HalSerialUSB usb_serial;
HalSerialUART uart_serial;
uint16_t HAL_adc_result;

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

void HAL_adc_start_conversion(const uint8_t adc_pin)
{
	HAL_adc_result = analogRead(adc_pin);
}

uint16_t HAL_adc_get_result(void)
{
	return HAL_adc_result;
}

#endif
