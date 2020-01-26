#if defined(STM32GENERIC) && defined(STM32F767xx)

//#include "../../inc/MarlinConfig.h"
#include <stdlib.h>
#include "Arduino.h"
//#include "HAL.h"
#include "stm32f7xx_hal.h"
#include "../../inc/MarlinConfig.h"
#include "timers.h"
void delay(const int milis)
{
	HAL_Delay(milis);
}



// IO functions
// As defined by Arduino INPUT(0x0), OUTPUT(0x1), INPUT_PULLUP(0x2)
void pinMode(const pin_t pin, const uint8_t mode) {
  //if (!VALID_PIN(pin)) return;

}

void digitalWrite(pin_t pin, uint8_t pin_status) {
  //if (!VALID_PIN(pin)) return;
	HAL_IOWrite(pin, pin_status);
}

bool digitalRead(pin_t pin) {
  //if (!VALID_PIN(pin)) return false;
  return false;
}

void analogWrite(pin_t pin, int pwm_value) {  // 1 - 254: pwm_value, 0: LOW, 255: HIGH
  //if (!VALID_PIN(pin)) return;

	switch(pin) {
		case E0_AUTO_FAN_PIN:
			HAL_timer_set_PWM(FAN_TIMER_NUM, TIM_CHANNEL_1, pwm_value);
			break;
		case FAN_PIN:
			HAL_timer_set_PWM(FAN_TIMER_NUM, TIM_CHANNEL_3, pwm_value);
			break;
		case CONTROLLER_FAN_PIN:
			HAL_timer_set_PWM(FAN_TIMER_NUM, TIM_CHANNEL_4, pwm_value);
			break;
	}

}

uint16_t analogRead(pin_t adc_pin) {
  //if (!VALID_PIN(DIGITAL_PIN_TO_ANALOG_PIN(adc_pin))) return 0;
  return 500;
}

char *dtostrf(double __val, signed char __width, unsigned char __prec, char *__s) {
  char format_string[20];
  snprintf(format_string, 20, "%%%d.%df", __width, __prec);
  sprintf(__s, format_string, __val);
  return __s;
}

int32_t random(int32_t max) {
  return rand() % max;
}

int32_t random(int32_t min, int32_t max) {
  return min + rand() % (max - min);
}

#endif
