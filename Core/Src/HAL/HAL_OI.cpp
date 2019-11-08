#include "HAL/HAL_IO.h"
#include "stm32f7xx_hal.h"

#define IO_PORTS 7

GPIO_TypeDef* FastIOPortMap[IO_PORTS];


void HAL_IOinit(void)
{
	FastIOPortMap[PORTA] = _STM_PORT(A);
	FastIOPortMap[PORTB] = _STM_PORT(B);
	FastIOPortMap[PORTC] = _STM_PORT(C);
	FastIOPortMap[PORTD] = _STM_PORT(D);
	FastIOPortMap[PORTE] = _STM_PORT(E);
	FastIOPortMap[PORTF] = _STM_PORT(F);
	FastIOPortMap[PORTG] = _STM_PORT(G);
}


uint8_t HAL_IORead(pin_t pin)
{
	return HAL_GPIO_ReadPin(FastIOPortMap[STM_PIN_PORT(pin)], STM_PIN(pin));
}

void HAL_IOWrite(pin_t pin, uint8_t pin_status)
{
	GPIO_PinState io_status = pin_status == 0 ? GPIO_PIN_RESET : GPIO_PIN_SET;
	uint8_t portIndex = STM_PIN_PORT(pin);
	HAL_GPIO_WritePin(FastIOPortMap[portIndex], STM_PIN(pin), io_status);
}
