//#include "../../inc/MarlinConfig.h"
//#include "HAL.h"

#include "stm32f7xx_hal.h"

void u8g_Delay(uint16_t val) {
	HAL_Delay(val);
}
void u8g_MicroDelay(void)
{
   usleep(1);
}
void u8g_10MicroDelay(void)
{
   usleep(10);
}
