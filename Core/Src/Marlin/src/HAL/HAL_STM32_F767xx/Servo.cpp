/**
 * Marlin 3D Printer Firmware
 * Copyright (c) 2019 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
 *
 * Based on Sprinter and grbl.
 * Copyright (c) 2011 Camiel Gubbels / Erik van der Zalm
 * Copyright (c) 2017 Victor Perez
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

#if defined(STM32GENERIC) && defined(STM32F767xx)

#include "../../inc/MarlinConfig.h"

//#if HAS_SERVOS

#include "Servo.h"

libServo::libServo()
:angle(0)
{

}

bool libServo::attach(const int32_t pin)
{
  return true;
}

/*bool libServo::attach(const int32_t inPin, const int32_t inMinAngle, const int32_t inMaxAngle)
{
  return false;
}*/

void libServo::move(const uint8_t value)
{
	angle = value;
	uint16_t tau = uint16_t(map(angle,  SERVO_DEFAULT_MIN_ANGLE, SERVO_DEFAULT_MAX_ANGLE, SERVO_DEFAULT_MIN_PW, SERVO_DEFAULT_MAX_PW));
	HAL_timer_set_PWM_uS(SERVO_TIMER_NUM, TIM_CHANNEL_1, tau);
    #if ENABLED(DEACTIVATE_SERVOS_AFTER_MOVE)
      //detach();
    #endif

}
bool libServo::detach()
{
	return false;
}

int8_t libServo::read() const
{
	return angle;
}

//#endif // HAS_SERVOS
#endif // STM32GENERIC && STM32F767xx
