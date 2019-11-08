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
#pragma once

#include "HAL/HAL_IO.h"

#ifndef PWM
  #define PWM OUTPUT
#endif

void FastIO_init();

#define READ(IO)                HAL_IORead(IO)
#define WRITE(IO,V)             HAL_IOWrite(IO,V)

#define _GET_MODE(IO)
#define _SET_MODE(IO,M)         pinMode(IO, M)
#define _SET_OUTPUT(IO)         pinMode(IO, OUTPUT)                               /*!< Output Push Pull Mode & GPIO_NOPULL   */

#define OUT_WRITE(IO,V)         do{ _SET_OUTPUT(IO); WRITE(IO,V); }while(0)

#define SET_INPUT(IO)           _SET_MODE(IO, INPUT)                              /*!< Input Floating Mode                   */
#define SET_INPUT_PULLUP(IO)    _SET_MODE(IO, INPUT_PULLUP)                       /*!< Input with Pull-up activation         */
#define SET_INPUT_PULLDOWN(IO)  _SET_MODE(IO, INPUT_PULLDOWN)                     /*!< Input with Pull-down activation       */
#define SET_OUTPUT(IO)          OUT_WRITE(IO, LOW)
#define SET_PWM(IO)             _SET_MODE(IO, PWM)

#define TOGGLE(IO)              OUT_WRITE(IO, !READ(IO))

#define IS_INPUT(IO)
#define IS_OUTPUT(IO)

#define PWM_PIN(P)              true

// digitalRead/Write wrappers
#define extDigitalRead(IO)    digitalRead(IO)
#define extDigitalWrite(IO,V) digitalWrite(IO,V)


