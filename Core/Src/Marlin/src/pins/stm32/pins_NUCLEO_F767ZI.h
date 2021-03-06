/**
 * Marlin 3D Printer Firmware
 * Copyright (c) 2019 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
 *
 * Based on Sprinter and grbl.
 * Copyright (c) 2011 Camiel Gubbels / Erik van der Zalm
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

#ifndef STM32F767xx
  #error "Oops! Select an STM32F767xx board in 'Tools > Board.'"
#elif HOTENDS > 3 || E_STEPPERS > 3
  #error "The-Borg supports up to 3 hotends / E-steppers."
#endif

#define BOARD_INFO_NAME      "NUCLEO_3D"
#define DEFAULT_MACHINE_NAME BOARD_INFO_NAME

#define E2END 0xFFF   // EEPROM end address

// Ignore temp readings during development.
//#define BOGUS_TEMPERATURE_GRACE_PERIOD 2000

//
// Limit Switches
//
#define X_MIN_PIN          PE8
#define Y_MIN_PIN          PF4
#define Z_MIN_PIN          PF5
//#define X_MAX_PIN          PE10
//#define Y_MAX_PIN          PE8
//#define Z_MAX_PIN          PG0
//#define E_MIN_PIN          PE2
//#define E_MAX_PIN          PE3

//
// Z Probe (when not Z_MIN_PIN)
//
//#ifndef Z_MIN_PROBE_PIN
//  #define Z_MIN_PROBE_PIN  PF10
//#endif

//
// Steppers
//
#define STEPPER_ENABLE_PIN PE1

#define X_STEP_PIN         PG14
#define X_DIR_PIN          PF12
#define X_ENABLE_PIN       PG4
#ifndef X_CS_PIN
  #define X_CS_PIN         PD10
#endif

#define Y_STEP_PIN         PE0
#define Y_DIR_PIN          PF11
#define Y_ENABLE_PIN       PG5
#ifndef Y_CS_PIN
  #define Y_CS_PIN         PG8
#endif

#define Z_STEP_PIN         PE11
#define Z_DIR_PIN          PE9
#define Z_ENABLE_PIN       PF15
#ifndef Z_CS_PIN
  #define Z_CS_PIN         PF3
#endif

#define Z2_STEP_PIN        PE15
#define Z2_DIR_PIN         PE14
#define Z2_ENABLE_PIN      PF13
#ifndef Z2_CS_PIN
  #define Z2_CS_PIN        PE13
#endif

#define E0_STEP_PIN        PD14
#define E0_DIR_PIN         PE7
#define E0_ENABLE_PIN      PF14
#ifndef E0_CS_PIN
  #define E0_CS_PIN        PD15
#endif

// Direction buttons
//#define BTN_EN1           PD13
//#define BTN_EN2          PD12
#define BTN_BACK          PD11
#define BTN_ENC           PE10
#define BTN_UP            PD12
#define BTN_DWN           PD13
//#define BTN_LFT          PD11
//#define BTN_RT           PE10

#if ENABLED(TOUCH_BUTTONS)
  #define TOUCH_CS_PIN     PG15   // SPI6_NSS
  #define TOUCH_SCK_PIN    PG13  // SPI6_SCK
  #define TOUCH_MISO_PIN   PB12  // SPI6_MISO
  #define TOUCH_MOSI_PIN   PB14  // SPI6_MOSI
  #define TOUCH_INT_PIN    PG11
#endif

//
// Temperature Sensors
//
#define TEMP_0_PIN         PA6   // Analog Input
//#define TEMP_1_PIN         PC2   // Analog Input
//#define TEMP_2_PIN         PC1   // Analog Input
//#define TEMP_3_PIN         PC0   // Analog Input
//#define TEMP_BED_PIN       PF10  // Analog Input


//
// Heaters / Fans
//
//#define ALFAWISE_UX0             // used for Open drain mosfets
#define HEATER_0_INVERTING true
#define HEATER_0_PIN       PE12
//#define HEATER_1_PIN       PD14
//#define HEATER_BED_PIN     PF6

#define FAN_INVERTING      true
//#define FAN_SOFT_PWM
#ifndef FAN_PIN
  #define FAN_PIN          PB10
#endif
//#define FAN1_PIN           PB10
#define CONTROLLER_FAN_PIN PA3
#define ORIG_E0_AUTO_FAN_PIN  PA0   // Use this by NOT overriding E0_AUTO_FAN_PIN

#if HAS_CUTTER
  #define SPINDLE_LASER_PWM_PIN     PC6
  #define SPINDLE_LASER_ENA_PIN     PF10
#endif

//
// Misc. Functions
//

//#define CASE_LIGHT_PIN_CI PF13
//#define CASE_LIGHT_PIN_DO PF14
//#define NEOPIXEL_PIN      PF13

//
// Průša i3 MK2 Multi Material Multiplexer Support
//

//#define E_MUX0_PIN         PG3
//#define E_MUX1_PIN         PG4

//
// Servos
//
#define SERVO0_PIN         PB0   // XS2-5
//#define SERVO1_PIN         PF7   // XS1-5
//#define SERVO2_PIN         PF8   // XS1-6

#define SDSS               PA8
#define SS_PIN             PA8
//#define LED_PIN            PA2   // Alive
#define PS_ON_PIN          PA3
#define KILL_PIN           -1 //PD5   // EXP2-10
//#define PWR_LOSS           PG5   // Power loss / nAC_FAULT

//
// MAX7219_DEBUG
//
//#define MAX7219_CLK_PIN    PG10  // EXP1-1
//#define MAX7219_DIN_PIN    PD7   // EXP1-3
//#define MAX7219_LOAD_PIN   PD1   // EXP1-5

//
// LCD / Controller
//
//#define LCD_RESET_PIN      PF6
//#define LCD_BACKLIGHT_PIN  PG11
#define FSMC_CS_PIN        PG12  // NE4
#define FSMC_RS_PIN        PF0   // A0

