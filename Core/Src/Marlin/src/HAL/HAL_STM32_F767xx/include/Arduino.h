/*
 * Arduino.h
 *
 *  Created on: 13.09.2019 г.
 *      Author: ceco
 */

#pragma once

#include <stdint.h>
#include <stddef.h>
#include <cstring>
#include <stdio.h>
#include <pinmapping.h>
#include <HAL_IO.h>

#define HIGH         0x01
#define LOW          0x00

#define INPUT          0x00
#define OUTPUT         0x01
#define INPUT_PULLUP   0x02
#define INPUT_PULLDOWN 0x03

#define PROGMEM
#define PSTR(v) (v)
#define PGM_P const char *

#define constrain(value, arg_min, arg_max) ((value) < (arg_min) ? (arg_min) :((value) > (arg_max) ? (arg_max) : (value)))

typedef uint8_t byte;

uint32_t millis();

//IO functions
void pinMode(const pin_t, const uint8_t);
void digitalWrite(pin_t, uint8_t);
bool digitalRead(pin_t);
void analogWrite(pin_t, int);
uint16_t analogRead(pin_t);

// Program Memory
#define pgm_read_ptr(addr)        (*((void**)(addr)))
#define pgm_read_byte_near(addr)  (*((uint8_t*)(addr)))
#define pgm_read_float_near(addr) (*((float*)(addr)))
#define pgm_read_word_near(addr)  (*((uint16_t*)(addr)))
#define pgm_read_dword_near(addr) (*((uint32_t*)(addr)))
#define pgm_read_byte(addr)       pgm_read_byte_near(addr)
#define pgm_read_float(addr)      pgm_read_float_near(addr)
#define pgm_read_word(addr)       pgm_read_word_near(addr)
#define pgm_read_dword(addr)      pgm_read_dword_near(addr)

int32_t random(int32_t);
int32_t random(int32_t, int32_t);
int map(uint16_t x, uint16_t in_min, uint16_t in_max, uint16_t out_min, uint16_t out_max);

using std::memcpy;
#define memcpy_P memcpy
#define sprintf_P sprintf
#define strstr_P strstr
#define strncpy_P strncpy
#define vsnprintf_P vsnprintf
#define strcpy_P strcpy
#define snprintf_P snprintf
#define strlen_P strlen

char *dtostrf(double __val, signed char __width, unsigned char __prec, char *__s);


extern "C" {
  void delay(const int milis);
}
