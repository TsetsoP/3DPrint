/**
 * Marlin 3D Printer Firmware
 *
 * Copyright (c) 2020 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
 * Copyright (c) 2016 Bob Cousins bobcousins42@googlemail.com
 * Copyright (c) 2015-2016 Nico Tonnhofer wurstnase.reprap@gmail.com
 * Copyright (c) 2016 Victor Perez victor_pv@hotmail.com
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

#include "../../inc/MarlinConfigPre.h"
#if defined(STM32GENERIC) && defined(STM32F767xx) && ENABLED(EEPROM_SETTINGS)

#include "../shared/persistent_store_api.h"
#include "../../core/serial.h"
#include "w25qxx/w25qxx.h"

#define SECTOR_ADDRESS  4096
#define BUFFER_SIZE 1024

static uint8_t buffer[BUFFER_SIZE];
static bool is_page_durty = false;

static void load_sector()
{
	 W25qxx_ReadSector(buffer, SECTOR_ADDRESS, 0, BUFFER_SIZE);
}

static void save_sector()
{
	 W25qxx_EraseSector(SECTOR_ADDRESS);
	 W25qxx_WriteSector(buffer, SECTOR_ADDRESS, 0, BUFFER_SIZE);
	 is_page_durty = false;
}

static void write_byte_buffered(int pos, uint8_t data)
{
	buffer[pos] = data;
	is_page_durty = true;
}


static uint8_t read_byte_buffered(int pos)
{
	return buffer[pos];
}


bool PersistentStore::access_start()
{
	load_sector();
	return true;
}

bool PersistentStore::access_finish()
{
	if (is_page_durty)
	{
		save_sector();
	}
	return true;
}

bool PersistentStore::write_data(int &pos, const uint8_t *value, size_t size, uint16_t *crc)
{
	while (size--)
	{
		uint8_t v = *value;
		write_byte_buffered(pos, v);
		crc16(crc, &v, 1);
		pos++;
		value++;
	};
	return false;
}

bool PersistentStore::read_data(int &pos, uint8_t *value, size_t size, uint16_t *crc, const bool writing/*=true*/)
{
	do
	{
		uint8_t read_byte = read_byte_buffered(pos);
		if (writing)
		{
			*value = read_byte;
		}
		crc16(crc, &read_byte, 1);
		pos++;
		value++;
	}
	while (--size);

	return false;
}

size_t PersistentStore::capacity()
{
	return E2END + 1;
}

#endif // EEPROM_SETTINGS
#endif // STM32GENERIC && (STM32F4 || STM32F7)
