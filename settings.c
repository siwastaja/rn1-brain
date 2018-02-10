/*
	PULUROBOT RN1-BRAIN RobotBoard main microcontroller firmware project

	(c) 2017-2018 Pulu Robotics and other contributors
	Maintainer: Antti Alhonen <antti.alhonen@iki.fi>

	This program is free software; you can redistribute it and/or modify
	it under the terms of the GNU General Public License version 2, as 
	published by the Free Software Foundation.

	This program is distributed in the hope that it will be useful,
	but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
	GNU General Public License for more details.

	GNU General Public License version 2 is supplied in file LICENSING.


	Non-volatile settings. Right now, I guess we don't have anything here yet, so
	this is not tested too well.

*/

#include <stdint.h>

#include "ext_include/stm32f2xx.h"

#include "flash.h"
#include "settings.h"
#include "main.h"

settings_t settings __attribute__((section(".settings"))) =
{
	.magic = 0x1357acef,
	.version = 10

};

// Memory addresses from the linker script:
extern unsigned int _SETTINGS_BEGIN;
extern unsigned int _SETTINGS_END;
extern unsigned int _SETTINGSI_BEGIN;

#define SETTINGS_FLASH_SECTOR 1

int verify_settings()
{
	uint32_t* settings_begin  = (uint32_t*)&_SETTINGS_BEGIN;
	uint32_t* settings_end    = (uint32_t*)&_SETTINGS_END;
	uint32_t* settingsi_begin = (uint32_t*)&_SETTINGSI_BEGIN;

	while(settings_begin < settings_end)
	{
		if(*settings_begin != *settingsi_begin)
		{
			return -1;
		}
		settings_begin++;
		settingsi_begin++;
	}
	return 0;
}

void program_setting_page()
{
	unlock_flash();
	flash_erase_sector(SETTINGS_FLASH_SECTOR);

	uint32_t* settings_begin  = (uint32_t*)&_SETTINGS_BEGIN;
	uint32_t* settings_end    = (uint32_t*)&_SETTINGS_END;
	volatile uint32_t* settingsi_begin = (uint32_t*)&_SETTINGSI_BEGIN;

	FLASH->CR = 0b10UL<<8 /*32-bit access*/ | 1UL /*activate programming*/;

	// When accessing flash, we don't use the remapped 0x0000 0000 address space, but the
	// real address space at FLASH_OFFSET. These being pointers to uint32_t (not bytes), we increment
	// the address by FLASH_OFFSET/4.
	#define FLASH_OFFSET 0x08000000
	settingsi_begin += FLASH_OFFSET/4;
	while(settings_begin < settings_end)
	{
		*settingsi_begin = *settings_begin;

		settings_begin++;
		settingsi_begin++;
		while(FLASH->SR & (1UL<<16)) ; // Poll busy bit
	}

	FLASH->CR = 0; // Clear programming bit.

	lock_flash();
}

void save_settings()
{
	program_setting_page();
	if(verify_settings())
	{
		program_setting_page();
		if(verify_settings())
		{
			error(7);
		}
	}

}
