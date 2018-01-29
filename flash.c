/*
	PULUROBOT RN1-BRAIN RobotBoard main microcontroller firmware project

	(c) 2017-2018 Pulu Robotics and other contributors

	This program is free software; you can redistribute it and/or modify
	it under the terms of the GNU General Public License version 2, as 
	published by the Free Software Foundation.

	This program is distributed in the hope that it will be useful,
	but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
	GNU General Public License for more details.

	GNU General Public License version 2 is supplied in file LICENSING.


	For in-system firmware update through UART.
	Flasher functions are designed to be located in a separate flash sector,
	which cannot be erased using these functions. (see stm32.ld)

*/



#include "ext_include/stm32f2xx.h"

#include "main.h" // for DO_KILL_PWR
#include "flash.h"

extern unsigned int _SETTINGS_BEGIN;
extern unsigned int _SETTINGS_END;
extern unsigned int _SETTINGSI_BEGIN;

#define FLASH_OFFSET 0x08000000

void unlock_flash() __attribute__((section(".flasher")));
void unlock_flash()
{
	if(FLASH->CR & (1UL<<31))
	{
		FLASH->KEYR = 0x45670123;
		FLASH->KEYR = 0xCDEF89AB;
	}
}

void lock_flash() __attribute__((section(".flasher")));
void lock_flash()
{
	FLASH->CR |= 1UL<<31;
}

int flash_erase_sector(int sector) __attribute__((section(".flasher")));
int flash_erase_sector(int sector)
{
	if(sector < 0 || sector > 11)
		return 1;

	// STM32 flash programming manual says 32-bit parallelism speeds up erasing, too. I don't understand why, but let's believe that for now.

	while(FLASH->SR & (1UL<<16)) ; // Poll busy bit
	FLASH->CR |= 0b10UL<<8 /*32-bit parallelism*/ | sector<<3 | 1UL<<1 /*sector erase*/;
	FLASH->CR |= 1UL<<16; // Start
	__asm__ __volatile__ ("nop");
	while(FLASH->SR & (1UL<<16)) ; // Poll busy bit

	FLASH->CR = 0; // Clear erase bit.
	return 0;
}

int flash_program_from_uart(uint32_t start_addr, int size) __attribute__((section(".flasher")));
int flash_program_from_uart(uint32_t start_addr, int size)
{
	int i;

	if(start_addr < 0x08000000 || start_addr >= 0x08040000 || size < 1 || size > (16+16+16+16+64+128)*1024 || start_addr+size >= 0x08040000)
		return 50;

	FLASH->CR = 1UL; // Activate programming, with 8-bit access.

	uint8_t* p_flash = (uint8_t*)start_addr;

	for(i = 0; i < size; i++)
	{
		while(!(USART3->SR & (1UL<<5))) ;
		*p_flash = (uint8_t)(USART3->DR & 0xff);
		p_flash++;
		while(FLASH->SR & (1UL<<16)) ; // Poll busy bit
	}

	FLASH->CR = 0; // Clear programming bit.

	return 0;
}

void flash_read(uint32_t start_addr, int size) __attribute__((section(".flasher")));
void flash_read(uint32_t start_addr, int size)
{
	int i;

	while(!(USART3->SR & (1UL<<7))) ;
	if(start_addr < 0x08000000 || start_addr >= 0x08040000 || size < 1 || size > (16+16+16+16+64+128)*1024 || start_addr+size >= 0x08040000)
	{
		USART3->DR = 50;
		return;
	}
	else
	{
		USART3->DR = 0;
	}

	uint8_t* p_flash = (uint8_t*)start_addr;

	for(i = 0; i < size; i++)
	{
		while(!(USART3->SR & (1UL<<7))) ;

		USART3->DR = *p_flash;
		p_flash++;
	}
}

// Address is sent MSB first
uint32_t flash_usart_u32() __attribute__((section(".flasher")));
uint32_t flash_usart_u32()
{
	int i;
	uint32_t ret = 0;
	for(i=0; i<4; i++)
	{
		ret <<= 8;
		while(!(USART3->SR & (1UL<<5))) ;
		ret |= USART3->DR;
	}
	return ret;
}


/*

Erasing:
	Send:
	100
	Sector number
	Wait
	Get 0 OK, 1 failure

Writing:
	Send:
	101
	addr 4 bytes MSB first
	size 4 bytes MSB first
	All bytes
	Get 0 OK, 1 failure

Reading:
	Send:
	102
	addr 4 bytes MSB first
	size 4 bytes MSB first
	Get 0 OK, 1 failure
	Get all data

Reset:
	Send 150
	Hard reset (KILL_PWR) is done.
	(Remember that if the battery is low (<17.5V), it will work as a shutdown instead of reset.)

	Send 151:
	Soft CPU reset is done.

*/
void flasher() __attribute__((section(".flasher")));
void flasher()
{
	uint32_t addr, size;
	uint8_t sect;
	uint8_t ret;
	while(1)
	{
		while(!(USART3->SR & (1UL<<5))) ;
		switch(USART3->DR)
		{
			case 100:
			unlock_flash();
			while(!(USART3->SR & (1UL<<5))) ;
			sect = USART3->DR;
			ret = flash_erase_sector(sect);
			lock_flash();
			while(!(USART3->SR & (1UL<<7))) ;
			USART3->DR = ret;
			break;

			case 101:
			unlock_flash();
			addr = flash_usart_u32();
			size = flash_usart_u32();
			ret = flash_program_from_uart(addr, size);
			lock_flash();
			while(!(USART3->SR & (1UL<<7))) ;
			USART3->DR = ret;
			break;

			case 102:
			addr = flash_usart_u32();
			size = flash_usart_u32();
			flash_read(addr, size);
			break;

			case 151:
			NVIC_SystemReset();
			while(1);

			case 150:
			default:
			{
			DO_KILL_PWR();
			}
			while(1);
		}

	}
}

void run_flasher()
{
	__disable_irq();

	FLASH->ACR = 0UL<<10 /* Data cache disable */ | 0UL<<9 /* Instr cache disable */ | 1UL<<8 /*prefetch enable*/ | 3UL /*3 wait states*/;

	/*
		Reconfigure UART to 115200, no interrupts. Flasher uses polling, and we want to make sure it
		starts from a clean table.
	*/
	USART3->CR1 = 0; // Disable
	delay_us(10);
	USART3->SR = 0; // Clear flags
	USART3->BRR = 16UL<<4 | 4UL; // 115200
	USART3->CR1 = 1UL<<13 /*USART enable*/ | 1UL<<3 /*TX ena*/ | 1UL<<2 /*RX ena*/;
	delay_us(10);
	flasher();
	while(1);
}

