#include "ext_include/stm32f2xx.h"

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

	while(FLASH->SR & (1UL<<16)) ; // Poll busy bit
	FLASH->CR |= sector<<3 | 1UL<<1 /*sector erase*/;
	FLASH->CR |= 1UL<<16; // Start
	__asm__ __volatile__ ("nop");
	while(FLASH->SR & (1UL<<16)) ; // Poll busy bit

	FLASH->CR = 0; // Clear erase bit.
	return 0;
}

int flash_program(uint32_t start_addr, int size) __attribute__((section(".flasher")));
int flash_program(uint32_t start_addr, int size)
{
	int i;

	if(start_addr < 0x08000000 || start_addr > 0x0808FFFF || size < 1 || size > 128*1024)
		return 1;

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
	if(start_addr < 0x08000000 || start_addr > 0x0808FFFF || size < 1 || size > 128*1024)
	{
		USART3->DR = 1;
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

*/

#define DO_KILL_PWR() {GPIOD->BSRR = 1UL<<5;}

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
			ret = flash_program(addr, size);
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
