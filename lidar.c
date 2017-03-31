#include <stdint.h>
#include "ext_include/stm32f2xx.h"

#include "lidar.h"

volatile int lidar_initialized;
volatile lidar_datum_t lidar_full_rev[90];


/*
Reading the LIDAR is a bit tricky, because the start delimiter byte is not escaped and can reappear in the data.
The stream of data is rather continuous, and it's unreliable to rely to idle times.

Each packet is 22 bytes fixed.
Full revolution = 1980 bytes

*/

#define L_ON()  {GPIOC->BSRR = 1UL<<13;}
#define L_OFF() {GPIOC->BSRR = 1UL<<(13+16);}

void sync_lidar()
{
	int i;
	int shift = 20;
	__disable_irq();
	while(1)
	{
		if(USART1->SR & (1UL<<5)) // data ready
		{
			int data = USART1->DR;
			if(data == 0xFA)
			{
				while(!(USART1->SR & (1UL<<5))) ;
				data = USART1->DR;
				if(data == 0xA0)
				{
					for(i=0; i < shift; i++)
					{
						while(!(USART1->SR & (1UL<<5))) ;
						data = USART1->DR;
					}
					break;
				}

			}
		}
/*		int hex0 = (DMA2_Stream2->NDTR&0x0F);
		int hex1 = (lidar_full_rev[1].start&0xF0)>>4;
		int hex2 = (lidar_full_rev[1].start&0x0F);
		if(hex0 < 10) hex0 += '0'; else hex0 += 'A'-10;
		if(hex1 < 10) hex1 += '0'; else hex1 += 'A'-10;
		if(hex2 < 10) hex2 += '0'; else hex2 += 'A'-10;
		while((USART3->SR & (1UL<<7)) == 0) ;
		USART3->DR = hex0;
		while((USART3->SR & (1UL<<7)) == 0) ;
		USART3->DR = hex1;
		while((USART3->SR & (1UL<<7)) == 0) ;
		USART3->DR = hex2;
		while((USART3->SR & (1UL<<7)) == 0) ;
		USART3->DR = '\r';
		while((USART3->SR & (1UL<<7)) == 0) ;
		USART3->DR = '\n';
*/
	}
	DMA2_Stream2->CR = 4UL<<25 /*Channel*/ | 0b01UL<<16 /*med prio*/ | 0b00UL<<13 /*8-bit mem*/ | 0b00UL<<11 /*8-bit periph*/ |
	                   1UL<<10 /*mem increment*/ | 1UL<<8 /*circular*/;  // Disable
	USART1->SR = 0;
	USART1->CR3 = 1UL<<6 /*RX DMA*/;
	DMA2_Stream2->NDTR = 22*90;
	DMA2->LIFCR = 0xffffffff; // Clear all flags
	DMA2->HIFCR = 0xffffffff;
	DMA2_Stream2->CR |= 1UL; // Enable
	__enable_irq();
}

// Requires little-endian CPU
uint16_t lidar_calc_checksum(volatile lidar_datum_t* l)
{
	int i;
	uint32_t chk32 = 0;

	for(i=0; i < 10; i++)
	{
		chk32 = (chk32<<1) + l->u16[i];
	}

	uint32_t checksum = (chk32 & 0x7FFF) + (chk32 >> 15);
	checksum &= 0x7FFF;
	return checksum;
}

void init_lidar()
{
	// USART1 (lidar) = APB2 = 60 MHz
	// 16x oversampling
	// 115200bps -> Baudrate register = 32.5625 = 32 9/16
	// USART1 RX is mapped to DMA2, Stream2, Ch4

	DMA2_Stream2->PAR = (uint32_t)&(USART1->DR);
	DMA2_Stream2->M0AR = (uint32_t)(lidar_full_rev);
//	DMA2_Stream2->NDTR = 22*90-1;
//	DMA2_Stream2->CR = 4UL<<25 /*Channel*/ | 0b01UL<<16 /*med prio*/ | 0b00UL<<13 /*8-bit mem*/ | 0b00UL<<11 /*8-bit periph*/ |
//	                   1UL<<10 /*mem increment*/ | 1UL<<8 /*circular*/;

//	DMA2->LIFCR = 0xffffffff; // Clear all flags
//	DMA2->HIFCR = 0xffffffff;
//	DMA2_Stream2->CR |= 1UL; // Enable

	USART1->BRR = 32UL<<4 | 9UL;
//	USART1->CR3 = 1UL<<6 /*RX DMA*/;
	USART1->CR1 = 1UL<<13 /*USART enable*/ | 1UL<<3 /*TX ena*/ | 1UL<<2 /*RX ena*/;

	lidar_initialized = 1;

}

