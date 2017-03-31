#include <stdint.h>
#include "ext_include/stm32f2xx.h"

#include "lidar.h"

volatile int lidar_initialized;
volatile lidar_datum_t lidar_full_rev[90];
volatile int lidar_rpm_setpoint_x64 = (DEFAULT_LIDAR_RPM)*64;

/*
Reading the LIDAR is a bit tricky, because the start delimiter byte is not escaped and can reappear in the data.
The stream of data is rather continuous, and it's unreliable to rely to idle times.

Each packet is 22 bytes fixed.
Full revolution = 1980 bytes

*/

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


void resync_lidar()
{
	// Disable DMA.
	DMA2_Stream2->CR = 4UL<<25 /*Channel*/ | 0b01UL<<16 /*med prio*/ | 0b00UL<<13 /*8-bit mem*/ | 0b00UL<<11 /*8-bit periph*/ |
	                   1UL<<10 /*mem increment*/ | 1UL<<8 /*circular*/;  // Disable
	while(DMA2_Stream2->CR & 1UL) ;
	USART1->CR3 &= ~(1UL<<6) /*disable RX DMA*/;
	sync_lidar();
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


void lidar_ctrl_loop()
{
	static int cycle = 0;
	static int pwm_shadow_x256 = 400*256;
	int i;
	int actual_speed = 0;

	if(cycle<100)
	{
		cycle++;
		return;
	}
	cycle = 0;

	for(i=0; i<90; i++)
	{
		actual_speed += lidar_full_rev[i].speed;
	}
	actual_speed /= 90;

	int error = actual_speed - lidar_rpm_setpoint_x64;

	pwm_shadow_x256 -= error/16;
	if(pwm_shadow_x256 < 100*256) pwm_shadow_x256=100*256;
	else if(pwm_shadow_x256 > 700*256) pwm_shadow_x256=700*256;
	TIM4->CCR4 = pwm_shadow_x256>>8;
}

void init_lidar()
{
	// USART1 (lidar) = APB2 = 60 MHz
	// 16x oversampling
	// 115200bps -> Baudrate register = 32.5625 = 32 9/16
	// USART1 RX is mapped to DMA2, Stream2, Ch4

	// Do not enable the DMA yet.

	DMA2_Stream2->PAR = (uint32_t)&(USART1->DR);
	DMA2_Stream2->M0AR = (uint32_t)(lidar_full_rev);

	USART1->BRR = 32UL<<4 | 9UL;
	USART1->CR1 = 1UL<<13 /*USART enable*/ | 1UL<<3 /*TX ena*/ | 1UL<<2 /*RX ena*/;

	lidar_initialized = 1;

}

