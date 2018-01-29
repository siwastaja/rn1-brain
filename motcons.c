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


	Motor controller controller module :-).

	Basically just relays the commands / data back through SPI (using DMA) between us and the slave
	motor control MCUs (see rn1-motcon software project)

	You can even remote update the firmware of those slave MCU's, the code for that is kinda
	kludgy right now, sorry about that.

*/

#include <stdint.h>
#include "ext_include/stm32f2xx.h"

#include "motcons.h"
#include "main.h"

volatile motcon_rx_t motcon_rx[4];
volatile motcon_tx_t motcon_tx[4];

volatile int cur_motcon;
volatile int motcons_initialized;
volatile int motcon_errors;

void motcon_rx_done_inthandler()
{
	DMA2->LIFCR = 0b111101UL<<0;
	switch(cur_motcon)
	{
		case 0: {MC1_CS1();} break;
		case 1: {MC2_CS1();} break;
		#if NUM_MOTCONS >= 3
		case 2: {MC3_CS1();} break;
		#endif
		#if NUM_MOTCONS >= 4
		case 3: {MC4_CS1();} break;
		#endif
		default: break;
	}
}

void init_motcons()
{
	#if NUM_MOTCONS >= 4
	MC4_CS1();
	#endif
	#if NUM_MOTCONS >= 3
	MC3_CS1();
	#endif

	MC2_CS1();
	MC1_CS1();

	// DMA2 STREAM 0 ch 3 = motcon RX
	DMA2_Stream0->PAR = (uint32_t)&(SPI1->DR);
	DMA2_Stream0->NDTR = MOTCON_DATAGRAM_LEN;
	DMA2_Stream0->CR = 3UL<<25 /*Channel*/ | 0b10UL<<16 /*high prio*/ | 0b01UL<<13 /*16-bit mem*/ | 0b01UL<<11 /*16-bit periph*/ |
	                   1UL<<10 /*mem increment*/ | 1UL<<4 /*transfer complete interrupt*/;

	// DMA2 STREAM 3 ch 3 = motcon TX
	DMA2_Stream3->PAR = (uint32_t)&(SPI1->DR);
	DMA2_Stream3->NDTR = MOTCON_DATAGRAM_LEN;
	DMA2_Stream3->CR = 3UL<<25 /*Channel*/ | 0b10UL<<16 /*high prio*/ | 0b01UL<<13 /*16-bit mem*/ | 0b01UL<<11 /*16-bit periph*/ |
	                   1UL<<10 /*mem increment*/ | 0b01<<6 /*mem->periph*/;


	// SPI1 @ APB2 = 60 MHz
	SPI1->CR1 = 1UL<<11 /*16-bit frame*/ | 1UL<<9 /*Software slave management*/ | 1UL<<8 /*SSI bit must be high*/ |
		0b010UL<<3 /*div 8 = 7.5 MHz*/ | 1UL<<2 /*Master*/;

	SPI1->CR2 = 1UL<<1 /* TX DMA enable */ | 1UL<<0 /* RX DMA enable*/;

	SPI1->CR1 |= 1UL<<6; // Enable SPI

	NVIC_SetPriority(DMA2_Stream0_IRQn, 0b0000); // Priority is the most urgent; keep the ISR short
	NVIC_EnableIRQ(DMA2_Stream0_IRQn);
	motcons_initialized = 1;
}

/*
	Every time motcon_fsm is called, the next controller is accessed.
	Maximum access frequency per controller is approx. 50 kHz (with 8*16 bits of data), so,
	200kHz access to motcon_fsm is allowable! In practice, polling at
	10 kHz (i.e., 2.5 kHz per motor) should give good response.
*/
void motcon_fsm()
{
	if(!motcons_initialized || (DMA2_Stream0->CR & 1) || (DMA2_Stream3->CR & 1))
	{
		motcon_errors++;
		return;
	}

	if(cur_motcon == NUM_MOTCONS-1)
		cur_motcon = 0;
	else
		cur_motcon++;

	switch(cur_motcon)
	{
		case 0: {MC1_CS0();} break;
		case 1: {MC2_CS0();} break;
		#if NUM_MOTCONS >= 4
		case 2: {MC3_CS0();} break;
		#endif
		#if NUM_MOTCONS >= 4
		case 3: {MC4_CS0();} break;
		#endif
		default: break;
	}

	DMA2_Stream0->M0AR = (uint32_t)&motcon_rx[cur_motcon];
	DMA2_Stream3->M0AR = (uint32_t)&motcon_tx[cur_motcon];

	DMA2->LIFCR = 0b111101UL<<0;  DMA2_Stream0->CR |= 1UL; // Enable RX DMA
	DMA2->LIFCR = 0b111101UL<<22; DMA2_Stream3->CR |= 1UL; // Enable TX DMA

}


/*
	Remote firmware update system:
*/

int mc_flasher_num;

void mc_flasher_cs1()
{
	switch(mc_flasher_num)
	{
		case 1: MC1_CS1(); break;
		case 2: MC2_CS1(); break;
		#if NUM_MOTCONS >= 3
		case 3: MC3_CS1(); break;
		#endif
		#if NUM_MOTCONS >= 4
		case 4: MC4_CS1(); break;
		#endif
		default: LED_ON(); while(1);
	}
}

void mc_flasher_cs0()
{
	switch(mc_flasher_num)
	{
		case 1: MC1_CS0(); break;
		case 2: MC2_CS0(); break;
		#if NUM_MOTCONS >= 3
		case 3: MC3_CS0(); break;
		#endif
		#if NUM_MOTCONS >= 4
		case 4: MC4_CS0(); break;
		#endif
		default: LED_ON(); while(1);
	}
}

// Blocks until free space in the SPI TX FIFO
void spi1_poll_tx(uint16_t d)
{
	while(!(SPI1->SR & (1UL<<1))) ;
	SPI1->DR = d;
}

// Blocks until SPI TX FIFO empty and not busy.

void spi1_poll_tx_bsy(uint16_t d)
{
	while(!(SPI1->SR & (1UL<<1))) ;
	SPI1->DR = d;
	while(SPI1->SR&(0b11<<11)); // Wait for TX fifo empty
	while(SPI1->SR&(1<<7)) ; // Wait until not busy.
	delay_us(1);
}

void spi1_poll_bsy()
{
	while(SPI1->SR&(0b11<<11)); // Wait for TX fifo empty
	while(SPI1->SR&(1<<7)) ; // Wait until not busy.
	delay_us(1);
}

// Blocks until data available in SPI RX FIFO - so indefinitely unless you have issued a TX just before.
uint16_t spi1_poll_rx()
{
	while(!(SPI1->SR & (1UL<<0))) ;
	return SPI1->DR;
}

// Empties the rx fifo
void spi1_empty_rx()
{
	while(SPI1->SR&(0b11<<9)) SPI1->DR;
}

void usart3_poll_tx(uint8_t d) // blocks until current byte tx finished
{
	while(!(USART3->SR & (1UL<<7))) ;
	USART3->DR = d;
}

uint8_t usart3_poll_rx() // blocks until byte received
{
	while(!(USART3->SR & (1UL<<5))) ;
	return USART3->DR;
}

#define ETERNAL_BLINK() {while(1) {LED_ON(); delay_ms(50); LED_OFF(); delay_ms(50);}}

void mc_flasher(int mcnum)
{
	__disable_irq();
	if(mcnum < 1 || mcnum > 4)
	{
		ETERNAL_BLINK();
	}

	mc_flasher_num = mcnum;
	LED_OFF();
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

	/*
		Reconfigure SPI1 for polling, no interrupts.
	*/

	while(SPI1->SR&(0b11<<11)); // Wait for TX fifo empty
	while(SPI1->SR&(1<<7)) ; // Wait until not busy.

	#if NUM_MOTCONS >= 4
	MC4_CS1();
	#endif
	#if NUM_MOTCONS >= 3
	MC3_CS1();
	#endif
	MC2_CS1();
	MC1_CS1();

	delay_us(10);

	// disable DMA
	DMA2->LIFCR = 0b111101UL<<0;  DMA2_Stream0->CR = 0;
	DMA2->LIFCR = 0b111101UL<<22; DMA2_Stream3->CR = 0;
	
	// SPI1 @ APB2 = 60 MHz
	SPI1->CR1 = 0; // Disable SPI
	delay_us(1);
	spi1_empty_rx();

	SPI1->CR1 = 1UL<<11 /*16-bit frame*/ | 1UL<<9 /*Software slave management*/ | 1UL<<8 /*SSI bit must be high*/ |
		0b010UL<<3 /*div 8 = 7.5 MHz*/ | 1UL<<2 /*Master*/;
	SPI1->CR2 = 0;

	SPI1->CR1 |= 1UL<<6; // Enable SPI

	mc_flasher_cs0();
	delay_us(1);

	// Special magic sequence to put the motor controller into the flasher:
	spi1_poll_tx(0xfaaa);
	spi1_poll_tx(0x1234);
	spi1_poll_tx(0xabcd);
	spi1_poll_tx(0x420b);
	spi1_poll_tx(0xacdc);
	spi1_poll_tx(0xabba);
	spi1_poll_tx(0x1337);
	spi1_poll_tx_bsy(0xacab);

	mc_flasher_cs1();

	// Now, the motor controller is reconfiguring its SPI, and we need to make sure we keep the lines idle
	// as per the reference manual requirements during the initialization.

	delay_ms(30); // was 20

	spi1_empty_rx();

	while(1)
	{
		int i;
		int size = 0;
		uint8_t cmd = usart3_poll_rx();
		switch(cmd)
		{
			case 100: // Erase
			{
			int num_pages = usart3_poll_rx();
			if(num_pages > 30 || num_pages < 1)
			{
				ETERNAL_BLINK();
			}
			spi1_empty_rx();
			mc_flasher_cs0();
			spi1_poll_tx((100<<8) | num_pages);

			while(spi1_poll_rx() != 0xaaaa)
				spi1_poll_tx(0x1111); // Generate dummy data so that we know when the erase is done.

			spi1_poll_bsy();
			mc_flasher_cs1();
			usart3_poll_tx(0); // success code
			}
			break;


			case 101: // Write
			size = usart3_poll_rx()<<8;
			size |= usart3_poll_rx();

			if(size < 50 || size>30*1024 || size&1)
			{
				ETERNAL_BLINK();
			}
			size>>=1;

			spi1_empty_rx();
			mc_flasher_cs0();
			spi1_poll_tx(101<<8);
			spi1_poll_tx(size);
			spi1_empty_rx();

			for(i=0; i<size; i++)
			{
				spi1_empty_rx();
				int word = usart3_poll_rx();
				word    |= usart3_poll_rx()<<8;
				spi1_poll_tx(word);
			}

			spi1_poll_bsy();
			mc_flasher_cs1();
			usart3_poll_tx(0); // success code
			break;


			case 102: // Read
			{
			size = usart3_poll_rx()<<8;
			size |= usart3_poll_rx();

			if(size < 50 || size>30*1024 || size&1)
			{
				ETERNAL_BLINK();
			}
			size>>=1;

			spi1_empty_rx();
			mc_flasher_cs0();
			spi1_poll_tx(102<<8);
			spi1_poll_tx(size);
			delay_us(100); // Give the slave some time to parse the cmd and prepare the first data.
			spi1_empty_rx();
			int mask = 1;
			i=0;
			while(1)
			{
				int word = spi1_poll_rx();
				
				if(!mask)
				{
					usart3_poll_tx(word&0xff);
					usart3_poll_tx((word&0xff00)>>8);
				}
				else
				{
					delay_us(200);
					i=0;
				}

				if(++i > size) break;
				spi1_poll_tx(0x1111); // Generate dummies.
				if(word == 0xcccc)
				{
					mask = 0;
				}
			}

			spi1_poll_bsy();
			mc_flasher_cs1();

			}
			break;

			case 150:
			case 151:
			// Reset motcon and the main cpu.
			mc_flasher_cs0();
			spi1_poll_tx(150<<8);
			delay_ms(1);
			spi1_poll_bsy();
			mc_flasher_cs1();
			NVIC_SystemReset();
			while(1);

			default:
			break;

		}
	}
}

