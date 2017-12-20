#include <stdint.h>
#include "ext_include/stm32f2xx.h"

#include "motcons.h"

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
/*	for(int i = 0; i < 4; i++)
	{
		motcon_tx[i].state = 1;
		motcon_tx[i].speed = 2;
		motcon_tx[i].cur_limit = 3;
		motcon_tx[i].res3 = 4;
		motcon_tx[i].res4 = 5;
		motcon_tx[i].res5 = 6;
		motcon_tx[i].res6 = 7;
		motcon_tx[i].magic = 8;
	}
*/
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
