#include <stdint.h>
#include "ext_include/stm32f2xx.h"

#include "motcons.h"

//#define MOTCON_DEBUG

extern volatile motcon_t motcons[NUM_MOTCONS];

volatile int cur_motcon;
volatile int motcons_initialized;

volatile int mc_dbg_rx, mc_dbg_tx;

void spi1_inthandler()
{
#ifdef MOTCON_DEBUG
	MC4_CS1();
	mc_dbg_rx = SPI1->DR;
#else
	// Receive done, slave can be de-selected.
	switch(cur_motcon)
	{
		case 0: {MC1_CS1();} break;
		case 1: {MC2_CS1();} break;
		case 2: {MC3_CS1();} break;
		case 3: {MC4_CS1();} break;
		default: break;
	}
	uint16_t msg = SPI1->DR;

	motcons[cur_motcon].status.last_msg = msg;

	if(cur_motcon == NUM_MOTCONS-1)
		cur_motcon = 0;
	else
		cur_motcon++;
#endif
}

void init_motcons()
{
	MC4_CS1();
	MC3_CS1();
	MC2_CS1();
	MC1_CS1();


	// SPI1 @ APB2 = 60 MHz
	SPI1->CR1 = 1UL<<11 /*16-bit frame*/ | 1UL<<9 /*Software slave management*/ | 1UL<<8 /*SSI bit must be high*/ |
		0b010UL<<3 /*div 8 = 7.5 MHz*/ | 1UL<<2 /*Master*/;

	SPI1->CR2 = 1UL<<6 /*RX not empty interrupt*/;

	SPI1->CR1 |= 1UL<<6; // Enable SPI

	NVIC_SetPriority(SPI1_IRQn, 0b0101);
	NVIC_EnableIRQ(SPI1_IRQn);

	motcons_initialized = 1;
}


/*
	Every time motcon_fsm is called, the next controller is accessed.
	Maximum access frequency per controller is approx. 300 kHz, so,
	1.2MHz access to motcon_fsm is allowable! In practice, polling at
	10 kHz (i.e., 2.5 kHz per motor) should give good response.
*/
void motcon_fsm()
{
	if(!motcons_initialized)
		return;

#ifdef MOTCON_DEBUG
	MC4_CS0();
	SPI1->DR = mc_dbg_tx;
//	mc_dbg_tx = 0;
#else
	switch(cur_motcon)
	{
		case 0: {MC1_CS0();} break;
		case 1: {MC2_CS0();} break;
		case 2: {MC3_CS0();} break;
		case 3: {MC4_CS0();} break;
		default: break;
	}

	if(motcons[cur_motcon].cmd.speed >= 0)
		SPI1->DR = 11UL<<10 | (motcons[cur_motcon].cmd.speed & 0x3FF);
	else
		SPI1->DR = 12UL<<10 | ((motcons[cur_motcon].cmd.speed*-1) & 0x3FF);

#endif
}
