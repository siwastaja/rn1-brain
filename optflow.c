#include <stdint.h>
#include "ext_include/stm32f2xx.h"

#include "optflow.h"

extern volatile optflow_data_t latest_optflow;
extern volatile int optflow_errors;

int optflow_ready;
int optflow_dummy_data;
void init_optflow()
{
	// SPI2 @ APB1 = 30 MHz
	// ADNS3080 optical flow sensor
	// Frame rate 2000..6469 fps
	// Time between write commands min 50 us
	// Between write and read 50 us
	// After read 250 us
	// Time after address: 50 us (for motion+motion burst: 75 us)
	// 500 ns min period -> 2 MHz max

	// DMA1 STREAM 3 ch0 = flow RX
	DMA1_Stream3->PAR = (uint32_t)&(SPI2->DR);
	DMA1_Stream3->M0AR = (uint32_t)(&latest_optflow);
	DMA1_Stream3->NDTR = 8;
	DMA1_Stream3->CR = 0UL<<25 /*Channel*/ | 0b01UL<<16 /*med prio*/ | 0b00UL<<13 /*8-bit mem*/ | 0b00UL<<11 /*8-bit periph*/ |
	                   1UL<<10 /*mem increment*/;

	// DMA1 STREAM 4 ch0 = flow TX (dummy bytes)
	DMA1_Stream4->PAR = (uint32_t)&(SPI2->DR);
	DMA1_Stream4->M0AR = (uint32_t)(&optflow_dummy_data);
	DMA1_Stream4->NDTR = 7;
	DMA1_Stream4->CR = 0UL<<25 /*Channel*/ | 0b01UL<<16 /*med prio*/ | 0b00UL<<13 /*8-bit mem*/ | 0b00UL<<11 /*8-bit periph*/ |
	                   0b01<<6 /*mem->periph*/;


	SPI2->CR1 = 0UL<<11 /*8-bit frame*/ | 1UL<<9 /*Software slave management*/ | 1UL<<8 /*SSI bit must be high*/ |
		0b011UL<<3 /*div 16 = 1.875 MHz*/ | 1UL<<2 /*Master*/;
	SPI2->CR2 = 1UL<<1 /* TX DMA enable */ | 1UL<<0 /* RX DMA enable*/;

	SPI2->CR1 |= 1UL<<6; // Enable SPI

	optflow_ready = 1;
}


// Run this at 10kHz!
void optflow_fsm(int* dx, int* dy)
{
	if(!optflow_ready) return;

	static int cycle = 0;
	if(cycle == 0)
	{
		*dx = latest_optflow.dx;
		*dy = latest_optflow.dy;
		FLOW_CS0();
	}
	else if(cycle == 1)
	{
		DMA1->LIFCR = 0b111101UL<<22; DMA1_Stream3->CR |= 1UL; // Enable RX DMA
		SPI2->DR = 0x50; // motion_burst read
	}
	else if(cycle == 2)
	{
		DMA1->HIFCR = 0b111101UL; DMA1_Stream4->CR |= 1UL; // Enable TX DMA to send dummy data
	}
	else if(cycle == 5)
	{
		FLOW_CS1();
		if(DMA1_Stream3->CR & 1UL) // Error: RX DMA stream still on (not finished)
		{
			optflow_errors++;
			cycle = -20;
			return;
		}
	}
	else if(cycle >= OPTFLOW_POLL_RATE)
	{
		cycle = 0;
		return;
	}

	cycle++;
}

