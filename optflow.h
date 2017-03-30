#ifndef _OPTFLOW_H
#define _OPTFLOW_H

#include <stdint.h>

#define FLOW_CS1() {GPIOB->BSRR = 1UL<<12;}
#define FLOW_CS0() {GPIOB->BSRR = 1UL<<(12+16);}

#define OPTFLOW_POLL_RATE 100 // unit: 100us, must be at least 5 (=500us)

typedef struct __attribute__ ((__packed__))
{
	uint8_t dummy;
	uint8_t motion;
	int8_t  dx;
	int8_t  dy;
	uint8_t squal;
	uint8_t shutter_msb;
	uint8_t shutter_lsb;
	uint8_t max_pixel;
} optflow_data_t;

void init_optflow();
void optflow_fsm();


#endif
