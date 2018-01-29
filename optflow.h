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

*/

#ifndef _OPTFLOW_H
#define _OPTFLOW_H

#include <stdint.h>

#define FLOW_CS1() {GPIOB->BSRR = 1UL<<12;}
#define FLOW_CS0() {GPIOB->BSRR = 1UL<<(12+16);}
#define OPTFLOW_RST_HI()  {GPIOD->BSRR = 1UL<<1;}
#define OPTFLOW_RST_LO() {GPIOD->BSRR = 1UL<<(1+16);}


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
