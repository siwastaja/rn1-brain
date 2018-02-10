/*
	PULUROBOT RN1-BRAIN RobotBoard main microcontroller firmware project

	(c) 2017-2018 Pulu Robotics and other contributors
	Maintainer: Antti Alhonen <antti.alhonen@iki.fi>

	This program is free software; you can redistribute it and/or modify
	it under the terms of the GNU General Public License version 2, as 
	published by the Free Software Foundation.

	This program is distributed in the hope that it will be useful,
	but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
	GNU General Public License for more details.

	GNU General Public License version 2 is supplied in file LICENSING.

*/

#ifndef _MOTCONS_H
#define _MOTCONS_H

#ifdef PCB1A

#define MC4_CS1()  {GPIOE->BSRR = 1UL<<6;}
#define MC4_CS0() {GPIOE->BSRR = 1UL<<(6+16);}
#define MC3_CS1()  {GPIOA->BSRR = 1UL<<4;}
#define MC3_CS0() {GPIOA->BSRR = 1UL<<(4+16);}
#define MC2_CS1()  {GPIOC->BSRR = 1UL<<4;}
#define MC2_CS0() {GPIOC->BSRR = 1UL<<(4+16);}
#define MC1_CS1()  {GPIOC->BSRR = 1UL<<5;}
#define MC1_CS0() {GPIOC->BSRR = 1UL<<(5+16);}

#define NUM_MOTCONS 4

#endif


#ifdef PCB1B

#define MC2_CS1()  {GPIOC->BSRR = 1UL<<5;}
#define MC2_CS0() {GPIOC->BSRR = 1UL<<(5+16);}
#define MC1_CS1()  {GPIOC->BSRR = 1UL<<4;}
#define MC1_CS0() {GPIOC->BSRR = 1UL<<(4+16);}

#define NUM_MOTCONS 2


#endif



#define MOTCON_DATAGRAM_LEN 8
typedef struct __attribute__ ((packed))
{
	uint16_t status;
	int16_t speed;
	int16_t current;
	int16_t pos;
	uint8_t cur_limit_mul;
	uint8_t num_hard_limits;
	int16_t res5;
	int16_t res6;
	uint16_t crc;
} motcon_rx_t;

typedef struct  __attribute__ ((packed))
{
	uint16_t state;
	int16_t speed;
	int16_t cur_limit;
	uint16_t res3;
	uint16_t res4;
	uint16_t res5;
	uint16_t res6;
	uint16_t crc;
} motcon_tx_t;

extern volatile motcon_rx_t motcon_rx[4];
extern volatile motcon_tx_t motcon_tx[4];


void init_motcons();
void motcon_fsm();

#endif
