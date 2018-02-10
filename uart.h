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

#ifndef UART_H
#define UART_H

#include <stdint.h>

#define TX_BUFFER_LEN 2048
extern uint8_t txbuf[TX_BUFFER_LEN];

// busy-loop print zero-terminated C string.
void uart_print_string_blocking(const char *buf);

// busy-loop send binary uint8 buffer
void uart_send_blocking(const uint8_t *buf, int len);

// Handle a message, if there is any. Function returns quickly if there is no message.
// You can call this at 1 kHz.
void handle_uart_message();

// The uart ISR, should be of high priority to data register overruns (there is no DMA for UART in the lousy STM32!)
void uart_rx_handler();

int send_uart(void* buf, uint8_t header, int len);
int send_uart_volatile(volatile void* buf, uint8_t header, int len);

void uart_10k_fsm();

void uart_send_fsm();
void uart_send_critical1();
void uart_send_critical2();

int uart_busy();

void init_uart();




#endif
