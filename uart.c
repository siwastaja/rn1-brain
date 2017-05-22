/*

All uart-related message management thingies.

*/

#include <stdint.h>
#include <string.h>

#include "ext_include/stm32f2xx.h"

#define TX_BUFFER_LEN 2048
uint8_t txbuf[TX_BUFFER_LEN];

void uart_print_string_blocking(const char *buf)
{
	while(buf[0] != 0)
	{
		while((USART3->SR & (1UL<<7)) == 0) ;
		USART3->DR = buf[0];
		buf++;
	}
}

void uart_send_blocking(const uint8_t *buf, int len)
{
	while(len--)
	{
		while((USART3->SR & (1UL<<7)) == 0) ;
		USART3->DR = buf[0];
		buf++;
	}
}

/*
	Incoming UART messages are double-buffered;
	handling needs to happen relatively fast so that reading the next command does not finish before the processing of the previous command.

	Minimum message length is 3 bytes, so this means that, at 115200 bps, processing has to happen within 260 us.
	Note that higher priority interrupts may take up time from the rx handler.
*/

#define RX_BUFFER_LEN 128
volatile uint8_t rx_buffers[2][RX_BUFFER_LEN];
int rx_buf_loc = 0;

volatile uint8_t* gather_rx_buf;
volatile uint8_t* process_rx_buf;

volatile int do_handle_message;

static void handle_maintenance_msg()
{
	switch(process_rx_buf[4])
	{
		case 0x52:
		run_flasher();
		break;

		case 0x53:
		mc_flasher(process_rx_buf[5]);
		break;

		default: break;
	}
}

void handle_uart_message()
{
	if(!do_handle_message)
		return;

	switch(process_rx_buf[0])
	{
		case 0xfe:
		if(process_rx_buf[1] == 0x42 && process_rx_buf[2] == 0x11 && process_rx_buf[3] == 0x7a)
		{
			handle_maintenance_msg();
		}
		break;

		case 0x80:
		host_alive();
		move_arc_manual(((int16_t)(int8_t)(process_rx_buf[1]<<1)), ((int16_t)(int8_t)(process_rx_buf[2]<<1)));
		break;

		case 0x81:
		move_rel_twostep(I7I7_I16_lossy(process_rx_buf[1],process_rx_buf[2]), I7I7_I16_lossy(process_rx_buf[3],process_rx_buf[4]));
		break;

		case 0x82:
		move_xy_abs(I7x5_I32(process_rx_buf[1],process_rx_buf[2],process_rx_buf[3],process_rx_buf[4],process_rx_buf[5]),
		            I7x5_I32(process_rx_buf[6],process_rx_buf[7],process_rx_buf[8],process_rx_buf[9],process_rx_buf[10]), 1 /*auto decide reverse*/);
		break;

		case 0x8f:
		host_alive();
		break;

		case 0x91:
		do_re_compass = 1;
		break;

		case 0x92:
		sync_to_compass();
		break;

		// debug/dev messages
		case 0xd1:
		zero_angle();
		break;

		case 0xd2:
		zero_coords();
		break;

		case 0xd3:
		break;

		case 0xd4:
		break;

		default:
		break;
	}
	do_handle_message = 0;
}

void uart_rx_handler()
{
	// This SR-then-DR read sequence clears error flags:
	/*uint32_t flags = */USART3->SR;
	uint8_t byte = USART3->DR;

// TODO:
//	if(flags & 0b1011)
//	{
//		// At error, drop the packet.
//	}

	if(byte == 255) // End-of-command delimiter
	{
		volatile uint8_t* tmp = gather_rx_buf;
		gather_rx_buf = process_rx_buf;
		process_rx_buf = tmp;
		do_handle_message = 1;
		rx_buf_loc = 0;
	}
	else
	{
		if(byte > 127) // Start of command delimiter
			rx_buf_loc = 0;

		gather_rx_buf[rx_buf_loc] = byte;
		rx_buf_loc++;
		if(rx_buf_loc >= RX_BUFFER_LEN)  // TODO: this is kind of an error condition, handle it better.
			rx_buf_loc = 0;
	}
}


void send_gyro()
{
	msg_gyro_t msg;
	msg.status = 1;
	msg.int_x = I16_I14(latest_gyro->x);
	msg.int_y = I16_I14(latest_gyro->y);
	msg.int_z = I16_I14(latest_gyro->z);
	txbuf[0] = 128;
	memcpy(txbuf+1, &msg, sizeof(msg_gyro_t));
	usart_send(txbuf, sizeof(msg_gyro_t)+1);
}

/*
	Figures out what to send.

	Call this in the main thread during free time.

	TX itself is interrupt-driven.
*/

void uart_send_fsm()
{
	static int send_count = 0;

	send_count++;

	switch(send_count)
	{
		case 0:
		send_gyro();

		break;

		default:
		send_count = 0;
		break;
	}

}

