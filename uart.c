/*

All uart-related message management thingies.

*/

#include <stdint.h>
#include <string.h>

#include "ext_include/stm32f2xx.h"

#include "main.h"
#include "feedbacks.h"
#include "navig.h"
#include "comm.h"
#include "gyro_xcel_compass.h"
#include "optflow.h"
#include "uart.h"
#include "sonar.h"

uint8_t txbuf[TX_BUFFER_LEN];

extern int dbg[10];

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
		host_dead();
		run_flasher();
		break;

		case 0x53:
		host_dead();
		mc_flasher(process_rx_buf[5]);
		break;

		default: break;
	}
}

volatile int do_compass_round;

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
		host_alive();
		move_rel_twostep(I7I7_I16_lossy(process_rx_buf[1],process_rx_buf[2]), I7I7_I16_lossy(process_rx_buf[3],process_rx_buf[4]));
		break;

		case 0x82:
		host_alive();
		move_xy_abs(I7x5_I32(process_rx_buf[1],process_rx_buf[2],process_rx_buf[3],process_rx_buf[4],process_rx_buf[5]),
		            I7x5_I32(process_rx_buf[6],process_rx_buf[7],process_rx_buf[8],process_rx_buf[9],process_rx_buf[10]),
		            process_rx_buf[11], process_rx_buf[12], process_rx_buf[13]);
		break;


		case 0x86:
		if(process_rx_buf[1] == 5)
			daiju_mode_on();
		else
			daiju_mode_off();

		break;

		case 0x87:
		find_charger();

		break;


		case 0x88:
		set_obstacle_avoidance_margin(process_rx_buf[1]);

		break;

		case 0x89:
		{
			pos_t corr;
			corr.ang = ((uint32_t)(I7I7_I16_lossy(process_rx_buf[1],process_rx_buf[2])))<<16;
			corr.x = I7I7_I16_lossy(process_rx_buf[3],process_rx_buf[4])>>2;
			corr.y = I7I7_I16_lossy(process_rx_buf[5],process_rx_buf[6])>>2;
			correct_location_without_moving_external(corr);
		}
		break;

		case 0x8a:
		{
			pos_t new_pos;
			new_pos.ang = ((uint32_t)(I7I7_I16_lossy(process_rx_buf[1],process_rx_buf[2])))<<16;
			new_pos.x = I7x5_I32(process_rx_buf[3],process_rx_buf[4],process_rx_buf[5],process_rx_buf[6],process_rx_buf[7]);
			new_pos.y = I7x5_I32(process_rx_buf[8],process_rx_buf[9],process_rx_buf[10],process_rx_buf[11],process_rx_buf[12]);
			set_location_without_moving_external(new_pos);
		}
		break;


		break;

		case 0x8f:
		if(process_rx_buf[1] == 42)
			host_alive();
		else
			host_dead();
		break;

		case 0x91:
		do_compass_round = 1;
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

int uart_sending;
int uart_tx_loc;
int uart_tx_len;

void uart_10k_fsm()
{
	if(uart_sending && (USART3->SR & (1UL<<7)))
	{
		USART3->DR = txbuf[uart_tx_loc++];
		if(uart_tx_loc > uart_tx_len)
		{
			uart_sending = 0;
		}
	}
}

int send_uart(int len)
{
	if(uart_sending)
		return -1;

	uart_tx_loc = 0;
	uart_tx_len = len;
	uart_sending = 1;
	return 0;
}

int uart_busy()
{
	return uart_sending;
}


/*
	Figures out what to send, and sends it if the uart is free.

	Call this in the main thread during free time.

	TX itself is interrupt-driven.
*/

/*
		msg_xcel_t msgx;
		msgx.status = 1;
		msgx.int_x = I16_I14(latest_xcel->x);
		msgx.int_y = I16_I14(latest_xcel->y);
		msgx.int_z = I16_I14(latest_xcel->z);
		txbuf[0] = 129;
		memcpy(txbuf+1, &msgx, sizeof(msg_xcel_t));
		usart_send(txbuf, sizeof(msg_xcel_t)+1);

		msg_compass_t msgc;
		msgc.status = 1;
		msgc.x = I16_I14(latest_compass->x);
		msgc.y = I16_I14(latest_compass->y);
		msgc.z = I16_I14(latest_compass->z);
		txbuf[0] = 0x82;
		memcpy(txbuf+1, &msgc, sizeof(msg_compass_t));
		usart_send(txbuf, sizeof(msg_compass_t)+1);
*/

void uart_send_fsm()
{
	static int send_count = 0;

	if(uart_busy())
		return;

	send_count++;

	switch(send_count)
	{
		case 0:
		{
			msg_gyro_t msg;
			msg.status = 1;
			msg.int_x = I16_I14(latest_gyro->x);
			msg.int_y = I16_I14(latest_gyro->y);
			msg.int_z = I16_I14(latest_gyro->z);
			txbuf[0] = 128;
			memcpy(txbuf+1, &msg, sizeof(msg_gyro_t));
			send_uart(sizeof(msg_gyro_t)+1);
		}
		break;

		case 2:
		{
			txbuf[0] = 0xa1;
			txbuf[1] = 1;
			txbuf[2] = I16_MS(optflow_int_x);
			txbuf[3] = I16_LS(optflow_int_x);
			txbuf[4] = I16_MS(optflow_int_y);
			txbuf[5] = I16_LS(optflow_int_y);
			txbuf[6] = latest_optflow.squal>>1;
			txbuf[7] = latest_optflow.dx&0x7f;
			txbuf[8] = latest_optflow.dy&0x7f;
			txbuf[9] = latest_optflow.max_pixel>>1;
			txbuf[10] = latest_optflow.dummy>>1;
			txbuf[11] = latest_optflow.motion>>1;
			send_uart(12);
		}
		break;

		case 4:
		{
			int bat_v = get_bat_v();
			int bat_percentage = (100*(bat_v-15500))/(21000-15500);
			if(bat_percentage < 0) bat_percentage = 0;
			if(bat_percentage > 127) bat_percentage = 127;

			txbuf[0] = 0xa2;
			txbuf[1] = ((CHA_RUNNING())?1:0) | ((CHA_FINISHED())?2:0);
			txbuf[2] = I16_MS(bat_v);
			txbuf[3] = I16_LS(bat_v);
			txbuf[4] = bat_percentage;
			send_uart(5);
		}
		break;

		case 6:
		{
			point_t sons[NUM_SONARS];
			pos_t rpos;
			get_sonars(sons, &rpos);

			txbuf[0] = 0x85;
			txbuf[1] = (sons[2].valid<<2) | (sons[1].valid<<1) | (sons[0].valid);

			int tm = rpos.x;
			txbuf[2] = I32_I7_4(tm);
			txbuf[3] = I32_I7_3(tm);
			txbuf[4] = I32_I7_2(tm);
			txbuf[5] = I32_I7_1(tm);
			txbuf[6] = I32_I7_0(tm);
			tm = rpos.y;
			txbuf[7] = I32_I7_4(tm);
			txbuf[8] = I32_I7_3(tm);
			txbuf[9] = I32_I7_2(tm);
			txbuf[10] = I32_I7_1(tm);
			txbuf[11] = I32_I7_0(tm);

			for(int i=0; i<3; i++)
			{
				tm = sons[i].x;
				txbuf[10*i+12] = I32_I7_4(tm);
				txbuf[10*i+13] = I32_I7_3(tm);
				txbuf[10*i+14] = I32_I7_2(tm);
				txbuf[10*i+15] = I32_I7_1(tm);
				txbuf[10*i+16] = I32_I7_0(tm);
				tm = sons[i].y;
				txbuf[10*i+17] = I32_I7_4(tm);
				txbuf[10*i+18] = I32_I7_3(tm);
				txbuf[10*i+19] = I32_I7_2(tm);
				txbuf[10*i+20] = I32_I7_1(tm);
				txbuf[10*i+21] = I32_I7_0(tm);
			}

			send_uart(42);

		}
		break;

		case 8:
		{
			txbuf[0] = 0xd2;
			for(int i=0; i<10; i++)
			{
				int tm = dbg[i];
				txbuf[5*i+1] = I32_I7_4(tm);
				txbuf[5*i+2] = I32_I7_3(tm);
				txbuf[5*i+3] = I32_I7_2(tm);
				txbuf[5*i+4] = I32_I7_1(tm);
				txbuf[5*i+5] = I32_I7_0(tm);
			}
			send_uart(51);
		}
		break;

		case 10:
		{
			txbuf[0] = 0xa0;
			txbuf[1] = 1;
			int tm = cur_pos.ang>>16;
			txbuf[2] = I16_MS(tm);
			txbuf[3] = I16_LS(tm);
			txbuf[4] = 0;
			txbuf[5] = 0;
			txbuf[6] = 0;
			txbuf[7] = 0;
			tm = cur_pos.x;
			txbuf[8] = I32_I7_4(tm);
			txbuf[9] = I32_I7_3(tm);
			txbuf[10] = I32_I7_2(tm);
			txbuf[11] = I32_I7_1(tm);
			txbuf[12] = I32_I7_0(tm);
			tm = cur_pos.y;
			txbuf[13] = I32_I7_4(tm);
			txbuf[14] = I32_I7_3(tm);
			txbuf[15] = I32_I7_2(tm);
			txbuf[16] = I32_I7_1(tm);
			txbuf[17] = I32_I7_0(tm);

			send_uart(18);
		}
		break;

		case 12:
		{
			extern int cur_compass_angle;
			txbuf[0] = 0xa3;
			extern volatile int compass_round_on;
			txbuf[1] = compass_round_on;
			int tm = cur_compass_angle>>16;
			txbuf[2] = I16_MS(tm);
			txbuf[3] = I16_LS(tm);
			send_uart(4);
		}
		break;

		case 1:
		case 3:
		case 5:
		case 7:
		case 9:
		case 11:
		case 13:
		{
			txbuf[0] = 0xa5;
			txbuf[1] = 1;
			txbuf[2] = get_xy_id();
			int tm = get_xy_left();
			if(tm < 0) tm*=-1;
			else if(tm > 30000) tm = 30000;
			txbuf[3] = I16_MS(tm);
			txbuf[4] = I16_LS(tm);

			uint32_t t = get_obstacle_avoidance_stop_flags();

			txbuf[5] = I32_I7_4(t);
			txbuf[6] = I32_I7_3(t);
			txbuf[7] = I32_I7_2(t);
			txbuf[8] = I32_I7_1(t);
			txbuf[9] = I32_I7_0(t);

			t = get_obstacle_avoidance_action_flags();

			txbuf[10] = I32_I7_4(t);
			txbuf[11] = I32_I7_3(t);
			txbuf[12] = I32_I7_2(t);
			txbuf[13] = I32_I7_1(t);
			txbuf[14] = I32_I7_0(t);

			extern uint8_t feedback_stop_flags;
			extern int feedback_stop_param1, feedback_stop_param2;
			txbuf[15] = feedback_stop_flags&0x7f;
			tm = feedback_stop_param1;
			txbuf[16] = I16_MS(tm);
			txbuf[17] = I16_LS(tm);
			tm = feedback_stop_param2;
			txbuf[18] = I16_MS(tm);
			txbuf[19] = I16_LS(tm);

			send_uart(20);

		}
		break;		

		default:
		send_count = 0;
		break;
	}

}

void init_uart()
{
	gather_rx_buf = rx_buffers[0];
	process_rx_buf = rx_buffers[1];
}
