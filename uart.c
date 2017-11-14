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

extern void delay_ms(uint32_t i);
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

		case 0x54:
		host_dead();
		DO_KILL_PWR();
		delay_ms(100);
		
		case 0x55:
		host_dead();
		NVIC_SystemReset();
		while(1);

		default: break;
	}
}

volatile int do_compass_round = 1;
extern int accurate_turngo;

int ignore_cmds = 0;

void handle_uart_message()
{
	if(!do_handle_message)
		return;

	if(ignore_cmds && process_rx_buf[0] != 0xfe)
	{
		return;
	}

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
		dis_coll_avoid();
		accurate_turngo = process_rx_buf[6];
		move_rel_twostep(((int32_t)I7I7_I16_lossy(process_rx_buf[1],process_rx_buf[2]))<<16, I7I7_I16_lossy(process_rx_buf[3],process_rx_buf[4]), process_rx_buf[5]);
		break;

		case 0x82:
		host_alive();
		ena_coll_avoid();
		accurate_turngo = process_rx_buf[14];
		move_xy_abs(I7x5_I32(process_rx_buf[1],process_rx_buf[2],process_rx_buf[3],process_rx_buf[4],process_rx_buf[5]),
		            I7x5_I32(process_rx_buf[6],process_rx_buf[7],process_rx_buf[8],process_rx_buf[9],process_rx_buf[10]),
		            process_rx_buf[11], process_rx_buf[12], process_rx_buf[13]);
		break;

		case 0x83:
		host_alive();
		dis_coll_avoid();
		accurate_turngo = process_rx_buf[6];
		move_absa_rels_twostep(((int32_t)I7I7_I16_lossy(process_rx_buf[1],process_rx_buf[2]))<<16, I7I7_I16_lossy(process_rx_buf[3],process_rx_buf[4]), process_rx_buf[5]);
		break;

		case 0x84:
		host_alive();
		stop_movement();
		break;

		case 0x85:
		limit_speed(process_rx_buf[1]);
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
			set_lidar_id(process_rx_buf[7]);
		}
		break;

		case 0x8a:
		{
			pos_t new_pos;
			new_pos.ang = ((uint32_t)(I7I7_I16_lossy(process_rx_buf[1],process_rx_buf[2])))<<16;
			new_pos.x = I7x5_I32(process_rx_buf[3],process_rx_buf[4],process_rx_buf[5],process_rx_buf[6],process_rx_buf[7]);
			new_pos.y = I7x5_I32(process_rx_buf[8],process_rx_buf[9],process_rx_buf[10],process_rx_buf[11],process_rx_buf[12]);
			set_location_without_moving_external(new_pos);
			//reset_livelidar_images(-1);
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
		lidar_near_filter_on = 1;
		break;

		case 0xd2:
		lidar_near_filter_on = 0;
		break;

		case 0xd3:
		lidar_midlier_filter_on = 1;
		break;

		case 0xd4:
		lidar_midlier_filter_on = 0;
		break;

		case 0xd5:
		break;

		case 0xd6:
		break;

		default:
		break;
	}
	do_handle_message = 0;
}

void uart_rx_handler()
{
	// This SR-then-DR read sequence clears error flags:
	uint32_t flags = USART3->SR;
	uint8_t byte = USART3->DR;

// TODO:
	if(flags & 0b1011)
	{
		// At error, drop the packet.
	}

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

volatile int uart_sending;
int uart_tx_loc;
int uart_tx_len;
uint8_t uart_tx_checksum_accum;
uint8_t uart_tx_header;
void* p_txbuf;

#define CRC_INITIAL_REMAINDER 0x00
#define CRC_POLYNOMIAL 0x07 // As per CRC-8-CCITT

#define CALC_CRC(remainder) \
	for(int crc__bit = 8; crc__bit > 0; --crc__bit) \
	{ \
		if((remainder) & 0b10000000) \
		{ \
			(remainder) = ((remainder) << 1) ^ CRC_POLYNOMIAL; \
		} \
		else \
		{ \
			(remainder) = ((remainder) << 1); \
		} \
	}

void uart_10k_fsm()
{
	if(uart_sending && (USART3->SR & (1UL<<7)) /*uart tx free*/)
	{
		if(uart_tx_loc == -3)
		{
			USART3->DR = uart_tx_header;
			uart_tx_loc++;
		}
		else if(uart_tx_loc == -2)
		{
			USART3->DR = uart_tx_len & 0xff;
			uart_tx_loc++;
		}
		else if(uart_tx_loc == -1)
		{
			USART3->DR = (uart_tx_len & 0xff00)>>8;
			uart_tx_loc++;
		}
		else if(uart_tx_loc == uart_tx_len)
		{
			USART3->DR = uart_tx_checksum_accum;
			uart_sending = 0;
		}
		else
		{
			uint8_t byte = ((char*)p_txbuf)[uart_tx_loc];
			USART3->DR = byte;
			uart_tx_checksum_accum ^= byte;
			CALC_CRC(uart_tx_checksum_accum);
			uart_tx_loc++;
		}
	}
}

int send_uart(void* buf, uint8_t header, int len)
{
	if(uart_sending)
		return -1;

	__disable_irq();
	p_txbuf = buf;
	uart_tx_header = header;
	uart_tx_loc = -3;
	uart_tx_len = len;
	uart_tx_checksum_accum = CRC_INITIAL_REMAINDER;
	uart_sending = 1;
	__enable_irq();
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
		case 4:
		{
			int bat_v = get_bat_v();
			int bat_percentage = (100*(bat_v-15500))/(21000-15500);
			if(bat_percentage < 0) bat_percentage = 0;
			if(bat_percentage > 127) bat_percentage = 127;

			txbuf[0] = ((CHA_RUNNING())?1:0) | ((CHA_FINISHED())?2:0);
			txbuf[1] = I16_MS(bat_v);
			txbuf[2] = I16_LS(bat_v);
			txbuf[3] = bat_percentage;
			send_uart(txbuf, 0xa2, 4);
		}
		break;

		case 8:
		{
			for(int i=0; i<10; i++)
			{
				int tm = dbg[i];
				txbuf[5*i+0] = I32_I7_4(tm);
				txbuf[5*i+1] = I32_I7_3(tm);
				txbuf[5*i+2] = I32_I7_2(tm);
				txbuf[5*i+3] = I32_I7_1(tm);
				txbuf[5*i+4] = I32_I7_0(tm);
			}
			send_uart(txbuf, 0xd2, 50);
		}
		break;

		case 12:
		{
			extern int cur_compass_angle;
			extern volatile int compass_round_on;
			txbuf[0] = compass_round_on;
			int tm = cur_compass_angle>>16;
			txbuf[1] = I16_MS(tm);
			txbuf[2] = I16_LS(tm);
			send_uart(txbuf, 0xa3, 3);
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
			txbuf[0] = 1;
			txbuf[1] = get_xy_id();
			int tm = get_xy_left();
			if(tm < 0) tm*=-1;
			else if(tm > 30000) tm = 30000;
			txbuf[2] = I16_MS(tm);
			txbuf[3] = I16_LS(tm);

			uint32_t t = get_obstacle_avoidance_stop_flags();

			txbuf[4] = I32_I7_4(t);
			txbuf[5] = I32_I7_3(t);
			txbuf[6] = I32_I7_2(t);
			txbuf[7] = I32_I7_1(t);
			txbuf[8] = I32_I7_0(t);

			t = get_obstacle_avoidance_action_flags();

			txbuf[9] = I32_I7_4(t);
			txbuf[10] = I32_I7_3(t);
			txbuf[11] = I32_I7_2(t);
			txbuf[12] = I32_I7_1(t);
			txbuf[13] = I32_I7_0(t);

			extern uint8_t feedback_stop_flags;
			extern int feedback_stop_param1, feedback_stop_param2;
			txbuf[14] = feedback_stop_flags&0x7f;
			tm = feedback_stop_param1;
			txbuf[15] = I16_MS(tm);
			txbuf[16] = I16_LS(tm);
			tm = feedback_stop_param2;
			txbuf[17] = I16_MS(tm);
			txbuf[18] = I16_LS(tm);

			send_uart(txbuf, 0xa5, 19);
		}
		break;

		case 0:
		case 2:
		case 6:
		case 10:
		{
			txbuf[0] = 1;
			__disable_irq();
			int tm = cur_pos.ang;
			__enable_irq();
			tm>>=16;
			txbuf[1] = I16_MS(tm);
			txbuf[2] = I16_LS(tm);
			__disable_irq();
			tm = cur_pos.x;
			__enable_irq();
			if(tm < -1000000 || tm > 1000000) tm = 123456789;
			txbuf[3] = I32_I7_4(tm);
			txbuf[4] = I32_I7_3(tm);
			txbuf[5] = I32_I7_2(tm);
			txbuf[6] = I32_I7_1(tm);
			txbuf[7] = I32_I7_0(tm);
			__disable_irq();
			tm = cur_pos.y;
			__enable_irq();
			if(tm < -1000000 || tm > 1000000) tm = 123456789;
			txbuf[8] = I32_I7_4(tm);
			txbuf[9] = I32_I7_3(tm);
			txbuf[10] = I32_I7_2(tm);
			txbuf[11] = I32_I7_1(tm);
			txbuf[12] = I32_I7_0(tm);

			send_uart(txbuf, 0xa0, 13);
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
