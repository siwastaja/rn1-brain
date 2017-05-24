/*
Reading the LIDAR is a bit tricky, because the start delimiter byte is not escaped and can reappear in the data.
The stream of data is rather continuous, and it's unreliable to rely to idle times.

Each packet is 22 bytes fixed.
Full revolution = 1980 bytes

*/

#include <stdint.h>
#include "ext_include/stm32f2xx.h"

#include "lidar.h"
#include "lidar_corr.h" // for point_t

extern int dbg[10];

extern void delay_us(uint32_t i);
extern void delay_ms(uint32_t i);


volatile int lidar_initialized;
volatile lidar_datum_t lidar_full_rev[90];
volatile int lidar_rpm_setpoint_x64 = (DEFAULT_LIDAR_RPM)*64;

void lidar_reset_flags() 
{
	DMA2->LIFCR = 0b111101UL<<16;
}
void lidar_reset_complete_flag() 
{
	DMA2->LIFCR = 0b100000UL<<16;
}
void lidar_reset_half_flag() 
{
	DMA2->LIFCR = 0b010000UL<<16;
}

int lidar_is_complete()
{
	return DMA2->LISR & (1UL<<21);
}

int lidar_is_half()
{
	return DMA2->LISR & (1UL<<20);
}

uint8_t lidar_ignore[360];

// Process the data so that datapoints either in the ignore list, or having the "error" flag set, are set as 0, and copy to a continuous int16_t table.
// Data is always positive and 14 bits long.
void copy_lidar_half1(int16_t* dst_start)
{
	int i;
	int o = 359;
	for(i = 0; i < 45; i++)
	{
		dst_start[o--] = (lidar_ignore[i*4+0] || (lidar_full_rev[i].d[0].flags_distance&(1<<15))) ? 0 : (lidar_full_rev[i].d[0].flags_distance&0x3fff);
		dst_start[o--] = (lidar_ignore[i*4+1] || (lidar_full_rev[i].d[1].flags_distance&(1<<15))) ? 0 : (lidar_full_rev[i].d[1].flags_distance&0x3fff);
		dst_start[o--] = (lidar_ignore[i*4+2] || (lidar_full_rev[i].d[2].flags_distance&(1<<15))) ? 0 : (lidar_full_rev[i].d[2].flags_distance&0x3fff);
		dst_start[o--] = (lidar_ignore[i*4+3] || (lidar_full_rev[i].d[3].flags_distance&(1<<15))) ? 0 : (lidar_full_rev[i].d[3].flags_distance&0x3fff);
	}
}
void copy_lidar_half2(int16_t* dst_start)
{
	int i;
	int o = 179;
	for(i = 45; i < 90; i++)
	{
		dst_start[o--] = (lidar_ignore[i*4+0] || (lidar_full_rev[i].d[0].flags_distance&(1<<15))) ? 0 : (lidar_full_rev[i].d[0].flags_distance&0x3fff);
		dst_start[o--] = (lidar_ignore[i*4+1] || (lidar_full_rev[i].d[1].flags_distance&(1<<15))) ? 0 : (lidar_full_rev[i].d[1].flags_distance&0x3fff);
		dst_start[o--] = (lidar_ignore[i*4+2] || (lidar_full_rev[i].d[2].flags_distance&(1<<15))) ? 0 : (lidar_full_rev[i].d[2].flags_distance&0x3fff);
		dst_start[o--] = (lidar_ignore[i*4+3] || (lidar_full_rev[i].d[3].flags_distance&(1<<15))) ? 0 : (lidar_full_rev[i].d[3].flags_distance&0x3fff);
	}
}
void copy_lidar_full(int16_t* dst_start)
{
	int i;
	int o = 359;
	for(i = 0; i < 90; i++)
	{
		dst_start[o--] = (lidar_ignore[i*4+0] || (lidar_full_rev[i].d[0].flags_distance&(1<<15))) ? 0 : (lidar_full_rev[i].d[0].flags_distance&0x3fff);
		dst_start[o--] = (lidar_ignore[i*4+1] || (lidar_full_rev[i].d[1].flags_distance&(1<<15))) ? 0 : (lidar_full_rev[i].d[1].flags_distance&0x3fff);
		dst_start[o--] = (lidar_ignore[i*4+2] || (lidar_full_rev[i].d[2].flags_distance&(1<<15))) ? 0 : (lidar_full_rev[i].d[2].flags_distance&0x3fff);
		dst_start[o--] = (lidar_ignore[i*4+3] || (lidar_full_rev[i].d[3].flags_distance&(1<<15))) ? 0 : (lidar_full_rev[i].d[3].flags_distance&0x3fff);
	}

}

int corr_on = 1;

void lidar_corr_on()
{
	corr_on = 1;
}
void lidar_corr_off()
{
	corr_on = 0;
}

extern live_lidar_scan_t* p_livelidar_store;
extern point_t* p_livelid2d_store;
extern int* p_livelidar_num_samples_store; // For counting validness of data for lidar-based correction.
/*
	Call lidar_fsm() at 1 kHz.

	As lidar produces full revolutions at 5Hz, packets are generated at 450Hz.
	Data is prepared and stored with location info (cur_pos from feedbacks.c) on each sample.
*/

void lidar_fsm()
{
	static int prev_cur_packet;
	int packets_left = DMA2_Stream2->NDTR/22;
	int cur_packet = 89-packets_left; if(cur_packet == -1) cur_packet = 0;

	if(cur_packet != prev_cur_packet)
	{
		int valid;
		int idx = prev_cur_packet; // We read from previous packet, since writing it is finished.
		int odx = 89-idx; // We write starting from the end to mirror the lidar image.
		int valid_tbl_odx = odx/15; // Validness table includes total counts of valid points divided in six 60 degree segments.
		COPY_POS(p_livelidar_store->pos[odx], cur_pos);

		valid = !((lidar_ignore[idx*4+0]) || (lidar_full_rev[idx].d[0].flags_distance&(1<<15)));
		p_livelid2d_store[odx*4+3].valid = valid;
		if(valid) p_livelidar_num_samples_store[valid_tbl_odx]++;

		valid = !((lidar_ignore[idx*4+1]) || (lidar_full_rev[idx].d[1].flags_distance&(1<<15)));
		p_livelid2d_store[odx*4+2].valid = valid;
		if(valid) p_livelidar_num_samples_store[valid_tbl_odx]++;

		valid = !((lidar_ignore[idx*4+2]) || (lidar_full_rev[idx].d[2].flags_distance&(1<<15)));
		p_livelid2d_store[odx*4+1].valid = valid;
		if(valid) p_livelidar_num_samples_store[valid_tbl_odx]++;

		valid = !((lidar_ignore[idx*4+3]) || (lidar_full_rev[idx].d[3].flags_distance&(1<<15)));
		p_livelid2d_store[odx*4+0].valid = valid;
		if(valid) p_livelidar_num_samples_store[valid_tbl_odx]++;

		p_livelidar_store->scan[odx*4+3] = lidar_full_rev[idx].d[0].flags_distance&0x3fff;
		p_livelidar_store->scan[odx*4+2] = lidar_full_rev[idx].d[1].flags_distance&0x3fff;
		p_livelidar_store->scan[odx*4+1] = lidar_full_rev[idx].d[2].flags_distance&0x3fff;
		p_livelidar_store->scan[odx*4+0] = lidar_full_rev[idx].d[3].flags_distance&0x3fff;

		if(prev_cur_packet == 81)
		{
			// Time is running out: tell lidar_corr that calculation must be finished ASAP, or terminated.
			// It won't be terminated right away; the termination condition is not checked too frequently as
			// it would slow down the process.
			live_lidar_calc_must_be_finished();
		}

		if(prev_cur_packet == 89)
		{	
			// We just got the full round.


			int skip = livelidar_skip();


			// Now processing the two previous lidar images is (must be) finished, and the correction
			// has already been applied to the latter one. We still need to apply the same correction
			// to this new image we just finished storing:

			if(!skip)
				apply_corr_to_livelidar(p_livelidar_store);

			// Now, correction has been applied to both images: the previous one, and the one just finished.
			// We can swap the buffers to start gathering the new image, and start processing the latest scan:
			livelidar_storage_finished();

			// One more thing, we need to apply the correction to the robot coordinates right here,
			// so that we get the new coords applied to the new lidar scan from the start:
			extern pos_t latest_corr; // from lidar_corr.c
			if(!skip && corr_on)
				correct_location_without_moving(latest_corr);
		}
	}
	prev_cur_packet = cur_packet;
}

void generate_lidar_ignore()
{
	int i;
	for(i = 0; i < 360; i++) lidar_ignore[i] = 0;

	for(i = 0; i < 90; i++)
	{
		int o;
		for(o = 0; o < 4; o++)
		{
			if(!(lidar_full_rev[i].d[o].flags_distance&(1<<15)))
			{
				if((int)(lidar_full_rev[i].d[o].flags_distance&0x3fff) < LIDAR_IGNORE_LEN)
				{
					int cur = i*4+o;
					int next = cur+1; if(next > 359) next = 0;
					int prev = cur-1; if(prev < 0) prev = 359;
					lidar_ignore[prev] = 1;
					lidar_ignore[cur] = 1;
					lidar_ignore[next] = 1;
				}
			}
		}
	}
}


void sync_lidar()
{
	if(!lidar_initialized) return;
	int i;
	int shift = 20;
	__disable_irq();
	int timeout = 100 * 1000000;
	while(1)
	{
		if(USART1->SR & (1UL<<5)) // data ready
		{
			int data = USART1->DR;
			if(data == 0xFA)
			{
				while(!(USART1->SR & (1UL<<5)))
				{
					if(!(--timeout)) goto LIDAR_SYNC_TIMEOUT;
				}
				data = USART1->DR;
				if(data == 0xA0+(90/4))
				{
					for(i=0; i < shift; i++)
					{
						while(!(USART1->SR & (1UL<<5)))
						{
							if(!(--timeout)) goto LIDAR_SYNC_TIMEOUT;
						}
						data = USART1->DR;
					}
					break;
				}

			}
		}
		if(!(--timeout)) goto LIDAR_SYNC_TIMEOUT;
	}
	DMA2_Stream2->CR = 4UL<<25 /*Channel*/ | 0b01UL<<16 /*med prio*/ | 0b00UL<<13 /*8-bit mem*/ | 0b00UL<<11 /*8-bit periph*/ |
	                   1UL<<10 /*mem increment*/ | 1UL<<8 /*circular*/;  // Disable
	USART1->SR = 0;
	USART1->CR3 = 1UL<<6 /*RX DMA*/;
	DMA2_Stream2->NDTR = 22*90;
	DMA2->LIFCR = 0xffffffff; // Clear all flags
	DMA2->HIFCR = 0xffffffff;
	DMA2_Stream2->CR |= 1UL; // Enable

	__enable_irq();
	return;

	LIDAR_SYNC_TIMEOUT:
	deinit_lidar();
	__enable_irq();
	return;
}


void resync_lidar()
{
	if(!lidar_initialized) return;
	// Disable DMA.
	DMA2_Stream2->CR = 4UL<<25 /*Channel*/ | 0b01UL<<16 /*med prio*/ | 0b00UL<<13 /*8-bit mem*/ | 0b00UL<<11 /*8-bit periph*/ |
	                   1UL<<10 /*mem increment*/ | 1UL<<8 /*circular*/;  // Disable
	while(DMA2_Stream2->CR & 1UL) ;
	USART1->CR3 &= ~(1UL<<6) /*disable RX DMA*/;
	sync_lidar();
}

// Requires little-endian CPU
uint16_t lidar_calc_checksum(volatile lidar_datum_t* l)
{
	int i;
	uint32_t chk32 = 0;

	for(i=0; i < 10; i++)
	{
		chk32 = (chk32<<1) + l->u16[i];
	}

	uint32_t checksum = (chk32 & 0x7FFF) + (chk32 >> 15);
	checksum &= 0x7FFF;
	return checksum;
}

volatile int lidar_speed_in_spec = 0;

// run this at 1 kHz
void lidar_motor_ctrl_loop()
{
	static int in_spec_cnt = 0;
	static int cycle = 0;
	static int pwm_shadow_x256 = 350*256;
	int i;
	int actual_speed = 0;

	if(cycle<10) // actual code runs at 100Hz
	{
		cycle++;
		return;
	}
	cycle = 0;

	if(!lidar_initialized)
	{
		TIM4->CCR4 = 0;
		return;
	}

	for(i=0; i<90; i++)
	{
		actual_speed += lidar_full_rev[i].speed;
	}
	actual_speed /= 90;

	int error = actual_speed - lidar_rpm_setpoint_x64;

	if(actual_speed > (MAX_LIDAR_RPM)*64 || actual_speed < (MIN_LIDAR_RPM)*64)
	{
		in_spec_cnt = 0;
		lidar_speed_in_spec = 0;
	}
	else
		in_spec_cnt++;

	if(in_spec_cnt > 100) // require 1 sec of stability
		lidar_speed_in_spec = 1;

	pwm_shadow_x256 -= error/16;
	if(pwm_shadow_x256 < 100*256) pwm_shadow_x256=100*256;
	else if(pwm_shadow_x256 > 700*256) pwm_shadow_x256=700*256;
	TIM4->CCR4 = pwm_shadow_x256>>8;
}

void init_lidar()
{
	/*
		TIM4 generates PWM control for the LIDAR brushed DC motor.
	*/

	// Set the IO to alternate function
	GPIOD->MODER &= ~(1UL<<30);
	GPIOD->MODER |= 1UL<<31;
	TIM4->CR1 = 1UL<<7 /*auto preload*/ | 0b01UL<<5 /*centermode*/;
	TIM4->CCMR2 = 1UL<<11 /*CH4 preload*/ | 0b110UL<<12 /*PWMmode1*/;
	TIM4->CCER = 1UL<<12 /*CH4 out ena*/;
	TIM4->ARR = 1024;
	TIM4->CCR4 = 350;
	TIM4->CR1 |= 1UL; // Enable.

	delay_ms(300); // let the motor spin up

	// USART1 (lidar) = APB2 = 60 MHz
	// 16x oversampling
	// 115200bps -> Baudrate register = 32.5625 = 32 9/16
	// USART1 RX is mapped to DMA2, Stream2, Ch4

	// Do not enable the DMA yet.

	DMA2_Stream2->PAR = (uint32_t)&(USART1->DR);
	DMA2_Stream2->M0AR = (uint32_t)(lidar_full_rev);

	USART1->BRR = 32UL<<4 | 9UL;
	USART1->CR1 = 1UL<<13 /*USART enable*/ | 1UL<<3 /*TX ena*/ | 1UL<<2 /*RX ena*/;

	delay_ms(100);

	lidar_initialized = 1;
}

void deinit_lidar()
{
	TIM4->CR1 = 0; // Disable the motor PWM
	// Set the motor fet drive IO as normal output
	GPIOD->MODER &= ~(1UL<<31);
	GPIOD->MODER |= 1UL<<30;
	GPIOC->BSRR = 1UL<<(15+16); // FET gate down

	USART1->CR1 = 0; // Disable uart
	DMA2_Stream2->CR = 0; // Disable DMA

	lidar_initialized = 0;
}
