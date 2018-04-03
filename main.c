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


	This module provides:
	- main(), which does timing-insensitive, polled things.
	- 10kHz timebase interrupt handler which basically calls everything else
	- some helper functions

*/

#if (!RN1P4 && !PULU1 && !RN1P6 && !RN1P7 && !PROD1)
#error "Unsupported robot model"
#endif

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <math.h>

#include "main.h"
#include "ext_include/stm32f2xx.h"
#include "own_std.h"

#include "gyro_xcel_compass.h"
#include "optflow.h"
#include "lidar.h"
#include "motcons.h"
#include "flash.h"
#include "comm.h"
#include "sonar.h"
#include "feedbacks.h"
#include "navig.h"
#include "lidar_corr.h"
#include "uart.h"

#include "settings.h"

volatile int dbg[10];

void delay_us(uint32_t i)
{
	if(i==0) return;
	i *= 25;
	i -= 7;
	while(i--)
		__asm__ __volatile__ ("nop");
}

void delay_ms(uint32_t i)
{
	while(i--)
	{
		delay_us(1000);
	}
}

void error(int code)
{
	__disable_irq();
	int i = 0;
	while(1)
	{
		LED_ON();
		delay_ms(200);
		LED_OFF();
		delay_ms(200);
		i++;
		if(i == code)
		{
			delay_ms(800);
			i = 0;
		}
	}
}

/*
If you ever need this, here's a stub:
void adc_int_handler()
{
	ADC1->ISR |= 1UL<<7;  // clear intflag
}
*/

volatile optflow_data_t latest_optflow;
volatile int optflow_errors;

volatile int new_gyro, new_xcel, new_compass;
/*
	10kHz interrupts drive practically everything.

	Most of the stuff are ran at 1kHz, different things on each cycle.
*/

volatile int optflow_int_x, optflow_int_y;

volatile int seconds;
volatile int millisec;

volatile int us100;
extern volatile int do_compass_round;

void power_and_led_fsm();
void pulutof_fsm();

void timebase_10k_handler()
{
	static int sec_gen = 0;
	static int cnt_10k = 0;
	static int gyro_xcel_compass_status;

	us100++;

	TIM6->SR = 0; // Clear interrupt flag

//	int starttime = TIM6->CNT;

	// Things expecting 10kHz calls:
	gyro_xcel_compass_status |= gyro_xcel_compass_fsm();

	#ifdef SONARS_INSTALLED
	sonar_fsm_10k();
	#endif

	// Send one more character through UART.
	// With 115200 baud rate, this will produce 10 kbytes/s stream,
	// meaning 11.52 bits/byte, hence, with 1 start and 1 stop bit, 1.52 extra idle bytes on average.
	uart_10k_fsm();

	// Things expecting 1kHz calls:
	if(cnt_10k == 0)
	{
		handle_uart_message(); // 3.33 kHz min

		millisec++;
		sec_gen++;
		if(sec_gen >= 1000)
		{
			seconds++;
			sec_gen = 0;
		}
		motcon_fsm();

	#ifdef OPTFLOW_INSTALLED
		int dx = 0;
		int dy = 0;
		optflow_fsm(&dx, &dy);

		optflow_int_x += dx;
		optflow_int_y += dy;
	#endif

	}
	else if(cnt_10k == 1)
	{
		motcon_fsm();
	}
	else if(cnt_10k == 2)
	{
		handle_uart_message(); // 3.33 kHz min
		navig_fsm1();
	}
	else if(cnt_10k == 3)
	{
		navig_fsm2();
	}
	else if(cnt_10k == 4)
	{
		handle_uart_message(); // 3.33 kHz min
	}
	else if(cnt_10k == 5)
	{
		run_feedbacks(gyro_xcel_compass_status);
		gyro_xcel_compass_status = 0;
	}
	else if(cnt_10k == 6)
	{
		handle_uart_message(); // 3.33 kHz min
		lidar_fsm();
		motcon_fsm();
	}
	else if(cnt_10k == 7)
	{
		power_and_led_fsm();
		#ifdef PULUTOF1
			pulutof_fsm();
		#endif
	}
	else if(cnt_10k == 8)
	{
		handle_uart_message(); // 3.33 kHz min
		if(do_compass_round)
		{
			do_compass_round = 0;
			compass_fsm(1);
		}
		else
			compass_fsm(0);
	}
	else if(cnt_10k == 9)
	{
		motcon_fsm();
		static int adc_decim = 0;
		if(++adc_decim >= 10)
		{
			ADC1->CR2 |= 1UL<<30; // Do ADC seq at 100 Hz
			adc_decim = 0;
		}

	}


	cnt_10k++;
	if(cnt_10k > 9) cnt_10k = 0;
//	int tooktime = TIM6->CNT - starttime;
//	if(tooktime > dbg[0])
//	{
//		dbg[0] = tooktime;
//		dbg[1] = cnt_10k; // which takes longest.
//	}
}


extern volatile int i2c1_state;
extern volatile int last_sr1;
extern volatile int i2c1_fails;

extern volatile int gyro_timestep_len;
extern volatile int xcel_timestep_len;

extern int cur_compass_angle;

extern int64_t xcel_long_integrals[3];

volatile adc_data_t adc_data[ADC_SAMPLES];

int get_bat_v() // in mv
{
	// theoretical: 0.5*1/4096*3300*(470+75)/75 = 2.927246
	return (((int)(adc_data[0].bat_v + adc_data[1].bat_v))*29272 / 10000);
}

int get_cha_v() // in mv
{
	// theoretical: 0.5*1/4096*3300*(470+52.3)/52.3 = 4.02292867
	return (((adc_data[0].cha_v + adc_data[1].cha_v))*40229 / 10000);
}


int get_bat_percentage()
{
	int bat_v = get_bat_v();
	int bat_percentage = (100*(bat_v-16000))/(21000-16000);
	if(bat_percentage < 0) bat_percentage = 0;
	if(bat_percentage > 127) bat_percentage = 127;
	return bat_percentage;
}

volatile int leds_control_by_motion = 1;
volatile int leds_motion_blink_left;
volatile int leds_motion_blink_right;
volatile int leds_motion_forward;

volatile int bat_emerg_on;
volatile int bat_emerg_action;
volatile int robot_is_in_charger;

#define BAT_EMERG_LVL 21500 //21500 = 4.30V/cell

void power_and_led_fsm()
{
	static int bat_warn_cnt = 0;
	static int bat_emerg_cnt = 0;
	static int bat_emerg_rereaction_cnt = 0;
	static int bat_shdn_cnt = 0;
	static int leds_blink_low_bat = 0;

	int bat_v = get_bat_v();
	int cha_v = get_cha_v();
	int bat_perc = get_bat_percentage();

	static int led_cnt = 0;
	if(++led_cnt >= 1000) led_cnt = 0;

	if(cha_v > 20000)
		robot_is_in_charger = 1;
	else
		robot_is_in_charger = 0;



	if(bat_emerg_on)
	{
		FWD_LIGHT_OFF();
		if(led_cnt == 0 || led_cnt == 200 || led_cnt == 400 || led_cnt == 600 || led_cnt == 800)
		{
			LEFT_BLINKER_ON();
			RIGHT_BLINKER_ON();
		}
		else if(led_cnt == 100 || led_cnt == 300 || led_cnt == 500 || led_cnt == 700 || led_cnt == 900)
		{
			LEFT_BLINKER_OFF();
			RIGHT_BLINKER_OFF();
		}

		if(bat_v > BAT_EMERG_LVL)
		{
			// condition didn't clear
			bat_emerg_rereaction_cnt++;
			if(bat_emerg_rereaction_cnt > 30000)
			{
				bat_emerg_rereaction_cnt = 0;
				bat_emerg_action = 1;
				robot_is_in_charger = 0; // try with full power - at least the input fuse blows!
			}
		}

		return;
	}

	if(bat_v > BAT_EMERG_LVL) 
	{
		bat_emerg_cnt++;

		if(bat_emerg_cnt > 500)
		{
			bat_emerg_on = 1;
			bat_emerg_action = 1;
		}
	}
	else
	{
		bat_emerg_cnt = 0;
	}


	if(bat_perc < 30)
	{
		bat_warn_cnt++;

		if(bat_warn_cnt > 500)
		{
			leds_blink_low_bat = 1;
		}
	}
	else
	{
		bat_warn_cnt = 0;
		leds_blink_low_bat = 0;
	}

	if(bat_perc < 5)
	{
		bat_shdn_cnt++;

		if(bat_shdn_cnt > 500)
		{
			DO_KILL_PWR();
		}
	}
	else
	{
		bat_shdn_cnt = 0;
	}
	
	if(leds_blink_low_bat)
	{
		// Low battery warning: blink a distinct ti-ti-ti----ti-ti-ti----- pattern with both blinkers

		FWD_LIGHT_OFF();
		if(led_cnt == 0 || led_cnt == 200 || led_cnt == 400)
		{
			LEFT_BLINKER_ON();
			RIGHT_BLINKER_ON();
		}
		else if(led_cnt == 50 || led_cnt == 250 || led_cnt == 450)
		{
			LEFT_BLINKER_OFF();
			RIGHT_BLINKER_OFF();
		}
	}
	else if(leds_control_by_motion)
	{	
		if(leds_motion_blink_left && led_cnt < 500)  LEFT_BLINKER_ON();  else LEFT_BLINKER_OFF();
		if(leds_motion_blink_right && led_cnt < 500) RIGHT_BLINKER_ON(); else RIGHT_BLINKER_OFF();
		if(leds_motion_forward) FWD_LIGHT_ON(); else FWD_LIGHT_OFF();

		if(!leds_motion_blink_left && !leds_motion_blink_right && !leds_motion_forward)
		{
			if(led_cnt < 4)
			{
				LEFT_BLINKER_ON();
				RIGHT_BLINKER_ON();
				FWD_LIGHT_ON();
			}
			else
			{
				LEFT_BLINKER_OFF();
				RIGHT_BLINKER_OFF();
				FWD_LIGHT_OFF();
			}
		}
	}
	else
	{
		LEFT_BLINKER_OFF();
		RIGHT_BLINKER_OFF();
		FWD_LIGHT_OFF();
	}

}

volatile uint32_t random = 123;

extern volatile int lidar_scan_ready;

volatile int dbg_sending_lidar = 0;

void uart_send_dbg_teleportation_bug()
{
	if(uart_busy())
		return;
	
	if(dbg_teleportation_bug_report)
	{
		send_uart_volatile(&dbg_teleportation_bug_data, 0xEE, sizeof(dbg_teleportation_bug_data_t));
		while(uart_busy());
		send_uart_volatile(&dbg_teleportation_extra, 0xEF, sizeof(dbg_teleportation_extra_t));
		while(uart_busy());
		dbg_teleportation_bug_report = 0;
	}
}

volatile int send_settings;

#ifdef PULUTOF1


#define PULUTOF_CS1() {GPIOB->BSRR = 1UL<<12;}
#define PULUTOF_CS0() {GPIOB->BSRR = 1UL<<(12+16);}

typedef struct __attribute__((packed)) __attribute__((aligned(4)))
{
	uint32_t header;

	pos_t robot_pos;

} robot_to_pulutof_t;

typedef struct __attribute__((packed)) __attribute__((aligned(4)))
{
	uint32_t header;

	pos_t dummy;

} pulutof_to_robot_t;

volatile robot_to_pulutof_t robot_to_pulutof __attribute__((aligned(4))) = {0x01020304, {0x05060708,0x090a0b0c,0x0d0e0f01}};
volatile pulutof_to_robot_t pulutof_to_robot __attribute__((aligned(4)));

volatile int pulutof_init_ok = 0;
void init_pulutof()
{
	PULUTOF_CS1();

	// SPI2 @ APB1 = 30 MHz

	// DMA1 STREAM 3 ch0 = PULUTOF RX
	DMA1_Stream3->PAR = (uint32_t)&(SPI2->DR);
	DMA1_Stream3->M0AR = (uint32_t)(&pulutof_to_robot);
	DMA1_Stream3->NDTR = sizeof pulutof_to_robot;
	DMA1_Stream3->CR = 0UL<<25 /*Channel*/ | 0b01UL<<16 /*med prio*/ | 0b00UL<<13 /*8-bit mem*/ | 0b00UL<<11 /*8-bit periph*/ |
	                   1UL<<10 /*mem increment*/ | 0UL<<8 /*CIRCULAR off*/;
	// DMA1 STREAM 4 ch0 = PULUTOF TX
	DMA1_Stream4->PAR = (uint32_t)&(SPI2->DR);
	DMA1_Stream4->M0AR = (uint32_t)(&robot_to_pulutof);
	DMA1_Stream4->NDTR = sizeof robot_to_pulutof;
	DMA1_Stream4->CR = 0UL<<25 /*Channel*/ | 0b01UL<<16 /*med prio*/ | 0b00UL<<13 /*8-bit mem*/ | 0b00UL<<11 /*8-bit periph*/ |
	                   1UL<<10 /*mem increment*/ | 0b01<<6 /*mem->periph*/ | 0UL<<8 /*CIRCULAR off*/;

	SPI2->CR1 = 0UL<<11 /*8-bit frame*/ | 1UL<<9 /*Software slave management*/ | 1UL<<8 /*SSI bit must be high*/ |
		0b001UL<<3 /*div 4 = 7.5 MHz*/ | 1UL<<2 /*Master*/;
	SPI2->CR2 = 1UL<<1 /* TX DMA enable */ | 0UL<<0 /* RX DMA enable OFF*/ | 0UL<<2 /* SSOE OFF*/;


	SPI2->CR1 |= 1UL<<6; // Enable SPI

	delay_ms(10);
	pulutof_init_ok = 1;
}

// Run this at 1kHz!
void pulutof_fsm()
{
	if(!pulutof_init_ok) return;
	static int cycle = 0;
	if(cycle == 0)
	{	
		PULUTOF_CS0();
		robot_to_pulutof.robot_pos = cur_pos;
		DMA1->LIFCR = 0b111101UL<<22; DMA1_Stream3->CR |= 1UL; // Enable RX DMA
		DMA1->HIFCR = 0b111101UL; DMA1_Stream4->CR |= 1UL; // Enable TX DMA
		cycle = 1;
	}
	else
	{
		PULUTOF_CS1();
		cycle = 0;
	}
}
#endif


int main()
{

	__disable_irq();
	/*
	XTAL = HSE = 8 MHz
	PLLCLK = SYSCLK = 120 MHz (max)
	AHB = HCLK = 120 MHz (max) --> AHB prescaler = 1
	Cortex System Timer = AHB/8 = 15 MHz
	APB2 = high-speed APB = 60 MHz (max) --> APB2 prescaler = 2
	APB1 = low-speed APB  = 30 MHz (max) --> PAB1 prescaler = 4
	APB2 timers x2 = 120 MHz
	APB1 timers x2 = 60 MHz

	PLL CONFIG:
	Input: 8 MHz
	M (for PLL input)   =  4 -> 2 MHz (must be between 1..2MHz)
	N (PLL multiplier)  = 120 -> 240 MHz
	P (for main system) = 2  -> 120 MHz
	Q (for USB etc.)    = 5  -> 48MHz
	*/

	delay_ms(1); // to ensure voltage has ramped up

	/*
		Caches are totally unspecified, but it is highly likely that the "data cache" does not refer to SRAM, but data stored in flash.
		Hence, it's probably safe to turn it on.
	*/

	// delay loop: 13 sec -> 8 sec, when caches turned on.

	// 3 wait states for 120MHz and Vcc over 2.7V
	FLASH->ACR = 1UL<<10 /* Data cache enable */ | 1UL<<9 /* Instr cache enable */ | 1UL<<8 /*prefetch enable*/ | 3UL /*3 wait states*/;

	RCC->PLLCFGR = 5UL<<24 /*Q*/ | 1UL<<22 /*HSE as source*/ | 0b00UL<<16 /*P=2*/ | 120UL<<6 /*N*/ | 4UL /*M*/;
	RCC->CFGR = 0b100UL<<13 /*APB2 div 2*/ | 0b101UL<<10 /*APB1 div 4*/;

	RCC->CR |= 1UL<<16; // HSE clock on
	RCC->CR |= 1UL<<24; // PLL on

	while(!(RCC->CR & 1UL<<25)) ; // Wait for PLL
	RCC->CFGR |= 0b10; // Change PLL to system clock
	while((RCC->CFGR & (0b11UL<<2)) != (0b10UL<<2)) ; // Wait for switchover to PLL.


	RCC->AHB1ENR |= 0b111111111 /* PORTA to PORTI */ | 1UL<<22 /*DMA2*/ | 1UL<<21 /*DMA1*/;
	RCC->APB1ENR |= 1UL<<21 /*I2C1*/ | 1UL<<18 /*USART3*/ | 1UL<<14 /*SPI2*/ | 1UL<<4 /*TIM6*/;
	RCC->APB2ENR |= 1UL<<12 /*SPI1*/ | 1UL<<4 /*USART1*/ | 1UL<<8 /*ADC1*/;

	delay_us(10);

#ifdef PCB1A

	GPIOA->AFR[0] = 5UL<<20 | 5UL<<24 | 5UL<<28 /*SPI1*/;
	GPIOB->AFR[0] = 7UL<<24 | 7UL<<28 /*USART1*/;
	GPIOB->AFR[1] = 5UL<<20 | 5UL<<24 | 5UL<<28 /*SPI2*/ |
	                 4UL<<0 | 4UL<<4 /*I2C1*/;
	GPIOC->AFR[1] = 7UL<<8 | 7UL<<12; // USART3 alternate functions.

	             // Mode:
		     // 00 = General Purpose In
	             // 01 = General Purpose Out
	             // 10 = Alternate Function (in/out controlled by peripheral)
	             // 11 = Analog in (to ADC)

	             // Speed:
	             // 00 = low, 01 = medium, 10 = high, 11 = superhyper
	             //    15141312111009080706050403020100
	             //     | | | | | | | | | | | | | | | |
	GPIOA->MODER   = 0b01000000000000001010100100000000;
	GPIOA->OSPEEDR = 0b00000000000000000100010100000000;

	GPIOB->ODR     = 1UL<<8 | 1UL<<9; // I2C pins high.
	GPIOB->OTYPER  = 1UL<<8 | 1UL<<9; // Open drain for I2C.
	             //    15141312111009080706050403020100
	             //     | | | | | | | | | | | | | | | |
	GPIOB->MODER   = 0b10101001000001011010000000000000;
	GPIOB->OSPEEDR = 0b10001001000001010000010001000000;
	GPIOB->PUPDR   = 0b00000000010100000000000000000000;
	             //    15141312111009080706050403020100
	             //     | | | | | | | | | | | | | | | |
	GPIOC->MODER   = 0b00000100101000000000010100110000;
	GPIOC->OSPEEDR = 0b00000000000100000000010100000000;
	             //    15141312111009080706050403020100
	             //     | | | | | | | | | | | | | | | |
	GPIOD->MODER   = 0b01000000000000000000010101000100;
	GPIOD->OSPEEDR = 0b00000000000000000000000001000100;
	             //    15141312111009080706050403020100
	             //     | | | | | | | | | | | | | | | |
	GPIOE->MODER   = 0b01010101010101010101000000000000;
	GPIOE->OSPEEDR = 0b00000000000000000001000000000000;

#endif

#ifdef PCB1B

#ifdef PULUTOF1
	PULUTOF_CS1();
#endif

	GPIOA->AFR[0] = 5UL<<20 | 5UL<<24 | 5UL<<28 /*SPI1*/;
	GPIOB->AFR[0] = 7UL<<24 | 7UL<<28 /*USART1*/;
	GPIOB->AFR[1] = 5UL<<20 | 5UL<<24 | 5UL<<28 /*SPI2*/ |
//	GPIOB->AFR[1] = 5UL<<20 | 5UL<<24 | 5UL<<28 | 5UL<<16 /*SPI2*/ |
	                 4UL<<0 | 4UL<<4 /*I2C1*/;
	GPIOC->AFR[1] = 7UL<<8 | 7UL<<12; // USART3 alternate functions.

	             // Mode:
		     // 00 = General Purpose In
	             // 01 = General Purpose Out
	             // 10 = Alternate Function (in/out controlled by peripheral)
	             // 11 = Analog in (to ADC)

	             // Speed:
	             // 00 = low, 01 = medium, 10 = high, 11 = superhyper
	             //    15141312111009080706050403020100
	             //     | | | | | | | | | | | | | | | |
	GPIOA->MODER   = 0b01000100010100011010100000000000;
	GPIOA->OSPEEDR = 0b00000000000000000100010000000000;
	GPIOA->PUPDR   = 0b00000001000001000000000000000000;

	GPIOB->ODR     = 1UL<<8 | 1UL<<9; // I2C pins high.
	GPIOB->OTYPER  = 1UL<<8 | 1UL<<9; // Open drain for I2C.
	             //    15141312111009080706050403020100
	             //     | | | | | | | | | | | | | | | |
	GPIOB->MODER   = 0b10101001000101011010010100000001;
	GPIOB->OSPEEDR = 0b10001001000001010000010000000000;
	GPIOB->PUPDR   = 0b00000000000000000000000001000000;
	             //    15141312111009080706050403020100
	             //     | | | | | | | | | | | | | | | |
	GPIOC->MODER   = 0b00000100101000000000010111111111;
	GPIOC->OSPEEDR = 0b00000000000100000000010100000000;
	             //    15141312111009080706050403020100
	             //     | | | | | | | | | | | | | | | |
	GPIOD->MODER   = 0b00000100010101000000010000000000;
	GPIOD->OSPEEDR = 0b00000000000000000000000000000000;
	GPIOD->PUPDR   = 0b00000000000000000101000000000000;
	             //    15141312111009080706050403020100
	             //     | | | | | | | | | | | | | | | |
	GPIOE->MODER   = 0b00010000000000010101000101010000;
	GPIOE->OSPEEDR = 0b00000000000000000000000000000000;
	GPIOE->PUPDR   = 0b00000000000000000000010000000000;
#endif


	#ifdef SONARS_INSTALLED
	init_sonars();
	#endif
	init_uart();

	// Motor controller nCS signals must be high as early as possible. Motor controllers wait 100 ms at boot for this.
	#if NUM_MOTCONS >= 4
	MC4_CS1();
	#endif
	#if NUM_MOTCONS >= 3
	MC3_CS1();
	#endif
	MC2_CS1();
	MC1_CS1();

	/*
		Interrupts will have 4 levels of pre-emptive priority, and 4 levels of sub-priority.

		Interrupt with more urgent (lower number) pre-emptive priority will interrupt another interrupt running.

		If interrupts with similar or lower urgency happen during another ISR, the subpriority level will decide
		the order they will be run after finishing the more urgent ISR first.
	*/
	NVIC_SetPriorityGrouping(2);

	/*
		USART3 = the main USART @ APB1 = 30 MHz
		16x oversampling
		115200bps -> Baudrate register = 16.25 = 16 1/4 = 16 4/16

		STM32F205 DMA is a total and utter joke. There are practically no connections to anywhere.
		SPI2 and USART3 cannot be used at the same time. Even substituting UART4 doensn't help.
		So, there is no DMA for UART. So, we are just doing it interrupt-based.

		To make it worse, interrupts cannot be reliably used for TX and RX at the same time. At least the
		ISR will get complicated, since the TX interrupt cannot be reliably forced off for some reason.
		Maybe to be investigated later. In the meantime, RX works with interrupts, while TX works with
		polling. This is actually not a big deal, sending is done in the 10k timebase handler.
	*/

	USART3->BRR = 16UL<<4 | 4UL;
	USART3->CR1 = 1UL<<13 /*USART enable*/ | 1UL<<5 /*RX interrupt*/ | 1UL<<3 /*TX ena*/ | 1UL<<2 /*RX ena*/;

	/*
		TIM6 @ APB1 at 30MHz, but the counter runs at x2 = 60MHz
		Create 10 kHz timebase interrupt
	*/

	TIM6->DIER |= 1UL; // Update interrupt
	TIM6->ARR = 5999; // 60MHz -> 10 kHz
	TIM6->CR1 |= 1UL; // Enable


	#ifdef OPTFLOW_INSTALLED
	FLOW_CS1();
	#endif

	NVIC_SetPriority(USART3_IRQn, 0b0000); // highest prio really needed.
	NVIC_EnableIRQ(USART3_IRQn);

	delay_ms(100);

	int tries = 5+1;
	while(--tries && init_gyro_xcel_compass()) delay_ms(50);
	if(tries < 1) DO_KILL_PWR();

	set_obstacle_avoidance_margin(1);

	PSU12V_ENA();

	#ifdef OPTFLOW_INSTALLED
	init_optflow();
	#endif
	init_motcons();
	init_lidar();


	ADC->CCR = 1UL<<23 /* temp sensor and Vref enabled */ | 0b00<<16 /*prescaler 2 -> 30MHz*/;
	ADC1->CR1 = 1UL<<8 /* SCAN mode */;
	ADC1->CR2 = 1UL<<9 /* Magical DDS bit to actually enable DMA requests */ | 1UL<<8 /*DMA ena*/ | 0UL<<1 /*continuous OFF*/;
	ADC1->SQR1 = (ADC_ITEMS-1)<<20 /* sequence length */;

	#ifdef PCB1A
	ADC1->SMPR1 = 0b010UL<<6 /*ch12 (bat voltage): 28 cycles*/;
	ADC1->SQR3 = 12UL<<0; // Ch12 first in sequence: Vbat
	#endif

	#ifdef PCB1B
//	ADC1->SMPR1 = 0b010UL<<6 /*ch12 (bat voltage): 28 cycles*/ | 0b010UL<<9 /*ch13 (charger voltage): 28 cycles*/;
	ADC1->SMPR1 = 0b111UL<<6 /*ch12 (bat voltage): 480 cycles*/ | 0b111UL<<9 /*ch13 (charger voltage): 480 cycles*/;
	ADC1->SQR3 = 12UL<<0 /*Ch12 first in sequence: bat voltage*/ | 13UL<<5 /*2nd: Ch13 (charger voltage)*/;
	#endif

	ADC1->CR2 |= 1; // Enable ADC

	DMA2_Stream4->PAR = (uint32_t)&(ADC1->DR);
	DMA2_Stream4->M0AR = (uint32_t)(adc_data);
	DMA2_Stream4->NDTR = ADC_ITEMS*ADC_SAMPLES;
	DMA2_Stream4->CR = 0UL<<25 /*Channel*/ | 0b01UL<<16 /*med prio*/ | 0b01UL<<13 /*16-bit mem*/ | 0b01UL<<11 /*16-bit periph*/ |
	                   1UL<<10 /*mem increment*/ | 1UL<<8 /*circular*/;


	DMA2->LIFCR = 0b111101UL<<0;
	DMA2_Stream4->CR |= 1UL; // Enable ADC DMA

	delay_ms(1);

//	ADC1->CR2 |= 1UL<<30; // Start converting.



	#ifdef PCB1B

	LEFT_BLINKER_ON();
	delay_ms(200);
	LEFT_BLINKER_OFF();
	FWD_LIGHT_ON();
	delay_ms(200);
	FWD_LIGHT_OFF();
	RIGHT_BLINKER_ON();
	delay_ms(200);
	RIGHT_BLINKER_OFF();
	delay_ms(200);
	LEFT_BLINKER_ON();
	FWD_LIGHT_ON();
	RIGHT_BLINKER_ON();
	delay_ms(200);
	RIGHT_BLINKER_OFF();
	FWD_LIGHT_OFF();
	LEFT_BLINKER_OFF();

	#endif

	#ifdef PULUTOF1

	PO5_ON();
	init_pulutof();

	delay_ms(1000);

	#endif


	delay_ms(100);
	NVIC_SetPriority(TIM6_DAC_IRQn, 0b1010);
	NVIC_EnableIRQ(TIM6_DAC_IRQn);

	CHARGER_ENA();

	__enable_irq();

	int cnt = 0;

	init_lidar();

	lidar_on(2, 1);


	while(1)
	{
		random++;
		cnt++;

		static uint8_t sync_packet[8] = {0xff,0xff,0xff,0xff,  0xff,0xff,0x12,0xab};
		while(uart_busy()) random++;
		send_uart(sync_packet, 0xaa, 8);
		while(uart_busy()) random++;

		uart_send_dbg_teleportation_bug();

		LED_ON();
		lidar_scan_ready = 0;
		while(!lidar_scan_ready) ;
		LED_OFF();
		dbg_sending_lidar = 1;
		send_uart(prev_lidar_scan, 0x84, LIDAR_SIZEOF(*prev_lidar_scan));
		dbg_sending_lidar = 0;

		// Send stuff required to be sent often:
		while(uart_busy()) random++;
		uart_send_critical1(); 
		while(uart_busy()) random++;
		uart_send_critical2(); 
		while(uart_busy()) random++;
		uart_send_fsm(); // send something else.

#ifdef SONARS_INSTALLED
		{
			sonar_xyz_t* sonar;
			while( (sonar = get_sonar_point()) )
			{
				while(uart_busy()) random++;
				send_uart(sonar, 0x85, sizeof(sonar_xyz_t));
			}
		}
#endif

		while(uart_busy()) random++;
		uart_send_critical1(); // Send stuff required to be sent often.


		if(send_chafind_results)
		{
			send_chafind_results = 0;
			while(uart_busy()) random++;
			send_uart(&chafind_results, 0x95, sizeof(chafind_results_t));
		}

		if(send_settings)
		{
			send_settings = 0;
			while(uart_busy()) random++;
			send_uart(&settings, 0xd1, sizeof(settings_t));
		}

	}

}


