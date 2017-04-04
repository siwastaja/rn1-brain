#include <stdio.h>
#include <stdint.h>
#include <string.h>

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

#define LED_ON()  {GPIOC->BSRR = 1UL<<13;}
#define LED_OFF() {GPIOC->BSRR = 1UL<<(13+16);}
#define CHARGER_ENA() {GPIOA->BSRR = 1UL<<15;}
#define CHARGER_DIS() {GPIOA->BSRR = 1UL<<(15+16);}
#define PSU5V_ENA() {GPIOE->BSRR = 1UL<<15;}
#define PSU5V_DIS() {GPIOE->BSRR = 1UL<<(15+16);}
#define PSU12V_ENA() {GPIOD->BSRR = 1UL<<4;}
#define PSU12V_DIS() {GPIOD->BSRR = 1UL<<(4+16);}

uint8_t txbuf[1024];

void delay_us(uint32_t i)
{
	if(i==0) return;
	i *= 20;
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

void adc_int_handler()
{
	LED_ON();
	delay_ms(50);
	LED_OFF();
//	ADC1->ISR |= 1UL<<7;
}

void usart_print(const char *buf)
{
	while(buf[0] != 0)
	{
		while((USART3->SR & (1UL<<7)) == 0) ;
		USART3->DR = buf[0];
		buf++;
	}
}

volatile motcon_t motcons[NUM_MOTCONS];


void run_flasher()
{
	__disable_irq();
	USART3->CR1 = 0; // Disable
	delay_us(10);
	USART3->SR = 0; // Clear flags
	USART3->BRR = 16UL<<4 | 4UL; // 115200
	USART3->CR1 = 1UL<<13 /*USART enable*/ | 1UL<<3 /*TX ena*/ | 1UL<<2 /*RX ena*/;
	delay_us(10);
	flasher();
	while(1);
}

void uart_rx_handler()
{
	// This SR-then-DR read sequence clears error flags:
	char byte = USART3->DR;

	switch(byte)
	{
		case 'a': motcons[2].cmd.speed += 80; break;
		case 'q': motcons[2].cmd.speed -= 80; break;
		case 's': motcons[2].cmd.speed += 80; motcons[3].cmd.speed += 80; break;
		case 'w': motcons[2].cmd.speed -= 80; motcons[3].cmd.speed -= 80;break;
		case 'd': motcons[3].cmd.speed += 80; break;
		case 'e': motcons[3].cmd.speed -= 80; break;
		case ',': lidar_rpm_setpoint_x64 -= 320; break;
		case '.': lidar_rpm_setpoint_x64 += 320; break;
		case '9': run_flasher(); break;

		default: break;
	}
}

volatile int send_cnt;
volatile int send_len;

#define SEND(len)  {send_cnt=0; send_len=(len); USART3->CR1 |= 1UL<<7;}
#define NONREADY() (send_cnt < send_len)
#define READY()    (send_len == send_cnt)

void uart_inthandler()
{
	LED_ON();
	uint32_t status = USART3->SR;

	if(status & 1UL<<5)
	{
		uart_rx_handler();
	}

	if(status & 1UL<<3)
	{
		// RX Overrun, do something
	}

	if(status & 1UL<<7)
	{
		if(send_cnt < send_len)
		{
			USART3->DR = txbuf[send_cnt];
			send_cnt++;
			if(send_cnt == send_len)
			{
				USART3->CR1 &= ~(1UL<<7);
			}
		}
	}
	LED_OFF();
}

int latest_sonars[NUM_SONARS]; // in cm, 0 = no echo

volatile optflow_data_t latest_optflow;
volatile int optflow_errors;

void timebase_10k_handler()
{
	static int cnt_10k = 0;
	static int gyro_xcel_compass_cnt = 0;
	TIM6->SR = 0;
	cnt_10k++;

	optflow_fsm();

	// Motcon at 10 kHz
	motcon_fsm();

	sonar_fsm(); // at 10 kHz

	lidar_ctrl_loop();

	gyro_xcel_compass_cnt++;
	if(gyro_xcel_compass_cnt == 100) // gyro, xcel, compass at 100Hz
	{
		start_gyro_xcel_compass_sequence();
		gyro_xcel_compass_cnt = 0;
	}

}

volatile xcel_data_t latest_xcel;
volatile gyro_data_t latest_gyro;
volatile compass_data_t latest_compass;

extern volatile lidar_datum_t lidar_full_rev[90];

int main()
{
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

	// 3 wait states for 120MHz and Vcc over 2.7V
	FLASH->ACR = 1UL<<8 /*prefetch enable*/ | 3UL /*3 wait states*/;

	RCC->PLLCFGR = 5UL<<24 /*Q*/ | 1UL<<22 /*HSE as source*/ | 0b00UL<<16 /*P=2*/ | 120UL<<6 /*N*/ | 4UL /*M*/;
	RCC->CFGR = 0b100UL<<13 /*APB2 div 2*/ | 0b101UL<<10 /*APB1 div 4*/;

	RCC->CR |= 1UL<<16; // HSE clock on
	RCC->CR |= 1UL<<24; // PLL on

	while(!(RCC->CR & 1UL<<25)) ; // Wait for PLL
	RCC->CFGR |= 0b10; // Change PLL to system clock
	while((RCC->CFGR & (0b11UL<<2)) != (0b10UL<<2)) ; // Wait for switchover to PLL.


	RCC->AHB1ENR |= 0b111111111 /* PORTA to PORTI */ | 1UL<<22 /*DMA2*/ | 1UL<<21 /*DMA1*/;
	RCC->APB1ENR |= 1UL<<21 /*I2C1*/ | 1UL<<18 /*USART3*/ | 1UL<<14 /*SPI2*/ | 1UL<<2 /*TIM4*/ | 1UL<<4 /*TIM6*/;
	RCC->APB2ENR |= 1UL<<12 /*SPI1*/ | 1UL<<4 /*USART1*/;

	delay_us(10);

	GPIOA->AFR[0] = 5UL<<20 | 5UL<<24 | 5UL<<28 /*SPI1*/;
	GPIOB->AFR[0] = 7UL<<24 | 7UL<<28 /*USART1*/;
	GPIOB->AFR[1] = 5UL<<20 | 5UL<<24 | 5UL<<28 /*SPI2*/ |
	                 4UL<<0 | 4UL<<4 /*I2C1*/;
	GPIOC->AFR[1] = 7UL<<8 | 7UL<<12; // USART3 alternate functions.
	GPIOD->AFR[1] = 2UL<<28 /*TIM4*/;

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
	             //    15141312111009080706050403020100
	             //     | | | | | | | | | | | | | | | |
	GPIOB->MODER   = 0b10101001000010101010000000000000;
	GPIOB->OSPEEDR = 0b01000101000001010000010001000000;
	GPIOB->OTYPER  = 1UL<<8 | 1UL<<9; // Open drain for I2C.
	             //    15141312111009080706050403020100
	             //     | | | | | | | | | | | | | | | |
	GPIOC->MODER   = 0b00000100101000000000010100000000;
	GPIOC->OSPEEDR = 0b00000000000100000000010100000000;
	             //    15141312111009080706050403020100
	             //     | | | | | | | | | | | | | | | |
	GPIOD->MODER   = 0b10000000000000000000010101000000;
	GPIOD->OSPEEDR = 0b00000000000000000000000001000000;
	             //    15141312111009080706050403020100
	             //     | | | | | | | | | | | | | | | |
	GPIOE->MODER   = 0b01000000000000000001000000000000;
	GPIOE->OSPEEDR = 0b00000000000000000001000000000000;
	             //    15141312111009080706050403020100
	             //     | | | | | | | | | | | | | | | |
	GPIOF->MODER   = 0b00000000000000000000000000000000;
	GPIOF->OSPEEDR = 0b00000000000000000000000000000000;
	             //    15141312111009080706050403020100
	             //     | | | | | | | | | | | | | | | |
	GPIOG->MODER   = 0b00000000000000000000000000000000;
	GPIOG->OSPEEDR = 0b00000000000000000000000000000000;

	NVIC_SetPriorityGrouping(2);

	// USART3 = the main USART @ APB1 = 30 MHz
	// 16x oversampling
	// 115200bps -> Baudrate register = 16.25 = 16 1/4 = 16 4/16

	// STM32F205 DMA is a total and utter joke. There are practically no connections to anywhere.
	// SPI2 and USART3 cannot be used at the same time. Even substituting UART4 doensn't help.
	// So, there is no DMA for UART. So, we are just doing it interrupt-based.

	USART3->BRR = 16UL<<4 | 4UL;
	USART3->CR1 = 1UL<<13 /*USART enable*/ | 1UL<<5 /*RX interrupt*/ | 1UL<<3 /*TX ena*/ | 1UL<<2 /*RX ena*/;

	// TIM4 = Lidar motor

	TIM4->CR1 = 1UL<<7 /*auto preload*/ | 0b01UL<<5 /*centermode*/;
	TIM4->CCMR2 = 1UL<<11 /*CH4 preload*/ | 0b110UL<<12 /*PWMmode1*/;
	TIM4->CCER = 1UL<<12 /*CH4 out ena*/;
	TIM4->ARR = 1024;
	TIM4->CCR4 = 350;
	TIM4->CR1 |= 1UL; // Enable.

	/*
		TIM6 @ APB1 at 30MHz, but the counter runs at x2 = 60MHz
		Create 10 kHz timebase
	*/

	TIM6->DIER |= 1UL; // Update interrupt
	TIM6->ARR = 5999; // 60MHz -> 10 kHz
	TIM6->CR1 |= 1UL; // Enable


	FLOW_CS1();

	NVIC_SetPriority(I2C1_EV_IRQn, 0b0000);
	NVIC_EnableIRQ(USART3_IRQn);
	__enable_irq();

	usart_print("booty booty\r\n");
	delay_ms(1000);

	init_gyro_xcel_compass();
	usart_print("gyro,xcel,compass init ok\r\n");
	init_optflow();
	usart_print("optflow init ok\r\n");
	init_motcons();
	usart_print("motcons init ok\r\n");
	init_lidar();
	usart_print("lidar init ok\r\n");

	NVIC_EnableIRQ(TIM6_DAC_IRQn);

	delay_ms(100);


	PSU12V_ENA();
	CHARGER_ENA();

	usart_print("pre-syncing lidar... ");
	sync_lidar();
	usart_print("stablizing lidar... ");
	delay_ms(6000);
	usart_print("re-syncing lidar... ");
	resync_lidar();
	usart_print("done\r\n");

//	LED_OFF();

	int kakka = 0;
	while(1)
	{

		delay_ms(1);
		kakka++;

		if(kakka<100)
			continue;

		kakka = 0;
/*
		char buffer[4000];
		char* buf = buffer;


		buf = o_str_append(buf, " gyro=");
		buf = o_utoa8_fixed(latest_gyro.status_reg, buf);
		buf = o_str_append(buf, "  ");
		buf = o_itoa16_fixed(latest_gyro.x, buf);
		buf = o_str_append(buf, ", ");
		buf = o_itoa16_fixed(latest_gyro.y, buf);
		buf = o_str_append(buf, ", ");
		buf = o_itoa16_fixed(latest_gyro.z, buf);
		buf = o_str_append(buf, " xcel=");
		buf = o_utoa8_fixed(latest_xcel.status_reg, buf);
		buf = o_str_append(buf, "  ");
		buf = o_itoa16_fixed(latest_xcel.x, buf);
		buf = o_str_append(buf, ", ");
		buf = o_itoa16_fixed(latest_xcel.y, buf);
		buf = o_str_append(buf, ", ");
		buf = o_itoa16_fixed(latest_xcel.z, buf);
		buf = o_str_append(buf, " compass=");
		buf = o_utoa8_fixed(latest_compass.status_reg, buf);
		buf = o_str_append(buf, "  ");
		buf = o_itoa16_fixed(latest_compass.x, buf);
		buf = o_str_append(buf, ", ");
		buf = o_itoa16_fixed(latest_compass.y, buf);
		buf = o_str_append(buf, ", ");
		buf = o_itoa16_fixed(latest_compass.z, buf);
		buf = o_str_append(buf, " optflow=");
		buf = o_utoa8_fixed(latest_optflow.motion, buf);
		buf = o_str_append(buf, " dx=");
		buf = o_itoa8_fixed(latest_optflow.dx, buf);
		buf = o_str_append(buf, " dy=");
		buf = o_itoa8_fixed(latest_optflow.dy, buf);
		buf = o_str_append(buf, " Q=");
		buf = o_utoa8_fixed(latest_optflow.squal, buf);
		buf = o_str_append(buf, " shutter=");
		buf = o_utoa16_fixed(latest_optflow.shutter_msb<<8 | latest_optflow.shutter_lsb, buf);
		buf = o_str_append(buf, " max=");
		buf = o_utoa8_fixed(latest_optflow.max_pixel, buf);
		buf = o_str_append(buf, " errs=");
		buf = o_utoa16_fixed(optflow_errors, buf);

		buf = o_str_append(buf, "\r\n\r\n");
		usart_print(buffer);

*/
		while(NONREADY()) ;
		msg_gyro_t msg;
		msg.status = 1;
		msg.int_x = I16_I14(latest_gyro.x);
		msg.int_y = I16_I14(latest_gyro.y);
		msg.int_z = I16_I14(latest_gyro.z);
		txbuf[0] = 128;
		memcpy(txbuf+1, &msg, sizeof(msg_gyro_t));
		SEND(1+sizeof(msg_gyro_t));

		delay_ms(500);

		while(NONREADY()) ;
		msg_xcel_t msgx;
		msgx.status = 1;
		msgx.int_x = I16_I14(latest_xcel.x);
		msgx.int_y = I16_I14(latest_xcel.y);
		msgx.int_z = I16_I14(latest_xcel.z);
		txbuf[0] = 129;
		memcpy(txbuf+1, &msgx, sizeof(msg_xcel_t));
		SEND(1+sizeof(msg_xcel_t));

		while(NONREADY()) ;
		txbuf[0] = 0x84;
		txbuf[1] = 1;
		int i;
		for(i = 0; i < 90; i++)
		{
			int o;
			for(o = 0; o < 4; o++)
			{
				if(lidar_full_rev[i].d[o].flags_distance&(1<<15))
				{
					txbuf[2+8*i+2*o] = 0;
					txbuf[2+8*i+2*o+1] = 0;
				}
				else
				{
					txbuf[2+8*i+2*o] = lidar_full_rev[i].d[o].flags_distance&0x7f;
					txbuf[2+8*i+2*o+1] = (lidar_full_rev[i].d[o].flags_distance>>7)&0x7f;
				}
			}
		}
		SEND(90*4*2+2);
	}



}
