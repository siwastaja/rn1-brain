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

#define LED_ON()  {GPIOC->BSRR = 1UL<<13;}
#define LED_OFF() {GPIOC->BSRR = 1UL<<(13+16);}
#define CHARGER_ENA() {GPIOA->BSRR = 1UL<<15;}
#define CHARGER_DIS() {GPIOA->BSRR = 1UL<<(15+16);}
#define PSU5V_ENA() {GPIOE->BSRR = 1UL<<15;}
#define PSU5V_DIS() {GPIOE->BSRR = 1UL<<(15+16);}
#define PSU12V_ENA() {GPIOD->BSRR = 1UL<<4;}
#define PSU12V_DIS() {GPIOD->BSRR = 1UL<<(4+16);}


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


void uart_rx_handler()
{
	// This SR-then-DR read sequence clears error flags:
	LED_ON();
	uint32_t status = USART3->SR;
	char byte = USART3->DR;
	if(status & 1UL<<3)
	{
		// Overrun, do something
	}

	switch(byte)
	{
		case 'a': motcons[2].cmd.speed += 80; break;
		case 'q': motcons[2].cmd.speed -= 80; break;
		case 's': motcons[2].cmd.speed += 80; motcons[3].cmd.speed += 80; break;
		case 'w': motcons[2].cmd.speed -= 80; motcons[3].cmd.speed -= 80;break;
		case 'd': motcons[3].cmd.speed += 80; break;
		case 'e': motcons[3].cmd.speed -= 80; break;
		default: break;
	}
}



volatile optflow_data_t latest_optflow;
volatile int optflow_errors;

volatile int cnt_10k;
void timebase_10k_handler()
{
	TIM6->SR = 0;
	cnt_10k++;

	optflow_fsm();

	// Motcon at 10 kHz
	motcon_fsm();

	if(cnt_10k % 100) // gyro, xcel, compass at 100Hz
	{
		start_gyro_xcel_compass_sequence();
		LED_OFF();
	}
}

volatile xcel_data_t latest_xcel;
volatile gyro_data_t latest_gyro;
volatile compass_data_t latest_compass;

typedef struct __attribute__ ((__packed__))
{
	uint8_t start;
	uint8_t idx;
	uint16_t speed;
	uint32_t data0;
	uint32_t data1;
	uint32_t data2;
	uint32_t data3;
	uint16_t checksum;
} lidar_data_t;

volatile lidar_data_t latest_lidar;

int main()
{
	int i;

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

	delay_ms(1);

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

	delay_us(100);

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
	GPIOD->MODER   = 0b10000000000000000000000100000000;
	GPIOD->OSPEEDR = 0b00000000000000000000000000000000;
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


	// USART3 = APB1 = 30 MHz
	// 16x oversampling
	// 115200bps -> Baudrate register = 16.25 = 16 1/4 = 16 4/16
	USART3->BRR = 16UL<<4 | 4UL;
	USART3->CR1 = 1UL<<13 /*USART enable*/ | 1UL<<5 /*RX interrupt*/ | 1UL<<3 /*TX ena*/ | 1UL<<2 /*RX ena*/;

	TIM4->CR1 = 1UL<<7 /*auto preload*/ | 0b01UL<<5 /*centermode*/;
	TIM4->CCMR2 = 1UL<<11 /*CH4 preload*/ | 0b110UL<<12 /*PWMmode1*/;
	TIM4->CCER = 1UL<<12 /*CH4 out ena*/;
	TIM4->ARR = 1024;
	TIM4->CCR4 = 500;
	TIM4->CR1 |= 1UL; // Enable.

	/*
		TIM6 @ APB1 at 30MHz, but the counter runs at x2 = 60MHz
		Create 10 kHz timebase
	*/

	TIM6->DIER |= 1UL; // Update interrupt
	TIM6->ARR = 5999; // 60MHz -> 10 kHz
	TIM6->CR1 |= 1UL; // Enable


	FLOW_CS1();

	NVIC_EnableIRQ(USART3_IRQn);
	NVIC_EnableIRQ(TIM6_DAC_IRQn);
	__enable_irq();

	usart_print("booty booty\r\n");
	delay_ms(1000);

	init_gyro_xcel_compass();
	usart_print("gyro,xcel,compass init ok\r\n");
	init_optflow();
	usart_print("optflow init ok\r\n");
	init_motcons();
	usart_print("motcons init ok\r\n");

	delay_ms(100);


	PSU12V_ENA();
	PSU5V_ENA();
	CHARGER_ENA();

	delay_ms(500); // Let the lidar boot.

	// USART1 (lidar) = APB2 = 60 MHz
	// 16x oversampling
	// 115200bps -> Baudrate register = 32.5625 = 32 9/16
	// USART1 RX is mapped to DMA2, Stream2, Ch4

	DMA2_Stream2->PAR = (uint32_t)&(USART1->DR);
	DMA2_Stream2->M0AR = (uint32_t)(&latest_lidar);
	DMA2_Stream2->NDTR = 22;
	DMA2_Stream2->CR = 4UL<<25 /*Channel*/ | 0b01UL<<16 /*med prio*/ | 0b00UL<<13 /*8-bit mem*/ | 0b00UL<<11 /*8-bit periph*/ |
	                   1UL<<10 /*mem increment*/ | 1UL<<8 /*circular*/;

	DMA2->LIFCR = 0xffffffff; // Clear all flags
	DMA2->HIFCR = 0xffffffff;
	DMA2_Stream2->CR |= 1UL; // Enable

	USART1->BRR = 32UL<<4 | 9UL;
	USART1->CR3 = 1UL<<6 /*RX DMA*/;
	USART1->CR1 = 1UL<<13 /*USART enable*/ | 1UL<<3 /*TX ena*/ | 1UL<<2 /*RX ena*/;


//	LED_OFF();
	int kakka = 0;

	while(1)
	{
		char buffer[1000];
		char* buf = buffer;

		delay_ms(1);
		kakka++;

		if(kakka<1000)
			continue;

		for(i=2; i<4; i++)
		{
			if(motcons[i].cmd.speed > 0)
				motcons[i].cmd.speed-=5;
			else if(motcons[i].cmd.speed < 0)
				motcons[i].cmd.speed+=5;
		}

		kakka = 0;
//		LED_OFF();


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


		buf = o_str_append(buf, "\r\nMC1 head=");
		buf = o_utoa32((motcons[0].status.last_msg>>10)&0b111111, buf);
		buf = o_str_append(buf, " data=");
		buf = o_utoa32(motcons[0].status.last_msg&0x3ff, buf);
		buf = o_str_append(buf, "  MC2 head=");
		buf = o_utoa32((motcons[1].status.last_msg>>10)&0b111111, buf);
		buf = o_str_append(buf, " data=");
		buf = o_utoa32(motcons[1].status.last_msg&0x3ff, buf);
		buf = o_str_append(buf, "  MC3 head=");
		buf = o_utoa32((motcons[2].status.last_msg>>10)&0b111111, buf);
		buf = o_str_append(buf, " data=");
		buf = o_utoa32(motcons[2].status.last_msg&0x3ff, buf);
		buf = o_str_append(buf, "  MC4 head=");
		buf = o_utoa32((motcons[3].status.last_msg>>10)&0b111111, buf);
		buf = o_str_append(buf, " data=");
		buf = o_utoa32(motcons[3].status.last_msg&0x3ff, buf);


/*
		buf = o_str_append(buf, " LIDAR: start=");
		buf = o_utoa16(latest_lidar.start, buf);
		buf = o_str_append(buf, " idx=");
		buf = o_utoa16_fixed(latest_lidar.idx, buf);
		buf = o_str_append(buf, " speed=");
		buf = o_utoa16_fixed(latest_lidar.speed, buf);
		buf = o_str_append(buf, " d0=");
		buf = o_utoa32(latest_lidar.data0, buf);
		buf = o_str_append(buf, " d1=");
		buf = o_utoa32(latest_lidar.data1, buf);
		buf = o_str_append(buf, " d2=");
		buf = o_utoa32(latest_lidar.data2, buf);
		buf = o_str_append(buf, " d3=");
		buf = o_utoa32(latest_lidar.data3, buf);
		buf = o_str_append(buf, " chk=");
		buf = o_utoa16(latest_lidar.checksum, buf);
*/

		buf = o_str_append(buf, "\r\n\r\n");
		usart_print(buffer);



//		SPI1->DR = 11UL<<10 | speed;

	}



}
