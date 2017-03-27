#include <stdio.h>
#include <stdint.h>
#include <string.h>

#include "main.h"
#include "ext_include/stm32f2xx.h"
#include "own_std.h"

#define LED_ON()  {GPIOC->BSRR = 1UL<<13;}
#define LED_OFF() {GPIOC->BSRR = 1UL<<(13+16);}
#define CHARGER_ENA() {GPIOA->BSRR = 1UL<<15;}
#define CHARGER_DIS() {GPIOA->BSRR = 1UL<<(15+16);}
#define PSU5V_ENA() {GPIOE->BSRR = 1UL<<15;}
#define PSU5V_DIS() {GPIOE->BSRR = 1UL<<(15+16);}
#define MC4_CS1()  {GPIOE->BSRR = 1UL<<6;}
#define MC4_CS0() {GPIOE->BSRR = 1UL<<(6+16);}
#define MC3_CS1()  {GPIOA->BSRR = 1UL<<4;}
#define MC3_CS0() {GPIOA->BSRR = 1UL<<(4+16);}
#define MC2_CS1()  {GPIOC->BSRR = 1UL<<4;}
#define MC2_CS0() {GPIOC->BSRR = 1UL<<(4+16);}
#define MC1_CS1()  {GPIOC->BSRR = 1UL<<5;}
#define MC1_CS0() {GPIOC->BSRR = 1UL<<(5+16);}
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

volatile int ovrs = 0;
volatile int txnotemptys = 0;
volatile int sr = 0;
volatile int cr1 = 0;

volatile uint16_t last_msg;

typedef struct
{
	int last_msg;
	int16_t cur_b;
	int16_t cur_c;
} motcon_status_t;

typedef struct
{
	int16_t speed;
	int16_t cur_limit;
} motcon_cmd_t;

typedef struct
{
	motcon_status_t status;
	motcon_cmd_t cmd;
} motcon_t;

#define NUM_MOTCONS 4
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



volatile int cur_motcon = 0;
void spi1_inthandler()
{
	// Receive done, slave can be de-selected.
	switch(cur_motcon)
	{
		case 0: {MC1_CS1();} break;
		case 1: {MC2_CS1();} break;
		case 2: {MC3_CS1();} break;
		case 3: {MC4_CS1();} break;
		default: break;
	}
	uint16_t msg = SPI1->DR;

	motcons[cur_motcon].status.last_msg = msg;

	if(cur_motcon == NUM_MOTCONS-1)
		cur_motcon = 0;
	else
		cur_motcon++;
}


/*
	STM32 I2C implementation is a total catastrophe, it requires almost bitbanging-like sequencing by software,
	and provides no proper buffer registers (let alone a FIFO) - so we need to poll some status bits and write
	the data register at the exactly correct timing!

	For maximum performance, we hardwrite the optimal sequence of reading the relevant data from the
	gyro, accelerometer and compass. Because using the I2C implementation requires a lot of waiting
	for status bits, DMA is of little use; we use an interrupt handler implementing a state machine.
*/
volatile int i2c1_ready = 1;
volatile int latest_gyro = 0;
void i2c1_inthandler()
{
	static int state = 0;

	uint32_t dummy;

	switch(state)
	{
		case 0:
		if(I2C1->SR1 & 1) // SB = Start Generated
		{
			I2C1->DR = 0x40;
			state++;
		}
		break;

		case 1:
		if(I2C1->SR1 & 2) // ADDR = Address sent
		{
			dummy = I2C1->SR2;
			I2C1->DR = 0x0C;
			I2C1->CR1 |= 1UL<<8; // START
			state++;
		}

		case 2:
		if(I2C1->SR1 & 1) // SB = Start Generated
		{
			I2C1->DR = 0x41;
			state++;
		}
		break;

		case 3:
		if(I2C1->SR1 & 2) // ADDR = Address sent
		{
			dummy = I2C1->SR2;
			I2C1->CR1 &= ~(1UL<<10); // Zero ACK to generate NACK (for last data)
			I2C1->CR1 |= 1UL<<9; // STOP
			state++;
		}
		break;

		case 4:
		if(I2C1->SR1 & (1UL<<6))
		{
			latest_gyro = I2C1->DR;
			state = 0;
		}
		break;
		default:
		break;
	}

}

int start_i2c1_sequence()
{
	if(!i2c1_ready)
		return -1;

	I2C1->CR1 |= 1UL<<8; // Instruct the START. The interrupt handler will take over.
		return 0;
}
/*
		// FXAS:
		// Single byte read:
		// Start, Write 0x40, get ACK, write regaddr, get ACK.
		// StartRepeated, Write 0x41, get ack, get data. Send NACK(yes!) and stop.
		// RegAddr 0x0C WHO_AM_I should return 0xD7
		I2C1->CR1 |= 1UL<<8; // START
		while(!(I2C1->SR1 & 1)) ; // Wait for SB (Start Generated)
		I2C1->DR = 0x40;
		while(!(I2C1->SR1 & 2)) ; // Wait for ADDR (Address sent)
		uint32_t dummy = I2C1->SR2;
		I2C1->DR = 0x0C;
		I2C1->CR1 |= 1UL<<8; // START
		while(!(I2C1->SR1 & 1)) ; // Wait for SB (Start Generated)
		I2C1->DR = 0x41;
		while(!(I2C1->SR1 & 2)) ; // Wait for ADDR (Address sent)
		dummy = I2C1->SR2;
		I2C1->CR1 &= ~(1UL<<10); // Zero ACK to generate NACK (for last data)
		I2C1->CR1 |= 1UL<<9; // STOP
		while(!(I2C1->SR1 & (1UL<<6))) ; // Wait for RxNE
		uint32_t reply = I2C1->DR;
*/

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


	RCC->AHB1ENR |= 0b111111111 /* PORTA to PORTI */;
	RCC->APB1ENR |= 1UL<<21 /*I2C1*/ | 1UL<<18 /*USART3*/ | 1UL<<14 /*SPI2*/ | 1UL<<2 /*TIM4*/;
	RCC->APB2ENR |= 1UL<<12 /*SPI1*/;

	delay_us(100);

	GPIOA->AFR[0] = 5UL<<20 | 5UL<<24 | 5UL<<28 /*SPI1*/;
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
	GPIOB->MODER   = 0b10101000000010100000000000000000;
	GPIOB->OSPEEDR = 0b00000000000001010000010001000000;
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


	// SPI1 @ APB2 = 60 MHz
	SPI1->CR1 = 1UL<<11 /*16-bit frame*/ | 1UL<<9 /*Software slave management*/ | 1UL<<8 /*SSI bit must be high*/ |
		0b010UL<<3 /*div 8 = 7.5 MHz*/ | 1UL<<2 /*Master*/;

	SPI1->CR2 = 1UL<<6 /*RX not empty interrupt*/;

	SPI1->CR1 |= 1UL<<6; // Enable SPI

	MC4_CS1();

	/*
		APB1 at 30MHz
		"Tpclk1" = 1/30MHz = 0.03333333us
		100kHz standard mode:
		Thigh = Tlow = 5us
		-> CCR = 5us/0.0333333us = 150

		TRISE: for standard mode,
		1us / 0.0333333us = 30 -> TRISE = 31
	*/
	I2C1->CR2 = 0b011110 /*APB1 bus is 30MHz*/ | 1UL<<10 /*Buffer IE*/ | 1UL<<9 /*Event IE*/;
	I2C1->CCR = 0UL<<15 /*Standard speed*/ | 150UL;
	I2C1->TRISE = 30UL;
	I2C1->CR1 |= 1UL; // Enable I2C

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

//	NVIC_EnableIRQ(I2C1_EV_IRQn);
	NVIC_EnableIRQ(SPI1_IRQn);
	NVIC_EnableIRQ(USART3_IRQn);
	__enable_irq();
	delay_ms(1);

	usart_print("booty booty\r\n");

	PSU12V_ENA();
	LED_OFF();
	int kakka = 0;
	uint16_t speed = 0;

	while(1)
	{
		char buffer[1000];
		char* buf = buffer;

		delay_ms(1);

		switch(cur_motcon)
		{
			case 0: {MC1_CS0();} break;
			case 1: {MC2_CS0();} break;
			case 2: {MC3_CS0();} break;
			case 3: {MC4_CS0();} break;
			default: break;
		}

		if(motcons[cur_motcon].cmd.speed >= 0)
			SPI1->DR = 11UL<<10 | (motcons[cur_motcon].cmd.speed & 0x3FF);
		else
			SPI1->DR = 12UL<<10 | ((motcons[cur_motcon].cmd.speed*-1) & 0x3FF);

		kakka++;
		if(kakka<100)
			continue;

		for(i=2; i<4; i++)
		{
			if(motcons[i].cmd.speed > 0)
				motcons[i].cmd.speed-=5;
			else if(motcons[i].cmd.speed < 0)
				motcons[i].cmd.speed+=5;
		}

		kakka = 0;
		LED_OFF();
//		buf = o_str_append(buf, " gyro_reply=");
//		buf = o_utoa32(latest_gyro, buf);
//		buf = o_str_append(buf, " last_msg=");
//		buf = o_utoa32(last_msg, buf);
		buf = o_str_append(buf, " MC1 head=");
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
		buf = o_str_append(buf, "\r\n");
		usart_print(buffer);

//		start_i2c1_sequence();


		SPI1->DR = 11UL<<10 | speed;

		CHARGER_ENA();
//		PSU5V_ENA();

	}



}
