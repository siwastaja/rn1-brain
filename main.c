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
#define SPI1_SSI1() {SPI1->CR1 |= 1UL<<8;}
#define SPI1_SSI0() {SPI1->CR1 &= ~(1UL<<8);}
#define MC4_CS1()  {GPIOE->BSRR = 1UL<<6;}
#define MC4_CS0() {GPIOE->BSRR = 1UL<<(6+16);}



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

void uart_rx_handler()
{
	// This SR-then-DR read sequence clears error flags:
	uint32_t status = USART3->SR;
	char byte = USART3->DR;
	if(status & 1UL<<3)
	{
		// Overrun, do something
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
void spi1_inthandler()
{
	// Receive done, slave can be de-selected.
	sr = SPI1->SR;
	MC4_CS1();
	uint16_t msg = SPI1->DR;
	cr1 = SPI1->CR1;
//	if(SPI1->SR & (1<<6))

//	if(!(SPI1->SR & (1<<1)))
	if(msg != 0)
	{
		txnotemptys++;
//		LED_ON();
	}

	last_msg = msg;
}

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
	RCC->APB1ENR |= 1UL<<21 /*I2C1*/ | 1UL<<18 /*USART3*/ | 1UL<<14 /*SPI2*/;
	RCC->APB2ENR |= 1UL<<12 /*SPI1*/;

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
	GPIOB->OSPEEDR = 0b00000000000001010000000000000000;
	GPIOB->OTYPER  = 1UL<<8 | 1UL<<9; // Open drain for I2C.
	             //    15141312111009080706050403020100
	             //     | | | | | | | | | | | | | | | |
	GPIOC->MODER   = 0b00000100101000000000010100000000;
	GPIOC->OSPEEDR = 0b00000000000100000000010100000000;
	             //    15141312111009080706050403020100
	             //     | | | | | | | | | | | | | | | |
	GPIOD->MODER   = 0b00000000000000000000000000000000;
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


	GPIOA->AFR[0] = 5UL<<20 | 5UL<<24 | 5UL<<28; // SPI1 alternate functions.
	GPIOB->AFR[1] = 5UL<<20 | 5UL<<24 | 5UL<<28 /*SPI2*/ |
	                 4UL<<0 | 4UL<<4 /*I2C1*/;
	GPIOC->AFR[1] = 7UL<<8 | 7UL<<12; // USART3 alternate functions.

	// SPI1 @ APB2 = 60 MHz
//	SPI1->CR1 = 1UL<<11 /*16-bit frame*/ | 1UL<<9 /*Software slave management*/ | 1UL<<8 /*SSI bit must be high*/ |
//		0b010UL<<3 /*div 8 = 7.5 MHz*/ | 1UL<<2 /*Master*/;


//	SPI1->CR2 = 1UL<<6 /*RX not empty interrupt*/;

//	SPI1->CR1 |= 1UL<<6; // Enable SPI

	SPI1->CR1 = 1UL<<9 | 1UL<<8 | 0b010UL<<3 | 1UL<<2;
	SPI1->CR1 |= 1UL<<6;

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
	I2C1->CR2 = 0b011110 /*APB1 bus is 30MHz*/;
	I2C1->CCR = 0UL<<15 /*Standard speed*/ | 150UL;
	I2C1->TRISE = 30UL;
	I2C1->CR1 |= 1UL; // Enable I2C

	// USART3 = APB1 = 30 MHz
	// 16x oversampling
	// 115200bps -> Baudrate register = 16.25 = 16 1/4 = 16 4/16
	USART3->BRR = 16UL<<4 | 4UL;
	USART3->CR1 = 1UL<<13 /*USART enable*/ | 1UL<<5 /*RX interrupt*/ | 1UL<<3 /*TX ena*/ | 1UL<<2 /*RX ena*/;

//	NVIC_EnableIRQ(SPI1_IRQn);
//	__enable_irq();
	delay_ms(1);

	usart_print("booty booty\r\n");

	

	LED_OFF();
//	int kakka = 0;
	uint16_t speed = 0;
	while(1)
	{

		char buffer[1000];
		char* buf = buffer;
		LED_ON();
		delay_ms(1000);
		LED_OFF();
		delay_ms(1000);

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
		LED_ON();
		buf = o_str_append(buf, "gyro_reply=");
		buf = o_utoa32(reply, buf);
		buf = o_str_append(buf, "\r\n");
		usart_print(buffer);

/*		SPI1->DR = 123;
		while(!(SPI1->SR & 1)) ;
		buf = o_str_append(buf, " data read=");
		buf = o_utoa32(SPI1->DR, buf);
		buf = o_str_append(buf, "\r\n");
		usart_print(buffer);
*/


/*		buf = o_str_append(buf, " sr now=");
		buf = o_utoa32(SPI1->SR, buf);
		buf = o_str_append(buf, " sr was=");
		buf = o_utoa32(sr, buf);
		buf = o_str_append(buf, " cr1 was=");
		buf = o_utoa32(cr1, buf);
		buf = o_str_append(buf, " ovrs=");
		buf = o_utoa32(ovrs, buf);
		buf = o_str_append(buf, " txnotemptys=");
		buf = o_utoa32(txnotemptys, buf);
		buf = o_str_append(buf, " last_msg=");
		buf = o_utoa16(last_msg, buf);
		buf = o_str_append(buf, "\r\n");
		usart_print(buffer);
*/

/*		kakka++;
		if(kakka<1000000)
			continue;
		else
			kakka=0;
*/
		MC4_CS0();
//		SPI1_SSI0();
//		SPI1->DR = 12345; // 11UL<<10 | speed;
//		while(!(SPI1->SR & 2)) ;
//		SPI1->DR = 23456;

		speed += 20;

		if(speed > 300)
			speed = 0;

		CHARGER_ENA();
//		PSU5V_ENA();

	}



}
