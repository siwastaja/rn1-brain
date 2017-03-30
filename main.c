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

#define FLOW_CS1() {GPIOB->BSRR = 1UL<<12;}
#define FLOW_CS0() {GPIOB->BSRR = 1UL<<(12+16);}


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
//	LED_ON();
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


#define OPTFLOW_POLL_RATE 100 // unit: 100us, must be at least 4

typedef struct __attribute__ ((__packed__))
{
	uint8_t dummy;
	uint8_t motion;
	int8_t  dx;
	int8_t  dy;
	uint8_t squal;
	uint8_t shutter_msb;
	uint8_t shutter_lsb;
	uint8_t max_pixel;
} optflow_data_t;

volatile optflow_data_t latest_optflow;

volatile int num_optflows;

// Run this at 10kHz
void optflow_fsm()
{
	static int cycle = 0;
	if(cycle == 0)
	{
		FLOW_CS0();
	}
	else if(cycle == 1)
	{
		DMA1->LIFCR = 0b111101UL<<22; DMA1_Stream3->CR |= 1UL; // Enable RX DMA
		SPI2->DR = 0x50; // motion_burst read
	}
	else if(cycle == 2)
	{
		DMA1->HIFCR = 0b111101UL; DMA1_Stream4->CR |= 1UL; // Enable TX DMA to send dummy data
	}
	else if(cycle >= OPTFLOW_POLL_RATE)
	{
		FLOW_CS1();
		if(DMA1_Stream3->CR & 1UL) // Error: RX DMA stream still on (not finished)
			LED_ON();
		cycle = -1;
		num_optflows++;
	}

	cycle++;
}

volatile int cnt_10k;
void timebase_10k_handler()
{
	TIM6->SR = 0;
	cnt_10k++;

	optflow_fsm();
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

typedef struct
{
	uint8_t status_reg;
	int16_t x;
	int16_t y;
	int16_t z;
} gyro_data_t;

volatile gyro_data_t latest_gyro;

typedef struct
{
	uint8_t status_reg;
	int16_t x;
	int16_t y;
	int16_t z;
} xcel_data_t;

volatile xcel_data_t latest_xcel;

typedef struct
{
	uint8_t status_reg;
	int16_t x;
	int16_t y;
	int16_t z;
} compass_data_t;

volatile uint8_t x1, x2;

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
			I2C1->DR = 0x00; // Status register address
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
			I2C1->CR1 |= 1UL<<10; // Generate ACK
			state++;
		}
		break;

		case 4:
		if(I2C1->SR1 & (1UL<<6))
		{
			latest_gyro.status_reg = I2C1->DR;
			I2C1->CR1 |= 1UL<<10; // Generate ACK
			state++;
		}
		break;

		case 5:
		if(I2C1->SR1 & (1UL<<6))
		{
			latest_gyro.x = (I2C1->DR)<<8;
			I2C1->CR1 |= 1UL<<10; // Generate ACK
			state++;
		}
		break;

		case 6:
		if(I2C1->SR1 & (1UL<<6))
		{
			latest_gyro.x |= (I2C1->DR);
			I2C1->CR1 |= 1UL<<10; // Generate ACK
			state++;
		}
		break;

		case 7:
		if(I2C1->SR1 & (1UL<<6))
		{
			latest_gyro.y = (I2C1->DR)<<8;
			I2C1->CR1 |= 1UL<<10; // Generate ACK
			state++;
		}
		break;

		case 8:
		if(I2C1->SR1 & (1UL<<6))
		{
			latest_gyro.y |= (I2C1->DR);
			I2C1->CR1 |= 1UL<<10; // Generate ACK
			state++;
		}
		break;

		case 9:
		if(I2C1->SR1 & (1UL<<6))
		{
			latest_gyro.z = (I2C1->DR)<<8;
			I2C1->CR1 &= ~(1UL<<10); // Zero ACK to generate NACK (for the last data)
			I2C1->CR1 |= 1UL<<9; // generate STOP
			state++;
		}
		break;

		case 10:
		if(I2C1->SR1 & (1UL<<6))
		{
			latest_gyro.z |= (I2C1->DR);
			I2C1->CR1 |= 1UL<<8; // Generate start
			state++;
		}
		break;




		case 11:
		if(I2C1->SR1 & 1) // SB = Start Generated
		{
			I2C1->DR = 0x3A;
			state++;
		}
		break;

		case 12:
		if(I2C1->SR1 & 2) // ADDR = Address sent
		{
			dummy = I2C1->SR2;
			I2C1->DR = 0x27 | 0x80; // Status register w/ autoincr
			I2C1->CR1 |= 1UL<<8; // START
			state++;
		}

		case 13:
		if(I2C1->SR1 & 1) // SB = Start Generated
		{
			I2C1->DR = 0x3B;
			state++;
		}
		break;

		case 14:
		if(I2C1->SR1 & 2) // ADDR = Address sent
		{
			dummy = I2C1->SR2;
			I2C1->CR1 |= 1UL<<10; // Generate ACK
			state++;
		}
		break;

		case 15:
		if(I2C1->SR1 & (1UL<<6))
		{
			latest_xcel.status_reg = I2C1->DR;
			I2C1->CR1 |= 1UL<<10; // Generate ACK
			state++;
		}
		break;

		case 16:
		if(I2C1->SR1 & (1UL<<6))
		{
			latest_xcel.x = (I2C1->DR);
			I2C1->CR1 |= 1UL<<10; // Generate ACK
			state++;
		}
		break;

		case 17:
		if(I2C1->SR1 & (1UL<<6))
		{
			latest_xcel.x |= (I2C1->DR)<<8;
			I2C1->CR1 |= 1UL<<10; // Generate ACK
			state++;
		}
		break;

		case 18:
		if(I2C1->SR1 & (1UL<<6))
		{
			latest_xcel.y = (I2C1->DR);
			I2C1->CR1 |= 1UL<<10; // Generate ACK
			state++;
		}
		break;

		case 19:
		if(I2C1->SR1 & (1UL<<6))
		{
			latest_xcel.y |= (I2C1->DR)<<8;
			I2C1->CR1 |= 1UL<<10; // Generate ACK
			state++;
		}
		break;

		case 20:
		if(I2C1->SR1 & (1UL<<6))
		{
			latest_xcel.z = (I2C1->DR);
			I2C1->CR1 &= ~(1UL<<10); // Zero ACK to generate NACK (for the last data)
			I2C1->CR1 |= 1UL<<9; // generate STOP
			state++;
		}
		break;

		case 21:
		if(I2C1->SR1 & (1UL<<6))
		{
			latest_xcel.z |= (I2C1->DR)<<8;
			I2C1->CR1 |= 1UL<<8; // Generate start
			state++;
		}
		break;




		case 22:
		if(I2C1->SR1 & 1) // SB = Start Generated
		{
			I2C1->DR = 0x3C;
			state++;
		}
		break;

		case 23:
		if(I2C1->SR1 & 2) // ADDR = Address sent
		{
			dummy = I2C1->SR2;
			I2C1->DR = 0x27; // Status register address
			I2C1->CR1 |= 1UL<<8; // START
			state++;
		}

		case 24:
		if(I2C1->SR1 & 1) // SB = Start Generated
		{
			I2C1->DR = 0x3D;
			state++;
		}
		break;

		case 25:
		if(I2C1->SR1 & 2) // ADDR = Address sent
		{
			dummy = I2C1->SR2;
			I2C1->CR1 |= 1UL<<10; // Generate ACK
			state++;
		}
		break;

		case 26:
		if(I2C1->SR1 & (1UL<<6))
		{
			latest_compass.status_reg = I2C1->DR;
			I2C1->CR1 |= 1UL<<10; // Generate ACK
			state++;
		}
		break;

		case 27:
		if(I2C1->SR1 & (1UL<<6))
		{
			latest_compass.x = (I2C1->DR);
			I2C1->CR1 |= 1UL<<10; // Generate ACK
			state++;
		}
		break;

		case 28:
		if(I2C1->SR1 & (1UL<<6))
		{
			latest_compass.x |= (I2C1->DR)<<8;
			I2C1->CR1 |= 1UL<<10; // Generate ACK
			state++;
		}
		break;

		case 29:
		if(I2C1->SR1 & (1UL<<6))
		{
			latest_compass.y = (I2C1->DR);
			I2C1->CR1 |= 1UL<<10; // Generate ACK
			state++;
		}
		break;

		case 30:
		if(I2C1->SR1 & (1UL<<6))
		{
			latest_compass.y |= (I2C1->DR)<<8;
			I2C1->CR1 |= 1UL<<10; // Generate ACK
			state++;
		}
		break;

		case 31:
		if(I2C1->SR1 & (1UL<<6))
		{
			latest_compass.z = (I2C1->DR);
			I2C1->CR1 &= ~(1UL<<10); // Zero ACK to generate NACK (for the last data)
			I2C1->CR1 |= 1UL<<9; // generate STOP
			state++;
		}
		break;

		case 32:
		if(I2C1->SR1 & (1UL<<6))
		{
			latest_compass.z |= (I2C1->DR)<<8;
			state=0;
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

int init_i2c1_devices()
{
	uint32_t dummy;
	// Init gyro

	I2C1->CR1 |= 1UL<<8; // START
	while(!(I2C1->SR1 & 1)) ; // Wait for SB (Start Generated)
	I2C1->DR = 0x40;
	while(!(I2C1->SR1 & 2)) ; // Wait for ADDR (Address sent)
	dummy = I2C1->SR2;
	while(!(I2C1->SR1 & 1UL<<7)) ;
	I2C1->DR = 0x13;
	while(!(I2C1->SR1 & 1UL<<7)) ;
	I2C1->DR = 0b10; // Go to ACTIVE mode
	while(!(I2C1->SR1 & 1UL<<7)) ;
	I2C1->CR1 |= 1UL<<9; // STOP
	while(!(I2C1->SR1 & 1UL<<2)) ; // Wait for BYTE TRANSFER FINISHED

	// Init Accel

	I2C1->CR1 |= 1UL<<8; // START
	while(!(I2C1->SR1 & 1)) ; // Wait for SB (Start Generated)
	I2C1->DR = 0x3A;
	while(!(I2C1->SR1 & 2)) ; // Wait for ADDR (Address sent)
	dummy = I2C1->SR2;
	while(!(I2C1->SR1 & 1UL<<7)) ;
	I2C1->DR = 0x20; // CTRL_REG1_A
	while(!(I2C1->SR1 & 1UL<<7)) ;
	I2C1->DR = 0b101<<4 /*400Hz*/ | 1<<3 /*Must be set*/ | 0b111 /*Z,Y,X ena*/;
	while(!(I2C1->SR1 & 1UL<<7)) ;
	I2C1->CR1 |= 1UL<<9; // STOP
	while(!(I2C1->SR1 & 1UL<<2)) ; // Wait for BYTE TRANSFER FINISHED


	// Init Compass

	I2C1->CR1 |= 1UL<<8; // START
	while(!(I2C1->SR1 & 1)) ; // Wait for SB (Start Generated)
	I2C1->DR = 0x3C;
	while(!(I2C1->SR1 & 2)) ; // Wait for ADDR (Address sent)
	dummy = I2C1->SR2;
	while(!(I2C1->SR1 & 1UL<<7)) ;
	I2C1->DR = 0x20; // CTRL_REG1_M
	while(!(I2C1->SR1 & 1UL<<7)) ;
	I2C1->DR = 0b11<<5 /*Ultra-high performance mode*/ | 0b101<<2 /*20Hz*/ | 0 /* SELF TEST*/;
	while(!(I2C1->SR1 & 1UL<<7)) ;
	I2C1->CR1 |= 1UL<<9; // STOP
	while(!(I2C1->SR1 & 1UL<<2)) ; // Wait for BYTE TRANSFER FINISHED

	I2C1->CR1 |= 1UL<<8; // START
	while(!(I2C1->SR1 & 1)) ; // Wait for SB (Start Generated)
	I2C1->DR = 0x3C;
	while(!(I2C1->SR1 & 2)) ; // Wait for ADDR (Address sent)
	dummy = I2C1->SR2;
	while(!(I2C1->SR1 & 1UL<<7)) ;
	I2C1->DR = 0x21; // CTRL_REG2_M
	while(!(I2C1->SR1 & 1UL<<7)) ;
	I2C1->DR = 0b11<<5 /*must be set*/;
	while(!(I2C1->SR1 & 1UL<<7)) ;
	I2C1->CR1 |= 1UL<<9; // STOP
	while(!(I2C1->SR1 & 1UL<<2)) ; // Wait for BYTE TRANSFER FINISHED

	I2C1->CR1 |= 1UL<<8; // START
	while(!(I2C1->SR1 & 1)) ; // Wait for SB (Start Generated)
	I2C1->DR = 0x3C;
	while(!(I2C1->SR1 & 2)) ; // Wait for ADDR (Address sent)
	dummy = I2C1->SR2;
	while(!(I2C1->SR1 & 1UL<<7)) ;
	I2C1->DR = 0x22; // CTRL_REG3_M
	while(!(I2C1->SR1 & 1UL<<7)) ;
	I2C1->DR = 0b00<0 /*continuous*/;
	while(!(I2C1->SR1 & 1UL<<7)) ;
	I2C1->CR1 |= 1UL<<9; // STOP
	while(!(I2C1->SR1 & 1UL<<2)) ; // Wait for BYTE TRANSFER FINISHED

	I2C1->CR1 |= 1UL<<8; // START
	while(!(I2C1->SR1 & 1)) ; // Wait for SB (Start Generated)
	I2C1->DR = 0x3C;
	while(!(I2C1->SR1 & 2)) ; // Wait for ADDR (Address sent)
	dummy = I2C1->SR2;
	while(!(I2C1->SR1 & 1UL<<7)) ;
	I2C1->DR = 0x23; // CTRL_REG4_M
	while(!(I2C1->SR1 & 1UL<<7)) ;
	I2C1->DR = 0b11<2 /*Z in ultra-high, too*/;
	while(!(I2C1->SR1 & 1UL<<7)) ;
	I2C1->CR1 |= 1UL<<9; // STOP
	while(!(I2C1->SR1 & 1UL<<2)) ; // Wait for BYTE TRANSFER FINISHED

	I2C1->CR1 |= 1UL<<8; // START
	while(!(I2C1->SR1 & 1)) ; // Wait for SB (Start Generated)
	I2C1->DR = 0x3C;
	while(!(I2C1->SR1 & 2)) ; // Wait for ADDR (Address sent)
	dummy = I2C1->SR2;
	while(!(I2C1->SR1 & 1UL<<7)) ;
	I2C1->DR = 0x24; // CTRL_REG5_M
	while(!(I2C1->SR1 & 1UL<<7)) ;
	I2C1->DR = 1<<6 /*"block data update"; must be set*/;
	while(!(I2C1->SR1 & 1UL<<7)) ;
	I2C1->CR1 |= 1UL<<9; // STOP
	while(!(I2C1->SR1 & 1UL<<2)) ; // Wait for BYTE TRANSFER FINISHED


	I2C1->SR1 = 0; // Zero regs.

	return 0;

/*	I2C1->CR1 |= 1UL<<8; // START
	while(!(I2C1->SR1 & 1)) ; // Wait for SB (Start Generated)
	I2C1->DR = 0x40;
	while(!(I2C1->SR1 & 2)) ; // Wait for ADDR (Address sent)
	uint32_t dummy = I2C1->SR2;
	I2C1->DR = 0x13;
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
}

int dummy_data = 42;

int main()
{
	int i, dummy;

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


	// SPI1 @ APB2 = 60 MHz
	SPI1->CR1 = 1UL<<11 /*16-bit frame*/ | 1UL<<9 /*Software slave management*/ | 1UL<<8 /*SSI bit must be high*/ |
		0b010UL<<3 /*div 8 = 7.5 MHz*/ | 1UL<<2 /*Master*/;

	SPI1->CR2 = 1UL<<6 /*RX not empty interrupt*/;

	SPI1->CR1 |= 1UL<<6; // Enable SPI

	// SPI2 @ APB1 = 30 MHz
	// ADNS3080 optical flow sensor
	// Frame rate 2000..6469 fps
	// Time between write commands min 50 us
	// Between write and read 50 us
	// After read 250 us
	// Time after address: 50 us (for motion+motion burst: 75 us)
	// 500 ns min period -> 2 MHz max
	// SPI2 RX is mapped to DMA1, Stream3, Ch0

	// DMA1 STREAM 3 = flow RX
	DMA1_Stream3->PAR = (uint32_t)&(SPI2->DR);
	DMA1_Stream3->M0AR = (uint32_t)(&latest_optflow);
	DMA1_Stream3->NDTR = 8;
	DMA1_Stream3->CR = 0UL<<25 /*Channel*/ | 0b01UL<<16 /*med prio*/ | 0b00UL<<13 /*8-bit mem*/ | 0b00UL<<11 /*8-bit periph*/ |
	                   1UL<<10 /*mem increment*/;

	// DMA1 STREAM 4 = flow TX (dummy bytes)
	DMA1_Stream4->PAR = (uint32_t)&(SPI2->DR);
	DMA1_Stream4->M0AR = (uint32_t)(&dummy_data);
	DMA1_Stream4->NDTR = 7;
	DMA1_Stream4->CR = 0UL<<25 /*Channel*/ | 0b01UL<<16 /*med prio*/ | 0b00UL<<13 /*8-bit mem*/ | 0b00UL<<11 /*8-bit periph*/ |
	                   0b01<<6 /*mem->periph*/;


	SPI2->CR1 = 0UL<<11 /*8-bit frame*/ | 1UL<<9 /*Software slave management*/ | 1UL<<8 /*SSI bit must be high*/ |
		0b011UL<<3 /*div 16 = 1.875 MHz*/ | 1UL<<2 /*Master*/;
	SPI2->CR2 = 1UL<<1 /* TX DMA enable */ | 1UL<<0 /* RX DMA enable*/;

	SPI2->CR1 |= 1UL<<6; // Enable SPI


	MC4_CS1();

	/*
		I2C1 @ APB1 at 30MHz
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

	/*
		TIM6 @ APB1 at 30MHz, but the counter runs at x2 = 60MHz
		Create 10 kHz timebase
	*/

	TIM6->DIER |= 1UL; // Update interrupt
	TIM6->ARR = 6000; // 60MHz -> 10 kHz
	TIM6->CR1 |= 1UL; // Enable


	FLOW_CS1();

//	NVIC_EnableIRQ(SPI1_IRQn);
//	NVIC_EnableIRQ(USART3_IRQn);
	NVIC_EnableIRQ(TIM6_DAC_IRQn);
	__enable_irq();
	delay_ms(1000);

	usart_print("booty booty\r\n");

//	init_i2c1_devices();

	delay_ms(100);

//	NVIC_EnableIRQ(I2C1_EV_IRQn);



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
	uint16_t speed = 0;

	while(1)
	{
		char buffer[1000];
		char* buf = buffer;
		buf = o_str_append(buf, " num=");
		buf = o_utoa16_fixed(num_optflows, buf);
		buf = o_str_append(buf, "  flow: ");
		buf = o_utoa16_fixed(latest_optflow.motion, buf);
		buf = o_str_append(buf, "  X ");
		buf = o_itoa16_fixed(latest_optflow.dx, buf);
		buf = o_str_append(buf, "  Y ");
		buf = o_itoa16_fixed(latest_optflow.dy, buf);
		buf = o_str_append(buf, "  ");
		buf = o_utoa16_fixed(latest_optflow.squal, buf);
		buf = o_str_append(buf, "  ");
		buf = o_utoa16_fixed(latest_optflow.shutter_msb<<8 | latest_optflow.shutter_lsb, buf);
		buf = o_str_append(buf, "  ");
		buf = o_utoa16_fixed(latest_optflow.max_pixel, buf);
		buf = o_str_append(buf, "\r\n");
		usart_print(buffer);

		delay_ms(200);
	}

/*
	while(1)
	{
		char buffer[1000];
		char* buf = buffer;
		uint8_t data[7];
		FLOW_CS0();
		delay_us(100);
		SPI2->DR = 0x50; // motion_burst read
		while(!(SPI2->SR & 1));
		dummy = SPI2->DR;
		delay_us(150);
		for(i = 0; i < 7; i++)
		{
			SPI2->DR = 0; // dummy write
			while(!(SPI2->SR & 1));
			data[i] = SPI2->DR;
		}
		delay_us(100);
		FLOW_CS1();


		buf = o_str_append(buf, " flow: ");
		buf = o_utoa16_fixed(data[0], buf);
		buf = o_str_append(buf, "  X ");
		buf = o_itoa16_fixed((int8_t)data[1], buf);
		buf = o_str_append(buf, "  Y ");
		buf = o_itoa16_fixed((int8_t)data[2], buf);
		buf = o_str_append(buf, "  ");
		buf = o_utoa16_fixed(data[3], buf);
		buf = o_str_append(buf, "  ");
		buf = o_utoa16_fixed(data[4], buf);
		buf = o_str_append(buf, "  ");
		buf = o_utoa16_fixed(data[5], buf);
		buf = o_str_append(buf, "  ");
		buf = o_utoa16_fixed(data[6], buf);
		buf = o_str_append(buf, "  ");
		buf = o_str_append(buf, "\r\n");
		usart_print(buffer);

		delay_ms(200);
	}
*/

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

		if(kakka==100)
		{
			start_i2c1_sequence();
		}

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


/*		buf = o_str_append(buf, " gyro=");
		buf = o_utoa16_fixed(latest_gyro.status_reg, buf);
		buf = o_str_append(buf, "  ");
		buf = o_itoa16_fixed(latest_gyro.x, buf);
		buf = o_str_append(buf, ", ");
		buf = o_itoa16_fixed(latest_gyro.y, buf);
		buf = o_str_append(buf, ", ");
		buf = o_itoa16_fixed(latest_gyro.z, buf);


		buf = o_str_append(buf, " xcel=");
		buf = o_utoa16_fixed(latest_xcel.status_reg, buf);
		buf = o_str_append(buf, "  ");
		buf = o_itoa16_fixed(latest_xcel.x, buf);
		buf = o_str_append(buf, ", ");
		buf = o_itoa16_fixed(latest_xcel.y, buf);
		buf = o_str_append(buf, ", ");
		buf = o_itoa16_fixed(latest_xcel.z, buf);

		buf = o_str_append(buf, " compass=");
		buf = o_utoa16_fixed(latest_compass.status_reg, buf);
		buf = o_str_append(buf, "  ");
		buf = o_itoa16_fixed(latest_compass.x, buf);
		buf = o_str_append(buf, ", ");
		buf = o_itoa16_fixed(latest_compass.y, buf);
		buf = o_str_append(buf, ", ");
		buf = o_itoa16_fixed(latest_compass.z, buf);

*/
/*
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
*/

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

		buf = o_str_append(buf, "\r\n");
		usart_print(buffer);



		SPI1->DR = 11UL<<10 | speed;

	}



}
