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

#define BINARY_OUTPUT

#define LED_ON()  {GPIOC->BSRR = 1UL<<13;}
#define LED_OFF() {GPIOC->BSRR = 1UL<<(13+16);}
#define CHARGER_ENA() {GPIOA->BSRR = 1UL<<15;}
#define CHARGER_DIS() {GPIOA->BSRR = 1UL<<(15+16);}
#define PSU5V_ENA() {GPIOE->BSRR = 1UL<<15;}
#define PSU5V_DIS() {GPIOE->BSRR = 1UL<<(15+16);}
#define PSU12V_ENA() {GPIOD->BSRR = 1UL<<4;}
#define PSU12V_DIS() {GPIOD->BSRR = 1UL<<(4+16);}
#define CHA_RUNNING() (!(GPIOB->IDR & (1<<11)))
#define CHA_FINISHED() (!(GPIOB->IDR & (1<<10)))

volatile int dbg[10];

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

void usart_send(const uint8_t *buf, int len)
{
	while(len--)
	{
		while((USART3->SR & (1UL<<7)) == 0) ;
		USART3->DR = buf[0];
		buf++;
	}
}


void run_flasher()
{
	__disable_irq();
	/*
		Reconfigure UART to 115200, no interrupts. Flasher uses polling, and we want to make sure it
		starts from a clean table.
	*/
	USART3->CR1 = 0; // Disable
	delay_us(10);
	USART3->SR = 0; // Clear flags
	USART3->BRR = 16UL<<4 | 4UL; // 115200
	USART3->CR1 = 1UL<<13 /*USART enable*/ | 1UL<<3 /*TX ena*/ | 1UL<<2 /*RX ena*/;
	delay_us(10);
	flasher();
	while(1);
}

int mc_flasher_num;

void mc_flasher_cs1()
{
	switch(mc_flasher_num)
	{
		case 1: MC1_CS1(); break;
		case 2: MC2_CS1(); break;
		case 3: MC3_CS1(); break;
		case 4: MC4_CS1(); break;
		default: LED_ON(); while(1);
	}
}

void mc_flasher_cs0()
{
	switch(mc_flasher_num)
	{
		case 1: MC1_CS0(); break;
		case 2: MC2_CS0(); break;
		case 3: MC3_CS0(); break;
		case 4: MC4_CS0(); break;
		default: LED_ON(); while(1);
	}
}

// Blocks until free space in the SPI TX FIFO
void spi1_poll_tx(uint16_t d)
{
	while(!(SPI1->SR & (1UL<<1))) ;
	SPI1->DR = d;
}

// Blocks until SPI TX FIFO empty and not busy.

void spi1_poll_tx_bsy(uint16_t d)
{
	while(!(SPI1->SR & (1UL<<1))) ;
	SPI1->DR = d;
	while(SPI1->SR&(0b11<<11)); // Wait for TX fifo empty
	while(SPI1->SR&(1<<7)) ; // Wait until not busy.
	delay_us(1);
}

void spi1_poll_bsy()
{
	while(SPI1->SR&(0b11<<11)); // Wait for TX fifo empty
	while(SPI1->SR&(1<<7)) ; // Wait until not busy.
	delay_us(1);
}

// Blocks until data available in SPI RX FIFO - so indefinitely unless you have issued a TX just before.
uint16_t spi1_poll_rx()
{
	while(!(SPI1->SR & (1UL<<0))) ;
	return SPI1->DR;
}

// Empties the rx fifo
void spi1_empty_rx()
{
	while(SPI1->SR&(0b11<<9)) SPI1->DR;
}

void usart3_poll_tx(uint8_t d) // blocks until current byte tx finished
{
	while(!(USART3->SR & (1UL<<7))) ;
	USART3->DR = d;
}

uint8_t usart3_poll_rx() // blocks until byte received
{
	while(!(USART3->SR & (1UL<<5))) ;
	return USART3->DR;
}

#define ETERNAL_BLINK() {while(1) {LED_ON(); delay_ms(50); LED_OFF(); delay_ms(50);}}

void mc_flasher(int mcnum)
{
	__disable_irq();
	if(mcnum < 1 || mcnum > 4)
	{
		ETERNAL_BLINK();
	}

	mc_flasher_num = mcnum;
	LED_OFF();
	/*
		Reconfigure UART to 115200, no interrupts. Flasher uses polling, and we want to make sure it
		starts from a clean table.
	*/
	USART3->CR1 = 0; // Disable
	delay_us(10);
	USART3->SR = 0; // Clear flags
	USART3->BRR = 16UL<<4 | 4UL; // 115200
	USART3->CR1 = 1UL<<13 /*USART enable*/ | 1UL<<3 /*TX ena*/ | 1UL<<2 /*RX ena*/;
	delay_us(10);

	/*
		Reconfigure SPI1 for polling, no interrupts.
	*/

	while(SPI1->SR&(0b11<<11)); // Wait for TX fifo empty
	while(SPI1->SR&(1<<7)) ; // Wait until not busy.
	MC4_CS1();
	MC3_CS1();
	MC2_CS1();
	MC1_CS1();

	delay_us(10);

	// disable DMA
	DMA2->LIFCR = 0b111101UL<<0;  DMA2_Stream0->CR = 0;
	DMA2->LIFCR = 0b111101UL<<22; DMA2_Stream3->CR = 0;
	
	// SPI1 @ APB2 = 60 MHz
	SPI1->CR1 = 0; // Disable SPI
	delay_us(1);
	spi1_empty_rx();

	SPI1->CR1 = 1UL<<11 /*16-bit frame*/ | 1UL<<9 /*Software slave management*/ | 1UL<<8 /*SSI bit must be high*/ |
		0b010UL<<3 /*div 8 = 7.5 MHz*/ | 1UL<<2 /*Master*/;
	SPI1->CR2 = 0;

	SPI1->CR1 |= 1UL<<6; // Enable SPI

	mc_flasher_cs0();
	delay_us(1);

	// Special magic sequence to put the motor controller into the flasher:
	spi1_poll_tx(0xfaaa);
	spi1_poll_tx(0x1234);
	spi1_poll_tx(0xabcd);
	spi1_poll_tx(0x420b);
	spi1_poll_tx(0xacdc);
	spi1_poll_tx(0xabba);
	spi1_poll_tx(0x1337);
	spi1_poll_tx_bsy(0xacab);

	mc_flasher_cs1();

	// Now, the motor controller is reconfiguring its SPI, and we need to make sure we keep the lines idle
	// as per the reference manual requirements during the initialization.

	delay_ms(30); // was 20

	spi1_empty_rx();

	while(1)
	{
		int i;
		int size = 0;
		uint8_t cmd = usart3_poll_rx();
		switch(cmd)
		{
			case 100: // Erase
			{
			int num_pages = usart3_poll_rx();
			if(num_pages > 30 || num_pages < 1)
			{
				ETERNAL_BLINK();
			}
			spi1_empty_rx();
			mc_flasher_cs0();
			spi1_poll_tx((100<<8) | num_pages);

			while(spi1_poll_rx() != 0xaaaa)
				spi1_poll_tx(0x1111); // Generate dummy data so that we know when the erase is done.

			spi1_poll_bsy();
			mc_flasher_cs1();
			usart3_poll_tx(0); // success code
			}
			break;


			case 101: // Write
			size = usart3_poll_rx()<<8;
			size |= usart3_poll_rx();

			if(size < 50 || size>30*1024 || size&1)
			{
				ETERNAL_BLINK();
			}
			size>>=1;

			spi1_empty_rx();
			mc_flasher_cs0();
			spi1_poll_tx(101<<8);
			spi1_poll_tx(size);
			spi1_empty_rx();

			for(i=0; i<size; i++)
			{
				spi1_empty_rx();
				int word = usart3_poll_rx();
				word    |= usart3_poll_rx()<<8;
				spi1_poll_tx(word);
			}

			spi1_poll_bsy();
			mc_flasher_cs1();
			usart3_poll_tx(0); // success code
			break;


			case 102: // Read
			{
			size = usart3_poll_rx()<<8;
			size |= usart3_poll_rx();

			if(size < 50 || size>30*1024 || size&1)
			{
				ETERNAL_BLINK();
			}
			size>>=1;

			spi1_empty_rx();
			mc_flasher_cs0();
			spi1_poll_tx(102<<8);
			spi1_poll_tx(size);
			delay_us(100); // Give the slave some time to parse the cmd and prepare the first data.
			spi1_empty_rx();
			int mask = 1;
			i=0;
			while(1)
			{
				int word = spi1_poll_rx();
				
				if(!mask)
				{
					usart3_poll_tx(word&0xff);
					usart3_poll_tx((word&0xff00)>>8);
				}
				else
				{
					delay_us(200);
					i=0;
				}

				if(++i > size) break;
				spi1_poll_tx(0x1111); // Generate dummies.
				if(word == 0xcccc)
				{
					mask = 0;
				}
			}

			spi1_poll_bsy();
			mc_flasher_cs1();

			}
			break;

			case 150:
			case 151:
			// Reset motcon and the main cpu.
			mc_flasher_cs0();
			spi1_poll_tx(150<<8);
			delay_ms(1);
			spi1_poll_bsy();
			mc_flasher_cs1();
			NVIC_SystemReset();
			while(1);

			default:
			break;

		}
	}
}


/*
	Incoming UART messages are double-buffered;
	handling needs to happen relatively fast so that reading the next command does not finish before the processing of the previous command.
*/

#define RX_BUFFER_LEN 128
volatile uint8_t rx_buffers[2][RX_BUFFER_LEN];
int rx_buf_loc = 0;

volatile uint8_t* gather_rx_buf;
volatile uint8_t* process_rx_buf;

volatile int do_handle_message;


void handle_maintenance_msg()
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

volatile int do_re_compass = 0;

volatile int motcon_pi = 10;

volatile int16_t dbg_timing_shift = 5000;

volatile int test_seq = 0;
void handle_message()
{
	switch(process_rx_buf[0])
	{
		case 0xfe:
		if(process_rx_buf[1] == 0x42 && process_rx_buf[2] == 0x11 && process_rx_buf[3] == 0x7a)
		{
			handle_maintenance_msg();
		}
		break;

		case 0x80:
		move_arc_manual(((int16_t)(int8_t)(process_rx_buf[1]<<1)), ((int16_t)(int8_t)(process_rx_buf[2]<<1)));
		break;

		case 0x81:
		move_rel_twostep(I7I7_I16_lossy(process_rx_buf[1],process_rx_buf[2]), I7I7_I16_lossy(process_rx_buf[3],process_rx_buf[4]));
		break;

		case 0x91:
		do_re_compass = 1;
		break;

		case 0x92:
		sync_to_compass();
		break;

		// Motor controller debug/dev messages
		case 0xd1:
		test_seq = 1;
		break;

		case 0xd2:
		test_seq = 0;
		break;

		case 0xd3:
		dbg_timing_shift -= 50;
		break;

		case 0xd4:
		dbg_timing_shift += 50;
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
		do_handle_message = rx_buf_loc-1;
		rx_buf_loc = 0;
	}
	else
	{
		if(byte > 127) // Start of command delimiter
			rx_buf_loc = 0;

		gather_rx_buf[rx_buf_loc] = byte;
		rx_buf_loc++;
		if(rx_buf_loc >= RX_BUFFER_LEN)
			rx_buf_loc = 0;
	}
}

int latest_sonars[MAX_NUM_SONARS]; // in cm, 0 = no echo

volatile optflow_data_t latest_optflow;
volatile int optflow_errors;

volatile int new_gyro, new_xcel, new_compass;
/*
	10kHz interrupts drives practically everything.
*/

volatile int int_x, int_y;

volatile int seconds;

void timebase_10k_handler()
{
	static int sec_gen = 0;
	static int cnt_10k = 0;
	int status;

	TIM6->SR = 0; // Clear interrupt flag
	cnt_10k++;

	sec_gen++;
	if(sec_gen >= 10000)
	{
		seconds++;
		sec_gen = 0;
	}

	// Run code of all devices expecting 10kHz calls.
	int dx = 0;
	int dy = 0;
	optflow_fsm(&dx, &dy);

	int_x += dx;
	int_y += dy;


	motcon_fsm();
	sonar_fsm();
	lidar_ctrl_loop();
	status = gyro_xcel_compass_fsm();

	if(status & GYRO_NEW_DATA)
	{
		new_gyro++;
	}

	if(status & XCEL_NEW_DATA)
	{
		new_xcel++;
	}

	if(status & COMPASS_NEW_DATA)
	{
		new_compass++;
	}

	if(do_handle_message)
		handle_message();

	compass_fsm(do_re_compass);
	do_re_compass = 0;
	run_feedbacks(status);


}

extern volatile lidar_datum_t lidar_full_rev[90];


uint8_t lidar_ignore[360];

#define LIDAR_IGNORE_LEN 380 // mm

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


extern volatile int i2c1_state;
extern volatile int last_sr1;
extern volatile int i2c1_fails;

extern volatile int gyro_timestep_len;
extern volatile int xcel_timestep_len;

extern volatile int lidar_speed_in_spec;
extern int lidar_initialized;
extern int cur_angle;
extern int cur_compass_angle;

extern int64_t xcel_long_integrals[3];

#define ADC_ITEMS 1
#define ADC_SAMPLES 2

typedef struct  __attribute__ ((__packed__))
{
	uint16_t bat_v;
} adc_data_t;

volatile adc_data_t adc_data[ADC_SAMPLES];

int main()
{
	int i;
	gather_rx_buf = rx_buffers[0];
	process_rx_buf = rx_buffers[1];

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
	RCC->APB2ENR |= 1UL<<12 /*SPI1*/ | 1UL<<4 /*USART1*/ | 1UL<<8 /*ADC1*/;

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

	GPIOB->ODR     = 1UL<<8 | 1UL<<9; // I2C pins high.
	GPIOB->OTYPER  = 1UL<<8 | 1UL<<9; // Open drain for I2C.
	             //    15141312111009080706050403020100
	             //     | | | | | | | | | | | | | | | |
	GPIOB->MODER   = 0b10101001000001011010000000000000;
	GPIOB->OSPEEDR = 0b01000101000001010000010001000000;
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
	             //    15141312111009080706050403020100
	             //     | | | | | | | | | | | | | | | |
	GPIOF->MODER   = 0b00000000000000000000000000000000;
	GPIOF->OSPEEDR = 0b00000000000000000000000000000000;
	             //    15141312111009080706050403020100
	             //     | | | | | | | | | | | | | | | |
	GPIOG->MODER   = 0b00000000000000000000000000000000;
	GPIOG->OSPEEDR = 0b00000000000000000000000000000000;


	// Motor controller nCS signals must be high as early as possible. Motor controllers wait 100 ms at boot for this.
	MC4_CS1();
	MC3_CS1();
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
		polling. This is actually not a big deal, and can be later integrated with 10k timebase handler.
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


	FLOW_CS1();

	NVIC_SetPriority(USART3_IRQn, 0b0000); // highest prio really needed.
	NVIC_EnableIRQ(USART3_IRQn);

	delay_ms(100);

	GPIOE->BSRR = 1UL<<7; // DBG IO1

	init_gyro_xcel_compass();
	init_optflow();
	init_motcons();
	init_lidar();



	ADC->CCR = 1UL<<23 /* temp sensor and Vref enabled */ | 0b00<<16 /*prescaler 2 -> 30MHz*/;
	ADC1->CR1 = 1UL<<8 /* SCAN mode */;
	ADC1->CR2 = 1UL<<9 /* Magical DDS bit to actually enable DMA requests */ | 1UL<<8 /*DMA ena*/ | 1UL<<1 /*continuous*/;
	ADC1->SMPR1 = 0b010UL<<6 /*ch12 (bat voltage): 28 cycles*/;
	ADC1->SQR1 = (1  -1)<<20 /* sequence length */;
	ADC1->SQR3 = 12<<0; // Ch12 first in sequence

	ADC1->CR2 |= 1; // Enable ADC

	DMA2_Stream4->PAR = (uint32_t)&(ADC1->DR);
	DMA2_Stream4->M0AR = (uint32_t)(adc_data);
	DMA2_Stream4->NDTR = ADC_ITEMS*ADC_SAMPLES;
	DMA2_Stream4->CR = 0UL<<25 /*Channel*/ | 0b01UL<<16 /*med prio*/ | 0b01UL<<13 /*16-bit mem*/ | 0b01UL<<11 /*16-bit periph*/ |
	                   1UL<<10 /*mem increment*/ | 1UL<<8 /*circular*/;

	DMA2->LIFCR = 0b111101UL<<0;
	DMA2_Stream4->CR |= 1UL; // Enable ADC DMA

	delay_ms(1);

	ADC1->CR2 |= 1UL<<30; // Start converting.

	__enable_irq();


	delay_ms(100);
	NVIC_SetPriority(TIM6_DAC_IRQn, 0b1010);
	NVIC_EnableIRQ(TIM6_DAC_IRQn);

	PSU12V_ENA();
	CHARGER_ENA();

	/*
		Lidar requires two types of syncing:
		sync_lidar() is called when the motor is turning at a widely acceptable range of rpm, which is
		enough to produce _some_ data from the lidar. sync_lidar does sync the data frame to the struct boundaries,
		but may not sync to the degree field, so the image may be wrongly rotated.

		But, syncing to data frames allows the lidar control loop to read the rpm field and start regulating the motor speed.

		After the control loop has stabilized the rpm, the lidar
		produces fully correct data, so resync_lidar() must be called: it does not only sync the data frame, but it also
		syncs the whole table so that the 1 degree point is at the start of the table. At this point, lidar ignore table
		is generated from the closeby objects.

		TODO: Implement the trivial check of lidar data not being synced, and resync when necessary.
	*/

	sync_lidar();

	GPIOE->BSRR = 1UL<<14; // DBG IO8


	int cnt = 0;
	int lidar_resynced = 0;
	int do_generate_lidar_ignore = 0;
	while(1)
	{

		cnt++;
#ifdef TEXT_DEBUG
		char buffer[4000];
		char* buf = buffer;
#endif

		delay_ms(100); // Don't produce too much data now, for network reasons. WAS 100

		if(lidar_initialized && lidar_speed_in_spec && !lidar_resynced)
		{
			resync_lidar();
			lidar_resynced = 1;
			do_generate_lidar_ignore = 10;
		}


		if(do_generate_lidar_ignore)
		{
			do_generate_lidar_ignore--;
			if(do_generate_lidar_ignore == 0)
				generate_lidar_ignore();
		}


#ifdef BINARY_OUTPUT
/*
		msg_gyro_t msg;
		msg.status = 1;
		msg.int_x = I16_I14(latest_gyro->x);
		msg.int_y = I16_I14(latest_gyro->y);
		msg.int_z = I16_I14(latest_gyro->z);
		txbuf[0] = 128;
		memcpy(txbuf+1, &msg, sizeof(msg_gyro_t));
		usart_send(txbuf, sizeof(msg_gyro_t)+1);

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
		if(!(cnt&3))
		{
			txbuf[0] = 0x84;
			txbuf[1] = (lidar_initialized) | (lidar_speed_in_spec<<1);
			int i;
			for(i = 0; i < 90; i++)
			{
				int o;
				for(o = 0; o < 4; o++)
				{
					if((lidar_full_rev[i].d[o].flags_distance&(1<<15)) || lidar_ignore[i*4+o])
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
			usart_send(txbuf, 90*4*2+2);
		}

#endif

		// Do fancy calculation here :)

		/*
			Compass algorithm (to be moved from here)

			To compensate for robot-referenced magnetic fields and offset errors:
			Track max, min readings on both X, Y axes while the robot is turning.
			Scale readings so that they read zero on the middle of the range, e.g.,
			if X axis reads between 1000 and 3000, make 2000 read as 0.
			Then calculate the angle with arctan (atan2() handles the signs to resolve
			the correct quadrant).
		*/

//		degrees = cur_angle>>(4+16);

		int16_t cur_ang_t = cur_angle>>16;
		int16_t cur_c_ang_t = cur_compass_angle>>16;

#ifdef BINARY_OUTPUT
		txbuf[0] = 0xa0;
		txbuf[1] = 1;
		txbuf[2] = I16_MS(cur_ang_t);
		txbuf[3] = I16_LS(cur_ang_t);
		txbuf[4] = 0;
		txbuf[5] = 0;
		txbuf[6] = I16_MS(cur_c_ang_t);
		txbuf[7] = I16_LS(cur_c_ang_t);
		usart_send(txbuf, 8);

		txbuf[0] = 0xa1;
		txbuf[1] = 1;
		txbuf[2] = I16_MS(int_x);
		txbuf[3] = I16_LS(int_x);
		txbuf[4] = I16_MS(int_y);
		txbuf[5] = I16_LS(int_y);
		txbuf[6] = latest_optflow.squal>>1;
		txbuf[7] = latest_optflow.dx&0x7f;
		txbuf[8] = latest_optflow.dy&0x7f;
		txbuf[9] = latest_optflow.max_pixel>>1;
		txbuf[10] = latest_optflow.dummy>>1;
		txbuf[11] = latest_optflow.motion>>1;

		usart_send(txbuf, 12);

		int bat_v = (adc_data[0].bat_v + adc_data[1].bat_v)<<2;
		txbuf[0] = 0xa2;
		txbuf[1] = ((CHA_RUNNING())?1:0) | ((CHA_FINISHED())?2:0);
		txbuf[2] = I16_MS(bat_v);
		txbuf[3] = I16_LS(bat_v);
		usart_send(txbuf, 4);

		txbuf[0] = 0x85;
		txbuf[1] = 0b111;
		int ts = latest_sonars[0]*10;
		txbuf[2] = ts&0x7f;
		txbuf[3] = (ts&(0x7f<<7)) >> 7;
		ts = latest_sonars[1]*10;
		txbuf[4] = ts&0x7f;
		txbuf[5] = (ts&(0x7f<<7)) >> 7;
		ts = latest_sonars[2]*10;
		txbuf[6] = ts&0x7f;
		txbuf[7] = (ts&(0x7f<<7)) >> 7;
		usart_send(txbuf, 8);

		// calc from xcel integral
		//1 xcel unit = 0.061 mg = 0.59841 mm/s^2; integrated at 10kHz timesteps, 1 unit = 0.059841 mm/s
		// to mm/sec: / 16.72 = *245 / 4094.183
		int speedx = (xcel_long_integrals[0]/**245*/)>>12;
		int speedy = (xcel_long_integrals[1]/**245*/)>>12;

/*		dbg[0] = motcon_rx[2].status;
		dbg[1] = motcon_rx[2].speed;
		dbg[2] = motcon_rx[2].current;
		dbg[3] = motcon_rx[2].pos;
		dbg[4] = motcon_rx[2].res4;
		dbg[5] = motcon_rx[2].res5;
		dbg[6] = motcon_rx[2].res6;
		dbg[7] = motcon_rx[2].crc;
*/
		txbuf[0] = 0xd2;
		for(i=0; i<10; i++)
		{
			int tm = dbg[i];
			txbuf[5*i+1] = I32_I7_4(tm);
			txbuf[5*i+2] = I32_I7_3(tm);
			txbuf[5*i+3] = I32_I7_2(tm);
			txbuf[5*i+4] = I32_I7_1(tm);
			txbuf[5*i+5] = I32_I7_0(tm);
		}
		usart_send(txbuf, 51);

#endif

#ifdef TEXT_DEBUG

		buf = o_str_append(buf, "\r\n");
		usart_print(buffer);
#endif


		if(seconds > 130)
		{
			CHARGER_ENA();
			seconds = 0;
		}
		else if(seconds > 120)
		{
			CHARGER_DIS();
		}

	}



}
