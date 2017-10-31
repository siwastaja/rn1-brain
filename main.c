#if (!RN1P4 && !PULU1 && !RN1P6 && !RN1P7)
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

volatile int dbg[10];

volatile int dbg_error_num = 1;

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
	code = dbg_error_num;
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

void adc_int_handler()
{
//	ADC1->ISR |= 1UL<<7;
}

void run_flasher()
{
	__disable_irq();

	FLASH->ACR = 0UL<<10 /* Data cache disable */ | 0UL<<9 /* Instr cache disable */ | 1UL<<8 /*prefetch enable*/ | 3UL /*3 wait states*/;

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

void timebase_10k_handler()
{
	static int sec_gen = 0;
	static int cnt_10k = 0;
	static int gyro_xcel_compass_status;

	us100++;

	TIM6->SR = 0; // Clear interrupt flag

	int starttime = TIM6->CNT;

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
	}


	cnt_10k++;
	if(cnt_10k > 9) cnt_10k = 0;
	int tooktime = TIM6->CNT - starttime;
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
	return (((adc_data[0].bat_v + adc_data[1].bat_v))*70920 / 22200);
}

volatile uint32_t random = 123;

extern volatile int lidar_scan_ready;

volatile int dbg_sending_lidar = 0;

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
	             //    15141312111009080706050403020100
	             //     | | | | | | | | | | | | | | | |
	GPIOF->MODER   = 0b00000000000000000000000000000000;
	GPIOF->OSPEEDR = 0b00000000000000000000000000000000;
	             //    15141312111009080706050403020100
	             //     | | | | | | | | | | | | | | | |
	GPIOG->MODER   = 0b00000000000000000000000000000000;
	GPIOG->OSPEEDR = 0b00000000000000000000000000000000;

	#ifdef SONARS_INSTALLED
	init_sonars();
	#endif
	init_uart();

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

	CHARGER_ENA();

	int cnt = 0;

	init_lidar();

	lidar_on(2, 1);

	while(1)
	{
		random++;
		cnt++;

		// calc from xcel integral
		//1 xcel unit = 0.061 mg = 0.59841 mm/s^2; integrated at 10kHz timesteps, 1 unit = 0.059841 mm/s
		// to mm/sec: / 16.72 = *245 / 4094.183
//		int speedx = (xcel_long_integrals[0]/**245*/)>>12;
//		int speedy = (xcel_long_integrals[1]/**245*/)>>12;

		while(uart_busy()) random++;
		static uint8_t sync_packet[8] = {0xff,0xff,0xff,0xff,  0xff,0xff,0x12,0xab};
		send_uart(sync_packet, 0xaa, 8);
		while(uart_busy()) random++;

		LED_ON();
		lidar_scan_ready = 0;
		while(!lidar_scan_ready) ;
		LED_OFF();
		dbg_sending_lidar = 1;
		send_uart(prev_lidar_scan, 0x84, LIDAR_SIZEOF(*prev_lidar_scan));
		dbg_sending_lidar = 0;

		while(uart_busy()) random++;
		uart_send_fsm(); // send something else.
		delay_ms(50);
		while(uart_busy()) random++;


#ifdef SONARS_INSTALLED
		{
			xyc_t* sonar;
			while( (sonar = get_sonar_point()) )
			{
				send_uart(sonar, 0x85, sizeof(xyc_t));
			}
		}
#endif

/*
		static int sensors_stabilized = 0;
		if(!sensors_stabilized && seconds > 10)
		{
			sensors_stabilized = 1;
			enable_collision_detection();
		}
*/
	}



}
