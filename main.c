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

//#define MOTCON_DEBUG
#define BINARY_OUTPUT
//#define TEXT_DEBUG

#define LED_ON()  {GPIOC->BSRR = 1UL<<13;}
#define LED_OFF() {GPIOC->BSRR = 1UL<<(13+16);}
#define CHARGER_ENA() {GPIOA->BSRR = 1UL<<15;}
#define CHARGER_DIS() {GPIOA->BSRR = 1UL<<(15+16);}
#define PSU5V_ENA() {GPIOE->BSRR = 1UL<<15;}
#define PSU5V_DIS() {GPIOE->BSRR = 1UL<<(15+16);}
#define PSU12V_ENA() {GPIOD->BSRR = 1UL<<4;}
#define PSU12V_DIS() {GPIOD->BSRR = 1UL<<(4+16);}

#ifdef MOTCON_DEBUG
extern volatile int mc_dbg_tx, mc_dbg_rx;
#endif

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


volatile motcon_t motcons[NUM_MOTCONS];


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

void mc_flasher(int mcnum)
{
	__disable_irq();
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

	delay_ms(2000);

	MC4_CS1();
	MC3_CS1();
	MC2_CS1();
	MC1_CS1();

	// SPI1 @ APB2 = 60 MHz
	SPI1->CR1 = 0; // Disable SPI
	delay_us(10);
	SPI1->CR1 = 1UL<<11 /*16-bit frame*/ | 1UL<<9 /*Software slave management*/ | 1UL<<8 /*SSI bit must be high*/ |
		0b010UL<<3 /*div 8 = 7.5 MHz*/ | 1UL<<2 /*Master*/;
	SPI1->CR2 = 0;

	SPI1->CR1 |= 1UL<<6; // Enable SPI

	switch(mcnum)
	{
		case 1: MC1_CS0(); break;
		case 2: MC2_CS0(); break;
		case 3: MC3_CS0(); break;
		case 4: MC4_CS0(); break;
		default: LED_ON(); while(1);
	}

	while(!(SPI1->SR & (1UL<<1))) ;
	SPI1->DR = 60<<10 | 234;
	while(!(SPI1->SR & (1UL<<1))) ;
	SPI1->DR = 55<<10 | 345;

	while(!(SPI1->SR & (1UL<<1))) ;
	SPI1->DR = (100<<8) | 0;
	while(1);
	while(1)
	{
		int i;
		int size = 0;
		while(!(USART3->SR & (1UL<<5))) ;
		switch(USART3->DR)
		{
			case 100:
			while(!(USART3->SR & (1UL<<5))) ;
			int num_pages = USART3->DR;
			if(num_pages > 30 || num_pages < 1)
			{
				LED_ON(); while(1);
			}
			while(!(SPI1->SR & (1UL<<1))) ;
			SPI1->DR = (100<<8) | num_pages;

			while(1)
			{
				while(!(SPI1->SR & (1UL<<0))) ;
				if(SPI1->DR == 0xaaaa) // Erase done.
					break;
				SPI1->DR = 0; // generate dummy data during erase so we can detect the end-of-erase condition
			}
			USART3->DR = 0; // Return success code 0.
			break;


			case 101:
			while(!(USART3->SR & (1UL<<5))) ;
			size = USART3->DR<<8;
			while(!(USART3->SR & (1UL<<5))) ;
			size |= USART3->DR;

			if(size < 50 || size>30*1024 || size&1)
			{
				LED_ON(); while(1);
			}
			size>>=1;
			while(!(SPI1->SR & (1UL<<1))) ;
			SPI1->DR = (101<<8);
			while(!(SPI1->SR & (1UL<<1))) ;
			SPI1->DR = size;
			for(i=0; i<size; i++)
			{
				int word = 0;
				while(!(USART3->SR & (1UL<<5))) ;
				word = USART3->DR<<8;
				while(!(USART3->SR & (1UL<<5))) ;
				word |= USART3->DR;
				while(!(SPI1->SR & (1UL<<1))) ;
				SPI1->DR = word;
			}
			USART3->DR = 0; // Return success code 0.
			break;


			case 102:
			while(!(USART3->SR & (1UL<<5))) ;
			size = USART3->DR<<8;
			while(!(USART3->SR & (1UL<<5))) ;
			size |= USART3->DR;

			if(size < 50 || size>30*1024 || size&1)
			{
				LED_ON(); while(1);
			}
			size>>=1;
			while(!(SPI1->SR & (1UL<<1))) ;
			SPI1->DR = (102<<8);
			while(!(SPI1->SR & (1UL<<1))) ;
			SPI1->DR = size;
			for(i=0; i<(size>>1); i++)
			{
				int word = 0;
				while(!(SPI1->SR & (1UL<<1))) ;
				SPI1->DR = 0; // Dummy data to generate clocking
				while(!(SPI1->SR & (1UL<<0))) ;
				word = SPI1->DR;

				while(!(USART3->SR & (1UL<<7))) ;
				USART3->DR = (word&0xff00)>>8;
				while(!(USART3->SR & (1UL<<7))) ;
				USART3->DR = word&0xff;
			}
			break;

			case 150:
			case 151:
			// Reset motcon and the main cpu.
			while(!(SPI1->SR & (1UL<<1))) ;
			SPI1->DR = 150<<8;
			delay_ms(1);
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

int speed_updated;  // For timeouting robot movements (for faulty communications)


int common_speed;
int angle;


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
		common_speed = ((int16_t)(int8_t)(process_rx_buf[1]<<1))*4;
		angle = ((int16_t)(int8_t)(process_rx_buf[2]<<1))*4;
		speed_updated = 5000; // robot is stopped if 0.5s is elapsed between the speed commands.
		break;
		default:
		break;
	}
	do_handle_message = 0;
}

void uart_rx_handler()
{
	// This SR-then-DR read sequence clears error flags:
	USART3->SR;
	uint8_t byte = USART3->DR;

#ifdef MOTCON_DEBUG
	mc_dbg_tx = byte;
#else
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
#endif
}

int latest_sonars[NUM_SONARS]; // in cm, 0 = no echo

volatile optflow_data_t latest_optflow;
volatile int optflow_errors;

volatile int new_gyro, new_xcel, new_compass;
/*
	10kHz interrupts drives practically everything.
*/

volatile int int_x, int_y;

void timebase_10k_handler()
{
	static int cnt_10k = 0;
	int status;

	TIM6->SR = 0; // Clear interrupt flag
	cnt_10k++;

	// Run code of all devices expecting 10kHz calls.
	int dx = 0;
	int dy = 0;
	optflow_fsm(&dx, &dy);

	int_x = dx;
	int_y = dy;


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

	run_feedbacks(status);


}

extern volatile lidar_datum_t lidar_full_rev[90];

extern volatile int i2c1_state;
extern volatile int last_sr1;
extern volatile int i2c1_fails;

extern volatile int gyro_timestep_len;
extern volatile int xcel_timestep_len;

int main()
{
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
		TIM4 generates PWM control for the LIDAR brushed DC motor.
	*/

	TIM4->CR1 = 1UL<<7 /*auto preload*/ | 0b01UL<<5 /*centermode*/;
	TIM4->CCMR2 = 1UL<<11 /*CH4 preload*/ | 0b110UL<<12 /*PWMmode1*/;
	TIM4->CCER = 1UL<<12 /*CH4 out ena*/;
	TIM4->ARR = 1024;
	TIM4->CCR4 = 350;
	TIM4->CR1 |= 1UL; // Enable.

	/*
		TIM6 @ APB1 at 30MHz, but the counter runs at x2 = 60MHz
		Create 10 kHz timebase interrupt
	*/

	TIM6->DIER |= 1UL; // Update interrupt
	TIM6->ARR = 5999; // 60MHz -> 10 kHz
	TIM6->CR1 |= 1UL; // Enable


	FLOW_CS1();

	NVIC_SetPriority(USART3_IRQn, 0b0101);
	NVIC_EnableIRQ(USART3_IRQn);

#ifdef TEXT_DEBUG
	usart_print("booty booty\r\n");
#endif
	delay_ms(100);

	init_gyro_xcel_compass();
#ifdef TEXT_DEBUG
	usart_print("gyro,xcel,compass init ok\r\n");
#endif
	init_optflow();
#ifdef TEXT_DEBUG
	usart_print("optflow init ok\r\n");
#endif
	init_motcons();
#ifdef TEXT_DEBUG
	usart_print("motcons init ok\r\n");
#endif
//	init_lidar();
#ifdef TEXT_DEBUG
	usart_print("lidar init ok\r\n");
#endif

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
		but does not sync the degree field, so the image will be wrongly rotated.

		But, syncing to data frames allows the lidar control loop to read the rpm field and start regulating the motor speed.

		After sufficienct time (at least 5-6 seconds) of runtime, the control loop has stabilized the rpm so that the lidar
		produces fully correct data, so resync_lidar() must be called: it does not only sync the data frame, but it also
		syncs the whole table so that the 1 degree point is at the start of the table.

		TODO: Implement the trivial check of lidar data not being synced, and resync when necessary.
	*/

#ifdef TEXT_DEBUG
//	usart_print("pre-syncing lidar... ");
#endif
//	sync_lidar();
#ifdef TEXT_DEBUG
//	usart_print("done\r\n");
#endif


	int compass_x_min = 0;
	int compass_x_max = 0;
	int compass_y_min = 0;
	int compass_y_max = 0;

	int first_compass = 3;
	int kakka = 0;
	while(1)
	{

#ifdef TEXT_DEBUG
		char buffer[4000];
		char* buf = buffer;
#endif

		delay_ms(100); // Don't produce too much data now, socat is broken.
/*
		if(kakka != -1)
			kakka++;

		if(kakka > 50)
		{
			resync_lidar();
			kakka=-1;
		}
*/

/*
		Code like this can be used for debugging purposes:
		buf = o_str_append(buf, " gyro=");
		buf = o_utoa8_fixed(latest_gyro->status_reg, buf);
*/

#ifdef MOTCON_DEBUG
		delay_ms(50);
		int val = mc_dbg_rx;
		buf = o_utoa16_fixed(val, buf);
		buf = o_str_append(buf, "   ");
		buf = o_itoa16_fixed(val, buf);

		buf = o_str_append(buf, "   ");
		buf = o_utoa8_fixed((val&0xff00)>>8, buf);
		buf = o_str_append(buf, "   ");
		buf = o_utoa8_fixed((uint8_t)val&0xff, buf);

		buf = o_str_append(buf, "   ");
		buf = o_itoa8_fixed((val&0xff00)>>8, buf);
		buf = o_str_append(buf, "   ");
		buf = o_itoa8_fixed((int8_t)val&0xff, buf);

		int o;
		buf = o_str_append(buf, "   ");
		for(o=15; o>=0; o--)
		{
			buf = o_str_append(buf, (val&(1<<o))?"1":"0");
			if(o==4 || o==8 || o==12)
				buf = o_str_append(buf, " ");
		}
#endif

#ifdef BINARY_OUTPUT
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
		usart_send(txbuf, 90*4*2+2);


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

		int cx = latest_compass->x;
		int cy = latest_compass->y;

		if(first_compass > -1)
			first_compass--;

		if(first_compass == 0)
		{
			compass_x_min = compass_x_max = cx;
			compass_y_min = compass_y_max = cy;
		}

		if(cx < compass_x_min)
			compass_x_min = cx;
		if(cy < compass_y_min)
			compass_y_min = cy;

		if(cx > compass_x_max)
			compass_x_max = cx;
		if(cy > compass_y_max)
			compass_y_max = cy;

		int degrees = 0;

		int dx = compass_x_max - compass_x_min;
		int dy = compass_y_max - compass_y_min;
		if(dx > 500 && dy > 500)
		{
			int dx2 = compass_x_max + compass_x_min;
			int dy2 = compass_y_max + compass_y_min;
			cx = cx - dx2/2;
			cy = cy - dy2/2;

			double heading = atan2(cx, cy);
			heading *= (360.0/(2.0*M_PI));
			degrees = heading;
		}

#ifdef BINARY_OUTPUT
		txbuf[0] = 0xa0;
		txbuf[1] = 1;
		txbuf[2] = degrees&0x7f;
		txbuf[3] = (degrees&(0x7f<<7)) >> 7;
		txbuf[4] = 0;
		txbuf[5] = 0;
		usart_send(txbuf, 6);


		txbuf[0] = 0xa1;
		txbuf[1] = 1;
		txbuf[2] = (I16_I14(int_x)&0xff00)>>8;
		txbuf[3] = I16_I14(int_x)&0xff;
		txbuf[4] = (I16_I14(int_y)&0xff00)>>8;
		txbuf[5] = I16_I14(int_y)&0xff;
		usart_send(txbuf, 6);

#endif

#ifdef TEXT_DEBUG

		buf = o_str_append(buf, "\r\n");
		usart_print(buffer);
#endif

	}



}
