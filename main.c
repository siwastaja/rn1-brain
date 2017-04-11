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
	USART3->CR1 = 0; // Disable
	delay_us(10);
	USART3->SR = 0; // Clear flags
	USART3->BRR = 16UL<<4 | 4UL; // 115200
	USART3->CR1 = 1UL<<13 /*USART enable*/ | 1UL<<3 /*TX ena*/ | 1UL<<2 /*RX ena*/;
	delay_us(10);
	flasher();
	while(1);
}

#define RX_BUFFER_LEN 128
volatile uint8_t rx_buffers[2][RX_BUFFER_LEN];
int rx_buf_loc = 0;

volatile uint8_t* gather_rx_buf;
volatile uint8_t* process_rx_buf;

volatile int do_handle_message;

int speed_updated;

/*
	Messages are double-buffered;
	handling needs to happen relatively fast so that reading the next command does not finish before the processing of the previous command.
*/

int common_speed;
int angle;


void handle_message()
{
	switch(process_rx_buf[0])
	{
		case 0xfe:
		if(process_rx_buf[1] == 0x42 && process_rx_buf[2] == 0x11 && process_rx_buf[3] == 0x7a)
		{
			if(process_rx_buf[4] == 0x52)
			{
				run_flasher();
			}
		}
		break;

		case 0x80:
//		LED_OFF();
		common_speed = ((int16_t)(int8_t)(process_rx_buf[1]<<1))*4;
		angle = ((int16_t)(int8_t)(process_rx_buf[2]<<1))*4;
		speed_updated = 3000;
		break;
		default:
		break;
	}
	do_handle_message = 0;
}

void uart_rx_handler()
{
	LED_ON();
	// This SR-then-DR read sequence clears error flags:
	USART3->SR;
	uint8_t byte = USART3->DR;

	if(byte == 255)
	{
		volatile uint8_t* tmp = gather_rx_buf;
		gather_rx_buf = process_rx_buf;
		process_rx_buf = tmp;
		do_handle_message = rx_buf_loc-1;
		rx_buf_loc = 0;
	}
	else
	{
		if(byte > 127)
			rx_buf_loc = 0;

		gather_rx_buf[rx_buf_loc] = byte;
		rx_buf_loc++;
		if(rx_buf_loc >= RX_BUFFER_LEN)
			rx_buf_loc = 0;
	}
	LED_OFF();
}

int latest_sonars[NUM_SONARS]; // in cm, 0 = no echo

volatile optflow_data_t latest_optflow;
volatile int optflow_errors;

volatile int new_gyro, new_xcel, new_compass;

void timebase_10k_handler()
{
	static int cnt_10k = 0;
	int status;
	static int speeda = 0;
	static int speedb = 0;

	TIM6->SR = 0;
	cnt_10k++;

	optflow_fsm();

	// Motcon at 10 kHz
	motcon_fsm();

	sonar_fsm(); // at 10 kHz

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

	if(speed_updated)
	{
		speed_updated--;

		int error = latest_gyro->z/16 - angle;

		speeda += error;
		speedb -= error;

		if(speeda > 200*256) speeda = 200*256;
		else if(speeda < -200*256) speeda = -200*256;
		if(speedb > 200*256) speedb = 200*256;
		else if(speedb < -200*256) speedb = -200*256;

		int a = common_speed + speeda/256;
		int b = common_speed + speedb/256;

		if(a > 400) a=400;
		else if(a < -400) a=-400;
		if(b > 400) b=400;
		else if(b < -400) b=-400;

		motcons[2].cmd.speed = a;
		motcons[3].cmd.speed = b;

	}
	else
	{
		speeda=0;
		speedb=0;
		motcons[2].cmd.speed = 0;
		motcons[3].cmd.speed = 0;
	}
}

extern volatile lidar_datum_t lidar_full_rev[90];

extern volatile int i2c1_state;
extern volatile int last_sr1;
extern volatile int i2c1_fails;

extern volatile int gyro_timestep_len;
extern volatile int xcel_timestep_len;

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

	gather_rx_buf = rx_buffers[0];
	process_rx_buf = rx_buffers[1];

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

	NVIC_SetPriority(USART3_IRQn, 0b0101);
	NVIC_EnableIRQ(USART3_IRQn);

#ifdef TEXT_DEBUG
	usart_print("booty booty\r\n");
#endif
	delay_ms(500);

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
	init_lidar();
#ifdef TEXT_DEBUG
	usart_print("lidar init ok\r\n");
#endif

	__enable_irq();

	delay_ms(100);
	NVIC_SetPriority(TIM6_DAC_IRQn, 0b1010);
	NVIC_EnableIRQ(TIM6_DAC_IRQn);



	PSU12V_ENA();
	CHARGER_ENA();

#ifdef TEXT_DEBUG
	usart_print("pre-syncing lidar... ");
#endif
	sync_lidar();
//#ifdef TEXT_DEBUG
//	usart_print("stablizing lidar... ");
//#endif
//	delay_ms(6000);
//#ifdef TEXT_DEBUG
//	usart_print("re-syncing lidar... ");
//#endif
	resync_lidar();
#ifdef TEXT_DEBUG
	usart_print("done\r\n");
#endif

//	LED_OFF();

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

		if(kakka != -1)
			kakka++;

		if(kakka > 50)
		{
			resync_lidar();
			kakka=-1;
		}


/*
		buf = o_str_append(buf, " gyro=");
		buf = o_utoa8_fixed(latest_gyro->status_reg, buf);
		buf = o_str_append(buf, "  ");
		buf = o_itoa16_fixed(latest_gyro->x, buf);
		buf = o_str_append(buf, ", ");
		buf = o_itoa16_fixed(latest_gyro->y, buf);
		buf = o_str_append(buf, ", ");
		buf = o_itoa16_fixed(latest_gyro->z, buf);
		buf = o_str_append(buf, " xcel=");
		buf = o_utoa8_fixed(latest_xcel->status_reg, buf);
		buf = o_str_append(buf, "  ");
		buf = o_itoa16_fixed(latest_xcel->x, buf);
		buf = o_str_append(buf, ", ");
		buf = o_itoa16_fixed(latest_xcel->y, buf);
		buf = o_str_append(buf, ", ");
		buf = o_itoa16_fixed(latest_xcel->z, buf);
		buf = o_str_append(buf, " compass=");
		buf = o_utoa8_fixed(latest_compass->status_reg, buf);
		buf = o_str_append(buf, "  ");
		buf = o_itoa16_fixed(latest_compass->x, buf);
		buf = o_str_append(buf, ", ");
		buf = o_itoa16_fixed(latest_compass->y, buf);
		buf = o_str_append(buf, ", ");
		buf = o_itoa16_fixed(latest_compass->z, buf);


		buf = o_str_append(buf, " gyros=");
		buf = o_utoa16_fixed(new_gyro, buf);
		buf = o_str_append(buf, " +=");
		buf = o_utoa16_fixed(gyro_timestep_plusses, buf);
		buf = o_str_append(buf, " -=");
		buf = o_utoa16_fixed(gyro_timestep_minuses, buf);
		buf = o_str_append(buf, " l=");
		buf = o_utoa16_fixed(gyro_timestep_len, buf);
		buf = o_str_append(buf, " xcels=");
		buf = o_utoa16_fixed(new_xcel, buf);
		buf = o_str_append(buf, " +=");
		buf = o_utoa16_fixed(xcel_timestep_plusses, buf);
		buf = o_str_append(buf, " -=");
		buf = o_utoa16_fixed(xcel_timestep_minuses, buf);
		buf = o_str_append(buf, " l=");
		buf = o_utoa16_fixed(xcel_timestep_len, buf);
		buf = o_str_append(buf, " comps=");
		buf = o_utoa16_fixed(new_compass, buf);

		buf = o_str_append(buf, " i2c1_state=");
		buf = o_utoa16_fixed(i2c1_state, buf);

		buf = o_str_append(buf, " i2c1_fails=");
		buf = o_utoa16_fixed(i2c1_fails, buf);

*/

/*		buf = o_str_append(buf, " optflow=");
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
*/


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
			Compass algorithm:

			To compensate for robot-referenced magnetic fields and offset errors:
			Track max, min readings on both X, Y axes while the robot is turning.
			Scale readings so that they read zero on the middle of the range, e.g.,
			if X axis reads between 1000 and 3000, make 2000 read as 0.
			Then calculate the angle with atan2.
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
#endif

#ifdef TEXT_DEBUG

		buf = o_str_append(buf, "\r\n\r\n");
		usart_print(buffer);
#endif

	}



}
