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
#include "feedbacks.h"
#include "navig.h"
#include "lidar_corr.h"
#include "uart.h"

void hwtest_main()
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

	// Motor controller nCS signals must be high as early as possible. Motor controllers wait 100 ms at boot for this.
	MC4_CS1();
	MC3_CS1();
	MC2_CS1();
	MC1_CS1();

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
	USART3->CR1 = 1UL<<13 /*USART enable*/ | 1UL<<3 /*TX ena*/ | 1UL<<2 /*RX ena*/;

	uart_print_string_blocking("R#1 OJABOTTI DEV PROTOTYPE REV.A HWTEST       PULUROBOTICS 2017\r\n");
	uart_print_string_blocking("If you read this, UART TX is working. If you read at 115200bps, clock gen (xtal+pll) is working.\r\n");

	int ena5v = 0;
	int ena12v = 0;
	
	while(1)
	{
		uart_print_string_blocking("\r\n\r\n[ 1 ]  Test motor controller 1\r\n");
		uart_print_string_blocking("[ 2 ]  Test motor controller 2\r\n");
		uart_print_string_blocking("[ 3 ]  Test motor controller 3\r\n");
		uart_print_string_blocking("[ 4 ]  Test motor controller 4\r\n");
		uart_print_string_blocking("[ 5 ]  Test I2C gyro, accelerometer, compass\r\n");

		if(!ena5v)
			uart_print_string_blocking("[ 6 ] (   5V off    ) Enable 5V 10A supply\r\n");
		else
			uart_print_string_blocking("[ 6 ] (5V turned ON ) Disable 5V 10A supply\r\n");

		if(!ena12v)
			uart_print_string_blocking("[ 7 ] (   12V off    ) Enable 12V 1.5A supply\r\n");
		else
			uart_print_string_blocking("[ 7 ] (12V turned ON ) Disable 12V 1.5A supply\r\n");


		uart_print_string_blocking("[ 8 ]  Test charger subsystem\r\n");


		char buffer[1000];

		while(!(USART3->SR & (1<<5))) ;
		uint8_t cmd = USART3->DR;

		switch(cmd)
		{
			case '1':
			case '2':
			case '3':
			case '4':
			{
				uart_print_string_blocking("Enable power with w. Stop with q.\r\n");

				int motcon = cmd-'1';
				init_motcons();

				int cnt = 0;
				while(1)
				{
					for(int i=0; i<250; i++)
					{
						motcon_fsm();
						delay_ms(1);
					}

					uint8_t subcmd = 0;
					if(USART3->SR & (1<<5)) subcmd = USART3->DR;
					if(subcmd == 'q')
					{
						motcon_tx[motcon].state = 0;
						motcon_fsm();
						delay_ms(1);
						motcon_fsm();
						delay_ms(1);
						motcon_fsm();
						delay_ms(1);
						motcon_fsm();
						delay_ms(1);
						motcon_fsm();
						break;
					}
					else if(subcmd == 'w')
					{
						motcon_tx[motcon].state = 5;
					}

					cnt++;

					if(cnt == 1)
						motcon_tx[motcon].speed = 600;
					else if(cnt == 10)
						motcon_tx[motcon].speed = 0;
					else if(cnt == 15)
						motcon_tx[motcon].speed = -600;
					else if(cnt == 25)
						motcon_tx[motcon].speed = 0;
					else if(cnt == 30)
						cnt = 0;

					char *p_buf = buffer;
					p_buf = o_str_append(p_buf, "status=");
					p_buf = o_utoa16_fixed(motcon_rx[motcon].status, p_buf);
					p_buf = o_str_append(p_buf, " speed=");
					p_buf = o_itoa16_fixed(motcon_rx[motcon].speed, p_buf);
					p_buf = o_str_append(p_buf, " current=");
					p_buf = o_itoa16_fixed(motcon_rx[motcon].current, p_buf);
					p_buf = o_str_append(p_buf, "   pos=");
					p_buf = o_itoa16_fixed(motcon_rx[motcon].pos, p_buf);
					p_buf = o_str_append(p_buf, "   res4=");
					p_buf = o_itoa16_fixed(motcon_rx[motcon].res4, p_buf);
					p_buf = o_str_append(p_buf, " res5=");
					p_buf = o_itoa16_fixed(motcon_rx[motcon].res5, p_buf);
					p_buf = o_str_append(p_buf, " res6=");
					p_buf = o_itoa16_fixed(motcon_rx[motcon].res6, p_buf);
					p_buf = o_str_append(p_buf, " crc=");
					p_buf = o_utoa16_fixed(motcon_rx[motcon].crc, p_buf);
					p_buf = o_str_append(p_buf, "\r\n");
					uart_print_string_blocking(buffer);
				}
			}
			break;

			case '5':
			{
				int ret;
				while( (ret=init_gyro_xcel_compass()) != 0)
				{
					char *p_buf = buffer;
					p_buf = o_str_append(p_buf, "gyro_xcel_compass init failed with code=");
					p_buf = o_itoa16(ret, p_buf);
					p_buf = o_str_append(p_buf, ", retrying... (q to give up)\r\n");
					uart_print_string_blocking(buffer);
					delay_ms(500);
					uint8_t subcmd = 0;
					if(USART3->SR & (1<<5)) subcmd = USART3->DR;
					if(subcmd == 'q')
						break;
				}
				uart_print_string_blocking("Stop with q.\r\n");

				while(1)
				{
					for(int i=0; i<1000; i++)
					{
						gyro_xcel_compass_fsm();
						delay_us(90);
					}

					char *p_buf = buffer;
					p_buf = o_str_append(p_buf, "\r\n\r\nGYRO:    status=");
					p_buf = o_utoa8_fixed(latest_gyro->status_reg, p_buf);
					p_buf = o_str_append(p_buf, " X=");
					p_buf = o_itoa16_fixed(latest_gyro->x, p_buf);
					p_buf = o_str_append(p_buf, " Y=");
					p_buf = o_itoa16_fixed(latest_gyro->y, p_buf);
					p_buf = o_str_append(p_buf, " Z=");
					p_buf = o_itoa16_fixed(latest_gyro->z, p_buf);
					p_buf = o_str_append(p_buf, "\r\nXCEL:    status=");
					p_buf = o_utoa8_fixed(latest_xcel->status_reg, p_buf);
					p_buf = o_str_append(p_buf, " X=");
					p_buf = o_itoa16_fixed(latest_xcel->x, p_buf);
					p_buf = o_str_append(p_buf, " Y=");
					p_buf = o_itoa16_fixed(latest_xcel->y, p_buf);
					p_buf = o_str_append(p_buf, " Z=");
					p_buf = o_itoa16_fixed(latest_xcel->z, p_buf);
					p_buf = o_str_append(p_buf, "\r\nCOMPASS: status=");
					p_buf = o_utoa8_fixed(latest_compass->status_reg, p_buf);
					p_buf = o_str_append(p_buf, " X=");
					p_buf = o_itoa16_fixed(latest_compass->x, p_buf);
					p_buf = o_str_append(p_buf, " Y=");
					p_buf = o_itoa16_fixed(latest_compass->y, p_buf);
					p_buf = o_str_append(p_buf, " Z=");
					p_buf = o_itoa16_fixed(latest_compass->z, p_buf);
					uart_print_string_blocking(buffer);
					delay_ms(150);

					uint8_t subcmd = 0;
					if(USART3->SR & (1<<5)) subcmd = USART3->DR;
					if(subcmd == 'q')
						break;


				}
			}
			break;

			case '6':
			{
				if(!ena5v)
				{
					ena5v = 1;
					PSU5V_ENA();
				}
				else
				{
					ena5v = 0;
					PSU5V_DIS();
				}
			}
			break;

			case '7':
			{
				if(!ena12v)
				{
					ena12v = 1;
					PSU12V_ENA();
				}
				else
				{
					ena12v = 0;
					PSU12V_DIS();
				}
			}
			break;

			case '8':
			{

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

				uart_print_string_blocking("a = enable charging, s = disable charging, q = stop\r\n");

				while(1)
				{
					char *p_buf = buffer;
					p_buf = o_str_append(p_buf, "BATT VOLTAGE ADC = ");
					p_buf = o_utoa16_fixed(adc_data[0].bat_v, p_buf);
					p_buf = o_str_append(p_buf, ",");
					p_buf = o_utoa16_fixed(adc_data[1].bat_v, p_buf);
					p_buf = o_str_append(p_buf, "-->");
					p_buf = o_utoa16_fixed(get_bat_v(), p_buf);
					p_buf = o_str_append(p_buf, "mV ");

					if(CHA_RUNNING())
						p_buf = o_str_append(p_buf, " (  CHARGING  )");
					else
						p_buf = o_str_append(p_buf, " (not charging)");

					if(CHA_FINISHED())
						p_buf = o_str_append(p_buf, " (  FINISHED  )\r\n");
					else
						p_buf = o_str_append(p_buf, " (not finished)\r\n");

					uart_print_string_blocking(buffer);

					delay_ms(300);
					uint8_t subcmd = 0;
					if(USART3->SR & (1<<5)) subcmd = USART3->DR;
					if(subcmd == 'q')
						break;
					else if(subcmd == 'a')
					{
						uart_print_string_blocking("==> Charging enabled\r\n");
						CHARGER_ENA();
					}
					else if(subcmd == 's')
					{
						uart_print_string_blocking("==> Charging disabled\r\n");
						CHARGER_DIS();
					}

				}
				uart_print_string_blocking("Charging disabled\r\n");
				CHARGER_DIS();

			}
			break;

			default:
			break;

		}

	}

}
