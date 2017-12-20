#ifndef __MAIN_H
#define __MAIN_H

#include "sonar.h"
#include "optflow.h"

#ifdef PCB1A

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

	#define LIDAR_ENA() {GPIOD->BSRR = 1UL<<1;}
	#define LIDAR_DIS() {GPIOD->BSRR = 1UL<<(1+16);}

	#define DO_KILL_PWR() {GPIOD->BSRR = 1UL<<5;}

#endif

#ifdef PCB1B

	#define LED_ON()  {GPIOC->BSRR = 1UL<<13;}
	#define LED_OFF() {GPIOC->BSRR = 1UL<<(13+16);}
	#define CHARGER_ENA() {GPIOB->BSRR = 1UL<<4;}
	#define CHARGER_DIS() {GPIOB->BSRR = 1UL<<(4+16);}
	#define PSU5V_ENA() {GPIOE->BSRR = 1UL<<3;}
	#define PSU5V_DIS() {GPIOE->BSRR = 1UL<<(3+16);}
	#define PSU12V_ENA() {GPIOD->BSRR = 1UL<<5;}
	#define PSU12V_DIS() {GPIOD->BSRR = 1UL<<(5+16);}
	#define CHA_RUNNING() (!(GPIOB->IDR & (1<<3)))
	#define CHA_FINISHED() (!(GPIOD->IDR & (1<<6)))

	#define LIDAR_ENA() {GPIOB->BSRR = 1UL<<5;}
	#define LIDAR_DIS() {GPIOB->BSRR = 1UL<<(5+16);}

	#define DO_KILL_PWR() {GPIOE->BSRR = 1UL<<2;}

	#define LEFT_BLINKER_ON()  {GPIOD->BSRR = 1UL<<10;}
	#define LEFT_BLINKER_OFF() {GPIOD->BSRR = 1UL<<(10+16);}
	#define RIGHT_BLINKER_ON()  {GPIOE->BSRR = 1UL<<8;}
	#define RIGHT_BLINKER_OFF() {GPIOE->BSRR = 1UL<<(8+16);}
	#define FWD_LIGHT_ON()  {GPIOD->BSRR = 1UL<<9;}
	#define FWD_LIGHT_OFF() {GPIOD->BSRR = 1UL<<(9+16);}

#endif

void error(int code);
void run_flasher();
void mc_flasher(int mcnum);
void delay_ms(uint32_t i);
void delay_us(uint32_t i);


extern volatile int optflow_int_x, optflow_int_y;

extern volatile optflow_data_t latest_optflow;
extern volatile int optflow_errors;

#define ADC_ITEMS 1
#define ADC_SAMPLES 2

typedef struct  __attribute__ ((__packed__))
{
	uint16_t bat_v;
} adc_data_t;

int get_bat_v();
extern volatile adc_data_t adc_data[ADC_SAMPLES];


#endif
