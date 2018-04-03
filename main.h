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

	#define ADC_ITEMS 1
	typedef struct  __attribute__ ((__packed__))
	{
		uint16_t bat_v;
	} adc_data_t;


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

	#define LEFT_BLINKER_ON()  do{GPIOD->BSRR = 1UL<<10;}while(0)
	#define LEFT_BLINKER_OFF() do{GPIOD->BSRR = 1UL<<(10+16);}while(0)
	#define RIGHT_BLINKER_ON()  do{GPIOE->BSRR = 1UL<<8;}while(0)
	#define RIGHT_BLINKER_OFF() do{GPIOE->BSRR = 1UL<<(8+16);}while(0)
	#define FWD_LIGHT_ON()  do{GPIOD->BSRR = 1UL<<9;}while(0)
	#define FWD_LIGHT_OFF() do{GPIOD->BSRR = 1UL<<(9+16);}while(0)

	#define PO1_ON()  do{GPIOA->BSRR = 1UL<<(8);}while(0)
	#define PO1_OFF() do{GPIOA->BSRR = 1UL<<(8+16);}while(0)
	#define PO2_ON()  do{GPIOA->BSRR = 1UL<<(10);}while(0)
	#define PO2_OFF() do{GPIOA->BSRR = 1UL<<(10+16);}while(0)
	#define PO3_ON()  do{GPIOA->BSRR = 1UL<<(11);}while(0)
	#define PO3_OFF() do{GPIOA->BSRR = 1UL<<(11+16);}while(0)
	#define PO4_ON()  do{GPIOA->BSRR = 1UL<<(13);}while(0)
	#define PO4_OFF() do{GPIOA->BSRR = 1UL<<(13+16);}while(0)
	#define PO5_ON()  do{GPIOE->BSRR = 1UL<<(4);}while(0)
	#define PO5_OFF() do{GPIOE->BSRR = 1UL<<(4+16);}while(0)
	#define PO6_ON()  do{GPIOE->BSRR = 1UL<<(6);}while(0)
	#define PO6_OFF() do{GPIOE->BSRR = 1UL<<(6+16);}while(0)


	#define ADC_ITEMS 2
	typedef struct  __attribute__ ((__packed__))
	{
		uint16_t bat_v;
		uint16_t cha_v;
	} adc_data_t;


#endif

void error(int code);
void run_flasher();
void mc_flasher(int mcnum);
void delay_ms(uint32_t i);
void delay_us(uint32_t i);


extern volatile int optflow_int_x, optflow_int_y;

extern volatile optflow_data_t latest_optflow;
extern volatile int optflow_errors;



int get_bat_v();
int get_bat_percentage();
int get_cha_v(); // in mv


#define ADC_SAMPLES 2

extern volatile adc_data_t adc_data[ADC_SAMPLES];

extern volatile int leds_control_by_motion;
extern volatile int leds_motion_blink_left;
extern volatile int leds_motion_blink_right;
extern volatile int leds_motion_forward;

extern volatile int bat_emerg_on;
extern volatile int bat_emerg_action;
extern volatile int robot_is_in_charger;



#endif
