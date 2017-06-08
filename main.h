#ifndef __MAIN_H
#define __MAIN_H

#include "sonar.h"
#include "optflow.h"

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

#define DO_KILL_PWR() {GPIOD->BSRR = 1UL<<5;}


void error(int code);
void run_flasher();
void mc_flasher(int mcnum);

extern volatile int optflow_int_x, optflow_int_y;

extern volatile optflow_data_t latest_optflow;
extern volatile int optflow_errors;

int get_bat_v();

extern int latest_sonars[MAX_NUM_SONARS]; // in cm, 0 = no echo


#endif
