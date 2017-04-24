#ifndef _SONAR_H
#define _SONAR_H

#define SONAR_PULSE_ON()  {GPIOD->BSRR = 1UL<<3;}
#define SONAR_PULSE_OFF() {GPIOD->BSRR = 1UL<<(3+16);}
#define SONAR1_ECHO() (GPIOD->IDR & (1UL<<2))
#define SONAR2_ECHO() (GPIOD->IDR & (1UL<<0))
#define SONAR3_ECHO() (GPIOC->IDR & (1UL<<12))
//#define SONAR4_ECHO() (GPIOD->IDR & (1UL<<1))

#define MAX_NUM_SONARS 3
#define NUM_SONARS 3


void sonar_fsm();

#endif
