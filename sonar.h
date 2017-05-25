#ifndef _SONAR_H
#define _SONAR_H

#include "lidar_corr.h" // for point_t, which should be moved to a more generic header.


#define SONAR_PULSE_ON()  {GPIOD->BSRR = 1UL<<3;}
#define SONAR_PULSE_OFF() {GPIOD->BSRR = 1UL<<(3+16);}
#define SONAR1_ECHO() (GPIOC->IDR & (1UL<<12))
#define SONAR2_ECHO() (GPIOD->IDR & (1UL<<0))
#define SONAR3_ECHO() (GPIOD->IDR & (1UL<<2))
//#define SONAR4_ECHO() (GPIOD->IDR & (1UL<<1))

#define MAX_NUM_SONARS 3
#define NUM_SONARS 3

extern int latest_sonars[MAX_NUM_SONARS];

void sonar_fsm_10k();
void init_sonars();

void get_sonars(point_t* out); // outputs NUM_SONARS point_ts.


#endif
