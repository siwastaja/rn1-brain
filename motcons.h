#ifndef _MOTCONS_H
#define _MOTCONS_H

#define MC4_CS1()  {GPIOE->BSRR = 1UL<<6;}
#define MC4_CS0() {GPIOE->BSRR = 1UL<<(6+16);}
#define MC3_CS1()  {GPIOA->BSRR = 1UL<<4;}
#define MC3_CS0() {GPIOA->BSRR = 1UL<<(4+16);}
#define MC2_CS1()  {GPIOC->BSRR = 1UL<<4;}
#define MC2_CS0() {GPIOC->BSRR = 1UL<<(4+16);}
#define MC1_CS1()  {GPIOC->BSRR = 1UL<<5;}
#define MC1_CS0() {GPIOC->BSRR = 1UL<<(5+16);}


#define NUM_MOTCONS 4


typedef struct
{
	int last_msg;
	int16_t cur_b;
	int16_t cur_c;
} motcon_status_t;

typedef struct
{
	int16_t speed;
	int16_t cur_limit;
} motcon_cmd_t;

typedef struct
{
	motcon_status_t status;
	motcon_cmd_t cmd;
} motcon_t;


void init_motcons();
void motcon_fsm();


#endif
