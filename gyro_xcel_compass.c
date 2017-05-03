#include <stdint.h>
#include "ext_include/stm32f2xx.h"

#include "gyro_xcel_compass.h"

extern void delay_us(uint32_t i);
extern void delay_ms(uint32_t i);


/*
	STM32 I2C implementation is a total catastrophe, it requires almost bitbanging-like sequencing by software,
	and provides no proper buffer registers (let alone a FIFO) - so we need to poll some status bits and write
	the data register at the exactly correct timing!

	For maximum performance, we hardwrite the optimal sequence of reading the relevant data from the
	gyro, accelerometer and compass. Because using the I2C implementation requires a lot of waiting
	for status bits, DMA is of little use; we use an interrupt handler implementing a state machine.

	First versions of this routine checked that the actual status bits we are expecting are indeed set. Now
	we just don't care - i2c1 is configured to only give interrupts from the relevant status bits. (Addition:
	because of a buggy i2c1 implementation, some states still require checking for the status bits.)

	All I2C transactions transfer 10 bytes + acks = 90 bits total.
	At 100kHz, this means max. 1 kHz poll rate for everything total -> use 200 Hz for gyro and xcel.

	In other words, reading gyro, xcel or compass each takes 1 ms, and we'll do each of them every 5 ms.

	We don't need more, and it's best to keep the I2C1 at 100kHz, since this leaves a lot of time between
	the interrupts, which allows processing all the other interrupts in the robot -- because we need to keep
	i2c1 interrupt at high priority, so it needs to pre-empt others, so we don't want to have bursts of
	high duty; we rather have constant flow of these stupid interrupts but enough time between them for
	other modules to do something useful.

	We save on IO and PCB routing by not using the interrupt lines from the sensors; I don't find it
	necessary. It's not a catastrophe even if we double read a certain sample time point; or if we miss one.
	Anyway, we do our best to sync to the data stream.

	Gyro and xcel generate data at 200Hz, but their clock accuracies are unspecified.

	We start reading at 200Hz, but if we get an override status bit or "data not ready" status bit, we
	adjust the timing point AND finetune the reading frequency.

	We don't use the FIFO features on the chips, because low latency is a lot more important than not missing a single data point,
	so we would try to keep the FIFO at 1 level deep state anyway.

	For all this to work, call gyro_xcel_compass_fsm() at 10 kHz.

	The code is horrible spaghetti, but with the STM32's well known, almost legendarily broken i2c, this is inevitable.

	Bitbanging the whole I2C will be seriously considered in the future.

*/
#define LED_ON()  {GPIOC->BSRR = 1UL<<13;}
#define LED_OFF() {GPIOC->BSRR = 1UL<<(13+16);}



#define I2C1_FSM_FREQ 10000
#define I2C1_DATA_FREQ 200
#define COMPASS_DATA_FREQ 5

#define I2C1_NUM_TIMESTEPS ((I2C1_FSM_FREQ) / (I2C1_DATA_FREQ))
#define I2C1_NUM_COMPASS_TIMESTEPS ((I2C1_FSM_FREQ) / (COMPASS_DATA_FREQ))

#define I2C1_TIMESTEP_PLUS_ADJUSTMENT  (I2C1_NUM_TIMESTEPS/4)   // Data not ready --> make data reading happen later by this amount
#define I2C1_TIMESTEP_MINUS_ADJUSTMENT (I2C1_NUM_TIMESTEPS/4)   // Data overrun --> make data reading happen earlier by this amount

volatile gyro_data_t gyro_data[2];
volatile xcel_data_t xcel_data[2];
volatile compass_data_t compass_data[2];

volatile gyro_data_t *latest_gyro;
volatile xcel_data_t *latest_xcel;
volatile compass_data_t *latest_compass;

volatile gyro_data_t *buffer_gyro;
volatile xcel_data_t *buffer_xcel;
volatile compass_data_t *buffer_compass;

volatile int i2c1_ready = 0;


volatile int gyro_timestep_len = I2C1_NUM_TIMESTEPS;
volatile int gyro_timestep = 0;
volatile int gyro_timestep_plusses;
volatile int gyro_timestep_minuses;

volatile int xcel_timestep_len = I2C1_NUM_TIMESTEPS;
volatile int xcel_timestep = 0;
volatile int xcel_timestep_plusses;
volatile int xcel_timestep_minuses;

volatile int compass_timestep_len = I2C1_NUM_COMPASS_TIMESTEPS;
volatile int compass_timestep = 0;


volatile int gyro_new_data;
volatile int xcel_new_data;
volatile int compass_new_data;

typedef enum {I2C1_GYRO = 0, I2C1_XCEL, I2C1_COMPASS} i2c1_device_t;

i2c1_device_t i2c1_cur_device;

volatile int i2c1_state = 0;
volatile int last_sr1;
volatile int i2c1_fails;

void i2c1_gyro_handler()
{
	int tmp;
	uint8_t status;

	switch(i2c1_state)
	{
		case 0:
		break;

		case 1: // Expect SB
		I2C1->SR1;
		I2C1->DR = 0x40;
		i2c1_state++;
		break;

		case 2: // Expect ADDR
		if(I2C1->SR1 & 2) // For some reason, there is a stupid extra interrupt, so we need to check this.
		{
			I2C1->SR2;
			I2C1->DR = 0x00; // Status register address
			I2C1->CR1 |= 1UL<<8; // START
			i2c1_state++;
		}
		break;

		case 3: // Expect SB
		if(I2C1->SR1 & 1) // For some reason, there is a stupid extra interrupt, so we need to check this.
		{
			I2C1->DR = 0x41;
			i2c1_state++;
		}
		break;

		case 4: // Expect ADDR
		I2C1->SR1;
		I2C1->SR2;
		I2C1->CR1 |= 1UL<<10; // Generate ACK
		i2c1_state++;
		break;

		case 5:
		I2C1->SR1;
		buffer_gyro->status_reg = status = I2C1->DR;
//		if((status & 0b111) != 0b111) // Complete dataset not ready - don't read further; adjust timing to sync
		if(!(status & 0b1000)) // Complete dataset not ready - don't read further; adjust timing to sync
		{
			I2C1->CR1 &= ~(1UL<<10); // Zero ACK to generate NACK (for the last data)
			I2C1->CR1 |= 1UL<<9; // generate STOP
			i2c1_state=12; // We will get one dummy data...

			// Adjust timing:
			tmp = gyro_timestep;
			tmp += I2C1_TIMESTEP_PLUS_ADJUSTMENT;
			if(tmp >= gyro_timestep_len) tmp -= gyro_timestep_len;
			gyro_timestep = tmp;
			gyro_timestep_plusses++;
			break; // Let the NACK be. Don't read all.
		}

//		if((status & 0b01110000)) // Any of the axis has overwrite condition
		if((status & 0b10000000)) // Any of the axis has overwrite condition
		{
			// Go on and read it all, but adjust timing to avoid future overwrites.
			tmp = gyro_timestep;
			tmp -= I2C1_TIMESTEP_MINUS_ADJUSTMENT;
			if(tmp < 0) tmp += gyro_timestep_len;
			gyro_timestep = tmp;
			gyro_timestep_minuses++;
		}
		// Read the rest:
		I2C1->CR1 |= 1UL<<10; // Generate ACK
		i2c1_state++;
		break;

		case 6:
		I2C1->SR1;
		buffer_gyro->x = (I2C1->DR)<<8;
		I2C1->CR1 |= 1UL<<10; // Generate ACK
		i2c1_state++;
		break;

		case 7:
		I2C1->SR1;
		buffer_gyro->x |= (I2C1->DR);
		I2C1->CR1 |= 1UL<<10; // Generate ACK
		i2c1_state++;
		break;

		case 8:
		I2C1->SR1;
		buffer_gyro->y = (I2C1->DR)<<8;
		I2C1->CR1 |= 1UL<<10; // Generate ACK
		i2c1_state++;
		break;

		case 9:
		I2C1->SR1;
		buffer_gyro->y |= (I2C1->DR);
		I2C1->CR1 |= 1UL<<10; // Generate ACK
		i2c1_state++;
		break;

		case 10:
		I2C1->SR1;
		buffer_gyro->z = (I2C1->DR)<<8;
		I2C1->CR1 &= ~(1UL<<10); // Zero ACK to generate NACK (for the last data)
		I2C1->CR1 |= 1UL<<9; // generate STOP
		i2c1_state++;
		break;

		case 11:
		I2C1->SR1;
		buffer_gyro->z |= (I2C1->DR);
		gyro_new_data = 1;
		i2c1_state=0;
		break;

		case 12:
		I2C1->SR1;
		I2C1->DR; // Dummy read needed to clear interrupt
		i2c1_state=0;
		break;
		default: break;
	}
}

void i2c1_xcel_handler()
{
	int tmp;
	uint8_t status;
	switch(i2c1_state)
	{
		case 1: // Expect SB
		I2C1->SR1;
		I2C1->DR = 0x3A;
		i2c1_state++;
		break;

		case 2: // Expect ADDR
		if(I2C1->SR1 & 2) // For some reason, there is a stupid extra interrupt, so we need to check this.
		{
			I2C1->SR2;
			I2C1->DR = 0x27 | 0x80; // Status register w/ autoincr
			I2C1->CR1 |= 1UL<<8; // START
			i2c1_state++;
		}
		break;

		case 3: // Expect SB
		if(I2C1->SR1 & 1) // For some reason, there is a stupid extra interrupt, so we need to check this.
		{
			I2C1->DR = 0x3B;
			i2c1_state++;
		}
		break;

		case 4: // Expect ADDR
		I2C1->SR1;
		I2C1->SR2;
		I2C1->CR1 |= 1UL<<10; // Generate ACK
		i2c1_state++;
		break;

		case 5:
		I2C1->SR1;
		buffer_xcel->status_reg = status = I2C1->DR;
		if((status & 0b111) != 0b111) // Complete dataset not ready - don't read further; adjust timing to sync
		{
			I2C1->CR1 &= ~(1UL<<10); // Zero ACK to generate NACK (for the last data)
			I2C1->CR1 |= 1UL<<9; // generate STOP
			i2c1_state=12; // We will get one dummy data...

			// Adjust timing:
			tmp = xcel_timestep;
			tmp += I2C1_TIMESTEP_PLUS_ADJUSTMENT;
			if(tmp >= xcel_timestep_len) tmp -= xcel_timestep_len;
			xcel_timestep = tmp;
			xcel_timestep_plusses++;
			break; // Let the NACK be. Don't read all.
		}

		if((status & 0b01110000)) // Any of the axis has overwrite condition
		{
			// Go on and read it all, but adjust timing to avoid future overwrites.
			tmp = xcel_timestep;
			tmp -= I2C1_TIMESTEP_MINUS_ADJUSTMENT;
			if(tmp < 0) tmp += xcel_timestep_len;
			xcel_timestep = tmp;
			xcel_timestep_minuses++;
		}
		// Read the rest:
		I2C1->CR1 |= 1UL<<10; // Generate ACK
		i2c1_state++;
		break;

		case 6:
		I2C1->SR1;
		buffer_xcel->x = (I2C1->DR);
		I2C1->CR1 |= 1UL<<10; // Generate ACK
		i2c1_state++;
		break;

		case 7:
		I2C1->SR1;
		buffer_xcel->x |= (I2C1->DR)<<8;
		I2C1->CR1 |= 1UL<<10; // Generate ACK
		i2c1_state++;
		break;

		case 8:
		I2C1->SR1;
		buffer_xcel->y = (I2C1->DR);
		I2C1->CR1 |= 1UL<<10; // Generate ACK
		i2c1_state++;
		break;

		case 9:
		I2C1->SR1;
		buffer_xcel->y |= (I2C1->DR)<<8;
		I2C1->CR1 |= 1UL<<10; // Generate ACK
		i2c1_state++;
		break;

		case 10:
		I2C1->SR1;
		buffer_xcel->z = (I2C1->DR);
		I2C1->CR1 &= ~(1UL<<10); // Zero ACK to generate NACK (for the last data)
		I2C1->CR1 |= 1UL<<9; // generate STOP
		i2c1_state++;
		break;

		case 11:
		I2C1->SR1;
		buffer_xcel->z |= (I2C1->DR)<<8;
		xcel_new_data = 1;
		i2c1_state=0;
		break;

		case 12:
		I2C1->SR1;
		I2C1->DR; // Dummy read needed to clear interrupt
		i2c1_state=0;
		break;

		default: break;

	}
}

void i2c1_compass_handler()
{
	switch(i2c1_state)
	{
		case 1: // Expect SB
		I2C1->SR1;
		I2C1->DR = 0x3C;
		i2c1_state++;
		break;

		case 2: // Expect ADDR
		if(I2C1->SR1 & 2) // For some reason, there is a stupid extra interrupt, so we need to check this.
		{
			I2C1->SR2;
			I2C1->DR = 0x27; // Status register address
			I2C1->CR1 |= 1UL<<8; // START
			i2c1_state++;
		}
		break;

		case 3: // Expect SB
		if(I2C1->SR1 & 1) // For some reason, there is a stupid extra interrupt, so we need to check this.
		{
			I2C1->DR = 0x3D;
			i2c1_state++;
		}
		break;

		case 4: // Expect ADDR
		I2C1->SR1;
		I2C1->SR2;
		I2C1->CR1 |= 1UL<<10; // Generate ACK
		i2c1_state++;
		break;

		case 5:
		I2C1->SR1;
		buffer_compass->status_reg = I2C1->DR;
		I2C1->CR1 |= 1UL<<10; // Generate ACK
		i2c1_state++;
		break;

		case 6:
		I2C1->SR1;
		buffer_compass->x = (I2C1->DR);
		I2C1->CR1 |= 1UL<<10; // Generate ACK
		i2c1_state++;
		break;

		case 7:
		I2C1->SR1;
		buffer_compass->x |= (I2C1->DR)<<8;
		I2C1->CR1 |= 1UL<<10; // Generate ACK
		i2c1_state++;
		break;

		case 8:
		I2C1->SR1;
		buffer_compass->y = (I2C1->DR);
		I2C1->CR1 |= 1UL<<10; // Generate ACK
		i2c1_state++;
		break;

		case 9:
		I2C1->SR1;
		buffer_compass->y |= (I2C1->DR)<<8;
		I2C1->CR1 |= 1UL<<10; // Generate ACK
		i2c1_state++;
		break;

		case 10:
		I2C1->SR1;
		buffer_compass->z = (I2C1->DR);
		I2C1->CR1 &= ~(1UL<<10); // Zero ACK to generate NACK (for the last data)
		I2C1->CR1 |= 1UL<<9; // generate STOP
		i2c1_state++;
		break;

		case 11:
		I2C1->SR1;
		buffer_compass->z |= (I2C1->DR)<<8;
		compass_new_data = 1;
		i2c1_state=0;
		break;

		default: break;
	}

}

extern volatile int new_gyro;


void i2c1_inthandler()
{
//	last_sr1 = I2C1->SR1;
//	if(last_sr1 & (1UL<<8/*bus error*/ | 1UL<<9/*arbitration loss*/ | 1UL<<10 /*ack fail*/))
/*	{
		I2C1->SR1 = 0;
		i2c1_fails++;
		i2c1_state = 0;
		NVIC_ClearPendingIRQ(I2C1_EV_IRQn);
		NVIC_DisableIRQ(I2C1_EV_IRQn); // broken I2C implementation workaround
		return;
	}
*/
	switch(i2c1_cur_device)
	{
		case I2C1_GYRO: i2c1_gyro_handler(); break;
		case I2C1_XCEL: i2c1_xcel_handler(); break;
		case I2C1_COMPASS: i2c1_compass_handler(); break;
		default: break;
	}
	I2C1->SR1 = 0;
	NVIC_ClearPendingIRQ(I2C1_EV_IRQn);
}


int start_i2c1_sequence(i2c1_device_t d)
{
	if(!i2c1_ready || i2c1_state != 0)
		return -1;

	i2c1_cur_device = d;
	i2c1_state = 1;

//	I2C1->SR1 = 0;
//	NVIC_ClearPendingIRQ(I2C1_EV_IRQn);
//	NVIC_EnableIRQ(I2C1_EV_IRQn);

	I2C1->CR1 |= 1UL<<8; // Instruct the START. The interrupt handler will take over.
	return 0;
}

void i2c1_config_byte(int dev_addr, int reg_addr, int data)
{
	delay_us(100);
	I2C1->CR1 |= 1UL<<8; // START
	while(!(I2C1->SR1 & 1)) ; // Wait for SB (Start Generated)
	I2C1->DR = dev_addr;
	while(!(I2C1->SR1 & 2)) ; // Wait for ADDR (Address sent)
	I2C1->SR2;
	while(!(I2C1->SR1 & 1UL<<7)) ;
	I2C1->DR = reg_addr;
	while(!(I2C1->SR1 & 1UL<<7)) ;
	I2C1->DR = data;
	while(!(I2C1->SR1 & 1UL<<7)) ;
	I2C1->CR1 |= 1UL<<9; // STOP
	while(!(I2C1->SR1 & 1UL<<2)) ; // Wait for BYTE TRANSFER FINISHED
	delay_us(100);
}

int init_gyro_xcel_compass()
{
	buffer_gyro = &gyro_data[0];
	latest_gyro = &gyro_data[1];

	buffer_xcel = &xcel_data[0];
	latest_xcel = &xcel_data[1];

	buffer_compass = &compass_data[0];
	latest_compass = &compass_data[1];

	GPIOE->BSRR = 1UL<<8; // DBG IO2

	/*
		I2C1 @ APB1 at 30MHz
		"Tpclk1" = 1/30MHz = 0.03333333us
		100kHz standard mode:
		Thigh = Tlow = 5us
		-> CCR = 5us/0.0333333us = 150

		TRISE: for standard mode,
		1us / 0.0333333us = 30 -> TRISE = 31
	*/

	GPIOB->BSRR = 1UL<<(8+16); // SCL low
	GPIOB->BSRR = 1UL<<(9+16); // SDA low
	GPIOB->MODER &= ~(0b1111UL<<(2*8));
	GPIOB->MODER |= (0b0101UL<<(2*8)); // general purpose outputs

	delay_ms(10);

	// Manually generate stop condition
	GPIOB->BSRR = 1UL<<(8); // SCL high
	delay_ms(10);
	GPIOB->BSRR = 1UL<<(9); // SDA high
	delay_ms(10);

	// Reset the stupid i2c
	RCC->APB1RSTR = 1UL<<21;
	delay_ms(10);
	RCC->APB1RSTR = 0;
	delay_ms(10);

	I2C1->CR2 = 0b011110 /*APB1 bus is 30MHz*/ | 1UL<<10 /*Buffer IE*/ | 1UL<<9 /*Event IE*/;
	I2C1->CCR = 0UL<<15 /*Standard speed*/ | 150UL;
	I2C1->TRISE = 30UL;

	GPIOE->BSRR = 1UL<<9; // DBG IO3

	I2C1->CR1 |= 1UL; // Enable I2C

	GPIOB->MODER &= ~(0b1111UL<<(2*8));
	GPIOB->MODER |= (0b1010UL<<(2*8));

	GPIOE->BSRR = 1UL<<10; // DBG IO4


	delay_us(1000);
//	delay_ms(100);


	GPIOE->BSRR = 1UL<<11; // DBG IO5

	// Init gyro

	i2c1_config_byte(0x40, 0x0d,
		 0b00<<6 /*64Hz LPF*/ | 0b11<<3 /*0.495Hz HPF*/ | 0<<2 /*Disable high-pass filter*/ | 0b01 /*+/- 1000 degr per second range, 1lsb = 31.25 mdeg/s*/);

	i2c1_config_byte(0x40, 0x13,
		2<<2 /*200Hz data rate*/ | 1<<1 /*ACTIVATE*/);


	// Init Accel

	i2c1_config_byte(0x3A, 0x20,
		0b100<<4 /*200Hz*/ | 1<<3 /*Must be set for proper operation*/ | 0b111 /*Z,Y,X ena*/);

	// configuring anything seems to break the xcel sensor. 200Hz setting luckily works.

//	i2c1_config_byte(0x3A, 0x21,
//		0b10<<5 /*This field ridiculously configures both LPF and HPF, when HighRes is set: LPF=200/9 Hz. HPF undefined, go figure.*/ |
//		0<<2 /* HPF off */);

//	i2c1_config_byte(0x3A, 0x23,
//		0b11<<6 /*50Hz BW AA*/ | 0b00<<4 /*+- 2g full scale*/ | 1<<3 /*Enable BW selection (lol)*/);



	// Init Compass

	i2c1_config_byte(0x3C, 0x20,
		0b11<<5 /*Ultra-high performance mode*/ | 0b100<<2 /*10Hz*/);

	i2c1_config_byte(0x3C, 0x21,
		0b11<<5 /*must be set*/);

	i2c1_config_byte(0x3C, 0x22,
		0b00<0 /*continuous*/);

	i2c1_config_byte(0x3C, 0x23,
		0b11<2 /*Z in ultra-high performance mode, too (why not?)*/);

	i2c1_config_byte(0x3C, 0x24,
		1<<6 /*"block data update"; must be set*/);

	GPIOE->BSRR = 1UL<<12; // DBG IO6


	delay_us(100);
	I2C1->SR1 = 0; // Zero regs to prevent unwanted interrupt.
	delay_us(1);
	NVIC_SetPriority(I2C1_EV_IRQn, 0b0000);
	NVIC_ClearPendingIRQ(I2C1_EV_IRQn);
	NVIC_EnableIRQ(I2C1_EV_IRQn);

	GPIOE->BSRR = 1UL<<13; // DBG IO7

	i2c1_ready = 1;

	return 0;
}


/*
	Returns bit 0='1' if the gyro has just now (during this call) given the newest reading
	   "    bit 1='1'   "    xcel             "
*/

int gyro_xcel_compass_fsm()
{
	int ret = 0;
	static int gyro_cnt = 0;
	static int xcel_cnt = 0;
	static int compass_cnt = 0;
	static int gyro_pending = 0;
	static int xcel_pending = 0;
	static int compass_pending = 0;
	static int plus_return_cnt = 0;
	static int minus_return_cnt = 0;
	static int prev_i2c1_state = -1;
	static int prev_prev_i2c1_state = -2;
	static int fix_fail = 0;

	gyro_cnt++;
	xcel_cnt++;
	compass_cnt++;

	plus_return_cnt++;
	minus_return_cnt++;
	if(minus_return_cnt >= 500000) // at 0.02Hz
	{
		minus_return_cnt = 0;
		if(xcel_timestep_minuses) xcel_timestep_minuses--;
		if(gyro_timestep_minuses) gyro_timestep_minuses--;
	}
	if(plus_return_cnt >= 5000) // at 2Hz
	{
		plus_return_cnt = 0;
		if(xcel_timestep_plusses) xcel_timestep_plusses--;
		if(gyro_timestep_plusses) gyro_timestep_plusses--;
	}

	if(xcel_timestep_minuses > 10)
	{
		xcel_timestep_minuses = 0;
		xcel_timestep_plusses = 0;
		xcel_timestep_len--;
		if(xcel_timestep >= xcel_timestep_len) xcel_timestep = xcel_timestep_len-1;
	}
	if(xcel_timestep_plusses > 100 && xcel_timestep_minuses < 3)
	{
		xcel_timestep_minuses = 0;
		xcel_timestep_plusses = 0;
		xcel_timestep_len++;
	}

	if(gyro_timestep_minuses > 10)
	{
		gyro_timestep_minuses = 0;
		gyro_timestep_plusses = 0;
		gyro_timestep_len--;

		if(gyro_timestep >= gyro_timestep_len) gyro_timestep = gyro_timestep_len-1;
	}
	if(gyro_timestep_plusses > 100 && gyro_timestep_minuses < 3)
	{
		gyro_timestep_minuses = 0;
		gyro_timestep_plusses = 0;
		gyro_timestep_len++;
	}

	if(gyro_cnt >= gyro_timestep_len) gyro_cnt = 0;
	if(xcel_cnt >= xcel_timestep_len) xcel_cnt = 0;
	if(compass_cnt >= compass_timestep_len) compass_cnt = 0;

	if(gyro_cnt == gyro_timestep) gyro_pending = 1;
	if(xcel_cnt == xcel_timestep) xcel_pending = 1;
	if(compass_cnt == compass_timestep) compass_pending = 1;

	if(i2c1_state == 0) // I2C1 ready
	{
		if(gyro_pending)
		{
			// Swap buffers
			volatile gyro_data_t *tmp;
			tmp = latest_gyro;
			latest_gyro = buffer_gyro;
			buffer_gyro = tmp;
			start_i2c1_sequence(I2C1_GYRO);
			gyro_pending = 0;
		}
		else if(xcel_pending)
		{
			volatile xcel_data_t *tmp;
			tmp = latest_xcel;
			latest_xcel = buffer_xcel;
			buffer_xcel = tmp;
			start_i2c1_sequence(I2C1_XCEL);
			xcel_pending = 0;
		}
		else if(compass_pending)
		{
			volatile compass_data_t *tmp;
			tmp = latest_compass;
			latest_compass = buffer_compass;
			buffer_compass = tmp;
			start_i2c1_sequence(I2C1_COMPASS);
			compass_pending = 0;
		}

	}
	else
	{
//		if(prev_prev_i2c1_state == prev_i2c1_state && prev_i2c1_state == i2c1_state && !fix_fail)
//		{
			// Stupid broken STM32 i2c implementation has crashed again, just try again.
//			I2C1->CR1 = 1UL<<15 /*Yes, they actually admit their shit is broken, and instruct to reset it like this!!!*/ | 1UL /*keep i2c1 on*/;
//			NVIC_ClearPendingIRQ(I2C1_EV_IRQn);
//			i2c1_fails++;
//			fix_fail = 10;
//		}

	}

/*	if(fix_fail)
	{
		fix_fail--;
		if(fix_fail == 0)
		{
			I2C1->CR1 = 1UL; // Release SW reset.
			NVIC_ClearPendingIRQ(I2C1_EV_IRQn);
			i2c1_state = 0;
		}
	}
*/
	prev_prev_i2c1_state = prev_i2c1_state;
	prev_i2c1_state = i2c1_state;


	if(gyro_new_data)
	{
		gyro_new_data = 0;
		ret |= GYRO_NEW_DATA;
	}

	if(xcel_new_data)
	{
		xcel_new_data = 0;
		ret |= XCEL_NEW_DATA;
	}

	if(compass_new_data)
	{
		compass_new_data = 0;
		ret |= COMPASS_NEW_DATA;
	}

	return ret;

}
