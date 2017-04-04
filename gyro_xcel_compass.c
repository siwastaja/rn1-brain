#include <stdint.h>
#include "ext_include/stm32f2xx.h"

#include "gyro_xcel_compass.h"

extern void delay_us(uint32_t i);
extern void delay_ms(uint32_t i);


extern volatile xcel_data_t latest_xcel;
extern volatile gyro_data_t latest_gyro;
extern volatile compass_data_t latest_compass;

volatile int i2c1_ready = 0;

/*
	STM32 I2C implementation is a total catastrophe, it requires almost bitbanging-like sequencing by software,
	and provides no proper buffer registers (let alone a FIFO) - so we need to poll some status bits and write
	the data register at the exactly correct timing!

	For maximum performance, we hardwrite the optimal sequence of reading the relevant data from the
	gyro, accelerometer and compass. Because using the I2C implementation requires a lot of waiting
	for status bits, DMA is of little use; we use an interrupt handler implementing a state machine.
*/
volatile int i2c1_state = 0;

void i2c1_inthandler()
{
	switch(i2c1_state)
	{
		case 0:
		if(I2C1->SR1 & 1) // SB = Start Generated
		{
			I2C1->DR = 0x40;
			i2c1_state++;
		}
		break;

		case 1:
		if(I2C1->SR1 & 2) // ADDR = Address sent
		{
			I2C1->SR2;
			I2C1->DR = 0x00; // Status register address
			I2C1->CR1 |= 1UL<<8; // START
			i2c1_state++;
		}

		case 2:
		if(I2C1->SR1 & 1) // SB = Start Generated
		{
			I2C1->DR = 0x41;
			i2c1_state++;
		}
		break;

		case 3:
		if(I2C1->SR1 & 2) // ADDR = Address sent
		{
			I2C1->SR2;
			I2C1->CR1 |= 1UL<<10; // Generate ACK
			i2c1_state++;
		}
		break;

		case 4:
		if(I2C1->SR1 & (1UL<<6))
		{
			latest_gyro.status_reg = I2C1->DR;
			I2C1->CR1 |= 1UL<<10; // Generate ACK
			i2c1_state++;
		}
		break;

		case 5:
		if(I2C1->SR1 & (1UL<<6))
		{
			latest_gyro.x = (I2C1->DR)<<8;
			I2C1->CR1 |= 1UL<<10; // Generate ACK
			i2c1_state++;
		}
		break;

		case 6:
		if(I2C1->SR1 & (1UL<<6))
		{
			latest_gyro.x |= (I2C1->DR);
			I2C1->CR1 |= 1UL<<10; // Generate ACK
			i2c1_state++;
		}
		break;

		case 7:
		if(I2C1->SR1 & (1UL<<6))
		{
			latest_gyro.y = (I2C1->DR)<<8;
			I2C1->CR1 |= 1UL<<10; // Generate ACK
			i2c1_state++;
		}
		break;

		case 8:
		if(I2C1->SR1 & (1UL<<6))
		{
			latest_gyro.y |= (I2C1->DR);
			I2C1->CR1 |= 1UL<<10; // Generate ACK
			i2c1_state++;
		}
		break;

		case 9:
		if(I2C1->SR1 & (1UL<<6))
		{
			latest_gyro.z = (I2C1->DR)<<8;
			I2C1->CR1 &= ~(1UL<<10); // Zero ACK to generate NACK (for the last data)
			I2C1->CR1 |= 1UL<<9; // generate STOP
			i2c1_state++;
		}
		break;

		case 10:
		if(I2C1->SR1 & (1UL<<6))
		{
			latest_gyro.z |= (I2C1->DR);
			I2C1->CR1 |= 1UL<<8; // Generate start
			i2c1_state++;
		}
		break;




		case 11:
		if(I2C1->SR1 & 1) // SB = Start Generated
		{
			I2C1->DR = 0x3A;
			i2c1_state++;
		}
		break;

		case 12:
		if(I2C1->SR1 & 2) // ADDR = Address sent
		{
			I2C1->SR2;
			I2C1->DR = 0x27 | 0x80; // Status register w/ autoincr
			I2C1->CR1 |= 1UL<<8; // START
			i2c1_state++;
		}

		case 13:
		if(I2C1->SR1 & 1) // SB = Start Generated
		{
			I2C1->DR = 0x3B;
			i2c1_state++;
		}
		break;

		case 14:
		if(I2C1->SR1 & 2) // ADDR = Address sent
		{
			I2C1->SR2;
			I2C1->CR1 |= 1UL<<10; // Generate ACK
			i2c1_state++;
		}
		break;

		case 15:
		if(I2C1->SR1 & (1UL<<6))
		{
			latest_xcel.status_reg = I2C1->DR;
			I2C1->CR1 |= 1UL<<10; // Generate ACK
			i2c1_state++;
		}
		break;

		case 16:
		if(I2C1->SR1 & (1UL<<6))
		{
			latest_xcel.x = (I2C1->DR);
			I2C1->CR1 |= 1UL<<10; // Generate ACK
			i2c1_state++;
		}
		break;

		case 17:
		if(I2C1->SR1 & (1UL<<6))
		{
			latest_xcel.x |= (I2C1->DR)<<8;
			I2C1->CR1 |= 1UL<<10; // Generate ACK
			i2c1_state++;
		}
		break;

		case 18:
		if(I2C1->SR1 & (1UL<<6))
		{
			latest_xcel.y = (I2C1->DR);
			I2C1->CR1 |= 1UL<<10; // Generate ACK
			i2c1_state++;
		}
		break;

		case 19:
		if(I2C1->SR1 & (1UL<<6))
		{
			latest_xcel.y |= (I2C1->DR)<<8;
			I2C1->CR1 |= 1UL<<10; // Generate ACK
			i2c1_state++;
		}
		break;

		case 20:
		if(I2C1->SR1 & (1UL<<6))
		{
			latest_xcel.z = (I2C1->DR);
			I2C1->CR1 &= ~(1UL<<10); // Zero ACK to generate NACK (for the last data)
			I2C1->CR1 |= 1UL<<9; // generate STOP
			i2c1_state++;
		}
		break;

		case 21:
		if(I2C1->SR1 & (1UL<<6))
		{
			latest_xcel.z |= (I2C1->DR)<<8;
			I2C1->CR1 |= 1UL<<8; // Generate start
			i2c1_state++;
		}
		break;




		case 22:
		if(I2C1->SR1 & 1) // SB = Start Generated
		{
			I2C1->DR = 0x3C;
			i2c1_state++;
		}
		break;

		case 23:
		if(I2C1->SR1 & 2) // ADDR = Address sent
		{
			I2C1->SR2;
			I2C1->DR = 0x27; // Status register address
			I2C1->CR1 |= 1UL<<8; // START
			i2c1_state++;
		}

		case 24:
		if(I2C1->SR1 & 1) // SB = Start Generated
		{
			I2C1->DR = 0x3D;
			i2c1_state++;
		}
		break;

		case 25:
		if(I2C1->SR1 & 2) // ADDR = Address sent
		{
			I2C1->SR2;
			I2C1->CR1 |= 1UL<<10; // Generate ACK
			i2c1_state++;
		}
		break;

		case 26:
		if(I2C1->SR1 & (1UL<<6))
		{
			latest_compass.status_reg = I2C1->DR;
			I2C1->CR1 |= 1UL<<10; // Generate ACK
			i2c1_state++;
		}
		break;

		case 27:
		if(I2C1->SR1 & (1UL<<6))
		{
			latest_compass.x = (I2C1->DR);
			I2C1->CR1 |= 1UL<<10; // Generate ACK
			i2c1_state++;
		}
		break;

		case 28:
		if(I2C1->SR1 & (1UL<<6))
		{
			latest_compass.x |= (I2C1->DR)<<8;
			I2C1->CR1 |= 1UL<<10; // Generate ACK
			i2c1_state++;
		}
		break;

		case 29:
		if(I2C1->SR1 & (1UL<<6))
		{
			latest_compass.y = (I2C1->DR);
			I2C1->CR1 |= 1UL<<10; // Generate ACK
			i2c1_state++;
		}
		break;

		case 30:
		if(I2C1->SR1 & (1UL<<6))
		{
			latest_compass.y |= (I2C1->DR)<<8;
			I2C1->CR1 |= 1UL<<10; // Generate ACK
			i2c1_state++;
		}
		break;

		case 31:
		if(I2C1->SR1 & (1UL<<6))
		{
			latest_compass.z = (I2C1->DR);
			I2C1->CR1 &= ~(1UL<<10); // Zero ACK to generate NACK (for the last data)
			I2C1->CR1 |= 1UL<<9; // generate STOP
			i2c1_state++;
		}
		break;

		case 32:
		if(I2C1->SR1 & (1UL<<6))
		{
			latest_compass.z |= (I2C1->DR)<<8;
			i2c1_state=0;
		}
		break;



		default:
		break;
	}

}

int start_gyro_xcel_compass_sequence()
{
	if(!i2c1_ready || i2c1_state != 0)
		return -1;

	I2C1->CR1 |= 1UL<<8; // Instruct the START. The interrupt handler will take over.
		return 0;
}

int init_gyro_xcel_compass()
{
	/*
		I2C1 @ APB1 at 30MHz
		"Tpclk1" = 1/30MHz = 0.03333333us
		100kHz standard mode:
		Thigh = Tlow = 5us
		-> CCR = 5us/0.0333333us = 150

		TRISE: for standard mode,
		1us / 0.0333333us = 30 -> TRISE = 31
	*/
	I2C1->CR2 = 0b011110 /*APB1 bus is 30MHz*/ | 1UL<<10 /*Buffer IE*/ | 1UL<<9 /*Event IE*/;
	I2C1->CCR = 0UL<<15 /*Standard speed*/ | 150UL;
	I2C1->TRISE = 30UL;
	I2C1->CR1 |= 1UL; // Enable I2C

	delay_ms(1);

	// Init gyro

	I2C1->CR1 |= 1UL<<8; // START
	while(!(I2C1->SR1 & 1)) ; // Wait for SB (Start Generated)
	I2C1->DR = 0x40;
	while(!(I2C1->SR1 & 2)) ; // Wait for ADDR (Address sent)
	I2C1->SR2;
	while(!(I2C1->SR1 & 1UL<<7)) ;
	I2C1->DR = 0x13;
	while(!(I2C1->SR1 & 1UL<<7)) ;
	I2C1->DR = 0b10; // Go to ACTIVE mode
	while(!(I2C1->SR1 & 1UL<<7)) ;
	I2C1->CR1 |= 1UL<<9; // STOP
	while(!(I2C1->SR1 & 1UL<<2)) ; // Wait for BYTE TRANSFER FINISHED

	// Init Accel

	I2C1->CR1 |= 1UL<<8; // START
	while(!(I2C1->SR1 & 1)) ; // Wait for SB (Start Generated)
	I2C1->DR = 0x3A;
	while(!(I2C1->SR1 & 2)) ; // Wait for ADDR (Address sent)
	I2C1->SR2;
	while(!(I2C1->SR1 & 1UL<<7)) ;
	I2C1->DR = 0x20; // CTRL_REG1_A
	while(!(I2C1->SR1 & 1UL<<7)) ;
	I2C1->DR = 0b101<<4 /*400Hz*/ | 1<<3 /*Must be set*/ | 0b111 /*Z,Y,X ena*/;
	while(!(I2C1->SR1 & 1UL<<7)) ;
	I2C1->CR1 |= 1UL<<9; // STOP
	while(!(I2C1->SR1 & 1UL<<2)) ; // Wait for BYTE TRANSFER FINISHED


	// Init Compass

	I2C1->CR1 |= 1UL<<8; // START
	while(!(I2C1->SR1 & 1)) ; // Wait for SB (Start Generated)
	I2C1->DR = 0x3C;
	while(!(I2C1->SR1 & 2)) ; // Wait for ADDR (Address sent)
	I2C1->SR2;
	while(!(I2C1->SR1 & 1UL<<7)) ;
	I2C1->DR = 0x20; // CTRL_REG1_M
	while(!(I2C1->SR1 & 1UL<<7)) ;
	I2C1->DR = 0b11<<5 /*Ultra-high performance mode*/ | 0b101<<2 /*20Hz*/ | 0 /* SELF TEST*/;
	while(!(I2C1->SR1 & 1UL<<7)) ;
	I2C1->CR1 |= 1UL<<9; // STOP
	while(!(I2C1->SR1 & 1UL<<2)) ; // Wait for BYTE TRANSFER FINISHED

	I2C1->CR1 |= 1UL<<8; // START
	while(!(I2C1->SR1 & 1)) ; // Wait for SB (Start Generated)
	I2C1->DR = 0x3C;
	while(!(I2C1->SR1 & 2)) ; // Wait for ADDR (Address sent)
	I2C1->SR2;
	while(!(I2C1->SR1 & 1UL<<7)) ;
	I2C1->DR = 0x21; // CTRL_REG2_M
	while(!(I2C1->SR1 & 1UL<<7)) ;
	I2C1->DR = 0b11<<5 /*must be set*/;
	while(!(I2C1->SR1 & 1UL<<7)) ;
	I2C1->CR1 |= 1UL<<9; // STOP
	while(!(I2C1->SR1 & 1UL<<2)) ; // Wait for BYTE TRANSFER FINISHED

	I2C1->CR1 |= 1UL<<8; // START
	while(!(I2C1->SR1 & 1)) ; // Wait for SB (Start Generated)
	I2C1->DR = 0x3C;
	while(!(I2C1->SR1 & 2)) ; // Wait for ADDR (Address sent)
	I2C1->SR2;
	while(!(I2C1->SR1 & 1UL<<7)) ;
	I2C1->DR = 0x22; // CTRL_REG3_M
	while(!(I2C1->SR1 & 1UL<<7)) ;
	I2C1->DR = 0b00<0 /*continuous*/;
	while(!(I2C1->SR1 & 1UL<<7)) ;
	I2C1->CR1 |= 1UL<<9; // STOP
	while(!(I2C1->SR1 & 1UL<<2)) ; // Wait for BYTE TRANSFER FINISHED

	I2C1->CR1 |= 1UL<<8; // START
	while(!(I2C1->SR1 & 1)) ; // Wait for SB (Start Generated)
	I2C1->DR = 0x3C;
	while(!(I2C1->SR1 & 2)) ; // Wait for ADDR (Address sent)
	I2C1->SR2;
	while(!(I2C1->SR1 & 1UL<<7)) ;
	I2C1->DR = 0x23; // CTRL_REG4_M
	while(!(I2C1->SR1 & 1UL<<7)) ;
	I2C1->DR = 0b11<2 /*Z in ultra-high, too*/;
	while(!(I2C1->SR1 & 1UL<<7)) ;
	I2C1->CR1 |= 1UL<<9; // STOP
	while(!(I2C1->SR1 & 1UL<<2)) ; // Wait for BYTE TRANSFER FINISHED

	I2C1->CR1 |= 1UL<<8; // START
	while(!(I2C1->SR1 & 1)) ; // Wait for SB (Start Generated)
	I2C1->DR = 0x3C;
	while(!(I2C1->SR1 & 2)) ; // Wait for ADDR (Address sent)
	I2C1->SR2;
	while(!(I2C1->SR1 & 1UL<<7)) ;
	I2C1->DR = 0x24; // CTRL_REG5_M
	while(!(I2C1->SR1 & 1UL<<7)) ;
	I2C1->DR = 1<<6 /*"block data update"; must be set*/;
	while(!(I2C1->SR1 & 1UL<<7)) ;
	I2C1->CR1 |= 1UL<<9; // STOP
	while(!(I2C1->SR1 & 1UL<<2)) ; // Wait for BYTE TRANSFER FINISHED

	I2C1->SR1 = 0; // Zero regs.

	NVIC_EnableIRQ(I2C1_EV_IRQn);

	i2c1_ready = 1;

	return 0;
}
