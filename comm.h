#ifndef _COMM_H
#define _COMM_H

// resolution-lossy conversion from int16_t to sint14
#define I16_I14(in) ((in&0xfe00)>>1 | (in&0x1fc)>>2)
#define I14_I16(in) (int16_t)(((in&0x7f00)<<1 | (in&0x7f)<<2))

typedef struct __attribute__ ((__packed__))
{
	uint8_t status;
	int16_t int_x;
	int16_t int_y;
	int16_t int_z;
} msg_gyro_t;

typedef struct __attribute__ ((__packed__))
{
	uint8_t status;
	int16_t int_x;
	int16_t int_y;
	int16_t int_z;
} msg_xcel_t;

typedef struct __attribute__ ((__packed__))
{
	uint8_t status;
	int16_t x;
	int16_t y;
	int16_t z;
} msg_compass_t;

#endif

