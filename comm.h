#ifndef _COMM_H
#define _COMM_H

// resolution-lossy conversion from int16_t to sint14
#define I16_I14(in) ((in&0xfe00)>>1 | (in&0x1fc)>>2)

typedef struct
{
	uint8_t status;
	int16_t int_x;
	int16_t int_y;
	int16_t int_z;
} msg_gyro_t;

typedef struct
{
	uint8_t status;
	int16_t int_x;
	int16_t int_y;
	int16_t int_z;
} msg_xcel_t;

#endif

