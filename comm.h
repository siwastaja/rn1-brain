#ifndef _COMM_H
#define _COMM_H

// resolution-lossy conversion from int16_t to sint14
#define I16_I14(in) (((in)&0xfe00)>>1 | ((in)&0x1fc)>>2)
#define I14_I16(in) (int16_t)((((in)&0x7f00)<<1 | ((in)&0x7f)<<2))
#define I16_MS(in)  (((int16_t)(in)&0xfe00)>>9)
#define I16_LS(in)  (((int16_t)(in)&0x1fc)>>2)

// Nonlossy, result ranges from -8192 to 8191
#define I7I7_I16(ms, ls) ((int16_t)( ((ms)<<9) | ((ls)<<2) )>>2)


#define I32_I7_0(in) ( ((int32_t)((in))) & 0x7f )
#define I32_I7_1(in) ( ((int32_t)((in)>>7)) & 0x7f )
#define I32_I7_2(in) ( ((int32_t)((in)>>14)) & 0x7f )
#define I32_I7_3(in) ( ((int32_t)((in)>>21)) & 0x7f )
#define I32_I7_4(in) ( ((int32_t)((in)>>28)) & 0x7f )
#define I7x5_I32(b4,b3,b2,b1,b0) ((int32_t)( ((b4)<<28) | ((b3)<<21) | ((b2)<<14) | ((b1)<<7) | ((b0)) ))

#define I16_I7_0(in) ( ((int32_t)((in))) & 0x7f )
#define I16_I7_1(in) ( ((int32_t)((in)>>7)) & 0x7f )
#define I16_I7_2(in) ( ((int32_t)((in)>>14)) & 0x7f )
#define I7x3_I16(b2,b1,b0) ((int32_t)( ((b2)<<14) | ((b1)<<7) | ((b0)) ))


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

