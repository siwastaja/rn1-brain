#ifndef _GYRO_XCEL_COMPASS_H
#define _GYRO_XCEL_COMPASS_H

#include <stdint.h>

typedef struct
{
	uint8_t status_reg;
	int16_t x;
	int16_t y;
	int16_t z;
} gyro_data_t;

typedef struct
{
	uint8_t status_reg;
	int16_t x;
	int16_t y;
	int16_t z;
} xcel_data_t;


typedef struct
{
	uint8_t status_reg;
	int16_t x;
	int16_t y;
	int16_t z;
} compass_data_t;

int init_gyro_xcel_compass();

#define GYRO_NEW_DATA (1)
#define XCEL_NEW_DATA (2)
#define COMPASS_NEW_DATA (4)
int gyro_xcel_compass_fsm();

#endif
