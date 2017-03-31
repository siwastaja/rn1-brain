#ifndef _LIDAR_H
#define _LIDAR_H

#include <stdint.h>


#define DEFAULT_LIDAR_RPM 300

extern volatile int lidar_rpm_setpoint_x64;

//typedef struct __attribute__ ((__packed__))
//{
//
//}

typedef union
{
	struct __attribute__ ((__packed__))
	{
		uint8_t start;
		uint8_t idx;
		uint16_t speed;
		uint32_t data0;
		uint32_t data1;
		uint32_t data2;
		uint32_t data3;
		uint16_t checksum;
	};
	uint16_t u16[11];
	uint8_t u8[22];
} lidar_datum_t;


void sync_lidar();
void resync_lidar();
void init_lidar();
uint16_t lidar_calc_checksum(volatile lidar_datum_t* l);
void lidar_ctrl_loop();


#endif
