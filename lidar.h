#ifndef _LIDAR_H
#define _LIDAR_H

#include <stdint.h>

#define LIDAR_IGNORE_LEN 380 // mm

#define DEFAULT_LIDAR_RPM 300
// For determining whether the lidar is turning within the specs, so that the data flow can be synchronized:
#define MAX_LIDAR_RPM 340
#define MIN_LIDAR_RPM 260

extern volatile int lidar_rpm_setpoint_x64;
extern uint8_t lidar_ignore[360];


typedef struct __attribute__ ((__packed__))
{
	uint16_t flags_distance;
	uint16_t signal;
} lidar_d_t;

typedef union
{
	struct __attribute__ ((__packed__))
	{
		uint8_t start;
		uint8_t idx;
		uint16_t speed;
		lidar_d_t d[4];
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
void deinit_lidar();

void generate_lidar_ignore();
void copy_lidar_half1(int16_t* dst_start);
void copy_lidar_half2(int16_t* dst_start);




#endif
