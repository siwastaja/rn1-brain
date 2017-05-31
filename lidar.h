#ifndef _LIDAR_H
#define _LIDAR_H

#include <stdint.h>
#include "feedbacks.h" // for pos_t

#define LIDAR_IGNORE_LEN 380 // mm, everything below this is marked in ignore list during ignore scan.
#define LIDAR_LIVE_IGNORE_LEN 300 // mm, everything below this is always ignored, during normal operation

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

typedef struct
{
	pos_t pos;
	int16_t scan[360];
} lidar_scan_t;

#define LIVELIDAR_INVALID 1

typedef struct
{
	int status;
	pos_t pos[90]; // Each four points share the same position.
	int16_t scan[360];
} live_lidar_scan_t;


void sync_lidar();
void resync_lidar();
void init_lidar();
uint16_t lidar_calc_checksum(volatile lidar_datum_t* l);
void lidar_motor_ctrl_loop();
void deinit_lidar();

void generate_lidar_ignore();
void copy_lidar_half1(int16_t* dst_start);
void copy_lidar_half2(int16_t* dst_start);
void copy_lidar_full(int16_t* dst_start);

void lidar_reset_flags();
void lidar_reset_complete_flag(); 
void lidar_reset_half_flag(); 
int lidar_is_complete();
int lidar_is_half();

void lidar_fsm();

void reset_livelidar_images();

void lidar_mark_invalid();


/*
 Lidar-based 2D MAP on uart:

num_bytes
 1	uint8 start byte
 1	uint7 status
 2	int14 cur_ang (at the middle point of the lidar scan)  (not used for turning the image, just to include robot coords)
 5	int32 cur_x   ( " " )
 5	int32 cur_y   ( " " )
 1	int7  correction return value
 2	int14 ang_corr (for information only)
 2	int14 x_corr (for information only)
 2	int14 y_corr (for information only) 
1440	360 * point
	  2	int14  x referenced to cur_x
	  2	int14  y referenced to cur_y

	Total: 1461
	Time to tx at 115200: ~130 ms

*/


#endif
