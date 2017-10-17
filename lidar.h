#ifndef _LIDAR_H
#define _LIDAR_H

#include <stdint.h>
#include "feedbacks.h" // for pos_t

typedef struct
{
	int valid;
	int32_t x;
	int32_t y;
} point_t;


#if defined(RN1P4) || defined(RN1P6) || defined(RN1P5)
	#define LIDAR_IGNORE_LEN 350 // mm, everything below this is marked in ignore list during ignore scan.
	#define LIDAR_IGNORE_LEN_FRONT 200 // mm, everything below this is marked in ignore list during ignore scan.
#endif
#ifdef PULU1
	#define LIDAR_IGNORE_LEN 250
	#define LIDAR_IGNORE_LEN_FRONT 120
#endif

typedef struct
{
	pos_t pos_at_start;
	point_t scan[720];
} lidar_scan_t;

#define LIVELIDAR_INVALID 1

typedef struct
{
	int status;
	int id;
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

void reset_livelidar_images(int id);

void lidar_mark_invalid();


/*
 Lidar-based 2D MAP on uart:

num_bytes
 1	uint8 start byte
 1	uint7 status
 1      uint7 id to identify when new robot coordinates have been applied.
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
