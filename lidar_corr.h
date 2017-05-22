#ifndef LIDAR_CORR_H
#define LIDAR_CORR_H

#include "lidar.h"

typedef struct
{
	int valid;
	int32_t x;
	int32_t y;
} point_t;

int do_lidar_corr(lidar_scan_t* scan1, lidar_scan_t* scan2, pos_t* corr);

void live_lidar_calc_must_be_finished();
void apply_corr_to_livelidar(live_lidar_scan_t* lid);
void livelidar_storage_finished();


#endif
