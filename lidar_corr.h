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


#endif
