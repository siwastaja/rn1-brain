#ifndef LIDAR_CORR_H
#define LIDAR_CORR_H

#include "lidar.h"

int do_lidar_corr(lidar_scan_t* scan1, lidar_scan_t* scan2, pos_t* corr);

void live_lidar_calc_must_be_finished();
void apply_corr_to_livelidar(live_lidar_scan_t* lid);
void livelidar_storage_finished();
int livelidar_fsm(int allowed_to_send_lidar);

int livelidar_skip();

void reset_lidar_corr_images();


#endif
