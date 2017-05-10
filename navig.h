#ifndef NAVIG_H
#define NAVIG_H

#include "lidar.h"

void move_rel_twostep(int angle, int fwd /*in mm*/);
void navig_fsm();
lidar_scan_t* move_get_valid_lidar(int idx);

void stop_navig_fsms();

#endif
