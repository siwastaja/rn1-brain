#ifndef NAVIG_H
#define NAVIG_H

void move_rel_twostep(int angle, int fwd /*in mm*/);
void navig_fsm();
lidar_scan_t* get_valid_before_lidar();
lidar_scan_t* get_valid_after_lidar();
lidar_scan_t* move_get_valid_lidar(int idx);

void stop_navig_fsms();

#endif
