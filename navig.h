#ifndef NAVIG_H
#define NAVIG_H

void move_rel_twostep(int angle, int fwd /*in mm*/);
void navig_fsm();
lidar_scan_t* get_valid_before_lidar();
lidar_scan_t* get_valid_after_lidar();




#endif
