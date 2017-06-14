#ifndef NAVIG_H
#define NAVIG_H

#include "lidar.h"

void move_rel_twostep(int angle, int fwd /*in mm*/);
void move_xy_abs(int32_t x, int32_t y, int back_mode, int id, int speedlim);
void navig_fsm1();
void navig_fsm2();


lidar_scan_t* move_get_valid_lidar(int idx);
lidar_scan_t* move_get_lidar(int idx);

void move_mark_lidar_nonread(int idx);

void stop_navig_fsms();

int get_xy_left();
int get_xy_id();
uint32_t get_obstacle_avoidance_stop_flags();
uint32_t get_obstacle_avoidance_action_flags();

void set_obstacle_avoidance_margin(int cm);

void find_charger();

void daiju_mode_on();
void daiju_mode_off();


#endif
