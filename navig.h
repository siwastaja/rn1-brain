/*
	PULUROBOT RN1-BRAIN RobotBoard main microcontroller firmware project

	(c) 2017-2018 Pulu Robotics and other contributors

	This program is free software; you can redistribute it and/or modify
	it under the terms of the GNU General Public License version 2, as 
	published by the Free Software Foundation.

	This program is distributed in the hope that it will be useful,
	but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
	GNU General Public License for more details.

	GNU General Public License version 2 is supplied in file LICENSING.

*/

#ifndef NAVIG_H
#define NAVIG_H

#include "lidar.h"

void move_rel_twostep(int angle32, int fwd /*in mm*/, int speedlim);
void move_absa_rels_twostep(int angle32, int fwd /*in mm*/, int speedlim);
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

void ena_coll_avoid();
void dis_coll_avoid();
void stop_movement();

void limit_speed(int speed);
void micronavi_point_in(int32_t x, int32_t y, int16_t z, int stop_if_necessary, int source);


typedef struct
{
	int16_t first_movement_needed; // distance the robot needed to go fwd or back as the very first operation. 0 if within tolerances. in mm.
	uint8_t turning_passes_needed; // optical positioning needed to move the robot this many passes without needing to back of / go forward again (adjusting angle was enough alone)
	uint8_t vexling_passes_needed; // optical positioning needed to move the robot this many passes, doing a back-off-go-forward pass.
	uint8_t accepted_pos;          // 1, if optical positioning succesful. 0 if failed there.
	int16_t dist_before_push;      // after succesful optical positioning, the measured distance to the charger right before the push. in mm.
	uint8_t result;                // 100 = success. Others = failure.
} chafind_results_t;

extern chafind_results_t chafind_results;
extern volatile int send_chafind_results;

#endif
