/*
	PULUROBOT RN1-BRAIN RobotBoard main microcontroller firmware project

	(c) 2017-2018 Pulu Robotics and other contributors
	Maintainer: Antti Alhonen <antti.alhonen@iki.fi>

	This program is free software; you can redistribute it and/or modify
	it under the terms of the GNU General Public License version 2, as 
	published by the Free Software Foundation.

	This program is distributed in the hope that it will be useful,
	but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
	GNU General Public License for more details.

	GNU General Public License version 2 is supplied in file LICENSING.

*/

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
