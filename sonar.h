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

#ifndef _SONAR_H
#define _SONAR_H

typedef struct
{
	int32_t x;
	int32_t y;
	int16_t z; // from robot floor level
	int8_t  c; // classification
} sonar_xyz_t;


void sonar_fsm_10k();
void init_sonars();

sonar_xyz_t* get_sonar_point();

#endif
