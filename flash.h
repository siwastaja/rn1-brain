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

#ifndef _FLASHER_H
#define _FLASHER_H

void unlock_flash() __attribute__((section(".flasher")));
void lock_flash() __attribute__((section(".flasher")));
void flasher() __attribute__((section(".flasher")));
int flash_erase_sector(int sector) __attribute__((section(".flasher")));


#endif
