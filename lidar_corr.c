/*
	Lidar-based error correction
	Tries to remove any (ang,x,y) error between two adjacent robot positions with two lidar scans.

	Inputs:
	lidar_scan_t *before, lidar_scan_t *after, both include the assumed (ang,x,y) coordinates.

	Outputs:
	(ang,x,y) corrections
	
*/

#include <stdint.h>
#include "lidar_corr.h"


