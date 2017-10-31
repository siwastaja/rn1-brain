#ifndef _SONAR_H
#define _SONAR_H

#define NUM_SONARS 2

typedef struct
{
	int32_t x;
	int32_t y;
	int16_t z;
} sonar_xyz_t;


void sonar_fsm_10k();
void init_sonars();

xyc_t* get_sonar_point();

#endif
