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
