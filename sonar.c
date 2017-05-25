#include "ext_include/stm32f2xx.h"

#include "feedbacks.h" // for robot position
#include "sonar.h"
#include "lidar_corr.h" // for point_t, which should be moved to a more generic header.
#include "sin_lut.h"

int latest_sonars[MAX_NUM_SONARS]; // in cm

typedef struct
{
	int mm[MAX_NUM_SONARS];
	pos_t robot_pos;
} sonar_data_t;

sonar_data_t sonars[2];

// To read, take local copy of sonar_rd in case buffers are swapped in the mid of reading.
// Even if they get swapped mid processing, it takes at least 5 ms before anything is written, so you have plenty of processing time
// before data corruption occurs.
sonar_data_t* sonar_wr;
sonar_data_t* sonar_rd;

void init_sonars()
{
	sonar_wr = &sonars[0];
	sonar_rd = &sonars[1];
}

void get_sonars(point_t* out) // outputs NUM_SONARS point_ts.
{
	sonar_data_t* s = sonar_rd;

	uint32_t angle;
	int x_idx, y_idx, x, y;

	// Middle sonar is 145mm forward from the robot origin
	if(s->mm[1])
	{
		angle = s->robot_pos.ang;
		y_idx = (angle)>>SIN_LUT_SHIFT;
		x_idx = (1073741824-angle)>>SIN_LUT_SHIFT;
		out[1].x = s->robot_pos.x + (((int32_t)sin_lut[x_idx] * (int32_t)(s->mm[1]+145))>>15);
		out[1].y = s->robot_pos.y + (((int32_t)sin_lut[y_idx] * (int32_t)(s->mm[1]+145))>>15);
		out[1].valid = 1;
	}
	else
		out[1].valid = 0;

	// Left sonar is 145mm forward and 208mm left from the robot origin.
	if(s->mm[0])
	{
		angle = s->robot_pos.ang;
		y_idx = (angle)>>SIN_LUT_SHIFT;
		x_idx = (1073741824-angle)>>SIN_LUT_SHIFT;
		x = s->robot_pos.x + (((int32_t)sin_lut[x_idx] * (int32_t)(s->mm[0]+145))>>15);
		y = s->robot_pos.y + (((int32_t)sin_lut[y_idx] * (int32_t)(s->mm[0]+145))>>15);

		// Shift the result 208mm to the left:
		angle -= (uint32_t)(90*ANG_1_DEG);
		y_idx = (angle)>>SIN_LUT_SHIFT;
		x_idx = (1073741824-angle)>>SIN_LUT_SHIFT;
		x += (((int32_t)sin_lut[x_idx] * (int32_t)(208))>>15);
		y += (((int32_t)sin_lut[y_idx] * (int32_t)(208))>>15);

		out[0].x = x; out[0].y = y;

		out[0].valid = 1;
	}
	else
		out[0].valid = 0;

	// Right sonar is 145mm forward and 208mm right from the robot origin.
	if(s->mm[2])
	{
		angle = s->robot_pos.ang;
		y_idx = (angle)>>SIN_LUT_SHIFT;
		x_idx = (1073741824-angle)>>SIN_LUT_SHIFT;
		x = s->robot_pos.x + (((int32_t)sin_lut[x_idx] * (int32_t)(s->mm[2]+145))>>15);
		y = s->robot_pos.y + (((int32_t)sin_lut[y_idx] * (int32_t)(s->mm[2]+145))>>15);

		// Shift the result 208mm to the right:
		angle += (uint32_t)(90*ANG_1_DEG);
		y_idx = (angle)>>SIN_LUT_SHIFT;
		x_idx = (1073741824-angle)>>SIN_LUT_SHIFT;
		x += (((int32_t)sin_lut[x_idx] * (int32_t)(208))>>15);
		y += (((int32_t)sin_lut[y_idx] * (int32_t)(208))>>15);

		out[2].x = x; out[2].y = y;

		out[2].valid = 1;
	}
	else
		out[2].valid = 0;


}


// Must be called at 10 kHz
void sonar_fsm_10k()
{
	static int cnt_sonar;
	static int sonar_times[MAX_NUM_SONARS];
	cnt_sonar++;
	if(cnt_sonar == 1000) // Sonar with 100ms intervals
	{       // Acquisition starts
		SONAR_PULSE_ON();
		sonar_times[0] = 0;
		sonar_times[1] = 0;
		sonar_times[2] = 0;

		// Swap the buffers
		sonar_data_t* tmp = sonar_wr;
		sonar_wr = sonar_rd;
		sonar_rd = tmp;
	}
	else if(cnt_sonar == 1001)
	{
		SONAR_PULSE_OFF();
	}
	else if(cnt_sonar > 1000+300) // 30000us pulse = 517 cm top limit
	{	// Acquisition ends.
		cnt_sonar = 0;
		if(sonar_times[0] != -1) latest_sonars[0] = 0;
		if(sonar_times[1] != -1) latest_sonars[1] = 0;
		if(sonar_times[2] != -1) latest_sonars[2] = 0;
		sonar_wr->mm[0] = latest_sonars[0]*10;
		sonar_wr->mm[1] = latest_sonars[1]*10;
		sonar_wr->mm[2] = latest_sonars[2]*10;
	}
	else if(cnt_sonar > 1001) // Wait for signals
	{
		if(sonar_times[0] == 0 && SONAR1_ECHO())
			sonar_times[0] = cnt_sonar;
		else if(sonar_times[0] > 0 && !SONAR1_ECHO())
		{
			latest_sonars[0] = ((100*(cnt_sonar-sonar_times[0]))+29/*rounding*/)/58;
			sonar_times[0] = -1;
		}
		if(sonar_times[1] == 0 && SONAR2_ECHO())
			sonar_times[1] = cnt_sonar;
		else if(sonar_times[1] > 0 && !SONAR2_ECHO())
		{
			latest_sonars[1] = ((100*(cnt_sonar-sonar_times[1]))+29/*rounding*/)/58;
			sonar_times[1] = -1;
		}
		if(sonar_times[2] == 0 && SONAR3_ECHO())
			sonar_times[2] = cnt_sonar;
		else if(sonar_times[2] > 0 && !SONAR3_ECHO())
		{
			latest_sonars[2] = ((100*(cnt_sonar-sonar_times[2]))+29/*rounding*/)/58;
			sonar_times[2] = -1;
		}
/*
		if(sonar_times[3] == 0 && SONAR4_ECHO())
			sonar_times[3] = cnt_sonar;
		else if(sonar_times[3] > 0 && !SONAR4_ECHO())
		{
			latest_sonars[3] = ((100*(cnt_sonar-sonar_times[3]))+29)/58;
			sonar_times[3] = -1;
		}
*/
	}

	if(cnt_sonar == 1050)
	{
		// Save robot coords
		COPY_POS(sonar_wr->robot_pos, cur_pos);
	}


}

