#include <inttypes.h>

#include "ext_include/stm32f2xx.h"
#include "feedbacks.h"
#include "gyro_xcel_compass.h"
#include "motcons.h"

#define LED_ON()  {GPIOC->BSRR = 1UL<<13;}
#define LED_OFF() {GPIOC->BSRR = 1UL<<(13+16);}

extern volatile int dbg1, dbg2, dbg3, dbg4;

extern volatile motcon_t motcons[NUM_MOTCONS];

extern int common_speed;

int64_t gyro_long_integrals[3];
int32_t gyro_short_integrals[3];

int gyro_timing_issues;

int cur_angle = 0; // in 1/16th degrees times 256
int aim_angle = 0;

int ang_accel = 100;
int ang_top_speed = 500000;
int ang_p = 1000;

int speed_updated;  // For timeouting robot movements (for faulty communications)
int manual_control;
int common_speed;
int manual_ang_speed;

void zero_gyro_short_integrals()
{
	gyro_short_integrals[0] = 0;
	gyro_short_integrals[1] = 0;
	gyro_short_integrals[2] = 0;
}

void move_rel_twostep(int angle, int fwd)
{
	aim_angle += angle;
	manual_control = 0;
	speed_updated = 50000;
}

void move_arc_manual(int comm, int ang)
{
	common_speed = comm;
	manual_ang_speed = ang;
	speed_updated = 5000; // robot is stopped if 0.5s is elapsed between the speed commands.
	manual_control = 1;
}

// Run this at 10 kHz
void run_feedbacks(int sens_status)
{
	int i;
	static int cnt = 0;
	static int prev_gyro_cnt = 0;
	static int speeda = 0;
	static int speedb = 0;

	static int idle = 1;

	static int ang_speed = 0;
	static int ang_speed_limit = 0;

	cnt++;

	dbg1 = aim_angle;
	dbg2 = cur_angle;

	int ang_err = aim_angle - cur_angle;
	if(ang_err < -4 || ang_err > 4)
	{
		if(idle)
		{
			idle = 0;
			ang_speed_limit = 0;
			zero_gyro_short_integrals();
		}

		// Calculate angular speed with P loop from the gyro integral.
		// Limit the value by using acceleration ramp. P loop handles the deceleration.

		ang_speed_limit += ang_accel;
		if(ang_speed_limit > ang_top_speed) ang_speed_limit = ang_top_speed;

		int new_ang_speed = ang_err*ang_p;  // 30 degrees, p=1000: 480000
		if(new_ang_speed < 0 && new_ang_speed < -1*ang_speed_limit) new_ang_speed = -1*ang_speed_limit;
		if(new_ang_speed > 0 && new_ang_speed > ang_speed_limit) new_ang_speed = ang_speed_limit;

		ang_speed = new_ang_speed;
		common_speed = 0;

	}
	else
	{
		idle=1;
		ang_speed = 0;
	}

	if(sens_status & GYRO_NEW_DATA)
	{
		int gyro_dt = 1; //cnt - prev_gyro_cnt;
/*		if(gyro_dt < 1 || gyro_dt > 5)
		{
			gyro_dt = 1;
			gyro_timing_issues++;
		}
*/
		int latest[3] = {latest_gyro->x*gyro_dt, latest_gyro->y*gyro_dt, latest_gyro->z*gyro_dt};

		for(i=0; i<3; i++)
		{
			if(latest[i] < -GYRO_LONG_INTEGRAL_IGNORE_LEVEL || latest[i] > GYRO_LONG_INTEGRAL_IGNORE_LEVEL)
				gyro_long_integrals[i] += latest[i];

			gyro_short_integrals[i] += latest[i];
		}

		//1 unit = 31.25 mdeg/s; at 200Hz timestep, it's 0.15625 mdeg/s, which is 2.5m 1/16th steps/s
		cur_angle += latest[3]
	}

	int error;
	if(manual_control)
		error = -1*latest_gyro->z/32 - manual_ang_speed;
	else
		error = -1*latest_gyro->z/32 - (ang_speed>>12);

	speeda += error>>2;
	speedb -= error>>2;

	if((cnt&0x3f) == 0x3f) // slow decay
	{
		if(speeda > 0) speeda--;
		else if(speeda < 0) speeda++;

		if(speedb > 0) speedb--;
		else if(speedb < 0) speedb++;
	}

	if(speeda > MAX_DIFFERENTIAL_SPEED*256) speeda = MAX_DIFFERENTIAL_SPEED*256;
	else if(speeda < -MAX_DIFFERENTIAL_SPEED*256) speeda = -MAX_DIFFERENTIAL_SPEED*256;
	if(speedb > MAX_DIFFERENTIAL_SPEED*256) speedb = MAX_DIFFERENTIAL_SPEED*256;
	else if(speedb < -MAX_DIFFERENTIAL_SPEED*256) speedb = -MAX_DIFFERENTIAL_SPEED*256;

	int a = common_speed + speeda/512;
	int b = common_speed + speedb/512;

	a>>=1;
	b>>=1;

	if(a > MAX_SPEED) a=MAX_SPEED;
	else if(a < -MAX_SPEED) a=-MAX_SPEED;
	if(b > MAX_SPEED) b=MAX_SPEED;
	else if(b < -MAX_SPEED) b=-MAX_SPEED;

	if(speed_updated)
	{
		LED_OFF();
		speed_updated--;
		motcons[2].cmd.speed = a;
		motcons[3].cmd.speed = -1*b;
	}
	else
	{
		LED_ON();
		motcons[2].cmd.speed = 0;
		motcons[3].cmd.speed = 0;
		speeda=0;
		speedb=0;
	}

}
