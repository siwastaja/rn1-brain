/*
Mechanical feedback module.

Keeps track of position & angle, controls the motors.
*/

#include <inttypes.h>
#include <math.h>

#include "ext_include/stm32f2xx.h"
#include "feedbacks.h"
#include "gyro_xcel_compass.h"
#include "motcons.h"
#include "sonar.h"
#include "sin_lut.h"
#include "navig.h"

#define LED_ON()  {GPIOC->BSRR = 1UL<<13;}
#define LED_OFF() {GPIOC->BSRR = 1UL<<(13+16);}

extern volatile int dbg[10];
extern int latest_sonars[MAX_NUM_SONARS]; // in cm, 0 = no echo

int64_t gyro_long_integrals[3];
int64_t gyro_short_integrals[3];

int64_t xcel_long_integrals[3];
int64_t xcel_short_integrals[3];

int xcel_dc_corrs[3];
int gyro_dc_corrs[3];

pos_t cur_pos;

int gyro_timing_issues;
int xcel_timing_issues;

int cur_compass_angle = 0;
int aim_angle = 0; // same

int ang_accel = 300; // was 220
int ang_top_speed;
int ang_p = 1000; //1350; // 1500

int aim_fwd;
int final_fwd_accel = 400;
int fwd_accel = 350; // was 250, kinda sluggish
int fwd_top_speed;
int fwd_p = 3500; // 5000 gives rather strong deceleration; 2500 feels sluggish

volatile int manual_control;
volatile int manual_common_speed;
volatile int manual_ang_speed;

int robot_nonmoving_cnt = 0;
int robot_nonmoving = 0;

static void robot_moves()
{
	robot_nonmoving_cnt = 0;
	robot_nonmoving = 0;
}

int robot_moving()
{
	return !robot_nonmoving;
}

static int host_alive_watchdog;
void host_alive()
{
	host_alive_watchdog = 5000;
}

void zero_gyro_short_integrals()
{
	gyro_short_integrals[0] = 0;
	gyro_short_integrals[1] = 0;
	gyro_short_integrals[2] = 0;
}

void zero_xcel_short_integrals()
{
	xcel_short_integrals[0] = 0;
	xcel_short_integrals[1] = 0;
	xcel_short_integrals[2] = 0;
}

void zero_xcel_long_integrals()
{
	xcel_long_integrals[0] = 0;
	xcel_long_integrals[1] = 0;
	xcel_long_integrals[2] = 0;
}

static int fwd_speed_limit;
static int speed_limit_lowered;

void take_control()
{
	manual_control = 0;
}

void rotate_rel(int angle)
{
	aim_angle += angle;

	speed_limit_lowered = 0;
	ang_top_speed = 300000; // was 220000
	manual_control = 0;
	robot_moves();
}

void rotate_abs(int angle)
{
	aim_angle = angle;

	speed_limit_lowered = 0;
	ang_top_speed = 220000; // 150000
	manual_control = 0;
	robot_moves();
}

void change_angle_abs(int angle)
{
	aim_angle = angle;
}

void straight_rel(int fwd /*in mm*/)
{
	speed_limit_lowered = 0;
	fwd_accel = final_fwd_accel/4;
	fwd_speed_limit = fwd_accel*80; // use starting speed that equals to 80ms of acceleration
	aim_fwd = fwd*10; // in 0.1mm
	fwd_top_speed = 800000; // was 600000
	manual_control = 0;
	robot_moves();
}

void change_straight_rel(int fwd /*in mm*/)
{
	aim_fwd = fwd*10; // in 0.1mm
}

void reset_movement()
{
	aim_fwd = 0;
	aim_angle = cur_pos.ang;
}

static int auto_keepstill;

static int do_correct_angle = 0;
static int do_correct_fwd = 0;

static int angular_allowed = 1;
static int straight_allowed = 1;

void allow_angular(int yes)
{
	angular_allowed = yes;
}

void allow_straight(int yes)
{
	straight_allowed = yes;
}

void auto_disallow(int yes)
{
	auto_keepstill = yes;
}

static int ang_idle = 1;
static int fwd_idle = 1;

int correcting_angle()
{
	return do_correct_angle || (ang_idle < 700); // was 500
}

int correcting_straight()
{
	return do_correct_fwd || (fwd_idle < 700); // was 500
}

int correcting_either()
{
	return do_correct_angle || (ang_idle < 700) || do_correct_fwd || (fwd_idle < 700);
}

void zero_angle()
{
	aim_angle = 0;
	cur_pos.ang = 0;
}

void zero_coords()
{
	cur_pos.x = 0;
	cur_pos.y = 0;
}

void correct_location_without_moving(pos_t corr)
{
	cur_pos.x += corr.x;
	cur_pos.y += corr.y;
	cur_pos.ang += corr.ang;
	aim_angle += corr.ang;
}


void compass_fsm(int cmd)
{
	static int compass_x_min = 0;
	static int compass_x_max = 0;
	static int compass_y_min = 0;
	static int compass_y_max = 0;

	static int state = 0;

	if(cmd == 1)
		state = 1;

	int cx = latest_compass->x;
	int cy = latest_compass->y;

	int ang_err = cur_pos.ang - aim_angle;
	if(state == 1)
	{
		compass_x_min = compass_x_max = cx;
		compass_y_min = compass_y_max = cy;

//		aim_angle += (65536/3)<<16; // instruct 120 deg turn
		aim_angle += 1431655765;
		ang_top_speed = 100000;
		manual_control = 0;
		state = 2;
	}
	else if(state == 2)
	{
//		if(ang_err < (65536/6)<<16 && ang_err > (-65536/6)<<16); // when 60 deg remaining, instruct 120 deg more.
		if(ang_err < 60000000 && ang_err > -60000000)
		{
//			aim_angle += (65536/3)<<16;
			aim_angle += 1431655765;
			state = 3;
		}
	}
	else if(state == 3)
	{
//		if(ang_err < (65536/6)<<16 && ang_err > (-65536/6)<<16); // when 60 deg remaining, instruct 120 deg more.
		if(ang_err < 60000000 && ang_err > -60000000)
		{
//			aim_angle += (65536/3)<<16;
			aim_angle += 1431655765;
			state = 4;
		}
	}
	else if(state == 4)
	{
//		if(ang_err < (65536/6)<<16 && ang_err > (-65536/6)<<16); // when 60 deg remaining, instruct 120 deg more.
		if(ang_err < 60000000 && ang_err > -60000000)
		{
//			aim_angle += (65536/3)<<16;
			aim_angle += 1431655765;
			state = 5;
		}
	}
	else if(state == 5)
	{
//		if(ang_err < (65536/6)<<16 && ang_err > (-65536/6)<<16); // when 60 deg remaining, instruct 120 deg more.
		if(ang_err < 60000000 && ang_err > -60000000)
		{
//			aim_angle += (65536/3)<<16;
			aim_angle += 1431655765;
			state = 6;
		}
	}
	else if(state == 6)
	{
//		if(ang_err < (65536/6)<<16 && ang_err > (-65536/6)<<16); // when 60 deg remaining, instruct 120 deg more.
		if(ang_err < 60000000 && ang_err > -60000000)
		{
//			aim_angle += (65536/3)<<16;
			aim_angle += 1431655765;
			state = 0;
		}
	}

//	dbg4 = state;

	/*
		Compass algorithm

		To compensate for robot-referenced magnetic fields and offset errors:
		Track max, min readings on both X, Y axes while the robot is turning.
		Scale readings so that they read zero on the middle of the range, e.g.,
		if X axis reads between 1000 and 3000, make 2000 read as 0.
		Then calculate the angle with arctan (atan2() handles the signs to resolve
		the correct quadrant).
	*/

	if(cx < compass_x_min)
		compass_x_min = cx;
	if(cy < compass_y_min)
		compass_y_min = cy;

	if(cx > compass_x_max)
		compass_x_max = cx;
	if(cy > compass_y_max)
		compass_y_max = cy;

	int dx = compass_x_max - compass_x_min;
	int dy = compass_y_max - compass_y_min;
	if(dx > 500 && dy > 500)
	{
		int dx2 = compass_x_max + compass_x_min;
		int dy2 = compass_y_max + compass_y_min;
		cx = cx - dx2/2;
		cy = cy - dy2/2;

		double heading = atan2(cx, cy);
		heading /= (2.0*M_PI);
		heading *= -65536.0*65536.0;
		cur_compass_angle = (int)heading;// + 2147483648 /*180 deg*/;
	}



}

void sync_to_compass()
{
	cur_pos.ang = aim_angle = cur_compass_angle;
}

void move_arc_manual(int comm, int ang)
{
	manual_common_speed = comm<<5;
	manual_ang_speed = ang<<5;
	manual_control = 1;
	robot_moves();
}

extern volatile int test_seq;
extern volatile int16_t dbg_timing_shift;


#define ANG_1_DEG 11930465
#define ANG_01_DEG 1193047

int nearest_sonar()
{
	int n = 99999;
	if(latest_sonars[0] && latest_sonars[0] < n) n = latest_sonars[0];
	if(latest_sonars[1] && latest_sonars[1] < n) n = latest_sonars[1];
	if(latest_sonars[2] && latest_sonars[2] < n) n = latest_sonars[2];
	return n;
}


// Run this at 1 kHz
void run_feedbacks(int sens_status)
{
	static int fwd_nonidle;
	static int first = 100;
	int i;
	static int cnt = 0;
	static int prev_gyro_cnt = 0;
	static int prev_xcel_cnt = 0;
	static int speeda = 0;
	static int speedb = 0;

	static int ang_speed = 0;
	static int ang_speed_limit = 0;

	static int fwd_speed = 0;

	static int expected_fwd_accel = 0;

	static int16_t prev_wheel_counts[2];

	static int rear_wheels_in_line = 0;

	cnt++;


	if(robot_nonmoving_cnt > 2000)
		robot_nonmoving = 1;
	else
	{
		robot_nonmoving = 0;
		robot_nonmoving_cnt++;
	}

	int ang_err = cur_pos.ang - aim_angle;
	int fwd_err = aim_fwd;


	int son = nearest_sonar();
	if((speed_limit_lowered == 0 && son < 40) ||
	   (speed_limit_lowered == 1 && son < 28) ||
	   (speed_limit_lowered == 2 && son < 15))
	{
		speed_limit_lowered++;
		ang_speed_limit >>= 1;
		fwd_speed_limit >>= 1;
	}

	if(nearest_sonar() < 10)
	{
		if(fwd_err > 0) // allow backwards
		{
			reset_movement();
		//	dbg[9]++;
		}
	}


	if(straight_allowed && (fwd_err < -150 || fwd_err > 150))
	{
		do_correct_fwd = 1;
	}
	
	if(!straight_allowed || (fwd_err > -80 && fwd_err < 80))
	{
		do_correct_fwd = 0;
	}

	/*
	When going straight at the same time, smaller error in angle starts the correction.
	This causes the following style:
	- When turning first, angle is only fixed within +-3 deg
	- When the straight segment starts, angle is being fixed within +-0.5 deg.
	- This way, turning is quick, and there is no oscillation caused by the BLDC control at extremely small speeds / steps.
	- When the robot moves forward at the same time, tire speeds have common mode, and angular control only slighly changes the offset between the tires.
	  This works very well without oscillation. Also, during straight segment, the rear wheels are properly aligned.
	- Start doing the tighter angular control after 300 ms of straight segment
	*/

	if(angular_allowed && 
		(    (fwd_nonidle>300 && (ang_err < (-ANG_0_5_DEG) || ang_err > ANG_0_5_DEG))
		 || (                    (ang_err < (3*-ANG_1_DEG) || ang_err > 3*ANG_1_DEG)) ) )
	{
		do_correct_angle = 1;
	}

	if(!angular_allowed || 
		    (fwd_nonidle>300 && (ang_err > (-ANG_0_25_DEG)  && ang_err < (ANG_0_25_DEG)))
		|| (                    (ang_err > (2*-ANG_1_DEG)   && ang_err < (2*ANG_1_DEG))) )
	{
		do_correct_angle = 0;
	}


	if(ang_err > (-10*ANG_1_DEG) && ang_err < (10*ANG_1_DEG) && !fwd_idle)
		rear_wheels_in_line++;
	else
		rear_wheels_in_line = 0;

	if(rear_wheels_in_line > 800)
	{
		fwd_accel = final_fwd_accel;
	}

	if(auto_keepstill)
	{
		if(!do_correct_angle && ang_idle > 300 && !do_correct_fwd && fwd_idle > 300)
		{
			angular_allowed = 0;
			straight_allowed = 0;
		}
	}

	if(!manual_control && do_correct_angle) // && angular_allowed)
	{
		if(ang_idle)
		{
			ang_idle = 0;
			ang_speed_limit = ang_accel*20; // use starting speed that equals to 20ms of acceleration
		}

		// Calculate angular speed with P loop from the gyro integral.
		// Limit the value by using acceleration ramp. P loop handles the deceleration.

		if(ang_speed_limit < ang_top_speed) ang_speed_limit += ang_accel;
		else if(ang_speed_limit > ang_top_speed+ang_accel+1) ang_speed_limit -= ang_accel;
		else ang_speed_limit = ang_top_speed;

		int new_ang_speed = (ang_err>>20)*ang_p + ((ang_err>0)?22000:-22000) /*step-style feedforward for minimum speed*/;
		if(new_ang_speed < 0 && new_ang_speed < -1*ang_speed_limit) new_ang_speed = -1*ang_speed_limit;
		if(new_ang_speed > 0 && new_ang_speed > ang_speed_limit) new_ang_speed = ang_speed_limit;

		ang_speed = new_ang_speed;
	}
	else
	{
		if(ang_idle < 10000) ang_idle++;
		ang_speed = 0;
	}

	int16_t wheel_counts[2];
	wheel_counts[0] = motcon_rx[2].pos;
	wheel_counts[1] = -1*motcon_rx[3].pos;

	if(first)
	{
		first--;
		prev_wheel_counts[0] = wheel_counts[0];
		prev_wheel_counts[1] = wheel_counts[1];
	}

	int wheel_deltas[2] = {wheel_counts[0] - prev_wheel_counts[0], wheel_counts[1] - prev_wheel_counts[1]};

	int movement = ((wheel_deltas[0] + wheel_deltas[1])*85)>>1; // in 0.1mm

	aim_fwd -= movement;

	int y_idx = cur_pos.ang>>SIN_LUT_SHIFT;
	if(y_idx < 0) y_idx += SIN_LUT_POINTS;
	else if(y_idx >= SIN_LUT_POINTS) y_idx -= SIN_LUT_POINTS;
	int x_idx = SIN_LUT_POINTS/4 - y_idx;
	if(x_idx < 0) x_idx += SIN_LUT_POINTS;
	else if(x_idx >= SIN_LUT_POINTS) x_idx -= SIN_LUT_POINTS;
	cur_pos.x += ((int64_t)sin_lut[x_idx] * (int64_t)movement)>>15;
	cur_pos.y += ((int64_t)sin_lut[y_idx] * (int64_t)movement)>>15;


	prev_wheel_counts[0] = wheel_counts[0];
	prev_wheel_counts[1] = wheel_counts[1];

	int tmp_expected_accel = -1*fwd_speed;

	if(!manual_control && do_correct_fwd) // && straight_allowed)
	{
		if(fwd_idle)
		{
			fwd_idle = 0;
		}
		fwd_nonidle++;

		// Calculate linear speed with P loop from position error.
		// Limit the value by using acceleration ramp. P loop handles the deceleration.

		if(fwd_speed_limit < fwd_top_speed) fwd_speed_limit += fwd_accel;
		else if(fwd_speed_limit > fwd_top_speed+fwd_accel+1) fwd_speed_limit -= fwd_accel;
		else fwd_speed_limit = fwd_top_speed;

		int new_fwd_speed = (fwd_err>>4)*fwd_p + ((fwd_err>0)?100000:-100000) /*step-style feedforward for minimum speed*/;
		if(new_fwd_speed < 0 && new_fwd_speed < -1*fwd_speed_limit) new_fwd_speed = -1*fwd_speed_limit;
		if(new_fwd_speed > 0 && new_fwd_speed > fwd_speed_limit) new_fwd_speed = fwd_speed_limit;

		fwd_speed = new_fwd_speed;
	}
	else
	{
		if(fwd_idle < 10000) fwd_idle++;
		fwd_nonidle = 0;
		fwd_speed = 0;
	}

	if(ang_speed != 0 || fwd_speed != 0) robot_moves();


	tmp_expected_accel += fwd_speed;

	expected_fwd_accel = ((tmp_expected_accel<<16) + 63*expected_fwd_accel)>>6;


	if(sens_status & GYRO_NEW_DATA)
	{

		int latest[3] = {latest_gyro->x, latest_gyro->y, latest_gyro->z};

#define GYRO_MOVEMENT_DETECT_THRESHOLD_X 600
#define GYRO_MOVEMENT_DETECT_THRESHOLD_Y 300
#define GYRO_MOVEMENT_DETECT_THRESHOLD_Z 200
		if(latest[0] < -GYRO_MOVEMENT_DETECT_THRESHOLD_X || latest[0] > GYRO_MOVEMENT_DETECT_THRESHOLD_X ||
		   latest[1] < -GYRO_MOVEMENT_DETECT_THRESHOLD_Y || latest[1] > GYRO_MOVEMENT_DETECT_THRESHOLD_Y ||
		   latest[2] < -GYRO_MOVEMENT_DETECT_THRESHOLD_Z || latest[2] > GYRO_MOVEMENT_DETECT_THRESHOLD_Z)
		{
			robot_moves();
		}

		if(robot_nonmoving)
		{
			gyro_dc_corrs[0] = ((latest[0]<<15) + 255*gyro_dc_corrs[0])>>8;
			gyro_dc_corrs[1] = ((latest[1]<<15) + 255*gyro_dc_corrs[1])>>8;
			gyro_dc_corrs[2] = ((latest[2]<<15) + 255*gyro_dc_corrs[2])>>8;
		}

		int gyro_dt = cnt - prev_gyro_cnt;
		prev_gyro_cnt = cnt;
		// Gyro should give data at 200Hz, which is 5 steps at 1kHz
		if(gyro_dt < 2 || gyro_dt > 20)
		{
			// If the timing is clearly out of range, assume 200Hz data rate and log the error.
			gyro_dt = 5;
			gyro_timing_issues++;
		}

		for(i=0; i<3; i++)
		{
			latest[i] -= gyro_dc_corrs[i]>>15;
			latest[i] *= gyro_dt;
			gyro_long_integrals[i] += latest[i];
			gyro_short_integrals[i] += latest[i];
		}

		//1 gyro unit = 7.8125 mdeg/s; integrated at 1kHz timesteps, 1 unit = 7.8125 udeg
		// Correct ratio = (7.8125*10^-6)/(360/(2^32)) = 93.2067555555589653990
		// Approximated ratio = 763550/8192  = 763550>>13 = 93.20678710937500
		// Corrected empirically from there.

		int gyro_blank = robot_nonmoving?(40*5):(0); // Prevent slow gyro drifting during no operation

		if(latest[2] < -1*gyro_blank || latest[2] > gyro_blank)
			cur_pos.ang += ((int64_t)latest[2]*(int64_t)763300)>>13;
	}


	if(sens_status & XCEL_NEW_DATA)
	{
		int latest[3] = {latest_xcel->x, latest_xcel->y, latest_xcel->z};

		if(robot_nonmoving)
		{
			xcel_dc_corrs[0] = ((latest[0]<<15) + 255*xcel_dc_corrs[0])>>8;
			xcel_dc_corrs[1] = ((latest[1]<<15) + 255*xcel_dc_corrs[1])>>8;
			xcel_dc_corrs[2] = ((latest[2]<<15) + 255*xcel_dc_corrs[2])>>8;
		}

		int xcel_dt = cnt - prev_xcel_cnt;
		prev_xcel_cnt = cnt;
		// Xcel should give data at 200Hz, which is 5 steps at 1kHz
		if(xcel_dt < 2 || xcel_dt > 20)
		{
			// If the timing is clearly out of range, assume 200Hz data rate and log the error.
			xcel_dt = 5;
			xcel_timing_issues++;
		}

		for(i=0; i<3; i++)
		{
			latest[i] -= xcel_dc_corrs[i]>>15;
			latest[i] *= xcel_dt;
			if(latest[i] < -3000 || latest[i] > 3000)
				xcel_long_integrals[i] += (int64_t)latest[i];
			xcel_short_integrals[i] += (int64_t)latest[i];
		}


		int unexpected_accel = /*(expected_fwd_accel>>4)*/ 0 - latest[1];


		//1 xcel unit = 0.061 mg = 0.59841 mm/s^2; integrated at 1kHz timesteps, 1 unit = 0.59841 mm/s
	}

	int common_speed;
	if(manual_control)
	{
		speeda = -1*manual_ang_speed;
		speedb = manual_ang_speed;
		reset_movement(); stop_navig_fsms(); robot_moves(); // to prevent surprises when auto mode goes back up.
		common_speed = manual_common_speed;
	}
	else
	{
		speeda = -1*(ang_speed>>8);
		speedb = (ang_speed>>8);
		common_speed = fwd_speed>>8;
	}

	if(speeda > MAX_DIFFERENTIAL_SPEED*256) speeda = MAX_DIFFERENTIAL_SPEED*256;
	else if(speeda < -MAX_DIFFERENTIAL_SPEED*256) speeda = -MAX_DIFFERENTIAL_SPEED*256;
	if(speedb > MAX_DIFFERENTIAL_SPEED*256) speedb = MAX_DIFFERENTIAL_SPEED*256;
	else if(speedb < -MAX_DIFFERENTIAL_SPEED*256) speedb = -MAX_DIFFERENTIAL_SPEED*256;

	int a = (common_speed) + (speeda);
	int b = (common_speed) + (speedb);

	if(a > MAX_SPEED) a=MAX_SPEED;
	else if(a < -MAX_SPEED) a=-MAX_SPEED;
	if(b > MAX_SPEED) b=MAX_SPEED;
	else if(b < -MAX_SPEED) b=-MAX_SPEED;


	if(host_alive_watchdog)
	{
		host_alive_watchdog--;
		motcon_tx[2].state = 5;
		motcon_tx[3].state = 5;
		motcon_tx[2].speed = a;
		motcon_tx[3].speed = -1*b;
	}
	else
	{
		motcon_tx[2].state = 1;
		motcon_tx[3].state = 1;
		motcon_tx[2].speed = 0;
		motcon_tx[3].speed = 0;
		reset_movement(); stop_navig_fsms(); // to prevent surprises when we are back up.
	}

}
