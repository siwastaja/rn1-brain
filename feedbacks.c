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
#include "sin_lut.h"
#include "navig.h"

#define LED_ON()  {GPIOC->BSRR = 1UL<<13;}
#define LED_OFF() {GPIOC->BSRR = 1UL<<(13+16);}

extern volatile int dbg[10];

uint8_t feedback_stop_flags;

int64_t gyro_long_integrals[3];
int64_t gyro_short_integrals[3];

int64_t xcel_long_integrals[3];
int64_t xcel_short_integrals[3];

int xcel_dc_corrs[3];
int gyro_dc_corrs[3];

// cur x,y are being integrated at higher than 1mm resolution; the result is copied in mm to cur_pos.
static int64_t cur_x;
static int64_t cur_y;
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
int fwd_p = 1600; // 3100 gives rather strong deceleration; 1600 feels sluggish. 2200 oscillates sometimes.

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

void host_dead()
{
	host_alive_watchdog = 0;
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
static int ang_speed_limit = 0;
static int speed_limit_lowered;

void take_control()
{
	manual_control = 0;
}

int speed_limit_status()
{
	return speed_limit_lowered;
}

void lower_speed_limit()
{
	if(!speed_limit_lowered)
		ang_top_speed >>= 1;  // Lower angular speed limit once only.
	fwd_top_speed >>= 1;
	speed_limit_lowered++;
}

void speed_limit(int new_status)
{
	if(new_status < 0 || new_status > 4) return;

	while(new_status > speed_limit_lowered)
		lower_speed_limit();
}

void reset_speed_limits()
{
	ang_top_speed = 170000;
	fwd_top_speed = 600000;
}

void set_ang_top_speed(int speed)
{
	ang_top_speed = speed;
}

void rotate_rel(int angle)
{
	feedback_stop_flags = 0;

	aim_angle += angle;

	speed_limit_lowered = 0;
	ang_top_speed = 170000;
	manual_control = 0;
	robot_moves();
}

void rotate_abs(int angle)
{
	feedback_stop_flags = 0;
	aim_angle = angle;

	speed_limit_lowered = 0;
	ang_top_speed = 170000;
	manual_control = 0;
	robot_moves();
}

void change_angle_abs(int angle)
{
	aim_angle = angle;
}

void change_angle_rel(int angle)
{
	aim_angle += angle;
}


void straight_rel(int fwd /*in mm*/)
{
	feedback_stop_flags = 0;
	speed_limit_lowered = 0;
	fwd_accel = final_fwd_accel/4;
	fwd_speed_limit = fwd_accel*80; // use starting speed that equals to 80ms of acceleration
	aim_fwd = fwd<<16;
	fwd_top_speed = 600000;
	manual_control = 0;
	robot_moves();
}

int get_fwd()
{
	return (aim_fwd>>16);
}

void change_straight_rel(int fwd /*in mm*/)
{
	aim_fwd = fwd<<16;
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
	return do_correct_angle || (ang_idle < 700);
}

int get_ang_err()
{
	return cur_pos.ang - aim_angle;
}

int angle_almost_corrected()
{
	int ang_err = cur_pos.ang - aim_angle;
	if(ang_err > (-5*ANG_1_DEG) && ang_err < (5*ANG_1_DEG))
		return 1;

	return 0;
}

int correcting_straight()
{
	return do_correct_fwd || (fwd_idle < 700);
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
	cur_x = cur_y = 0;
}

int64_t gyro_mul_neg = 763300LL<<16;
int64_t gyro_mul_pos = 763300LL<<16;

int gyro_avgd = 0;

void correct_location_without_moving(pos_t corr)
{
	cur_x += corr.x<<16;
	cur_y += corr.y<<16;
	cur_pos.ang += corr.ang;
	aim_angle += corr.ang;

	if(gyro_avgd < -100)
		gyro_mul_neg += corr.ang;
	else if(gyro_avgd > 100)
		gyro_mul_pos += corr.ang;

}

void correct_location_without_moving_external(pos_t corr)
{
	cur_x += corr.x<<16;
	cur_y += corr.y<<16;
	cur_pos.ang += corr.ang;
	aim_angle += corr.ang;
}

void set_location_without_moving_external(pos_t new_pos)
{
	cur_x = new_pos.x<<16;
	cur_y = new_pos.y<<16;
	cur_pos.ang = new_pos.ang;
	aim_angle = new_pos.ang;
}

void compass_fsm(int cmd)
{
	static int compass_x_min = 0;
	static int compass_x_max = 0;
	static int compass_y_min = 0;
	static int compass_y_max = 0;

	static int state = 0;

	if(cmd == 1 && state == 0)
		state = 1;

	int cx = latest_compass->x;
	int cy = latest_compass->y;

	int ang_err = cur_pos.ang - aim_angle;
	if(state == 1)
	{
		compass_x_min = compass_x_max = cx;
		compass_y_min = compass_y_max = cy;

		rotate_rel(120*ANG_1_DEG);
		ang_top_speed = 100000;

		state = 2;
	}
	else if(state >= 2 && state < 7)
	{
		if(ang_err > -60*ANG_1_DEG && ang_err < 60*ANG_1_DEG)
		{
			rotate_rel(120*ANG_1_DEG);
			ang_top_speed = 100000;
			state++;
		}
	}
	else if(state == 7)
	{
		state = 0;
	}

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

		double heading = atan2(cx, cy) + M_PI;
		if(heading < -1.0*M_PI) heading += 2.0*M_PI;
		else if(heading > 1.0*M_PI) heading -= 2.0*M_PI;
		heading /= (2.0*M_PI);
		heading *= -65536.0*65536.0;
		cur_compass_angle = (int)heading;
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

void unexpected_movement_detected()
{
	lidar_mark_invalid();
}

void collision_detected(int reason)
{
	feedback_stop_flags = reason;
	lidar_mark_invalid();
	reset_movement();
	stop_navig_fsms();
}

int coll_det_on;
void enable_collision_detection()
{
	coll_det_on = 1;
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

	static int fwd_speed = 0;

	static int expected_fwd_accel = 0;

	static int16_t prev_wheel_counts[2];

	static int rear_wheels_in_line = 0;

	cnt++;


	if(robot_nonmoving_cnt > 1500)
		robot_nonmoving = 1;
	else
	{
		robot_nonmoving = 0;
		robot_nonmoving_cnt++;
	}

	int ang_err = cur_pos.ang - aim_angle;
	int fwd_err = aim_fwd;

	if(straight_allowed && (fwd_err < -40*65536 || fwd_err > 40*65536))
	{
		if(ang_err > (-8*ANG_1_DEG) && ang_err < (8*ANG_1_DEG))
			do_correct_fwd = 1;
		else if(ang_err < (-12*ANG_1_DEG) && ang_err > (12*ANG_1_DEG))
			do_correct_fwd = 0;
		
	}
	if(!straight_allowed || (fwd_err > -15*65536 && fwd_err < 15*65536))
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
		 || (                    (ang_err < (-3*ANG_1_DEG) || ang_err > 3*ANG_1_DEG)) ) )
	{
		do_correct_angle = 1;
	}

	if(!angular_allowed || 
		    (fwd_nonidle>300 && (ang_err > (-ANG_0_25_DEG)  && ang_err < (ANG_0_25_DEG)))
		|| (                    (ang_err > (-2*ANG_1_DEG)   && ang_err < (2*ANG_1_DEG))) )
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

	if(!manual_control && do_correct_angle)
	{
		if(ang_idle)
		{
			ang_idle = 0;
			ang_speed_limit = ang_accel*20; // use starting speed that equals to 20ms of acceleration
		}

		// Calculate angular speed with P loop from the gyro integral.
		// Limit the value by using acceleration ramp. P loop handles the deceleration.

		if(ang_speed_limit < ang_top_speed) ang_speed_limit += ang_accel;
		else if(ang_speed_limit > ang_top_speed+ang_accel+1) ang_speed_limit -= 2*ang_accel;
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

	static int32_t prev_cur_ang = 0;
	static int32_t turned_by_wheels_integral = 0;
	static int32_t turned_by_gyro_integral;

	int16_t wheel_counts[2];
	wheel_counts[0] = motcon_rx[2].pos;
	wheel_counts[1] = -1*motcon_rx[3].pos;
	if(first)
	{
		first--;
		prev_wheel_counts[0] = wheel_counts[0];
		prev_wheel_counts[1] = wheel_counts[1];
		prev_cur_ang = cur_pos.ang;
		turned_by_wheels_integral = 0;
	}
	int wheel_deltas[2] = {wheel_counts[0] - prev_wheel_counts[0], wheel_counts[1] - prev_wheel_counts[1]};
	prev_wheel_counts[0] = wheel_counts[0];
	prev_wheel_counts[1] = wheel_counts[1];

	if(wheel_deltas[0] != 0 || wheel_deltas[1] != 0)
		robot_moves();

	int movement = (wheel_deltas[0] + wheel_deltas[1])*278528; // in 1mm/65536
	int turned_by_wheels = (wheel_deltas[0] - wheel_deltas[1])*15500000;
	turned_by_wheels_integral += turned_by_wheels;

	turned_by_gyro_integral += cur_pos.ang - prev_cur_ang;
	prev_cur_ang = cur_pos.ang;
	if(turned_by_wheels_integral < -10*ANG_1_DEG || turned_by_wheels_integral > 10*ANG_1_DEG)
	{
		int err = turned_by_wheels_integral - turned_by_gyro_integral;
		if(err < -5*ANG_1_DEG || err > 5*ANG_1_DEG)
		{
			collision_detected(2);
		}
		else if(err < -3*ANG_1_DEG || err > 3*ANG_1_DEG)
		{
			unexpected_movement_detected();
		}

		turned_by_wheels_integral = turned_by_gyro_integral = 0;
	}

	aim_fwd -= movement;

	int y_idx = cur_pos.ang>>SIN_LUT_SHIFT;
	if(y_idx < 0) y_idx += SIN_LUT_POINTS;
	else if(y_idx >= SIN_LUT_POINTS) y_idx -= SIN_LUT_POINTS;
	int x_idx = SIN_LUT_POINTS/4 - y_idx;
	if(x_idx < 0) x_idx += SIN_LUT_POINTS;
	else if(x_idx >= SIN_LUT_POINTS) x_idx -= SIN_LUT_POINTS;

	cur_x += ((int64_t)sin_lut[x_idx] * (int64_t)movement)>>15;
	cur_y += ((int64_t)sin_lut[y_idx] * (int64_t)movement)>>15;

	// Copy accurate accumulation variables to cur_pos here:
	cur_pos.x = cur_x>>16;
	cur_pos.y = cur_y>>16;


	int tmp_expected_accel = -1*fwd_speed;

	if(!manual_control && do_correct_fwd)
	{
		if(fwd_idle)
		{
			fwd_idle = 0;
		}
		fwd_nonidle++;

		int top_speed = fwd_top_speed;
		if(ang_err < -30*ANG_1_DEG || ang_err > 30*ANG_1_DEG) { top_speed = 0;}
		else if(ang_err < -25*ANG_1_DEG || ang_err > 25*ANG_1_DEG) { if(top_speed > 30000) top_speed = 30000;}
		else if(ang_err < -20*ANG_1_DEG || ang_err > 20*ANG_1_DEG) { if(top_speed > 60000) top_speed = 60000;}
		else if(ang_err < -15*ANG_1_DEG || ang_err > 15*ANG_1_DEG) { if(top_speed > 150000) top_speed = 150000;}
		else if(ang_err < -10*ANG_1_DEG || ang_err > 10*ANG_1_DEG) { if(top_speed > 300000) top_speed = 300000;}

		// Calculate linear speed with P loop from position error.
		// Limit the value by using acceleration ramp. P loop handles the deceleration.

		if(fwd_speed_limit < top_speed) fwd_speed_limit += fwd_accel;
		else if(fwd_speed_limit > top_speed+fwd_accel+1) fwd_speed_limit -= 2*fwd_accel;
		else fwd_speed_limit = top_speed;

		int new_fwd_speed = (fwd_err>>16)*fwd_p + ((fwd_err>0)?100000:-100000) /*step-style feedforward for minimum speed*/;
		if(new_fwd_speed < 0 && new_fwd_speed < -1*fwd_speed_limit) new_fwd_speed = -1*fwd_speed_limit;
		if(new_fwd_speed > 0 && new_fwd_speed > fwd_speed_limit) new_fwd_speed = fwd_speed_limit;

		fwd_speed = new_fwd_speed;
	}
	else
	{
		if(fwd_idle < 10000) fwd_idle++;
		fwd_nonidle = 0;
		if(fwd_speed)
		{
			fwd_speed -= 2*fwd_accel;
			if(fwd_speed < 0) fwd_speed = 0;
		}
	}

	if(ang_speed != 0 || fwd_speed != 0) robot_moves();


	tmp_expected_accel += fwd_speed;

	expected_fwd_accel = ((tmp_expected_accel<<16) + 63*expected_fwd_accel)>>6;


	if(sens_status & GYRO_NEW_DATA)
	{

		int latest[3] = {latest_gyro->x, latest_gyro->y, latest_gyro->z};

#define GYRO_MOVEMENT_DETECT_THRESHOLD_X 800
#define GYRO_MOVEMENT_DETECT_THRESHOLD_Y 600
#define GYRO_MOVEMENT_DETECT_THRESHOLD_Z 400
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

//		int gyro_blank = robot_nonmoving?(40*5):(0); // Prevent slow gyro drifting during no operation

		int gyro_blank = robot_nonmoving?(100*5):(50*5); // Prevent slow gyro drifting during no operation

		gyro_avgd = ((latest[2]<<8) + 3*gyro_avgd)>>2;

		if(latest[2] < -1*gyro_blank)  // Negative = left
			cur_pos.ang += ((int64_t)latest[2]*(int64_t)(gyro_mul_neg>>16))>>13;
		else if(latest[2] > gyro_blank) // Positive = right
			cur_pos.ang += ((int64_t)latest[2]*(int64_t)(gyro_mul_pos>>16))>>13;
	}


	if(sens_status & XCEL_NEW_DATA)
	{
		int latest[3] = {latest_xcel->x, latest_xcel->y, latest_xcel->z};
		static int xcel_flt[2];

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

		// Moving average 7/8
		xcel_flt[0] = ((latest[0]<<8) + 7*xcel_flt[0])>>3;
		xcel_flt[1] = ((latest[1]<<8) + 7*xcel_flt[1])>>3;

		if(coll_det_on && (xcel_flt[0] < XCEL_X_NEG_WARN || xcel_flt[0] > XCEL_X_POS_WARN ||
		   xcel_flt[1] < XCEL_Y_NEG_WARN || xcel_flt[1] > XCEL_Y_POS_WARN))
			unexpected_movement_detected();

		if(coll_det_on && (xcel_flt[0] < XCEL_X_NEG_COLL || xcel_flt[0] > XCEL_X_POS_COLL ||
		   xcel_flt[1] < XCEL_Y_NEG_COLL || xcel_flt[1] > XCEL_Y_POS_COLL))
			collision_detected(1);

//		int unexpected_accel = /*(expected_fwd_accel>>4)*/ 0 - latest[1];


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
