#include <inttypes.h>
#include <math.h>

#include "ext_include/stm32f2xx.h"
#include "feedbacks.h"
#include "gyro_xcel_compass.h"
#include "motcons.h"
#include "sonar.h"

#define LED_ON()  {GPIOC->BSRR = 1UL<<13;}
#define LED_OFF() {GPIOC->BSRR = 1UL<<(13+16);}

extern volatile int dbg[10];
extern int latest_sonars[MAX_NUM_SONARS]; // in cm, 0 = no echo

int64_t gyro_long_integrals[3];
int64_t gyro_short_integrals[3];

int64_t xcel_long_integrals[3];
int64_t xcel_short_integrals[3];

int xcel_dc_corrs[3];


int gyro_timing_issues;
int xcel_timing_issues;

volatile int cur_x = 0;
volatile int cur_y = 0;

volatile int cur_angle = 0; // int32_t range --> -180..+180 deg; let it overflow freely. 1 unit = 83.81903171539 ndeg
volatile int cur_compass_angle = 0;
volatile int aim_angle = 0; // same
volatile int aim_speed = 0;

int ang_accel = 25; //100;
int ang_top_speed;
int ang_p = 1500;

volatile int cur_fwd;
volatile int aim_fwd;
int fwd_accel = 25;
int fwd_top_speed;
int fwd_p = 5000;

volatile int speed_updated;  // For timeouting robot movements (for faulty communications)
volatile int manual_control;
volatile int common_speed;
volatile int manual_ang_speed;

volatile int robot_nonmoving = 1;

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

volatile int wheel_integrals[2];
volatile int fwd_speed_limit;
volatile int speed_limit_lowered;


// 700 = 1860

void move_rel_twostep(int angle, int fwd)
{
	aim_angle += angle<<16;

	speed_limit_lowered = 0;
	wheel_integrals[0] = 0;
	wheel_integrals[1] = 0;
	fwd_speed_limit = fwd_accel*200; // use starting speed that equals to 20ms of acceleration
	aim_fwd = fwd*10;

	ang_top_speed = 150000;
	manual_control = 0;

	fwd_top_speed = 600000;

	speed_updated = 500000;
	robot_nonmoving = 0;
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

	int ang_err = cur_angle - aim_angle;
	if(state == 1)
	{
		compass_x_min = compass_x_max = cx;
		compass_y_min = compass_y_max = cy;

//		aim_angle += (65536/3)<<16; // instruct 120 deg turn
		aim_angle += 1431655765;
		ang_top_speed = 100000;
		manual_control = 0;
		speed_updated = 1000000;
		state = 2;
	}
	else if(state == 2)
	{
//		if(ang_err < (65536/6)<<16 && ang_err > (-65536/6)<<16); // when 60 deg remaining, instruct 120 deg more.
		if(ang_err < 60000000 && ang_err > -60000000)
		{
//			aim_angle += (65536/3)<<16;
			aim_angle += 1431655765;
			speed_updated = 1000000;
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
			speed_updated = 1000000;
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
			speed_updated = 1000000;
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
			speed_updated = 1000000;
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
			speed_updated = 1000000;
			state = 0;
		}
	}

//	dbg4 = state;


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
	cur_angle = aim_angle = cur_compass_angle;
}

void move_arc_manual(int comm, int ang)
{
	if(comm == 0 && ang == 0)
	{
		zero_xcel_long_integrals();
		robot_nonmoving = 1;

	}
	else
		robot_nonmoving = 0;


	common_speed = comm<<5;
	manual_ang_speed = ang<<5;
	speed_updated = 5000; // robot is stopped if 0.5s is elapsed between the speed commands.
	manual_control = 1;
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



// Run this at 10 kHz
void run_feedbacks(int sens_status)
{
	int i;
	static int cnt = 0;
	static int prev_gyro_cnt = 0;
	static int prev_xcel_cnt = 0;
	static int robot_nonmoving_cnt = 0;
	static int speeda = 0;
	static int speedb = 0;

	static int ang_idle = 1;
	static int correct_angle = 0;

	static int fwd_idle = 1;
	static int correct_fwd = 0;
	static int correct_fwd_pending = 0;

	static int ang_speed = 0;
	static int ang_speed_limit = 0;

	static int fwd_speed = 0;

	static int16_t prev_wheel_counts[2];

	cnt++;

	dbg[1] = aim_fwd;
	dbg[2] = cur_fwd;

	int ang_err = cur_angle - aim_angle;
	int fwd_err = aim_fwd - cur_fwd;

	dbg[3] = fwd_err;

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
			aim_angle = cur_angle;
			aim_fwd = cur_fwd;
		}
	}


	if(manual_control)
	{
		correct_angle = 0;
	}
	else if((ang_err < (-ANG_1_DEG) || ang_err > ANG_1_DEG))
	{
		correct_angle = 1;
	}
	else if((ang_err < (-ANG_1_DEG)/2 || ang_err > (ANG_1_DEG)/2))
	{
		correct_angle = 0;
	}

	if(manual_control)
	{
		correct_fwd_pending = 0;
	}
	else if(fwd_err < -100 || fwd_err > 100)
	{
		correct_fwd_pending = 1;
	}
	else if(fwd_err < -50 || fwd_err > 50)
	{
		correct_fwd_pending = 0;
	}

	if(correct_fwd_pending)
	{
		if((ang_err > 3*(-ANG_1_DEG) && ang_err < 3*ANG_1_DEG))
			correct_fwd = 1;
	}
	else
	{
		correct_fwd = 0;
	}

	dbg[4] = correct_fwd;
	dbg[5] = fwd_idle;

	if(correct_angle)
	{
		if(ang_idle)
		{
			ang_idle = 0;
			ang_speed_limit = ang_accel*200; // use starting speed that equals to 20ms of acceleration
			zero_gyro_short_integrals();
		}

		// Calculate angular speed with P loop from the gyro integral.
		// Limit the value by using acceleration ramp. P loop handles the deceleration.

		if(ang_speed_limit < ang_top_speed) ang_speed_limit += ang_accel;
		else if(ang_speed_limit > ang_top_speed+ang_accel+1) ang_speed_limit -= ang_accel;
		else ang_speed_limit = ang_top_speed;

		int new_ang_speed = (ang_err>>20)*ang_p;
		if(new_ang_speed < 0 && new_ang_speed < -1*ang_speed_limit) new_ang_speed = -1*ang_speed_limit;
		if(new_ang_speed > 0 && new_ang_speed > ang_speed_limit) new_ang_speed = ang_speed_limit;

		ang_speed = new_ang_speed;
	}
	else
	{
		ang_idle=1;
		ang_speed = 0;
	}

	int16_t wheel_counts[2];
	wheel_counts[0] = motcon_rx[2].pos;
	wheel_counts[1] = -1*motcon_rx[3].pos;

	wheel_integrals[0] += wheel_counts[0] - prev_wheel_counts[0];
	wheel_integrals[1] += wheel_counts[1] - prev_wheel_counts[1];

	dbg[6] = wheel_integrals[0];
	dbg[7] = wheel_integrals[1];

	cur_fwd = ((wheel_integrals[0] + wheel_integrals[1])*85)>>1;

	prev_wheel_counts[0] = wheel_counts[0];
	prev_wheel_counts[1] = wheel_counts[1];

	if(correct_fwd)
	{
		if(fwd_idle)
		{
			fwd_idle = 0;
		}

		// Calculate linear speed with P loop from position error.
		// Limit the value by using acceleration ramp. P loop handles the deceleration.

		if(fwd_speed_limit < fwd_top_speed) fwd_speed_limit += fwd_accel;
		else if(fwd_speed_limit > fwd_top_speed+fwd_accel+1) fwd_speed_limit -= fwd_accel;
		else fwd_speed_limit = fwd_top_speed;

		int new_fwd_speed = (fwd_err>>4)*fwd_p;
		if(new_fwd_speed < 0 && new_fwd_speed < -1*fwd_speed_limit) new_fwd_speed = -1*fwd_speed_limit;
		if(new_fwd_speed > 0 && new_fwd_speed > fwd_speed_limit) new_fwd_speed = fwd_speed_limit;

		fwd_speed = new_fwd_speed;
	}
	else
	{
		fwd_idle=1;
		fwd_speed = 0;
	}

	if(sens_status & GYRO_NEW_DATA)
	{
		int gyro_dt = cnt - prev_gyro_cnt;
		prev_gyro_cnt = cnt;
		// Gyro should give data at 200Hz, which is 50 steps at 10kHz
		if(gyro_dt < 30 || gyro_dt > 200)
		{
			// If the timing is clearly out of range, assume 200Hz data rate and log the error.
			gyro_dt = 50;
			gyro_timing_issues++;
		}


		int latest[3] = {latest_gyro->x*gyro_dt, latest_gyro->y*gyro_dt, latest_gyro->z*gyro_dt};

		for(i=0; i<3; i++)
		{
//			if(latest[i] < -GYRO_LONG_INTEGRAL_IGNORE_LEVEL || latest[i] > GYRO_LONG_INTEGRAL_IGNORE_LEVEL)
				gyro_long_integrals[i] += latest[i];

			gyro_short_integrals[i] += latest[i];
		}

		//1 gyro unit = 31.25 mdeg/s; integrated at 10kHz timesteps, 1 unit = 3.125 udeg
		// Correct ratio = (3.125*10^-6)/(360/(2^32)) = 37.28270222222358615960
		// Approximated ratio = 76355/2048  = 76355>>11 = 37.28271484375
		// Error = -0.00003385%
		// Corrected empirically: 75900>>11

		int gyro_blank = ang_idle?(100*50):(60*50);

		if(latest[2] < -1*gyro_blank || latest[2] > gyro_blank)
			cur_angle += ((int64_t)latest[2]*(int64_t)75900)>>11;
	}

	if(sens_status & XCEL_NEW_DATA)
	{
		int latest[3] = {latest_xcel->x, latest_xcel->y, latest_xcel->z};

		if(robot_nonmoving)
		{
			if(robot_nonmoving_cnt < 50)
			{
				robot_nonmoving_cnt++;
			}
			else
			{
				xcel_dc_corrs[0] = ((latest[0]<<8) + 63*xcel_dc_corrs[0])>>6;
				xcel_dc_corrs[1] = ((latest[1]<<8) + 63*xcel_dc_corrs[1])>>6;
				xcel_dc_corrs[2] = ((latest[2]<<8) + 63*xcel_dc_corrs[2])>>6;
			}
		}
		else
			robot_nonmoving_cnt = 0;

		int xcel_dt = cnt - prev_xcel_cnt;
		prev_xcel_cnt = cnt;
		// Xcel should give data at 200Hz, which is 50 steps at 10kHz
		if(xcel_dt < 30 || xcel_dt > 200)
		{
			// If the timing is clearly out of range, assume 200Hz data rate and log the error.
			xcel_dt = 50;
			xcel_timing_issues++;
		}

		for(i=0; i<3; i++)
		{
			latest[i] -= xcel_dc_corrs[i]>>8;
			latest[i] *= xcel_dt;
			if(latest[i] < -3000 || latest[i] > 3000)
				xcel_long_integrals[i] += (int64_t)latest[i];
			xcel_short_integrals[i] += (int64_t)latest[i];
		}

//		dbg1 = xcel_dc_corrs[0]>>8;
//		dbg2 = xcel_dc_corrs[1]>>8;

		//1 xcel unit = 0.061 mg = 0.59841 mm/s^2; integrated at 10kHz timesteps, 1 unit = 0.059841 mm/s
	}

	if(manual_control)
	{
		speeda = -1*manual_ang_speed;
		speedb = manual_ang_speed;
	}
	else
	{
		speeda = -1*(ang_speed>>8);
		speedb = (ang_speed>>8);
		common_speed = fwd_speed>>8;
	}
/*
	if((cnt&0x3f) == 0x3f) // slow decay
	{
		if(speeda > 0) speeda--;
		else if(speeda < 0) speeda++;

		if(speedb > 0) speedb--;
		else if(speedb < 0) speedb++;
	}
*/
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


	if(speed_updated)
	{
		LED_OFF();
		speed_updated--;
		motcon_tx[2].state = 5;
		motcon_tx[3].state = 5;
		motcon_tx[2].speed = a;
		motcon_tx[3].speed = -1*b;
	}
	else
	{
		LED_ON();
		robot_nonmoving = 1;
		motcon_tx[2].speed = 0;
		motcon_tx[3].speed = 0;
		motcon_tx[2].state = 1;
		motcon_tx[3].state = 1;

		speeda=0;
		speedb=0;
	}

/*
	motcon_tx[3].crc = dbg_timing_shift;
	if(test_seq > 0)
	{
		motcon_tx[2].state = 1;
		motcon_tx[3].state = 5;

		test_seq++;

		if(test_seq == 3*10000)  
		{
			motcon_tx[2].speed = motcon_tx[3].speed = 50; // Absolute minimum creeping should be observed
		}

		if(test_seq == 3*30000)  
		{
			motcon_tx[2].speed = motcon_tx[3].speed = -50; // Absolute minimum creeping should be observed
		}

		if(test_seq == 3*50000)  
		{
			motcon_tx[2].speed = motcon_tx[3].speed = 250;
		}

		if(test_seq == 3*70000)  
		{
			motcon_tx[2].speed = motcon_tx[3].speed = -250;
		}

		if(test_seq == 3*90000)  
		{
			motcon_tx[2].speed = motcon_tx[3].speed = 1000;
		}

		if(test_seq == 3*100000)  
		{
			motcon_tx[2].speed = motcon_tx[3].speed = -1000;
		}

		if(test_seq == 3*110000)  
		{
			motcon_tx[2].speed = motcon_tx[3].speed = 5000;
		}

		if(test_seq == 3*120000)  
		{
			motcon_tx[2].speed = motcon_tx[3].speed = -5000;
		}

		if(test_seq == 3*130000) 
		{
			motcon_tx[2].speed = motcon_tx[3].speed = 0;
			test_seq = 0;
		}
	}
	else
	{
		motcon_tx[2].speed = motcon_tx[3].speed = 0;
		motcon_tx[2].state = 1;
		motcon_tx[3].state = 1;
	}

*/
}
