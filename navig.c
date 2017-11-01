/*
Low-level navigation module.

Collision avoidance, simple mechanical tasks.

*/

#include <stdint.h>
#include <math.h>

#include "sin_lut.h"
#include "lidar.h"
#include "sonar.h"
#include "navig.h"
#include "feedbacks.h"
#include "lidar_corr.h" // for point_t (for lidar collision avoidance)

extern point_t lidar_collision_avoidance[360];
volatile int lidar_collision_avoidance_new;
int enable_coll_avoidance = 1;
int collision_avoidance_on;

uint32_t store_stop_flags = 0;
uint32_t store_action_flags = 0;


extern volatile int dbg[10];

#define XY_DONT_AT_END_LEN 200 /*don't recalculate ang,straight at this many last millimeters to (x,y) destination*/


static int navi_speed;
#define MAXSPEED(x) do{if((x) < navi_speed) navi_speed = (x);} while(0)


typedef enum {
	MOVE_IDLE 		= 0, 
	MOVE_START          	= 1,
	MOVE_WAIT_ANGULAR	= 2,
	MOVE_WAIT_STRAIGHT	= 3,
	MOVE_END           	= 4
} move_state_t;

typedef struct
{
	int valid;
	move_state_t state;
	int abs_ang;
	int rel_fwd;
} move_t;

static move_t cur_move;
static int dest_x, dest_y;
static int correct_xy;
static int was_correcting_xy;
static int avoidance_in_action;

static int xy_left;
static int xy_id;


static int back_mode_hommel = 0;
static int speedlim_hommel = 50;

void move_fsm()
{
	switch(cur_move.state)
	{
		case MOVE_IDLE:
		break;

		case MOVE_START:
		{
			if(enable_coll_avoidance)
				collision_avoidance_on = 1;
			else
				collision_avoidance_on = 0;
			allow_angular(1);
			allow_straight(1);
			auto_disallow(0);
			rotate_abs(cur_move.abs_ang);
			straight_rel(cur_move.rel_fwd);
			set_top_speed(navi_speed);
			cur_move.state = MOVE_WAIT_STRAIGHT;
		}
		break;

/*
		case MOVE_WAIT_ANGULAR:
		{
			if(angle_almost_corrected())
			{
				straight_rel(cur_move.rel_fwd);
				set_top_speed_max(speedlim_hommel);
				allow_straight(1);
				cur_move.state++;
			}
		}
		break;
*/
		case MOVE_WAIT_STRAIGHT:


		if(correct_xy && (cur_move.rel_fwd < -1*XY_DONT_AT_END_LEN || cur_move.rel_fwd > XY_DONT_AT_END_LEN))
		{
			change_angle_abs(cur_move.abs_ang);
			change_straight_rel(cur_move.rel_fwd);
			set_top_speed(navi_speed);
		}

		if(!correcting_either())
		{
			cur_move.state++;
		}
		break;

		case MOVE_END:	
		{
			collision_avoidance_on = 0;
			was_correcting_xy = 0;
			correct_xy = 0;
			avoidance_in_action = 0;
			xy_left = 0;
			reset_movement();
			auto_disallow(0);
			allow_angular(1);
			allow_straight(1);
			cur_move.state = 0;
		}
		break;

		default:
		break;
	}
}

void move_rel_twostep(int angle32, int fwd /*in mm*/, int speedlim)
{
	reset_movement();
	take_control();
	was_correcting_xy = 0;
	correct_xy = 0;
	store_action_flags = store_stop_flags = 0;
	avoidance_in_action = 0;
	speedlim_hommel = speedlim;
	set_top_speed(speedlim);
	cur_move.state = MOVE_START;
	cur_move.abs_ang = cur_pos.ang + angle32;
	cur_move.rel_fwd = fwd;
	cur_move.valid = 1;
}

void ena_coll_avoid()
{
	enable_coll_avoidance = 1;
}
void dis_coll_avoid()
{
	enable_coll_avoidance = 0;
}

void move_absa_rels_twostep(int angle32, int fwd /*in mm*/, int speedlim)
{
	reset_movement();
	take_control();
	was_correcting_xy = 0;
	correct_xy = 0;
	store_action_flags = store_stop_flags = 0;
	avoidance_in_action = 0;
	speedlim_hommel = speedlim;
	set_top_speed(speedlim);
	cur_move.state = MOVE_START;
	cur_move.abs_ang = angle32;
	cur_move.rel_fwd = fwd;
	cur_move.valid = 1;
}

int get_xy_left()
{
	return xy_left;
}

int get_xy_id()
{
	return xy_id;
}

int32_t xy_original_ang;

void xy_fsm()
{
	if(!correct_xy)
	{
		return;
	}

	int dx = dest_x - cur_pos.x;
	int dy = dest_y - cur_pos.y;

	int new_fwd = sqrt(dx*dx + dy*dy);
	int new_ang = atan2(dy, dx)*(4294967296.0/(2.0*M_PI));

	if(back_mode_hommel == 1) // Force backwards
	{
		new_fwd *= -1;
		new_ang = (uint32_t)new_ang + 2147483648UL;
	}
	else if(back_mode_hommel == 2) // Auto decision
	{
		int ang_err = cur_pos.ang - new_ang;
		if((ang_err < -1610612736 || ang_err > 1610612736) && new_fwd < 1000) // 0.75*180deg
		{
			new_fwd *= -1;
			new_ang = (uint32_t)new_ang + 2147483648UL;
		}
	}

	xy_left = new_fwd;

	int32_t ang_changed = xy_original_ang - new_ang;

	if((ang_changed < -90*ANG_1_DEG || ang_changed > 90*ANG_1_DEG) &&
	   (new_fwd < -1*XY_DONT_AT_END_LEN || new_fwd > XY_DONT_AT_END_LEN))
	{
		store_stop_flags |= 1UL<<11;
		reset_movement();
		cur_move.state = 0;
	}

	cur_move.rel_fwd = new_fwd;
	cur_move.abs_ang = new_ang;
}


void move_xy_abs(int32_t x, int32_t y, int back_mode, int id, int speedlim)
{
	back_mode_hommel = back_mode;
	speedlim_hommel = speedlim;
	dest_x = x;
	dest_y = y;

	int dx = dest_x - cur_pos.x;
	int dy = dest_y - cur_pos.y;

	int new_fwd = sqrt(dx*dx + dy*dy);
	int new_ang = atan2(dy, dx)*(4294967296.0/(2.0*M_PI));

	// back_mode == 0 = always go forward.

	if(back_mode == 1) // Force backwards
	{
		new_fwd *= -1;
		new_ang = (uint32_t)new_ang + 2147483648UL;
	}
	else if(back_mode == 2) // Auto decision
	{
		int ang_err = cur_pos.ang - new_ang;
		if((ang_err < -1610612736 || ang_err > 1610612736) && new_fwd < 1000) // 0.75*180deg
		{
			new_fwd *= -1;
			new_ang = (uint32_t)new_ang + 2147483648UL;
		}
	}

	xy_left = new_fwd;

	xy_original_ang = new_ang;
	move_absa_rels_twostep(new_ang, new_fwd, speedlim);
	was_correcting_xy = 1;
	correct_xy = 1;
	xy_id = id;
}

void navig_fsm1()
{
	xy_fsm();

	static int speed_incr_cnt;
	speed_incr_cnt++;
	if(speed_incr_cnt > 250)
	{
		speed_incr_cnt = 0;
		if(navi_speed < speedlim_hommel)
		{
			if(navi_speed < 10)
				navi_speed+=1;
			else if(navi_speed < 20)
				navi_speed+=2;
			else if(navi_speed < 30)
				navi_speed+=3;
			else if(navi_speed < 40)
				navi_speed+=4;
			else
				navi_speed+=5;
		}
		if(navi_speed > speedlim_hommel)
			navi_speed = speedlim_hommel;
	}
}


#if defined(RN1P4) || defined(RN1P6) || defined(RN1P7)
#define robot_ys_TIGHT (480)
#define robot_xs_TIGHT (524)
#define robot_origin_to_front_TIGHT (150)
	#ifdef DELIVERY_APP
	#define robot_origin_to_back_TIGHT  (526)
	#else
	#define robot_origin_to_back_TIGHT  (400)
	#endif
#endif

#ifdef PULU1
#define robot_ys_TIGHT (300)
#define robot_xs_TIGHT (340)
#define robot_origin_to_front_TIGHT (60)
#define robot_origin_to_back_TIGHT  (250)
#endif


int robot_ys, robot_xs, robot_origin_to_front, robot_origin_to_back;
int robot_height = 1600;

void set_obstacle_avoidance_margin(int cm)
{
	robot_ys = robot_ys_TIGHT + 20*cm;
	robot_xs = robot_xs_TIGHT + 20*cm;
	robot_origin_to_front = robot_origin_to_front_TIGHT + 10*cm;
	robot_origin_to_back = robot_origin_to_back_TIGHT + 10*cm;
}

uint32_t get_obstacle_avoidance_stop_flags()
{
	return store_stop_flags;
}

uint32_t get_obstacle_avoidance_action_flags()
{
	return store_action_flags;
}


int lr_diff, y_diff, x_dist_to_charger;

// -100: no lidar image, try again
// -99: Mark not found.
int chafind_middle_mark()
{
	int nearest_ang = -100;
	if(lidar_collision_avoidance_new)
	{
		lidar_collision_avoidance_new = 0;

		int nearest_x = 400;
		int y_at_nearest_x = 0;
//		int avg_x = 0;
//		int avg_cnt = 0;
		for(int i = 0; i < 80; i++)
		{
			int ang = i-40;
			int ang_minus2 = i-40-2;
			if(ang < 0) ang += 360;
			if(ang_minus2 < 0) ang_minus2 += 360;

			if(!lidar_collision_avoidance[ang].valid) continue;

//			avg_x += lidar_collision_avoidance[ang].x;
//			avg_cnt++;

			if(lidar_collision_avoidance[ang].x < nearest_x)
			{
				// nearest point (the mark) must have clearly farther away points before it. If it's invalid, skip the test.
				if(!lidar_collision_avoidance[ang_minus2].valid || lidar_collision_avoidance[ang_minus2].x > (lidar_collision_avoidance[ang].x+7))
				{
					nearest_x = lidar_collision_avoidance[ang].x;
					y_at_nearest_x = lidar_collision_avoidance[ang].y;
					nearest_ang = ang;
				}
			}
		}

//		avg_x /= avg_cnt;

		if(nearest_x == 400 || (nearest_ang > 30 && nearest_ang < 330))
			return -1;

		int widen = 15;
		if(nearest_x < 200) widen = 30;
		else if(nearest_x < 280) widen = 22;
		else widen = 19;

		int x_right_avg = 0;
		int x_right_avg_cnt = 0;
		for(int find = -3; find <=3; find++)
		{
			int idx = nearest_ang + widen + find;
			if(idx > 359) idx-= 360;
			else if(idx < 0) idx += 360;

			if(!lidar_collision_avoidance[idx].valid) continue;
			x_right_avg += lidar_collision_avoidance[idx].x;
			x_right_avg_cnt++;
		}


		if(x_right_avg_cnt < 2)
		{
			return -1;	
		}
		x_right_avg /= x_right_avg_cnt;

		int x_left_avg = 0;
		int x_left_avg_cnt = 0;
		for(int find = -3; find <=3; find++)
		{
			int idx = nearest_ang - widen + find;
			if(idx > 359) idx-= 360;
			else if(idx < 0) idx += 360;

			if(!lidar_collision_avoidance[idx].valid) continue;
			x_left_avg += lidar_collision_avoidance[idx].x;
			x_left_avg_cnt++;
		}

		if(x_left_avg_cnt < 2)
		{
			return -1;	
		}
		x_left_avg /= x_left_avg_cnt;

		x_dist_to_charger = (x_left_avg + x_right_avg)/2;

		lr_diff = x_left_avg - x_right_avg;  // Defines angle to be turned

		if(lr_diff < -80 || lr_diff > 80)
		{
			return -1;
		}

		y_diff = y_at_nearest_x; // Defines distance to go sideways.

		if(y_diff < -150 || y_diff > 150)
			return -1;

		return 0;
	}

	return -2;
}


typedef enum {
	CHAFIND_IDLE 		= 0,
	CHAFIND_START          	= 1,
	CHAFIND_WAIT_ROTATED	= 2,
	CHAFIND_WAIT_BACK	= 3,
	CHAFIND_WAIT_FWD	= 4,
	CHAFIND_PUSH		= 5,
	CHAFIND_WAIT_PUSH	= 6,
	CHAFIND_FINISH		= 7,
	CHAFIND_FAIL		= 8
} chafind_state_t;

chafind_state_t chafind_state;

volatile int start_charger = 0;


void navig_fsm2_for_charger()
{
	static int rot_dir;
	static int back_amount;
	switch(chafind_state)
	{
		case CHAFIND_START:
		{
			set_top_speed_max(20);

			int ret = chafind_middle_mark();
			if(ret == 0)
			{
				auto_disallow(0);
				allow_angular(1);
				allow_straight(1);
				rotate_rel(lr_diff*2*ANG_0_1_DEG);
				chafind_state++;
			}
			else if(ret==-1)
				chafind_state = CHAFIND_FAIL;

		}
		break;

		case CHAFIND_WAIT_ROTATED:
		{
			if(!correcting_angle())
			{
				int ret = chafind_middle_mark();
				if(ret == 0)
				{
					if(y_diff > 15)
					{
						rot_dir = 0;
						// if we were to turn 45 deg, we would need to back off 1mm for each 1mm error.
						auto_disallow(0);
						allow_angular(1);
						allow_straight(1);
						rotate_rel(-10*ANG_1_DEG);

						int amount_fwd = -6*y_diff;
						if(amount_fwd < -500) amount_fwd = -500;
						back_amount = amount_fwd;
						straight_rel(amount_fwd);
						chafind_state++;
					}
					else if(y_diff < -15)
					{
						rot_dir = 1;
						auto_disallow(0);
						allow_angular(1);
						allow_straight(1);
						rotate_rel(10*ANG_1_DEG);

						int amount_fwd = 6*y_diff;
						if(amount_fwd < -500) amount_fwd = -500;
						back_amount = amount_fwd;
						straight_rel(amount_fwd);
						chafind_state++;
					}
					else
						chafind_state = CHAFIND_PUSH;
				}
				else if(ret == -1)
				{
					chafind_state = CHAFIND_FAIL;
				}

			}
		}
		break;

		case CHAFIND_WAIT_BACK:
		{
			if(!correcting_either())
			{
				chafind_state++;

				// Rotate back to where we were before backing.
				if(rot_dir == 0)
					rotate_rel(10*ANG_1_DEG);
				else
					rotate_rel(-10*ANG_1_DEG);

				straight_rel((-1*back_amount));

			}
		}
		break;

		case CHAFIND_WAIT_FWD:
		{
			if(!correcting_either())
			{
				int ret = chafind_middle_mark();
				if(ret == 0)
				{
					if(y_diff < -20 || y_diff > 20 || lr_diff < -10 || lr_diff > 10)
					{
						// Try again.
						chafind_state = CHAFIND_START;
					}
					else
						chafind_state++;
				}
				else if(ret == -1)
				{
					chafind_state = CHAFIND_FAIL;
				}
			}
		}
		break;

		case CHAFIND_PUSH:
		{
			if(!correcting_either())
			{
				set_top_speed_max(11);
				straight_rel(x_dist_to_charger-robot_origin_to_front_TIGHT+20);
				chafind_state++;
			}
		}
		break;

		case CHAFIND_WAIT_PUSH:
		{
			if(!correcting_either())
			{
				chafind_state++;

			}
		}
		break;

		default:
		case CHAFIND_FINISH:
		{
			reset_movement();
			start_charger = 1;
			chafind_state = 0;
		}
		break;

		case CHAFIND_FAIL:
		{
			reset_movement();
			chafind_state = 0;
		}
		break;

	}

}

extern int accurate_turngo;

void find_charger()
{
	accurate_turngo = 1;
	chafind_state = CHAFIND_START;
}

void stop_navig_fsms()
{
	cur_move.state = 0;
	chafind_state = 0;
}


#define GO_FWD 0
#define GO_BACK 1
#define TURN_LEFT 2
#define TURN_RIGHT 3
#define DO_NOTHING 4

void daiju_meininki_fsm()
{
	extern uint32_t random;

	static int state = 4;

	if(lidar_collision_avoidance_new)
	{
		allow_angular(1);
		allow_straight(1);
		auto_disallow(0);
		lidar_collision_avoidance_new = 0;

		int nearest_colliding_front = 9999;
		int nearest_colliding_back = 9999;

		int can_do[5] = {0,0,1,1,1};

		for(int i = (360-60); i < 360; i+=2)
		{
			if(!lidar_collision_avoidance[i].valid) continue;

			int dist_to_front = lidar_collision_avoidance[i].x - robot_origin_to_front;

			if(lidar_collision_avoidance[i].y > -1*(robot_ys/2+10) && lidar_collision_avoidance[i].y < (robot_ys/2+10))
			{
				if(dist_to_front < nearest_colliding_front) nearest_colliding_front = dist_to_front;
			}
		}
		for(int i = 0; i < 60; i+=2)
		{
			if(!lidar_collision_avoidance[i].valid) continue;

			int dist_to_front = lidar_collision_avoidance[i].x - robot_origin_to_front;

			if(lidar_collision_avoidance[i].y > -1*(robot_ys/2+10) && lidar_collision_avoidance[i].y < (robot_ys/2+10))
			{
				if(dist_to_front < nearest_colliding_front) nearest_colliding_front = dist_to_front;
			}
		}
		for(int i = 180-60; i < 180+60; i+=2)
		{
			if(!lidar_collision_avoidance[i].valid) continue;

			int dist_to_back  = -1*lidar_collision_avoidance[i].x - robot_origin_to_back;

			if(lidar_collision_avoidance[i].y > -1*(robot_ys/2+0) && lidar_collision_avoidance[i].y < (robot_ys/2+0))
			{
				if(dist_to_back  < nearest_colliding_back) nearest_colliding_back = dist_to_back;
			}
		}

		if(nearest_colliding_front > 80)
			can_do[GO_FWD] = 1;

		if(nearest_colliding_back > 80)
			can_do[GO_BACK] = 1;

		// Check the arse, to avoid hitting when turning.
		for(int i = 90-1; i < 90+35; i+=2)
		{
			if(!lidar_collision_avoidance[i].valid) continue;

			if(lidar_collision_avoidance[i].y < robot_ys/2+40)
			{
				if(lidar_collision_avoidance[i].x < 0 && lidar_collision_avoidance[i].x > -robot_origin_to_back)
				{
					can_do[TURN_LEFT] = 0;
					break;
				}
			}
		}


		// Check the arse, to avoid hitting when turning.
		for(int i = 180+35; i < 180+90+1; i+=2)
		{
			if(!lidar_collision_avoidance[i].valid) continue;

			if(lidar_collision_avoidance[i].y > -1*(robot_ys/2+40))
			{
				if(lidar_collision_avoidance[i].x < 0 && lidar_collision_avoidance[i].x > -(robot_origin_to_back+10))
				{
					can_do[TURN_RIGHT] = 0;
					break;
				}

			}
		}

		#ifdef SONARS_INSTALLED
		#endif

		int can_do_cnt = can_do[0]+can_do[1]+can_do[2]+can_do[3];

		int instruct_movement = 0;

		static int state_cnt = 0;
		static int nothing_cnt = 0;
		state_cnt++;
		if(!can_do[state] || state_cnt > 10)
		{
			state = random&0b11;
			if(can_do_cnt != 0)
			{
				nothing_cnt = 0;
				while(!can_do[state]) // find a state we can do.
				{
					state++;
					if(state > 3) state = 0;
				}
			}
			else
			{
				state = DO_NOTHING;
				nothing_cnt++;
				if(nothing_cnt > 5)
				{
					nothing_cnt = 0;
					state = random&0b11;
					if(state == GO_BACK) state = TURN_LEFT;
				}
			}
			instruct_movement = 1;
			state_cnt = 0;
		}
		

		if(instruct_movement)
		{
			set_top_speed_max(13);

			switch(state)
			{
				case GO_FWD:
				{
					int amount = nearest_colliding_front;
					if(amount > 300) amount = 300;
					else if(amount < 10) amount = 10;
					straight_rel(amount);
				}
				break;
				case GO_BACK:
				{
					set_top_speed_max(10);
					int amount = nearest_colliding_back;
					if(amount > 200) amount = 200;
					else if(amount < 5) amount = 5;
					straight_rel(-1*amount);
				}
				break;
				case TURN_LEFT:
				{
					rotate_rel(-8*ANG_1_DEG);
				}
				break;

				case TURN_RIGHT:
				{
					rotate_rel(8*ANG_1_DEG);
				}
				break;

				default:
				break;
			}
		}
	}
}

int daiju_meininki = 0;

void daiju_mode_on()
{
	daiju_meininki = 1;
	stop_navig_fsms();
}

void daiju_mode_off()
{
	daiju_meininki = 0;
	stop_navig_fsms();
}

void navig_fsm2()
{

	if(daiju_meininki)
	{
		daiju_meininki_fsm();
		return;
	}

	if(chafind_state)
	{
		navig_fsm2_for_charger();
		return;
	}

	move_fsm();
}


/*
	Call this function right after any sensor has observed a point at (x, y) in robot coord frame (+x = forward).
	Adjusts max speed, stops if necessary.
*/

void micronavi_point_in(int32_t x, int32_t y, int16_t z, int stop_if_necessary)
{
	if(z < 35 || z > robot_height)
		goto SKIP_MICRONAVI;

	if(correcting_straight())
	{
		int fwd_remain = get_fwd();
		int reverse;
		int dist_to_hit;

		if(fwd_remain < 0)
		{
			reverse = 1;
			fwd_remain *= -1;
			dist_to_hit = -1*x - robot_origin_to_back;
		}
		else
		{
			reverse = 0;
			dist_to_hit = x - robot_origin_to_front;
		}

		if(y < (robot_ys/2+0) && y > (-1*robot_ys/2-0))  // Direct collision course
		{
			if(dist_to_hit > -100) // skip things behind our way
			{
				if(dist_to_hit < 250)
				{
					if(fwd_remain < dist_to_hit-150) // going to stop, but don't want to scare anyone, limit speed.
					{
						MAXSPEED(20);
					}
					else if(fwd_remain < dist_to_hit) // going to barely stop, but don't want to scare anyone, limit speed.
					{
						MAXSPEED(10);
					}
					else if(stop_if_necessary) // gonna hit, stop as quickly as needed (try 30 mm before the obstacle)
					{
						//dbg[6] = x; dbg[7] = y;
						int amount = dist_to_hit-30;
						if(amount < 0) amount = 0;
						change_straight_rel((reverse?-1:1)*amount);
						cur_move.state = 0;
						store_stop_flags |= 1;
					}
					else
						MAXSPEED(10);
				}
				if(dist_to_hit < 500)
				{
					if(fwd_remain < dist_to_hit-300) // going to stop well in time anyway
					{
						MAXSPEED(40);
					}
					else if(fwd_remain < dist_to_hit-150) // going to stop, but don't want to scare anyone, limit speed.
					{
						MAXSPEED(30);
					}
					else if(fwd_remain < dist_to_hit) // going to barely stop, but don't want to scare anyone, limit speed.
					{
						MAXSPEED(20);
					}
					else if(stop_if_necessary) // gonna hit
					{
						//dbg[6] = x; dbg[7] = y;

						// Stop carefully, 5cm before the obstacle:
						change_straight_rel((reverse?-1:1)*(dist_to_hit-50));
						cur_move.state = 0;
						store_stop_flags |= 2;
					}
					else
						MAXSPEED(15);
				}
				else if(dist_to_hit < 1200)
				{
					if(fwd_remain < dist_to_hit-300) // going to stop well in time anyway
					{
						MAXSPEED(50);
					}
					else if(fwd_remain < dist_to_hit-150) // going to stop, but don't want to scare anyone, limit speed.
					{
						MAXSPEED(40);
					}
					else if(fwd_remain < dist_to_hit) // going to barely stop, but don't want to scare anyone, limit speed.
					{
						MAXSPEED(35);
					}
					else // gonna hit
					{
						MAXSPEED(30);
					}
				}
			}
		}
		else if(y < (robot_ys/2+80) && y > (-1*robot_ys/2-80))  // Obstacle very close to either side
		{
			if(dist_to_hit > -100) // skip things behind our way
			{
				if(dist_to_hit < 250)
				{
					if(fwd_remain < dist_to_hit-150) // going to stop, but don't want to scare anyone, limit speed.
					{
						MAXSPEED(25);
					}
					else if(fwd_remain < dist_to_hit) // going to barely stop, but don't want to scare anyone, limit speed.
					{
						MAXSPEED(15);
					}
					else
					{
						MAXSPEED(10); // Corner of the robot will almost hit, go very slowly
					}
				}
				if(dist_to_hit < 500)
				{
					if(fwd_remain < dist_to_hit-300) // going to stop well in time anyway
					{
						MAXSPEED(45);
					}
					else if(fwd_remain < dist_to_hit-150) // going to stop, but don't want to scare anyone, limit speed.
					{
						MAXSPEED(35);
					}
					else if(fwd_remain < dist_to_hit) // going to barely stop, but don't want to scare anyone, limit speed.
					{
						MAXSPEED(25);
					}
					else
					{
						MAXSPEED(20);
					}
				}
				else if(dist_to_hit < 1200)
				{
					if(fwd_remain < dist_to_hit-300) // going to stop well in time anyway
					{
						MAXSPEED(50);
					}
					else if(fwd_remain < dist_to_hit-150) // going to stop, but don't want to scare anyone, limit speed.
					{
						MAXSPEED(45);
					}
					else if(fwd_remain < dist_to_hit) // going to barely stop, but don't want to scare anyone, limit speed.
					{
						MAXSPEED(40);
					}
					else
					{
						MAXSPEED(35);
					}
				}
			}
		}
		else if(y < (robot_ys/2+160) && y > (-1*robot_ys/2-160))  // Obstacle kinda close to either side
		{
			if(dist_to_hit > -100) // skip things behind our way
			{
				if(dist_to_hit < 250)
				{
					if(fwd_remain < dist_to_hit-150)
					{
						MAXSPEED(30);
					}
					else if(fwd_remain < dist_to_hit)
					{
						MAXSPEED(20);
					}
					else
					{
						MAXSPEED(15);
					}
				}
				if(dist_to_hit < 500)
				{
					if(fwd_remain < dist_to_hit-300)
					{
						MAXSPEED(50);
					}
					else if(fwd_remain < dist_to_hit-150)
					{
						MAXSPEED(40);
					}
					else if(fwd_remain < dist_to_hit)
					{
						MAXSPEED(30);
					}
					else
					{
						MAXSPEED(25);
					}
				}
				else if(dist_to_hit < 1200)
				{
					if(fwd_remain < dist_to_hit-150)
					{
						MAXSPEED(55);
					}
					else if(fwd_remain < dist_to_hit)
					{
						MAXSPEED(45);
					}
					else
					{
						MAXSPEED(40);
					}
				}
			}
		}
		else if(y < (robot_ys/2+240) && y > (-1*robot_ys/2-240))  // Some obstacle not so close
		{
			if(dist_to_hit > -100) // skip things behind our way
			{
				if(dist_to_hit < 250)
				{
					MAXSPEED(40);
				}
				if(dist_to_hit < 500)
				{
					MAXSPEED(50);
				}
				else if(dist_to_hit < 1200)
				{
					MAXSPEED(60);
				}
			}
		}
	}


/*


       ###############################
       ###############################
       ###############################
       ###############################
       #######################O#######
       ###############################
       ###############################
       ###############################
       ###############################
             |
             |
             |  hit_remain_mm
             |
             |
             |
             X  <--obstacle seen (variables x,y)





       ###############################
       ###############################
       ###############################
       ###############################
       #######################O#######
       ###############################
       ###############################
       ###############################
       ###############################
             |
             |
             |
             |
             |
             |
             X  <--obstacle seen (variables x,y)
             |
             |
             |   turn_remain_mm (projected 90 deg from the robot, through x,y)
                 can be longer or shorter than hit_remain_mm. If longer, obviously going to hit.

*/


	if(correcting_angle())
	{
		int ang_remain = get_ang_err();

		if(ang_remain > 5*ANG_1_DEG || ang_remain < -5*ANG_1_DEG)
		{
			int hit_remain_mm;
			if(ang_remain > 0)
			{
				hit_remain_mm = y - robot_ys/2;
			}
			else
			{
				hit_remain_mm = -1*y - robot_ys/2;
			}

			uint32_t ang32 = ang_remain;
			int turn_remain_mm = (sin_lut[ang32>>SIN_LUT_SHIFT] * (-1*x))>>15;
			if(turn_remain_mm < 0) turn_remain_mm *= -1;

			if(hit_remain_mm > -150) // skip things behind
			{
				if(x > (-1*robot_origin_to_back)          && x < -20 && hit_remain_mm < 200) // arse in direct collision course, and soon.
				{
					if(turn_remain_mm < hit_remain_mm-150)
					{
						MAXSPEED(20);
					}
					if(turn_remain_mm < hit_remain_mm) // barely not going to hit, still go carefully
					{
						MAXSPEED(10);
					}
					else if(stop_if_necessary)  // going to hit, stop
					{
						//dbg[4] = hit_remain_mm;
						//dbg[5] = turn_remain_mm;
						//dbg[6] = x;
						//dbg[7] = y;
						change_angle_to_cur();
						store_stop_flags |= (1UL<<2);
					}
					else
						MAXSPEED(10);
				}
				else if(x > (-1*robot_origin_to_back)-60  && x < 0 && hit_remain_mm < 300) // arse getting close
				{
					if(turn_remain_mm < hit_remain_mm-250)
					{
						MAXSPEED(40);
					}
					if(turn_remain_mm < hit_remain_mm-150)
					{
						MAXSPEED(30);
					}
					else   // going to hit, in theory, let's see what happens later
					{
						MAXSPEED(20);
					}
				}
			}
		}
	}

	SKIP_MICRONAVI:

	set_top_speed(navi_speed);
}


void stop_movement()
{
	reset_movement();
	cur_move.state = 0;
}

void limit_speed(int speed)
{
	speedlim_hommel = speed;
	set_top_speed_max(speedlim_hommel);
}

