/*
Low-level navigation module.

Collision avoidance, simple mechanical tasks.

*/

#include <stdint.h>
#include <math.h>
#include <string.h> // memset

#include "sin_lut.h"
#include "lidar.h"
#include "sonar.h"
#include "navig.h"
#include "feedbacks.h"
#include "lidar_corr.h" // for point_t (for lidar collision avoidance)
#include "main.h"

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
   dbg_teleportation_bug(201);

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

   dbg_teleportation_bug(202);
}

void move_rel_twostep(int angle32, int fwd /*in mm*/, int speedlim)
{
   dbg_teleportation_bug(203);

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
   dbg_teleportation_bug(204);

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
   dbg_teleportation_bug(205);

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
   dbg_teleportation_bug(206);

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
   dbg_teleportation_bug(207);

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
   dbg_teleportation_bug(208);

}


void move_xy_abs(int32_t x, int32_t y, int back_mode, int id, int speedlim)
{
   dbg_teleportation_bug(209);

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
   dbg_teleportation_bug(210);

}

void navig_fsm1()
{
   dbg_teleportation_bug(211);

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

   dbg_teleportation_bug(212);

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

#if defined(PROD1)
#define robot_ys_TIGHT (650)
#define robot_xs_TIGHT (500)
#define robot_origin_to_front_TIGHT (142)
#define robot_origin_to_back_TIGHT  (650-142)
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


extern int accurate_turngo;

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

/*
	Call this function right after any sensor has observed a point at (x, y) in robot coord frame (+x = forward).
	Adjusts max speed, stops if necessary.
*/

void micronavi_point_in_normal(int32_t x, int32_t y, int16_t z, int stop_if_necessary)
{
	if(z < 35 || z > robot_height)
		goto SKIP_MICRONAVI;


   dbg_teleportation_bug(220);

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

   dbg_teleportation_bug(221);


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

   dbg_teleportation_bug(222);

	SKIP_MICRONAVI:

	set_top_speed(navi_speed);
}


int chafind_nearest_hit_x;
int chafind_nearest_hit_y;
int chafind_left_accum, chafind_left_accum_cnt;
int chafind_right_accum, chafind_right_accum_cnt;
int chafind_middle_min_y, chafind_middle_max_y;
int chafind_middle_cnt;
int chafind_total_front_accum, chafind_total_front_accum_cnt;

#define CHAFIND_FIRST_DIST 520
#define CHAFIND_MIDDLE_BAR_DEPTH 80
#define CHAFIND_MIDDLE_BAR_WIDTH 40
#define CHAFIND_FIRST_DIST_TOLERANCE 50
#define CHAFIND_LOOK_SIDEWAY_MAX 180
#define CHAFIND_LOOK_SIDEWAY_MIN 120
#define CHAFIND_SIDE (CHAFIND_LOOK_SIDEWAY_MAX+CHAFIND_LOOK_SIDEWAY_MIN)  //  /2 * 2  // y dist between the two side points looked at

#define CHAFIND_PASS1_ACCEPT_ANGLE (2*ANG_1_DEG) // was 1 deg
#define CHAFIND_PASS1_ACCEPT_SHIFT 20 // was 12mm

#define CHAFIND_PUSH_TUNE 100 // in mm, lower number = go further

#define CHAFIND_ACCEPT_MILLIVOLTS 21000 // Voltage to expect to stop the push

#define CHAFIND_AIM_Y_TUNE 10  // Positive = go more right



void chafind_empty_accum1()
{
	chafind_nearest_hit_x = 9999;
	chafind_nearest_hit_y = 0;
}

void chafind_empty_accum2()
{
	chafind_left_accum = chafind_left_accum_cnt = 0;
	chafind_right_accum = chafind_right_accum_cnt = 0;
	chafind_middle_min_y = 9999;
	chafind_middle_max_y = -9999;
	chafind_middle_cnt = 0;
	chafind_total_front_accum = 0;
	chafind_total_front_accum_cnt = 0;
}


void micronavi_point_in_chafind(int32_t x, int32_t y, int16_t z, int stop_if_necessary, int source)
{
	if(source != 0)
		return;

	y += CHAFIND_AIM_Y_TUNE;

//	int dist_to_hit;

//	dist_to_hit = x - robot_origin_to_front_TIGHT;

	if(y < (robot_ys_TIGHT/2+80) && y > (-1*robot_ys_TIGHT/2-80) && x > robot_origin_to_front_TIGHT+150)
	{
		// Look at area in front of the robot
		if(x < chafind_nearest_hit_x)
		{
			chafind_nearest_hit_x = x;
			chafind_nearest_hit_y = y;
		}
	}

//	dbg[] = chafind_nearest_hit_x;
//	dbg[] = chafind_nearest_hit_y;

	if(y < (chafind_nearest_hit_y-CHAFIND_LOOK_SIDEWAY_MIN) && y > (chafind_nearest_hit_y-CHAFIND_LOOK_SIDEWAY_MAX) &&
//		x > CHAFIND_FIRST_DIST-CHAFIND_FIRST_TOLERANCE-100+CHAFIND_MIDDLE_BAR_DEPTH && x < CHAFIND_FIRST_DIST+CHAFIND_FIRST_TOLERANCE+100+CHAFIND_MIDDLE_BAR_DEPTH)
		x > 100 && x < 700)
	{
		chafind_left_accum += x;
		chafind_left_accum_cnt++;
//		dbg[] = chafind_left_accum/chafind_left_accum_cnt;
//		dbg[] = chafind_left_accum_cnt;
	
	}
	else if(y > (chafind_nearest_hit_y+CHAFIND_LOOK_SIDEWAY_MIN) && y < (chafind_nearest_hit_y+CHAFIND_LOOK_SIDEWAY_MAX) &&
//		x > CHAFIND_FIRST_DIST-CHAFIND_FIRST_TOLERANCE-100+CHAFIND_MIDDLE_BAR_DEPTH && x < CHAFIND_FIRST_DIST+CHAFIND_FIRST_TOLERANCE+100+CHAFIND_MIDDLE_BAR_DEPTH)
		x > 100 && x < 700)
	{
		chafind_right_accum += x;
		chafind_right_accum_cnt++;
//		dbg[] = chafind_right_accum/chafind_right_accum_cnt;
//		dbg[] = chafind_right_accum_cnt;
	}

	if(x >= chafind_nearest_hit_x && x < chafind_nearest_hit_x+60 && y > chafind_nearest_hit_y-CHAFIND_MIDDLE_BAR_WIDTH-15 && y < chafind_nearest_hit_y+CHAFIND_MIDDLE_BAR_WIDTH+15)
	{
		chafind_middle_cnt++;
		if(y < chafind_middle_min_y) chafind_middle_min_y = y;
		if(y > chafind_middle_max_y) chafind_middle_max_y = y;
//		dbg[] = chafind_middle_cnt;
	}

	if(y < (robot_ys_TIGHT/2) && y > -1*(robot_ys_TIGHT/2) && x > 200 && x < 1000)
	{
		chafind_total_front_accum += x;
		chafind_total_front_accum_cnt++;
	}

}

int chafind_calc(int* p_ang, int* p_shift, int* p_dist)
{
	int left = chafind_left_accum/chafind_left_accum_cnt;
	int right = chafind_right_accum/chafind_right_accum_cnt;
	//int avgdist = (left+right+(chafind_nearest_hit_x+CHAFIND_MIDDLE_BAR_DEPTH))/3;

	int ang = atan2(right-left, CHAFIND_SIDE)*(4294967296.0/(2.0*M_PI));
	*p_ang = ang;
	dbg[1] = ang/ANG_0_1_DEG;

	int midmark_x = chafind_nearest_hit_x;
	int midmark_y = (chafind_middle_min_y + chafind_middle_max_y)/2;

	//dbg[] = midmark_x;
	//dbg[] = midmark_y;
	// Rotate middle mark coordinates to find required sideway drift

	int rotcos = sin_lut[(uint32_t)(ANG_90_DEG-ang)>>SIN_LUT_SHIFT];
	int rotsin = sin_lut[(uint32_t)(ang)>>SIN_LUT_SHIFT];

	int midmark_rot_x = (midmark_x * rotcos - midmark_y * rotsin)>>15;
	int midmark_rot_y = (midmark_x * rotsin + midmark_y * rotcos)>>15;

	*p_shift = midmark_rot_y;
	*p_dist = midmark_rot_x-robot_origin_to_front_TIGHT;

	dbg[2] = *p_shift;
	dbg[3] = *p_dist;

	return 0;
}


typedef enum {
	CHAFIND_IDLE 			= 0,
	CHAFIND_START          		= 1,
	CHAFIND_WAIT_DISTANCE		= 2,
	CHAFIND_WAIT_FWD1		= 3,
	CHAFIND_WAIT_FWD1_STOPEXTRA1	= 4,
	CHAFIND_WAIT_FWD1_STOPEXTRA2	= 5,
	CHAFIND_ACCUM_DATA		= 6,
	CHAFIND_WAIT_BACKING		= 7,
	CHAFIND_START_REBACKING		= 8,
	CHAFIND_WAIT_REBACKING		= 9,
	CHAFIND_WAIT_FWD2		= 10,
	CHAFIND_WAIT_FWD2_STOPEXTRA1	= 11,
	CHAFIND_ACCUM_FRONTAVG		= 12,
	CHAFIND_WAIT_PUSH		= 13,
	CHAFIND_SUCCESS                 = 14,
	CHAFIND_FAIL			= 99
} chafind_state_t;

chafind_state_t chafind_state = CHAFIND_IDLE;

volatile int start_charger = 0;

void micronavi_point_in(int32_t x, int32_t y, int16_t z, int stop_if_necessary, int source)
{
	dbg[0] = chafind_state;
	if(chafind_state)
	{
		micronavi_point_in_chafind(x,y,z,stop_if_necessary, source);
	}
	else
	{
		micronavi_point_in_normal(x,y,z,stop_if_necessary);
	}
}

chafind_results_t chafind_results;
volatile int send_chafind_results;

void navig_fsm2_for_charger()
{
	static int pass;
	static int timer;
	static int store_back, store_shift_ang;
	switch(chafind_state)
	{
		case CHAFIND_START:
		{
			memset(&chafind_results, 0, sizeof(chafind_results_t));
			pass = 0;
			dbg[1] = dbg[2] = dbg[3] = dbg[5] = dbg[5] = dbg[6] = dbg[7] = 0;
			chafind_empty_accum1();
			timer = 1000;
			chafind_state++;
		}
		break;

		case CHAFIND_WAIT_DISTANCE:
		{
			if(--timer == 0)
			{
				if(chafind_nearest_hit_x < 150 || chafind_nearest_hit_x > 1200) // todo: also check obstacles from back
				{
					chafind_state = CHAFIND_FAIL;
				}
				else
				{
					int movement = chafind_nearest_hit_x - CHAFIND_FIRST_DIST;
					if(movement < -CHAFIND_FIRST_DIST_TOLERANCE || movement > CHAFIND_FIRST_DIST_TOLERANCE)
					{
						set_top_speed_max(10);
						straight_rel(movement);
						chafind_results.first_movement_needed = movement;
						chafind_state = CHAFIND_WAIT_FWD1;
					}
					else
					{
						chafind_empty_accum2();
						chafind_results.first_movement_needed = 0;
						chafind_state = CHAFIND_ACCUM_DATA;
					}
				}
			}
		}
		break;

		case CHAFIND_WAIT_FWD1:
		{
			if(!correcting_either())
			{
				timer = 200;
				chafind_state++;
			}
		}
		break;

		case CHAFIND_WAIT_FWD1_STOPEXTRA1:
		{
			if(--timer == 0)
			{
				timer = 2700;
				chafind_empty_accum1();
				chafind_state++;
			}
		}
		break;

		case CHAFIND_WAIT_FWD1_STOPEXTRA2:
		{
			if(--timer == 0)
			{
				chafind_empty_accum2();
				chafind_state++;
			}
		}
		break;

		case CHAFIND_ACCUM_DATA:
		{
			if((pass == 0 && chafind_left_accum_cnt > 30 && chafind_right_accum_cnt > 30 && chafind_middle_cnt > 20) ||
			   (pass  > 0 && chafind_left_accum_cnt > 60 && chafind_right_accum_cnt > 60 && chafind_middle_cnt > 40) )
			{
				pass++;
				int ang, shift, dist;
				chafind_calc(&ang, &shift, &dist);

				if(ang > -1*CHAFIND_PASS1_ACCEPT_ANGLE && ang < CHAFIND_PASS1_ACCEPT_ANGLE &&
					shift > -1*CHAFIND_PASS1_ACCEPT_SHIFT && shift < CHAFIND_PASS1_ACCEPT_SHIFT)
				{
					chafind_results.accepted_pos++;

					dbg[4] = 11111;
					dbg[5] = dbg[6] = 0;
					set_top_speed_max(5);

					int dist2 = chafind_total_front_accum/chafind_total_front_accum_cnt;
					straight_rel(dist2-robot_origin_to_front_TIGHT-270);
					chafind_state = CHAFIND_WAIT_FWD2;
				}
				else
				{
					set_top_speed_max(6);

					allow_angular(1);
					auto_disallow(0);
					if(shift > -1*CHAFIND_PASS1_ACCEPT_SHIFT && shift < CHAFIND_PASS1_ACCEPT_SHIFT) // Only turning needed
					{
						chafind_results.turning_passes_needed++;
						dbg[4] = 22222;
						dbg[5] = ang/ANG_0_1_DEG;
						dbg[6] = 0;
						rotate_rel(-1*ang);
						chafind_state = CHAFIND_WAIT_REBACKING;

					}
					else // vexling needed
					{
						chafind_results.vexling_passes_needed++;
						dbg[4] = 33333;
						allow_straight(1);
						int shift_ang;
						if(shift > 170)       shift_ang = 30*ANG_1_DEG;
						else if(shift > 70)   shift_ang = 15*ANG_1_DEG;
						else if(shift > 0)    shift_ang =  8*ANG_1_DEG;
						else if(shift < -170) shift_ang = -30*ANG_1_DEG;
						else if(shift < -70)  shift_ang = -15*ANG_1_DEG;
						else                  shift_ang =  -8*ANG_1_DEG;

						int d = ((float)shift)/tan((float)shift_ang*(2.0*M_PI)/4294967296.0);
						rotate_rel(-1*(ang+shift_ang));
						dbg[5] = (-1*(ang+shift_ang))/ANG_0_1_DEG;
						dbg[6] = (-1*shift_ang)/ANG_0_1_DEG;
						dbg[7] = d;
						straight_rel(-1*d);
						store_back = d;
						store_shift_ang = shift_ang;
						chafind_state = CHAFIND_WAIT_BACKING;
					}
				}
				
			}
			else
			{
				dbg[4] = chafind_left_accum_cnt;
				dbg[5] = chafind_right_accum_cnt;
				dbg[6] = chafind_middle_cnt;
			}
	
		}
		break;

		case CHAFIND_WAIT_BACKING:
		{
			if(!correcting_either())
			{
				timer=200;
				chafind_state++;
			}
		}
		break;

		case CHAFIND_START_REBACKING:
		{
			if(--timer == 0)
			{
				rotate_rel(store_shift_ang);
				straight_rel(store_back);
				chafind_state++;
			}
		}
		break;


		case CHAFIND_WAIT_REBACKING:
		{
			if(!correcting_either())
			{
				timer = 200;
				chafind_state = CHAFIND_WAIT_FWD1_STOPEXTRA1;
			}
		}
		break;

		case CHAFIND_WAIT_FWD2:
		{
			if(!correcting_either())
			{
				timer = 200;
				chafind_state++;
			}
		}
		break;

		case CHAFIND_WAIT_FWD2_STOPEXTRA1:
		{
			if(--timer == 0)
			{
				chafind_state++;
				chafind_total_front_accum = 0;
				chafind_total_front_accum_cnt = 0;
			}
		}
		break;

		case CHAFIND_ACCUM_FRONTAVG:
		{
			/*
				Average enough samples directly from the front, to measure the distance to go. Go a bit further than that.
				This prevents excessive travel in case the charger is unpowered, or we are at the wrong place.
			*/
			if(chafind_total_front_accum_cnt > 300)
			{
				int dist = chafind_total_front_accum/chafind_total_front_accum_cnt;
				chafind_results.dist_before_push = dist;
				set_top_speed_max(0);
				straight_rel(dist-robot_origin_to_front_TIGHT-CHAFIND_PUSH_TUNE);
				chafind_state = CHAFIND_WAIT_PUSH;
			}
			else
			{
				dbg[4] = chafind_total_front_accum_cnt;
			}
		}
		break;


		case CHAFIND_WAIT_PUSH:
		{
			if(get_cha_v() > CHAFIND_ACCEPT_MILLIVOLTS)
			{
				chafind_state = CHAFIND_SUCCESS;
			}
			if(!correcting_either())
			{
				chafind_state = CHAFIND_FAIL;
			}
		}
		break;

		case CHAFIND_FAIL:
		{
			chafind_results.result = 0;
			reset_movement();
			send_chafind_results = 1;
			chafind_state = 0;
		}
		break;

		case CHAFIND_SUCCESS:
		{
			chafind_results.result = 100;
			reset_movement();
			send_chafind_results = 1;
			chafind_state = 0;
		}
		break;

		default:
		break;
	}

}


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

void navig_fsm2()
{

	if(daiju_meininki)
	{
		daiju_meininki_fsm();
		return;
	}

	if(chafind_state)
	{
		lidar_near_filter_on = 0;
		lidar_midlier_filter_on = 0;

		navig_fsm2_for_charger();
		return;
	}
	else
	{
		lidar_near_filter_on = 1;
		lidar_midlier_filter_on = 1;
	}

	move_fsm();
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

