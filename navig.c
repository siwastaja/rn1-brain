/*
Low-level navigation module.

Collision avoidance, simple mechanical tasks.

*/

#include <stdint.h>
#include <math.h>

#include "lidar.h"
#include "sonar.h"
#include "navig.h"
#include "feedbacks.h"
#include "lidar_corr.h" // for point_t (for lidar collision avoidance)

extern point_t lidar_collision_avoidance[360];
volatile int lidar_collision_avoidance_new;


extern int latest_sonars[MAX_NUM_SONARS]; // in cm, 0 = no echo

extern volatile int dbg[10];

#define XY_DONT_AT_END_LEN 250 /*don't recalculate ang,straight at this many last millimeters to (x,y) dstination*/


typedef enum {
	MOVE_IDLE 		= 0, 
	MOVE_START          	= 1,
	MOVE_WAIT_STRAIGHT	= 2,  // Wait for straight motion to end
	MOVE_END           	= 3
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


int nearest_sonar()
{
	int n = 99999;
	if(latest_sonars[0] && latest_sonars[0] < n) n = latest_sonars[0];
	if(latest_sonars[1] && latest_sonars[1] < n) n = latest_sonars[1];
	if(latest_sonars[2] && latest_sonars[2] < n) n = latest_sonars[2];
	return n;
}

void move_fsm()
{
	switch(cur_move.state)
	{
		case MOVE_IDLE:
		break;

		case MOVE_START:
		{
			allow_angular(1);
			allow_straight(1);
			auto_disallow(1);
			rotate_abs(cur_move.abs_ang);
			straight_rel(cur_move.rel_fwd);
			cur_move.state++;
		}
		break;

		case MOVE_WAIT_STRAIGHT:
		if(correct_xy && (cur_move.rel_fwd < -1*XY_DONT_AT_END_LEN || cur_move.rel_fwd > XY_DONT_AT_END_LEN))
		{
			change_angle_abs(cur_move.abs_ang);
			change_straight_rel(cur_move.rel_fwd);
		}

		if(!correcting_either())
		{
			cur_move.state++;
		}
		break;

		case MOVE_END:	
		{
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

void move_rel_twostep(int angle16, int fwd /*in mm*/)
{
	reset_movement();
	take_control();
	was_correcting_xy = 0;
	correct_xy = 0;
	avoidance_in_action = 0;
	cur_move.state = MOVE_START;
	cur_move.abs_ang = cur_pos.ang + (angle16<<16);
	cur_move.rel_fwd = fwd;
	cur_move.valid = 1;
}

void move_absa_rels_twostep(int angle32, int fwd /*in mm*/)
{
	reset_movement();
	take_control();
	was_correcting_xy = 0;
	correct_xy = 0;
	avoidance_in_action = 0;
	cur_move.state = MOVE_START;
	cur_move.abs_ang = angle32;
	cur_move.rel_fwd = fwd;
	cur_move.valid = 1;
}

static int back_mode_hommel = 0;

int get_xy_left()
{
	return xy_left;
}

int get_xy_id()
{
	return xy_id;
}


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

	cur_move.rel_fwd = new_fwd;
	cur_move.abs_ang = new_ang;
}


void move_xy_abs(int32_t x, int32_t y, int back_mode)
{
	back_mode_hommel = back_mode;
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

	move_absa_rels_twostep(new_ang, new_fwd);
	was_correcting_xy = 1;
	correct_xy = 1;
	xy_id++;
	if(xy_id > 99) xy_id = 0;

}

void navig_fsm1()
{
	xy_fsm();
}

#define ROBOT_YS (480+20)
#define ROBOT_XS (524+20)
#define ROBOT_ORIGIN_TO_FRONT (150+10)
#define ROBOT_ORIGIN_TO_BACK  (390+10)

void navig_fsm2()
{
	if(lidar_collision_avoidance_new)
	{
		lidar_collision_avoidance_new = 0;


		int fwd_remain = get_fwd();

		int reverse = 0;

		if(fwd_remain < 0)
		{
			reverse = 1;
			fwd_remain *= -1;
		}

		if(fwd_remain < 30 || reverse /*currently not implemented */)
		{
			goto SKIP_COLL_AVOID;
		}

/*
		lidar_collision_avoidance[360] is the latest lidar scan converted to 2d XY plane, referenced to the
		robot origin, always in this orientation:
                      ...
	  ##########   ...
	  ##########    idx359
	y #######O##     idx0
	  ##########    idx1
	  ##########   ...
               x      ...

*/

		// Check the right side:

		int limited = 0;
		int stop = 0;
		int turn[2] = {0, 0}; // to left, right
		int do_not_turn[2] = {0, 0};   // to right, left
		for(int d = 0; d < 2; d++)
		{
			for(int i = (d?295:20); i < (d?340:65); i++)
			{
				int ang = d?(360-i):(i);

				if(!lidar_collision_avoidance[i].valid) continue;

				int dist_to_front = lidar_collision_avoidance[i].x - ROBOT_ORIGIN_TO_FRONT;

				if((!d && lidar_collision_avoidance[i].y < ROBOT_YS/2) ||
				   ( d && lidar_collision_avoidance[i].y > -1*ROBOT_YS/2))
				{
					if(fwd_remain < dist_to_front)
					{
						// No need to avoid this point, we are going to stop anyway.
						continue;
					}

					if(dist_to_front < 70)
					{
						speed_limit(2);

						if(ang < 55)
						{
							dbg[4]++;
							stop = 1; break;
						}
					}
					else if(dist_to_front < 110)
					{
						speed_limit(2);
						if(ang > 45) {turn[d] = 4;} else {dbg[5]++;stop = 1; break;}
					}
					else if(dist_to_front < 140)
					{
						speed_limit(2);
						if(ang > 35) {turn[d] = 4;} else {dbg[5]++;stop = 1; break;}
					}
					else if(dist_to_front < 170)
					{
						speed_limit(2);
						if(ang > 25) {turn[d] = 4;} else {dbg[6]++;stop = 1; break;}
					}
					else if(dist_to_front < 320)
					{
						speed_limit(2);
						turn[d] = 4;
					}
					else if(dist_to_front < 450)
					{
						speed_limit(2);
						if(ang < 30)
						{
							turn[d] = 4;
						}
						else
						{if(turn[d] < 2) turn[d] = 2;}
					}
					else if(dist_to_front < 500)
					{
						speed_limit(2);
						limited = 1;
					}
					else if(dist_to_front < 1000)
					{
						speed_limit(1);
						limited = 1;
					}
				}
				else 
				if((!d && lidar_collision_avoidance[i].y < ROBOT_YS/2+70) ||
				   ( d && lidar_collision_avoidance[i].y > -1*(ROBOT_YS/2+70)))
				{
					if(dist_to_front < 450)
					{
						speed_limit(2);
						if(turn[d]<1) turn[d] = 1;
					}
					else if(dist_to_front < 500)
					{
						speed_limit(2);
						limited = 1;
					}
					else if(dist_to_front < 1000)
					{
						speed_limit(1);
						limited = 1;
					}

					if(dist_to_front < 250 && dist_to_front > -190)
					{
						do_not_turn[d] = 1;
					}

				}
				else 
				if((!d && lidar_collision_avoidance[i].y < ROBOT_YS/2+200) ||
				   ( d && lidar_collision_avoidance[i].y > -1*(ROBOT_YS/2+200)))
				{
					if(dist_to_front < 500)
					{
						speed_limit(2);
						limited = 1;
					}
					else if(dist_to_front < 1000)
					{
						speed_limit(1);
						limited = 1;
					}

				}

			}

/*
			// Check the arse, to avoid hitting when turning.
			for(int i = (d?(90-1):(180+35)); i < (d?(90+35):(180+90+1)); i++)
			{
				if(!lidar_collision_avoidance[i].valid) continue;

				int ang = d?(360-i):(i);

				if((!d && lidar_collision_avoidance[i].y < ROBOT_YS/2+50) ||
				   ( d && lidar_collision_avoidance[i].y > -1*(ROBOT_YS/2+50)))
				{
					if(lidar_collision_avoidance[i].x < 50 && lidar_collision_avoidance[i].x > -ROBOT_ORIGIN_TO_BACK)
					{
						do_not_turn[d] = 1;
					}

				}


			}
*/

			for(int i = (d?340:0); i < (d?360:20); i++)
			{
				if(!lidar_collision_avoidance[i].valid) continue;

				if((!d && lidar_collision_avoidance[i].y < ROBOT_YS/2) ||
				   ( d && lidar_collision_avoidance[i].y > -1*ROBOT_YS/2))
				{
					int dist_to_front = lidar_collision_avoidance[i].x - ROBOT_ORIGIN_TO_FRONT;

					if(fwd_remain > dist_to_front)
					{
						if(dist_to_front < 120)
						{
							stop = 1;
						}
						else if(dist_to_front < 500)
						{
							speed_limit(2);
							limited = 1;
						}
						else if(dist_to_front < 1000)
						{
							speed_limit(1);
							limited = 1;
						}
					}
				}

			}
		}

		if(turn[0] && turn[1])
		{
			stop = 1;
		}

		if(stop)
		{
			reset_movement();
			cur_move.state = 0;
		}
		else if(turn[0])
		{
			speed_limit(1);
			change_angle_rel(turn[0]*-5*ANG_0_25_DEG);
			set_ang_top_speed(90000 + turn[0]*10000);
			correct_xy = 0;
			avoidance_in_action = 1+turn[0];
		}
		else if(turn[1])
		{
			speed_limit(1);
			change_angle_rel(turn[1]*5*ANG_0_25_DEG);
			set_ang_top_speed(90000 + turn[1]*10000);
			correct_xy = 0;
			avoidance_in_action = 1+turn[1];
		}
		else if(do_not_turn[0] || do_not_turn[1])
		{
			avoidance_in_action = 1;
		}
		else
		{
			if(avoidance_in_action)
				avoidance_in_action--;
			else
				correct_xy = was_correcting_xy;
		}

		if(!stop && !turn[0] && !turn[1] && !do_not_turn[0] && !do_not_turn[1] && !limited && !avoidance_in_action)
		{
			reset_speed_limits();
		}

		SKIP_COLL_AVOID:;

	}


/*

	if(cur_move.state && cur_move.rel_fwd > 0) // check sonars if running positive forward.
	{
		int son = nearest_sonar();
		int limit_status = speed_limit_status();
		if((limit_status == 0 && son < 50) ||
		   (limit_status == 1 && son < 35) ||
		   (limit_status == 2 && son < 15))
		{
			lower_speed_limit();
		}

		if(son < 10)
		{
			reset_movement();
			cur_move.state = 0;
		}
	}
*/
	move_fsm();
}

void stop_navig_fsms()
{
	cur_move.state = 0;
}
