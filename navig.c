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
int collision_avoidance_on;

uint32_t store_stop_flags = 0;
uint32_t store_action_flags = 0;


extern int latest_sonars[MAX_NUM_SONARS]; // in cm, 0 = no echo

extern volatile int dbg[10];

#define XY_DONT_AT_END_LEN 200 /*don't recalculate ang,straight at this many last millimeters to (x,y) dstination*/


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
			collision_avoidance_on = 1;
			allow_angular(1);
			auto_disallow(1);
			rotate_abs(cur_move.abs_ang);
			cur_move.state++;
		}
		break;

		case MOVE_WAIT_ANGULAR:
		{
			if(angle_almost_corrected())
			{
				straight_rel(cur_move.rel_fwd);
				allow_straight(1);
				cur_move.state++;
			}
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
	store_action_flags = store_stop_flags = 0;
	avoidance_in_action = 0;
	collision_avoidance_on = 1;
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
	store_action_flags = store_stop_flags = 0;
	collision_avoidance_on = 1;
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


void move_xy_abs(int32_t x, int32_t y, int back_mode, int id)
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

	xy_left = new_fwd;

	move_absa_rels_twostep(new_ang, new_fwd);
	was_correcting_xy = 1;
	correct_xy = 1;
	xy_id = id;
}

void navig_fsm1()
{
	xy_fsm();
}

#define ROBOT_YS_TIGHT (480)
#define ROBOT_XS_TIGHT (524)
#define ROBOT_ORIGIN_TO_FRONT_TIGHT (150)
#define ROBOT_ORIGIN_TO_BACK_TIGHT  (390)

int ROBOT_YS, ROBOT_XS, ROBOT_ORIGIN_TO_FRONT, ROBOT_ORIGIN_TO_BACK;

void set_obstacle_avoidance_margin(int cm)
{
	ROBOT_YS = ROBOT_YS_TIGHT + 20*cm;
	ROBOT_XS = ROBOT_XS_TIGHT + 20*cm;
	ROBOT_ORIGIN_TO_FRONT = ROBOT_ORIGIN_TO_FRONT_TIGHT + 10*cm;
	ROBOT_ORIGIN_TO_BACK = ROBOT_ORIGIN_TO_BACK_TIGHT + 10*cm;
}

uint32_t get_obstacle_avoidance_stop_flags()
{
	return store_stop_flags;
}

uint32_t get_obstacle_avoidance_action_flags()
{
	return store_action_flags;
}

void navig_fsm2()
{
	move_fsm();

	if(!collision_avoidance_on)
	{
		lidar_collision_avoidance_new = 0;
		return;
	}

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

		if(reverse /*currently not implemented */)
		{
			goto SKIP_COLL_AVOID;
		}

		if(fwd_remain > 1200) fwd_remain = 1200; // Don't do any decisions based on a far-away destination.

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

		uint32_t stop_flags = 0;
		uint32_t action_flags = 0;

		int limited = 0;
		int stop = 0;
		int turn[2] = {0, 0}; // to left, right
		int do_not_turn[2] = {0, 0};   // to right, left
		for(int d = 0; d < 2; d++)
		{
			for(int i = (d?295:15); i < (d?345:65); i++)
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
							stop_flags |= 1UL<<(0+(d?16:0));
							stop = 1; break;
						}
					}
					else if(dist_to_front < 110)
					{
						speed_limit(2);
						if(ang > 38) {action_flags |= 1UL<<(1+(d?16:0)); turn[d] = 4;} else {stop_flags |= 1UL<<(1+(d?16:0)); stop = 1; break;}
					}
					else if(dist_to_front < 140)
					{
						speed_limit(2);
						if(ang > 30) {action_flags |= 1UL<<(2+(d?16:0)); turn[d] = 4;} else {stop_flags |= 1UL<<(2+(d?16:0)); stop = 1; break;}
					}
					else if(dist_to_front < 170)
					{
						speed_limit(2);
						if(ang > 22) {action_flags |= 1UL<<(3+(d?16:0)); turn[d] = 4;} else {stop_flags |= 1UL<<(3+(d?16:0)); stop = 1; break;}
					}
					else if(dist_to_front < 300)
					{
						speed_limit(2);
						action_flags |= 1UL<<(4+(d?16:0)); 
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
						action_flags |= 1UL<<(5+(d?16:0)); 
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
				if((!d && lidar_collision_avoidance[i].y < ROBOT_YS/2+80) ||
				   ( d && lidar_collision_avoidance[i].y > -1*(ROBOT_YS/2+80)))
				{
					if(dist_to_front < 450)
					{
						speed_limit(2);
						if(turn[d]<1) turn[d] = 1;
						action_flags |= 1UL<<(6+(d?16:0)); 
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

					if(dist_to_front < ((fwd_remain>350)?350:fwd_remain) && dist_to_front > -190)
					{
						action_flags |= 1UL<<(7+(d?16:0)); 
						do_not_turn[d] = 1;
					}

				}
				else 
				if((!d && lidar_collision_avoidance[i].y < ROBOT_YS/2+120) ||
				   ( d && lidar_collision_avoidance[i].y > -1*(ROBOT_YS/2+120)))
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

					if(dist_to_front < ((fwd_remain>350)?350:fwd_remain) && dist_to_front > 50)
					{
						action_flags |= 1UL<<(8+(d?16:0)); 
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


			// Check the arse, to avoid hitting when turning.
			for(int i = (d?(90-1):(180+35)); i < (d?(90+35):(180+90+1)); i++)
			{
				if(!lidar_collision_avoidance[i].valid) continue;

				//int ang = d?(360-i):(i);

				if((!d && lidar_collision_avoidance[i].y > -1*(ROBOT_YS/2+60)) ||
				   ( d && lidar_collision_avoidance[i].y < ROBOT_YS/2+60))
				{
					if(lidar_collision_avoidance[i].x < 0 && lidar_collision_avoidance[i].x > -ROBOT_ORIGIN_TO_BACK)
					{
						speed_limit(1);
						do_not_turn[d] |= 2;
						action_flags |= 1UL<<(9+(d?16:0)); 
						break;
					}

				}
				else
				if((!d && lidar_collision_avoidance[i].y > -1*(ROBOT_YS/2+200)) ||
				   ( d && lidar_collision_avoidance[i].y < ROBOT_YS/2+200))
				{
					if(lidar_collision_avoidance[i].x < 0 && lidar_collision_avoidance[i].x > -1*(ROBOT_ORIGIN_TO_BACK+150))
					{
						speed_limit(1);
						limited = 1;
						break;
					}

				}



			}

			for(int i = (d?345:0); i < (d?360:15); i++)
			{
				if(!lidar_collision_avoidance[i].valid) continue;

				if((!d && lidar_collision_avoidance[i].y < ROBOT_YS/2) ||
				   ( d && lidar_collision_avoidance[i].y > -1*ROBOT_YS/2))
				{
					int dist_to_front = lidar_collision_avoidance[i].x - ROBOT_ORIGIN_TO_FRONT;

					if(fwd_remain > dist_to_front)
					{
						if(dist_to_front < 100)
						{
							stop_flags |= 1UL<<(4+(d?16:0));
							stop = 1;
						}
						else if(dist_to_front < 150)
						{
							speed_limit(3);
							limited = 1;
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

		/*
		Use sonars for secondary information.
		
		We can maneuver left or right to avoid an obstacle, but we won't know how long it takes to actually pass it, since
		it disappears from the sonar's view when we turn, so we just make assumptions.
		*/

		int sonar_assumption_made = 0;

		int nearest_son = nearest_sonar();
		if(nearest_son < 80)
		{ speed_limit(1); limited = 1; avoidance_in_action = 5;}
		else if(nearest_son < 45)
		{ speed_limit(2); limited = 1; avoidance_in_action = 5;}
		else if(nearest_son < 10)
		{
			stop_flags |= 1UL<<5;
			stop = 1;
		}
		int fwd_remain_or = fwd_remain; if(fwd_remain_or > 400) fwd_remain_or = 400;

		if(latest_sonars[0] && (latest_sonars[0]*10-ROBOT_ORIGIN_TO_FRONT) < fwd_remain_or &&
		   (!latest_sonars[1] || latest_sonars[1]*10 > fwd_remain) &&
		   (!latest_sonars[2] || latest_sonars[2]*10 > fwd_remain))
		{
			// Leftmost sonar sees an obstacle, two other do not; turn right if we already won't by the lidar.
			if(turn[1] < 4) turn[1] = 4;
			action_flags |= 1UL<<11; 
			speed_limit(2);
			sonar_assumption_made = 1;
		} 
		else
		if(latest_sonars[2] && (latest_sonars[2]*10-ROBOT_ORIGIN_TO_FRONT) < fwd_remain_or &&
		   (!latest_sonars[1] || latest_sonars[1]*10 > fwd_remain) &&
		   (!latest_sonars[0] || latest_sonars[0]*10 > fwd_remain))
		{
			// Same, but opposite -> turn left.
			if(turn[0] < 4) turn[0] = 4;
			action_flags |= 1UL<<12; 
			speed_limit(2);
			sonar_assumption_made = 1;
		}

		fwd_remain_or = fwd_remain-50; if(fwd_remain_or > 200) fwd_remain_or = 200;

		if((latest_sonars[1] && latest_sonars[1]*10-ROBOT_ORIGIN_TO_FRONT < fwd_remain_or) ||
		   (latest_sonars[1] && latest_sonars[1]*10-ROBOT_ORIGIN_TO_FRONT < fwd_remain_or &&
		   latest_sonars[2] && latest_sonars[2]*10-ROBOT_ORIGIN_TO_FRONT < fwd_remain_or))
		{
			// Stop if middle sonar sees an obstacle, or if both left and right see it.
			stop = 1;
			stop_flags |= 1UL<<6;
		}


		if(turn[0] && turn[1])
		{
//			if(turn[0] + turn[1] > 7)
//			{
//				stop = 1;
//				stop_flags |= 1UL<<7;
//			}
//			else
			{
				action_flags |= 1UL<<13; 
				if(turn[0] > turn[1]) turn[0] -= turn[1];
				else turn[1] -= turn[0];
			}
		}

		if((turn[0] && do_not_turn[1]) || (turn[1] && do_not_turn[0]))
		{
			speed_limit(2);
		}

		int ang_err = get_ang_err();

		static int see_if_this_goes_away2 = 0;

		if((do_not_turn[1]&2) && ang_err > 8*ANG_1_DEG) // || (do_not_turn[1] && ang_err > 8*ANG_1_DEG))
		{
			see_if_this_goes_away2++;
			do_not_turn[1] |= 1; // prevent turning
			if(see_if_this_goes_away2 > 5)
			{
				stop_flags |= 1UL<<8;
				stop = 1;
			}
		}
		else
			see_if_this_goes_away2 = 0;

		static int see_if_this_goes_away1 = 0;
		if((do_not_turn[0]&2) && ang_err < -8*ANG_1_DEG) // || (do_not_turn[0] && ang_err < -8*ANG_1_DEG))
		{
			see_if_this_goes_away1++;
			do_not_turn[0] |= 1; // prevent turning
			if(see_if_this_goes_away1 > 5)
			{
				stop_flags |= 1UL<<9;
				stop = 1;
			}
		}
		else
			see_if_this_goes_away1 = 0;

		if(cur_move.state && stop)
		{
			dbg[8] = store_stop_flags = stop_flags;
			dbg[9] = store_action_flags = action_flags;
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
			avoidance_in_action = 1+turn[0]+sonar_assumption_made*2;
		}
		else if(turn[1])
		{
			speed_limit(1);
			change_angle_rel(turn[1]*5*ANG_0_25_DEG);
			set_ang_top_speed(90000 + turn[1]*10000);
			correct_xy = 0;
			avoidance_in_action = 1+turn[1]+sonar_assumption_made*2;
		}
		else if((do_not_turn[0]&1) || (do_not_turn[1]&1))
		{
			avoidance_in_action = 3;
		} // &2 is relevant only when turning fast (and going to actually hit right now), and would hinder obstacle passing.
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


}

void stop_navig_fsms()
{
	cur_move.state = 0;
}
