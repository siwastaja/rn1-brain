/*
Low-level navigation module.

Collision avoidance, simple mechanical tasks.

*/

#include <stdint.h>

#include "lidar.h"
#include "sonar.h"
#include "navig.h"
#include "feedbacks.h"

extern volatile int dbg[10];


// State machine for lidar-synced turn-then-straight segment.
// Lidar data is obtained when the robot is not moving for non-distorted image.
typedef enum {
	MOVE_IDLE 		= 0, 
	MOVE_START          	= 1,
	MOVE_LIDAR_SYNC_0	= 2,  // Wait for lidar scan to complete, ignoring the scan
	MOVE_LIDAR_STORE_0A	= 3,  // Keep the robot standstill, wait for half the scan, then copy the first half
	MOVE_LIDAR_STORE_0B	= 4,  // While keeping still, wait for & copy the second half; instruct straight motion
	MOVE_WAIT_ROTATION	= 5,  // Rotate to the final angle; wait until angle is fixed
	MOVE_LIDAR_SYNC_1	= 6,  
	MOVE_LIDAR_STORE_1A	= 7,  
	MOVE_LIDAR_STORE_1B	= 8,
	MOVE_WAIT_CALC_1	= 9,
	MOVE_WAIT_STRAIGHT	= 10,  // Wait for straight motion to end
	MOVE_LIDAR_SYNC_2	= 11, 
	MOVE_LIDAR_STORE_2A	= 12,
	MOVE_LIDAR_STORE_2B	= 13,
	MOVE_WAIT_CALC_2	= 14
} move_state_t;

typedef struct
{
	int valid;
	move_state_t state;
	int abs_angle;
	int rel_fwd;
	int lidar_nonread[3];
	lidar_scan_t lidars[3]; // Before turning; after turning; after straight segment. Including the assumed (ang,x,y).
} move_t;

static move_t cur_move;

lidar_scan_t* move_get_valid_lidar(int idx)
{
	if(idx < 0 || idx > 2)
		return 0;

	if(cur_move.lidar_nonread[idx])
	{
		cur_move.lidar_nonread[idx] = 0;
		return &cur_move.lidars[idx];
	}
	return 0;
}

lidar_scan_t* move_get_lidar(int idx)
{
	if(idx < 0 || idx > 2)
		return 0;

	return &cur_move.lidars[idx];
}

extern volatile int lidar_calc_req;

void move_fsm()
{
	static int dcnt = 0;

	dcnt++;
	switch(cur_move.state)
	{
		case MOVE_IDLE:
		dcnt = 0;
		break;

		case MOVE_START:
		dcnt = 0;

		// TODO: check if robot has been nonmoving already, and skip the extra wait.
		// TODO: Timeout and error if robot is moving; it shouldn't be.
		allow_angular(0);
		allow_straight(0);
		if(!robot_moving())
		{
			dbg[2] = dbg[3] = dbg[4] = dbg[5] = 0;
			cur_move.lidars[0].pos.ang = 0;
			cur_move.lidars[0].pos.x = 9999;	
			cur_move.lidars[0].pos.y = 9999;
			for(int i = 0; i < 360; i+=4)
			{
				cur_move.lidars[0].scan[i] = 8000;
				cur_move.lidars[1].scan[i] = 8000;
				cur_move.lidars[2].scan[i] = 8000;
			}
			cur_move.lidars[1].pos.ang = 0;
			cur_move.lidars[1].pos.x = 9999;	
			cur_move.lidars[1].pos.y = 9999;

			cur_move.lidars[2].pos.ang = 0;
			cur_move.lidars[2].pos.x = 9999;	
			cur_move.lidars[2].pos.y = 9999;

			lidar_reset_flags();
			cur_move.state++;
		}
		break;

		case MOVE_LIDAR_SYNC_0:
		if(lidar_is_complete())
		{
			lidar_reset_flags();
			cur_move.state++;
		}
		break;

		case MOVE_LIDAR_STORE_0A:
		if(lidar_is_half())
		{
			/* Half of the lidar data is there, we can process and copy it, there is
			   no risk of it being overwritten, since the LIDAR is writing the second half right now. 
			   Since the robot is not moving, it shouldn't matter whether we sample cur_angle, cur_x, cur_y
			   at the start, middle or end of the lidar scan, but to be on the safe side, we do it in the middle.
			*/
			COPY_POS_PER10(cur_move.lidars[0].pos, cur_pos);
			copy_lidar_half1(cur_move.lidars[0].scan);
			cur_move.state++;
		}
		break;

		case MOVE_LIDAR_STORE_0B:
		if(lidar_is_complete())
		{
			copy_lidar_half2(cur_move.lidars[0].scan);
			cur_move.lidar_nonread[0] = 1;
			allow_angular(1);
			auto_disallow(1);
			rotate_abs(cur_move.abs_angle);
			cur_move.state++;
		}
		break;

		case MOVE_WAIT_ROTATION:
		if(!correcting_angle())
		{
//			dbg[3] = dcnt; dcnt = 0;
			lidar_reset_flags();
			cur_move.state++;
		}
		break;

		case MOVE_LIDAR_SYNC_1:
		if(lidar_is_complete())
		{
//			dbg[4] = dcnt; dcnt = 0;
			lidar_reset_flags();
			cur_move.state++;
		}
		break;

		case MOVE_LIDAR_STORE_1A:
		if(lidar_is_half())
		{
//			dbg[5] = dcnt; dcnt = 0;
			COPY_POS_PER10(cur_move.lidars[1].pos, cur_pos);
			copy_lidar_half1(cur_move.lidars[1].scan);
			cur_move.state++;
		}
		break;

		case MOVE_LIDAR_STORE_1B:
		if(lidar_is_complete())
		{
//			dbg[6] = dcnt; dcnt = 0;
			copy_lidar_half2(cur_move.lidars[1].scan);
			cur_move.lidar_nonread[1] = 1;
//			lidar_calc_req = 1;
			cur_move.state++;
		}
		break;

		case MOVE_WAIT_CALC_1:
		if(lidar_calc_req == 0)
		{
			allow_angular(1);
			allow_straight(1);
			auto_disallow(1);
			straight_rel(cur_move.rel_fwd);
			cur_move.state++;
		}
		break;

		case MOVE_WAIT_STRAIGHT:
		if(!correcting_straight() && !correcting_angle())
		{
//			dbg[7] = dcnt; dcnt = 0;
			lidar_reset_flags();
			cur_move.state++;
		}
		break;

		case MOVE_LIDAR_SYNC_2:
		if(lidar_is_complete())
		{
//			dbg[8] = dcnt; dcnt = 0;
			lidar_reset_flags();
			cur_move.state++;
		}
		break;

		case MOVE_LIDAR_STORE_2A:
		if(lidar_is_half())
		{
//			dbg[9] = dcnt; dcnt = 0;
			COPY_POS_PER10(cur_move.lidars[2].pos, cur_pos);
			copy_lidar_half1(cur_move.lidars[2].scan);
			cur_move.state++;
		}
		break;

		case MOVE_LIDAR_STORE_2B:
		if(lidar_is_complete())
		{
			reset_movement();
			auto_disallow(0);
			allow_angular(1);
			allow_straight(1);
			copy_lidar_half2(cur_move.lidars[2].scan);
			cur_move.lidar_nonread[2] = 1;
			lidar_calc_req = 2;
			cur_move.state++;
		}
		break;

		case MOVE_WAIT_CALC_2:
		if(lidar_calc_req == 0)
		{
			cur_move.state = 0;
		}
		break;

		default:
		dcnt = 0;

		break;
	}
}


void move_rel_twostep(int angle, int fwd /*in mm*/)
{
	dbg[8]+=100;
	reset_movement();
	take_control();
	cur_move.state = MOVE_START;
	cur_move.abs_angle = cur_pos.ang + (angle<<16);
	cur_move.rel_fwd = fwd;
	cur_move.valid = 1;
	cur_move.lidar_nonread[0] = 0;
	cur_move.lidar_nonread[1] = 0;
	cur_move.lidar_nonread[2] = 0;
}


void navig_fsm()
{
	move_fsm();
	dbg[1] = cur_move.state;
}

void stop_navig_fsms()
{
	cur_move.state = 0;
}
