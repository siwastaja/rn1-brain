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
	MOVE_START		= 1, // Command rotation to final angle
	MOVE_WAIT_ROTATION	= 2, // Wait until angle is fixed
	MOVE_LIDAR_SYNC_1	= 3, // Wait for lidar scan to complete, ignoring the scan
	MOVE_LIDAR_STORE_1A	= 4, // Keep the robot standstill, wait for half the scan, then copy the first half
	MOVE_LIDAR_STORE_1B	= 5, // While keeping still, wait for & copy the second half; instruct straight motion
	MOVE_WAIT_STRAIGHT	= 6, // Wait for straight motion to end
	MOVE_LIDAR_SYNC_2	= 7, // Wait for lidar scan to complete, to ignore it again (data during the movement)
	MOVE_LIDAR_STORE_2A	= 8, // Like above, to get the "after" scan.
	MOVE_LIDAR_STORE_2B	= 9
} move_state_t;

typedef struct
{
	int valid;
	move_state_t state;
	int abs_angle;
	int rel_fwd;
	int lidar_before_valid;
	lidar_scan_t lidar_before;
	int lidar_after_valid;
	lidar_scan_t lidar_after;
} move_t;

static move_t cur_move;

lidar_scan_t* get_valid_before_lidar()
{
	if(cur_move.lidar_before_valid)
	{
		cur_move.lidar_before_valid = 0;
		return &(cur_move.lidar_before);
	}
	return 0;
}

lidar_scan_t* get_valid_after_lidar()
{
	if(cur_move.lidar_after_valid)
	{
		cur_move.lidar_after_valid = 0;
		return &(cur_move.lidar_after);
	}
	return 0;
}


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
		allow_angular(1);
		allow_straight(1);
		dcnt = 0;

		rotate_abs(cur_move.abs_angle);
		cur_move.state++;
		break;

		case MOVE_WAIT_ROTATION:
		if(!correcting_angle())
		{
			dbg[3] = dcnt; dcnt = 0;
			lidar_reset_flags();
			cur_move.state++;
		}
		break;

		case MOVE_LIDAR_SYNC_1:
		if(lidar_is_complete())
		{
			dbg[4] = dcnt; dcnt = 0;
			allow_angular(0);
			allow_straight(0);
			lidar_reset_flags();
			cur_move.state++;
		}
		break;

		case MOVE_LIDAR_STORE_1A:
		if(lidar_is_half())
		{
			dbg[5] = dcnt; dcnt = 0;
			/* Half of the lidar data is there, we can process and copy it, there is
			   no risk of it being overwritten, since the LIDAR is writing the second half right now. 
			   Since the robot is not moving, it shouldn't matter whether we sample cur_angle, cur_x, cur_y
			   at the start, middle or end of the lidar scan, but to be on the safe side, we do it in the middle.
			*/
			cur_move.lidar_before.angle = cur_angle;
			cur_move.lidar_before.x = cur_x;
			cur_move.lidar_before.y = cur_y;
			copy_lidar_half1(cur_move.lidar_before.scan);
			cur_move.state++;
		}
		break;

		case MOVE_LIDAR_STORE_1B:
		if(lidar_is_complete())
		{
			dbg[6] = dcnt; dcnt = 0;
			allow_angular(1);
			allow_straight(1);
			straight_rel(cur_move.rel_fwd);
			copy_lidar_half2(cur_move.lidar_before.scan);
			cur_move.lidar_before_valid = 1;
			cur_move.state++;
		}
		break;

		case MOVE_WAIT_STRAIGHT:
		if(!correcting_straight())
		{
			dbg[7] = dcnt; dcnt = 0;
			lidar_reset_flags();
			cur_move.state++;
		}
		break;

		case MOVE_LIDAR_SYNC_2:
		if(lidar_is_complete())
		{
			dbg[8] = dcnt; dcnt = 0;
			allow_angular(0);
			allow_straight(0);
			lidar_reset_flags();
			cur_move.state++;
		}
		break;

		case MOVE_LIDAR_STORE_2A:
		if(lidar_is_half())
		{
			dbg[9] = dcnt; dcnt = 0;
			cur_move.lidar_after.angle = cur_angle;
			cur_move.lidar_after.x = cur_x;
			cur_move.lidar_after.y = cur_y;
			copy_lidar_half1(cur_move.lidar_after.scan);
			cur_move.state++;
		}
		break;

		case MOVE_LIDAR_STORE_2B:
		if(lidar_is_complete())
		{
			allow_angular(1);
			allow_straight(1);
			copy_lidar_half2(cur_move.lidar_after.scan);
			cur_move.lidar_after_valid = 1;
			cur_move.state++;
		}
		break;


		default:
		dcnt = 0;

		break;
	}
}


void move_rel_twostep(int angle, int fwd /*in mm*/)
{
	cur_move.state = MOVE_START;
	cur_move.abs_angle = cur_angle + (angle<<16);
	cur_move.rel_fwd = fwd;
	cur_move.valid = 1;
	cur_move.lidar_before_valid = 0;
	cur_move.lidar_after_valid = 0;
}


void navig_fsm()
{
	move_fsm();
	dbg[1] = cur_move.state;
}
