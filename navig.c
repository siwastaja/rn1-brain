/*
Low-level navigation module.

Collision avoidance, simple mechanical tasks.

*/

#include <stdint.h>

#include "lidar.h"
#include "sonar.h"
#include "navig.h"
#include "feedbacks.h"


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
	MOVE_WAIT_STRAIGHT	= 9,  // Wait for straight motion to end
	MOVE_LIDAR_SYNC_2	= 10, 
	MOVE_LIDAR_STORE_2A	= 11,
	MOVE_LIDAR_STORE_2B	= 12
} move_state_t;

typedef struct
{
	move_state_t state;
	int abs_angle;
	int rel_fwd;
	lidar_scan_t lidars[3]; // Before turning; after turning; after straight segment. Including the assumed (ang,x,y).
} move_t;

static move_t cur_move;

void move_fsm()
{
	switch(cur_move.state)
	{
		case MOVE_IDLE:
		break;

		case MOVE_START:
		// TODO: check if robot has been nonmoving already, and skip the extra wait.
		// TODO: Timeout and error if robot is moving; it shouldn't be.
		if(!robot_moving())
		{	
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
			COPY_POS(cur_move.lidars[0], cur_pos);
			copy_lidar_half1(cur_move.lidars[0].scan);
			cur_move.state++;
		}
		break;

		case MOVE_LIDAR_STORE_0B:
		if(lidar_is_complete())
		{
			straight_rel(cur_move.rel_fwd);
			copy_lidar_half2(cur_move.lidars[0].scan);
			cur_move.state++;
		}
		break;


		case MOVE_ROTATE:
		rotate_abs(cur_move.abs_angle);
		cur_move.state++;
		break;

		case MOVE_WAIT_ROTATION:
		if(!correcting_angle())
		{
			lidar_reset_flags();
			cur_move.state++;
		}
		break;

		case MOVE_LIDAR_SYNC_1:
		if(lidar_is_complete())
		{
			lidar_reset_flags();
			cur_move.state++;
		}
		break;

		case MOVE_LIDAR_STORE_1A:
		if(lidar_is_half())
		{
			COPY_POS(cur_move.lidars[1], cur_pos);
			copy_lidar_half1(cur_move.lidars[1].scan);
			cur_move.state++;
		}
		break;

		case MOVE_LIDAR_STORE_1B:
		if(lidar_is_complete())
		{
			straight_rel(cur_move.rel_fwd);
			copy_lidar_half2(cur_move.lidars[1].scan);
			cur_move.state++;
		}
		break;

		case MOVE_WAIT_STRAIGHT:
		if(!correcting_straight())
		{
			lidar_reset_flags();
			cur_move.state++;
		}
		break;

		case MOVE_LIDAR_SYNC_2:
		if(lidar_is_complete())
		{
			lidar_reset_flags();
			cur_move.state++;
		}
		break;

		case MOVE_LIDAR_STORE_2A:
		if(lidar_is_half())
		{
			COPY_POS(cur_move.lidars[2], cur_pos);
			copy_lidar_half1(cur_move.lidars[2].scan);
			cur_move.state++;
		}
		break;

		case MOVE_LIDAR_STORE_2B:
		if(lidar_is_complete())
		{
			straight_rel(cur_move.rel_fwd);
			copy_lidar_half2(cur_move.lidars[2].scan);
			cur_move.state++;
		}
		break;


		default:
		break;
	}
}


void move_rel_twostep(int angle, int fwd /*in mm*/)
{
	aim_angle += angle<<16;

	speed_limit_lowered = 0;
	wheel_integrals[0] = 0;
	wheel_integrals[1] = 0;
	fwd_speed_limit = fwd_accel*200; // use starting speed that equals to 20ms of acceleration
	aim_fwd = fwd*10; // in 0.1mm

	ang_top_speed = 220000; // 150000
	manual_control = 0;

	fwd_top_speed = 600000;

	robot_moves();
}

void stop_navig_fsms()
{

}
