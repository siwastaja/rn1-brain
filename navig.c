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
	MOVE_ROTATE		= 1, // Rotate to final angle
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
	move_state_t state;
	int abs_angle;
	int rel_fwd;
	int16_t lidar_before[360];
	int angle_before;
	int64_t x_before;
	int64_t y_before;
	int16_t lidar_after[360];
	int angle_after;
	int64_t x_after;
	int64_t y_after;
} move_t;

static move_t cur_move;

void move_fsm()
{
	switch(cur_move.state)
	{
		case MOVE_IDLE:
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
			/* Half of the lidar data is there, we can process and copy it, there is
			   no risk of it being overwritten, since the LIDAR is writing the second half right now. 
			   Since the robot is not moving, it shouldn't matter whether we sample cur_angle, cur_x, cur_y
			   at the start, middle or end of the lidar scan, but to be on the safe side, we do it in the middle.
			*/
			cur_move.angle_before = cur_angle;
			cur_move.x_before = cur_x;
			cur_move.y_berofe = cur_y;
			copy_lidar_half1(cur_move.lidar_before);
			cur_move.state++;
		}
		break;

		case MOVE_LIDAR_STORE_1B:
		if(lidar_is_complete())
		{
			straight_rel(cur_move.rel_fwd);
			copy_lidar_half2(cur_move.lidar_before);
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
			cur_move.angle_after = cur_angle;
			cur_move.x_after = cur_x;
			cur_move.y_after = cur_y;
			copy_lidar_half1(cur_move.lidar_after);
			cur_move.state++;
		}
		break;

		case MOVE_LIDAR_STORE_2B:
		if(lidar_is_complete())
		{
			copy_lidar_half2(cur_move.lidar_after);
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
