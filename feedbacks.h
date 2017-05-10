#ifndef _FEEDBACKS_H
#define _FEEDBACKS_H

#define GYRO_LONG_INTEGRAL_IGNORE_LEVEL 0

#define MAX_DIFFERENTIAL_SPEED 3000
#define MAX_SPEED 6000

void run_feedbacks(int sens_status);
void move_arc_manual(int comm, int ang);
void compass_fsm(int cmd);
void sync_to_compass();
void host_alive();
void rotate_rel(int angle);
void rotate_abs(int angle);
void straight_rel(int fwd /*in mm*/);
int correcting_angle();
int correcting_straight();
int robot_moving();

typedef struct
{
	int32_t ang; // int32_t range --> -180..+180 deg; let it overflow freely. 1 unit = 83.81903171539 ndeg
	int32_t x;   // mm
	int32_t y;   // mm
} pos_t;

#define COPY_POS(to, from) { (to).ang = (from).ang; (to).x = (from).x; (to).y = (from).y; }



#endif
