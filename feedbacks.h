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

extern int cur_angle;
extern int64_t cur_x;
extern int64_t cur_y;

void zero_angle();
void zero_coords();

void allow_angular(int yes);
void allow_straight(int yes);


#endif
