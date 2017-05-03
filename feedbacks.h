#ifndef _FEEDBACKS_H
#define _FEEDBACKS_H

#define GYRO_LONG_INTEGRAL_IGNORE_LEVEL 0

#define MAX_DIFFERENTIAL_SPEED 3000
#define MAX_SPEED 6000

void run_feedbacks(int sens_status);
void move_rel_twostep(int angle, int fwd);
void move_arc_manual(int comm, int ang);
void compass_fsm(int cmd);
void sync_to_compass();




#endif
