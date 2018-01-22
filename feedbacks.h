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
void host_dead();
void rotate_rel(int angle);
void rotate_abs(int angle);
void straight_rel(int fwd /*in mm*/);
int correcting_angle();
int angle_almost_corrected();
int get_ang_err();
int correcting_straight();
int correcting_either();
int robot_moving();
void take_control();
void reset_movement();
void speed_limit(int new_status);

void set_top_speed_ang(int speed);
void set_top_speed_ang_max(int speed); // Can only lower the existing limits

void set_top_speed_fwd(int speed);
void set_top_speed_fwd_max(int speed); // Can only lower the existing limits

// Sets both angular and forward:
void set_top_speed(int speed);
void set_top_speed_max(int speed); // Can only lower the existing limits

void reset_speed_limits();

void enable_collision_detection();

void change_angle_to_cur();


int get_fwd();

#define ANG_180_DEG 2147483648UL
#define ANG_90_DEG  1073741824
#define ANG_2_5_DEG   29826162
#define ANG_1_DEG     11930465
#define ANG_0_5_DEG    5965232
#define ANG_0_25_DEG   2982616
#define ANG_0_125_DEG  1491308
#define ANG_0_1_DEG    1193047
#define ANG_0_05_DEG    596523
#define ANG_0_01_DEG    119305
#define ANG_0_001_DEG    11930

#define ANG_1PER16_DEG  745654  // cumulated full circle rounding error: 0.000006%


#define XCEL_X_NEG_WARN ((int)(-20000)*256)
#define XCEL_X_POS_WARN ((int)(20000)*256)
#define XCEL_Y_NEG_WARN ((int)(-16000)*256)
#define XCEL_Y_POS_WARN ((int)(16000)*256)

#define XCEL_X_NEG_COLL ((int)(-27000)*256)
#define XCEL_X_POS_COLL ((int)(27000)*256)
#define XCEL_Y_NEG_COLL ((int)(-22000)*256)
#define XCEL_Y_POS_COLL ((int)(22000)*256)


typedef struct
{
	int32_t ang; // int32_t range --> -180..+180 deg; let it overflow freely. 1 unit = 83.81903171539 ndeg
	int32_t x;   // mm
	int32_t y;   // mm
} pos_t;

#define COPY_POS(to, from) { (to).ang = (from).ang; (to).x = (from).x; (to).y = (from).y; }

extern volatile pos_t cur_pos;

void zero_angle();
void zero_coords();

void allow_angular(int yes);
void allow_straight(int yes);
void auto_disallow(int yes);

void correct_location_without_moving(pos_t corr);
void correct_location_without_moving_external(pos_t corr);
void set_location_without_moving_external(pos_t new_pos);

void change_angle_abs(int angle);
void change_angle_rel(int angle);
void change_straight_rel(int fwd /*in mm*/);

void dbg_teleportation_bug();

typedef struct __attribute__((packed))
{
	int32_t id;
	int32_t prev_id;
	int32_t prev2_id;
	int32_t prev3_id;
	int32_t prev4_id;
	int64_t prev_x;
	int64_t prev_y;
	int64_t cur_x;
	int64_t cur_y;
} dbg_teleportation_bug_data_t;

typedef struct __attribute__((packed))
{
	int32_t wd0;
	int32_t wd1;
	int32_t movement;
	int32_t x_idx;
	int32_t y_idx;
	int64_t dx;
	int64_t dy;
	int64_t x_before;
	int64_t y_before;
	int64_t x_after;
	int64_t y_after;

} dbg_teleportation_extra_t;

extern volatile dbg_teleportation_extra_t dbg_teleportation_extra;


extern volatile int dbg_teleportation_bug_report;
extern volatile dbg_teleportation_bug_data_t dbg_teleportation_bug_data;

// Temporarily here, relayed to motor controllers, for adjusting PID loops.
extern volatile uint8_t mc_pid_imax;
extern volatile uint8_t mc_pid_feedfwd;
extern volatile uint8_t mc_pid_p;
extern volatile uint8_t mc_pid_i;
extern volatile uint8_t mc_pid_d;


#endif
