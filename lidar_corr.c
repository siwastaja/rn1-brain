/*
	Lidar-based error correction
	Tries to remove any (ang,x,y) error between two adjacent robot positions with two lidar scans.

	Inputs:
	lidar_scan_t *before, lidar_scan_t *after, both include the assumed (ang,x,y) coordinates.

	Outputs:
	(ang,x,y) corrections
	
*/

#include <stdint.h>
#include <string.h>
#include "lidar_corr.h"
#include "sin_lut.h"
#include "comm.h"
#include "uart.h"

extern volatile int dbg[10];
extern volatile int dbg_error_num;

int latest_corr_ret;
pos_t latest_corr;


#define LIDAR_RANGE 5000
#define sq(x) ((x)*(x))


#define PASS1_NUM_A 7
static int PASS1_A[PASS1_NUM_A] =
{
	-3*ANG_1_DEG,
	-2*ANG_1_DEG,
	-1*ANG_1_DEG,
	0,
	1*ANG_1_DEG,
	2*ANG_1_DEG,
	3*ANG_1_DEG
};
static int PASS1_A_WEIGH[PASS1_NUM_A] =
{
	11,
	13,
	15,
	16,
	15,
	13,
	11
};

#define PASS1_NUM_X 13
static int PASS1_X[PASS1_NUM_X] =
{
	-190
	-150,
	-120,
	-90,
	-60,
	-30,
	0,
	30,
	60,
	90,
	120,
	150,
	190
};
static int PASS1_X_WEIGH[PASS1_NUM_X] =
{
	10,
	13,
	15,
	17,
	18,
	18,
	18,
	18,
	18,
	17,
	15,
	13,
	10
};


#define PASS2_NUM_A 5
static int PASS2_A[PASS2_NUM_A] =
{
	-2*ANG_0_5_DEG,
	-1*ANG_0_5_DEG,
	0,
	1*ANG_0_5_DEG,
	2*ANG_0_5_DEG
};

#define PASS2_NUM_X 9
static int PASS2_X[PASS2_NUM_X] =
{
	-40,
	-30,
	-20,
	-10,
	0,
	10,
	20,
	30,
	40
};


#define PASS3_NUM_A 5
static int PASS3_A[PASS3_NUM_A] =
{
	-2*ANG_0_25_DEG,
	-1*ANG_0_25_DEG,
	0,
	1*ANG_0_25_DEG,
	2*ANG_0_25_DEG
};

#define PASS3_NUM_X 7
static int PASS3_X[PASS3_NUM_X] =
{
	-9,
	-6,
	-3,
	0,
	3,
	6,
	9
};


// Use the same tables for X & Y
#define PASS1_NUM_Y PASS1_NUM_X
#define PASS1_Y PASS1_X
#define PASS1_Y_WEIGH PASS1_X_WEIGH
#define PASS2_NUM_Y PASS2_NUM_X
#define PASS2_Y PASS2_X
#define PASS3_NUM_Y PASS3_NUM_X
#define PASS3_Y PASS3_X



point_t img1[256];
point_t img2[256];

// For optimization purposes:
uint8_t o_starts[256];
uint8_t o_ranges[256];


// Zeroes out and marks valid points.
static void scan_to_2d_pre(lidar_scan_t* in, point_t* out)
{
	for(int i = 0; i < 256; i++)
	{
		int in_idx = (i*360)>>8;
		if(in->scan[in_idx] != 0)
			out[i].valid = 1;
		else
			out[i].valid = 0;
	}
}

static void scan_to_2d(lidar_scan_t* in, point_t* out)
{
	uint32_t angle = (int32_t)in->pos.ang;

	for(int i = 0; i < 256; i++)
	{
		int32_t y_idx = (angle)>>SIN_LUT_SHIFT;
		int32_t x_idx = (1073741824-angle)>>SIN_LUT_SHIFT;
		int in_idx = (i*360)>>8;
		out[i].x = in->pos.x + (((int32_t)sin_lut[x_idx] * (int32_t)in->scan[in_idx])>>15);
		out[i].y = in->pos.y + (((int32_t)sin_lut[y_idx] * (int32_t)in->scan[in_idx])>>15);
		angle += 16777216;
	}
}

// For lidar scans taken under robot movement, with 360 scan points, and 90 position points.
// out should already contain valid fields set.
static void scan_to_2d_live(live_lidar_scan_t* in, point_t* out, int32_t corr_a, int32_t corr_x, int32_t corr_y)
{
	for(int i = 0; i < 360; i++)
	{
		int pos_idx = i>>2;
		uint32_t angle = (uint32_t)in->pos[pos_idx].ang + (uint32_t)i*(uint32_t)ANG_1_DEG + (uint32_t)corr_a;
		int y_idx = (angle)>>SIN_LUT_SHIFT;
		int x_idx = (1073741824-angle)>>SIN_LUT_SHIFT;
		out[i].x = corr_x + in->pos[pos_idx].x + (((int32_t)sin_lut[x_idx] * (int32_t)in->scan[i])>>15);
		out[i].y = corr_y + in->pos[pos_idx].y + (((int32_t)sin_lut[y_idx] * (int32_t)in->scan[i])>>15);
	}
}

/*
 Lidar-based 2D MAP on uart:

num_bytes
 1	uint8 start byte
 1	uint7 status
 2	int14 cur_ang (at the middle point of the lidar scan)  (not used for turning the image, just to include robot coords)
 5	int32 cur_x   ( " " )
 5	int32 cur_y   ( " " )
 1	int7  correction return value
 2	int14 ang_corr (for information only)
 2	int14 x_corr (for information only)
 2	int14 y_corr (for information only) 
1440	360 * point
	  2	int14  x referenced to cur_x
	  2	int14  y referenced to cur_y

	Total: 1461
	Time to tx at 115200: ~130 ms

*/

static void send_2d_live_to_uart(live_lidar_scan_t* in, point_t* point2d /*for valid fields*/, int significant_for_mapping)
{
	uint8_t* buf = txbuf;

	int a_mid = in->pos[45].ang>>16;
	int x_mid = in->pos[45].x;
	int y_mid = in->pos[45].y;

	*(buf++) = 0x84;
	*(buf++) = significant_for_mapping;

	*(buf++) = I16_MS(a_mid);
	*(buf++) = I16_LS(a_mid);
	*(buf++) = I32_I7_4(x_mid);
	*(buf++) = I32_I7_3(x_mid);
	*(buf++) = I32_I7_2(x_mid);
	*(buf++) = I32_I7_1(x_mid);
	*(buf++) = I32_I7_0(x_mid);
	*(buf++) = I32_I7_4(y_mid);
	*(buf++) = I32_I7_3(y_mid);
	*(buf++) = I32_I7_2(y_mid);
	*(buf++) = I32_I7_1(y_mid);
	*(buf++) = I32_I7_0(y_mid);

	*(buf++) = latest_corr_ret;
	int tmp = latest_corr.ang>>16;
	*(buf++) = I16_MS(tmp);
	*(buf++) = I16_LS(tmp);
	tmp = latest_corr.x<<2;
	*(buf++) = I16_MS(tmp);
	*(buf++) = I16_LS(tmp);
	tmp = latest_corr.y<<2;
	*(buf++) = I16_MS(tmp);
	*(buf++) = I16_LS(tmp);


	for(int i = 0; i < 360; i++)
	{
		if(point2d[i].valid)
		{
			int pos_idx = i>>2;
			uint32_t angle = (uint32_t)in->pos[pos_idx].ang + (uint32_t)i*(uint32_t)ANG_1_DEG;
			int y_idx = (angle)>>SIN_LUT_SHIFT;
			int x_idx = (1073741824-angle)>>SIN_LUT_SHIFT;
			int x = in->pos[pos_idx].x + (((int32_t)sin_lut[x_idx] * (int32_t)in->scan[i])>>15)  - x_mid;
			int y = in->pos[pos_idx].y + (((int32_t)sin_lut[y_idx] * (int32_t)in->scan[i])>>15)  - y_mid;

			*(buf++) = I16_MS(x<<2);
			*(buf++) = I16_LS(x<<2);
			*(buf++) = I16_MS(y<<2);
			*(buf++) = I16_LS(y<<2);
		}
		else
		{
			*(buf++) = 0;
			*(buf++) = 0;
			*(buf++) = 0;
			*(buf++) = 0;
		}
	}

	send_uart(1460);
}


static int scan_num_points(point_t* img)
{
	int n = 0;
	for(int i = 0; i < 256; i++)
	{
		if(img[i].valid) n++;
	}
	return n;
}

/*

*/
void pre_search(point_t* img1, point_t* img2)
{
	int num_opers = 0;
	int num_img1_masked = 0;
	for(int i = 0; i < 256; i++)
	{
		if(!img1[i].valid) continue;

		int o_smallest = 1000;
		int o_biggest = -1000;

		for(int o = 0; o < 256; o++)
		{
			if(!img2[o].valid) continue;
			int dx = img2[o].x - img1[i].x;
			int dy = img2[o].y - img1[i].y;
			int dist = sq(dx) + sq(dy);

			/*
				We want to log minimum and maximum o index so that we can only scan the relevant possible area.
			*/
			if(dist < 400*400)
			{
				int o_aligned = (o<128)?o:o-256; // We want to have negative numbers from the high range, so that looking for min/max works.
				if(o_aligned < o_smallest) o_smallest = o_aligned;
				if(o_aligned > o_biggest) o_biggest = o_aligned;
			}
		}

		/*
			img1 is always the same; img2 is adjusted.
			If, without the adjustments, the nearest img2 point is very far away, it
			will be far away even with any adjustments, and thus, does not contribute.
			Masking it away makes the calculation significantly quicker.
		*/
		if(o_smallest == 1000) // not found
		{
			num_img1_masked++;
			img1[i].valid = 0;
		}
		else
		{
			o_smallest -= 2;
			int o_range = o_biggest-o_smallest+2;
			if(o_smallest < 0) o_smallest+=256;
			o_starts[i] = o_smallest;
			o_ranges[i] = o_range;
			num_opers += o_range;
		}

	}
//	dbg[8] = num_img1_masked;
}

// returns 256..14656, bigger = better
int32_t calc_match_lvl(point_t* img1, point_t* img2)
{
	/*
	For each point in the first image, search the nearest point in the second image; any valid point will do.

	Use 1/x function to scale the distance so that small distances are more meaningfull than large distances;
	this is to ignore objects that are really different between the images, trying to account for objects common
	for both images.

	Sum up the distances.

	Finally, divide by the number of points to get comparable value regardless of num of valid points.
	*/

	int32_t dist_sum = 0;
	for(int i = 0; i < 256; i++)
	{
		if(!img1[i].valid) continue;

		int smallest = 1000*1000;
		uint8_t odx = o_starts[i];
		int range = o_ranges[i];

		for(int o = 0; o < range; o++)
		{
			odx++;
			if(!img2[odx].valid) continue;
			int dx = img2[odx].x - img1[i].x;
			int dy = img2[odx].y - img1[i].y;
			int dist = sq(dx) + sq(dy);
			if(dist < smallest)
			{
				smallest = dist;
			}
		}

		// Non-optimized code going through all points:
/*
		for(int o = 0; o < 256; o++)
		{
			if(!img2[o].valid) continue;
			int dx = img2[o].x - img1[i].x;
			int dy = img2[o].y - img1[i].y;
			int dist = sq(dx) + sq(dy);
			if(dist < smallest)
			{
				smallest = dist;
			}
		}
*/

		// Divider offset: (to avoid division by zero and numbers too huge)
		// 50 breaks the results down
		// 100 seems to be nearly identical to 200, which is fine
		// 400 is again nearly identical to 200. More careful in the most difficult case (undercorrects instead of overcorrection)
		// 800 is like again like 200 and 400, but shows very slight improvement in one image of the 20.
		// 1600 gets the difficult one most right so far! This seems the best.
		// 3200 shows very, very small degradation in trivial cases - slight undercorrection. Still very good.
		// 6400: about the same.

		/*
			dist	dist_scaled
			0mm	14656
			1mm	14646
			2mm	14691
			10mm	13793
			100mm	2021
			300mm	256
		*/

		int32_t dist_scaled = (256*(400*400+1200))/(smallest+1200);
		dist_sum += dist_scaled;
	}

	return dist_sum>>8;
}



/*
	Specifically optimized version for live scans (scans 200ms apart, so smaller differences, but 360 points)

	in 200ms, robot can:
	* (turn 18 degrees at 4 sec/360deg., which is irrelevant since we know how much we have turned.)
	* go 33 cm at 6 km/h (1.67 m/s)

	So, I decided looking at points for +/- 10 deg is ok when slow robot speed is assumed (based on wheels/gyro)
	When higher speed is detected, PASS4 is skipped to give more time for slightly less final resolution, and
	+/- 13 deg search range is used instead.
*/

int angle_optim; // must be smaller than SEARCH_RANGE

#define SEARCH_RANGE 10
#define SEARCH_RANGE_HI 13

// 100 causes severe jumping
// 200 causes some jumping every now and then
// 400 works well.
#define MATCH_DIV_OFFSET 800
#define MATCH_DIV_OFFSET_HI 800

int32_t calc_match_lvl_live(point_t* img1, point_t* img2)
{
	int32_t dist_sum = 0;
	for(int i = 0; i < 360; i++)
	{
		if(!img1[i].valid) continue;
		// GCC didn't figure out this trivial optimization.
		// Only load img1[i].x and .y from memory when the i has changed.
		register int i1x = img1[i].x;
		register int i1y = img1[i].y;

		int smallest = 500*500;
		int o = i-SEARCH_RANGE+angle_optim;
		if(o < 0) o+=360;
		int o_end = o+2*SEARCH_RANGE;

		// let o run [i-SEARCH_RANGE..i+SEARCH_RANGE], with wrapping from 359->0.
		// Two separate loops prevent wrapping condition on each inner loop.
		if(o_end > 360)
		{
			o_end -= 360;
			for(; o < 360; o++)
			{
				if(!img2[o].valid) continue;
				int dx = img2[o].x - i1x;
				int dy = img2[o].y - i1y;
				int dist = sq(dx) + sq(dy);
				if(dist < smallest) smallest = dist;
			}
			o = 0;
		}

		for(; o < o_end; o++)
		{
			if(!img2[o].valid) continue;
			int dx = img2[o].x - i1x;
			int dy = img2[o].y - i1y;
			int dist = sq(dx) + sq(dy);
			if(dist < smallest) smallest = dist;
		}


		int32_t dist_scaled = (256*(200*200+MATCH_DIV_OFFSET))/(smallest+MATCH_DIV_OFFSET);
		dist_sum += dist_scaled;
	}

	return dist_sum>>8;
}


int32_t calc_match_lvl_live_high_movement(point_t* img1, point_t* img2)
{
	int32_t dist_sum = 0;
	for(int i = 0; i < 360; i++)
	{
		if(!img1[i].valid) continue;
		// GCC didn't figure out this trivial optimization.
		// Only load img1[i].x and .y from memory when the i has changed.
		register int i1x = img1[i].x;
		register int i1y = img1[i].y;

		int smallest = 500*500;
		int o = i-SEARCH_RANGE_HI+angle_optim;
		if(o < 0) o+=360;
		int o_end = o+2*SEARCH_RANGE_HI;

		// let o run [i-SEARCH_RANGE..i+SEARCH_RANGE], with wrapping from 359->0.
		// Two separate loops prevent wrapping condition on each inner loop.
		if(o_end > 360)
		{
			o_end -= 360;
			for(; o < 360; o++)
			{
				if(!img2[o].valid) continue;
				int dx = img2[o].x - i1x;
				int dy = img2[o].y - i1y;
				int dist = sq(dx) + sq(dy);
				if(dist < smallest) smallest = dist;
			}
			o = 0;
		}

		for(; o < o_end; o++)
		{
			if(!img2[o].valid) continue;
			int dx = img2[o].x - i1x;
			int dy = img2[o].y - i1y;
			int dist = sq(dx) + sq(dy);
			if(dist < smallest) smallest = dist;
		}


		int32_t dist_scaled = (256*(200*200+MATCH_DIV_OFFSET_HI))/(smallest+MATCH_DIV_OFFSET_HI);
		dist_sum += dist_scaled;
	}

	return dist_sum>>8;
}



/*

	scan1, scan2: before and after lidar scans, with pos fields set as correctly as possible
	corr: pointer to pos_t; corrections are written there. You can apply them. Will be written zero in case of failure.

	Return:
	0 on success
	>0 on failure

*/

extern void dev_send_hommel(lidar_scan_t* p1, lidar_scan_t* p2, int bonus);
extern void dev_send_jutsk(point_t* img, int id);

extern void delay_ms(uint32_t i);


int do_lidar_corr(lidar_scan_t* scan1, lidar_scan_t* scan2, pos_t* corr)
{
	// scan1 stays the same. scan2 goes through pos modifications and scan_to_2d is called again every time.

	/*
	Step 1:
	Convert lidar scans to absolute x,y coordinates on the same map, assuming that the original
	coordinates are right.
	*/

	corr->ang = 0;
	corr->x = 0;
	corr->y = 0;

	scan_to_2d_pre(scan1, img1);
	scan_to_2d    (scan1, img1);

	scan_to_2d_pre(scan2, img2);
	scan_to_2d    (scan2, img2);

	/*
	Step 2:
	Process both lidar scans to remove any points that are out of visible range seen from the other scan.
	LIDAR_RANGE must be slightly smaller than in real life, since we are assuming zero error in robot coordinates.
	*/

	for(int i = 0; i < 256; i++)
	{
		int32_t dist_img1_to_scan2_origin = sq(img1[i].x - scan2->pos.x) + sq(img1[i].y - scan2->pos.y);
		int32_t dist_img2_to_scan1_origin = sq(img2[i].x - scan1->pos.x) + sq(img2[i].y - scan2->pos.y);

		if(dist_img1_to_scan2_origin > sq(LIDAR_RANGE))
			img1[i].valid = 0;
		if(dist_img2_to_scan1_origin > sq(LIDAR_RANGE))
			img2[i].valid = 0;
	}

	/*
	Step 3:
	For optimization, run one full "slow" image matching round, generating optimization tables.
	*/
	pre_search(img1, img2);

	int points1 = scan_num_points(img1);
	int points2 = scan_num_points(img2);

	//dbg[6] = points1;
	//dbg[7] = points2;

	if(points1 < 40 || points2 < 40)
	{
		return 1;
	}
	
	// scan2 position is modified during processing. Instead of memcpying the whole scan2, we just store and restore the pos.
	pos_t scan2_very_orig_pos;
	pos_t orig_pos;
	COPY_POS(scan2_very_orig_pos, scan2->pos);
	COPY_POS(orig_pos, scan2->pos);

	// Run PASS 1

	int biggest_lvl = 0;
	int best_a, best_x, best_y;
	for(int a_corr = 0; a_corr < PASS1_NUM_A; a_corr++)
	{
		scan2->pos.ang = orig_pos.ang + PASS1_A[a_corr];
		for(int x_corr = 0; x_corr < PASS1_NUM_X; x_corr++)
		{
			scan2->pos.x = orig_pos.x + PASS1_X[x_corr];
			for(int y_corr = 0; y_corr < PASS1_NUM_Y; y_corr++)
			{
				scan2->pos.y = orig_pos.y + PASS1_Y[y_corr];

				scan_to_2d(scan2, img2);

//				dev_send_jutsk(img1, 0);
//				dev_send_jutsk(img2, 1);

				int lvl = calc_match_lvl(img1, img2);
				lvl = lvl * PASS1_A_WEIGH[a_corr] * PASS1_X_WEIGH[x_corr] * PASS1_Y_WEIGH[y_corr];
//				dev_send_hommel(scan1, scan2, lvl);

				if(lvl > biggest_lvl)
				{
					biggest_lvl = lvl;
					best_a = a_corr;
					best_x = x_corr;
					best_y = y_corr;
				}
			}
		}
	}

	if(biggest_lvl == 0)
	{
		COPY_POS(scan2->pos, scan2_very_orig_pos);
		return 2;
	}

	// Correct orig_pos to the best match.
	orig_pos.ang += PASS1_A[best_a];
	orig_pos.x   += PASS1_X[best_x];
	orig_pos.y   += PASS1_Y[best_y];
	corr->ang    += PASS1_A[best_a];
	corr->x      += PASS1_X[best_x];
	corr->y      += PASS1_Y[best_y];

//	COPY_POS(scan2->pos, orig_pos);
//	dev_send_hommel(scan1, scan2, biggest_lvl);
//	delay_ms(5000);

	// Run pass 2

	biggest_lvl = 0;

	for(int a_corr = 0; a_corr < PASS2_NUM_A; a_corr++)
	{
		scan2->pos.ang = orig_pos.ang + PASS2_A[a_corr];
		for(int x_corr = 0; x_corr < PASS2_NUM_X; x_corr++)
		{
			scan2->pos.x = orig_pos.x + PASS2_X[x_corr];
			for(int y_corr = 0; y_corr < PASS2_NUM_Y; y_corr++)
			{
				scan2->pos.y = orig_pos.y + PASS2_Y[y_corr];

				scan_to_2d(scan2, img2);

//				dev_send_jutsk(img1, 0);
//				dev_send_jutsk(img2, 1);

				int lvl = calc_match_lvl(img1, img2);
//				dev_send_hommel(scan1, scan2, lvl);

				if(lvl > biggest_lvl)
				{
					biggest_lvl = lvl;
					best_a = a_corr;
					best_x = x_corr;
					best_y = y_corr;
				}
			}
		}
	}


	if(biggest_lvl == 0)
	{
		COPY_POS(scan2->pos, scan2_very_orig_pos);
		return 3;
	}

	// Correct orig_pos to the best match.
	orig_pos.ang += PASS2_A[best_a];
	orig_pos.x   += PASS2_X[best_x];
	orig_pos.y   += PASS2_Y[best_y];
	corr->ang    += PASS2_A[best_a];
	corr->x      += PASS2_X[best_x];
	corr->y      += PASS2_Y[best_y];

//	COPY_POS(scan2->pos, orig_pos);
//	dev_send_hommel(scan1, scan2, biggest_lvl);
//	delay_ms(5000);
	

	// Run pass 3

	biggest_lvl = 0;
	for(int a_corr = 0; a_corr < PASS3_NUM_A; a_corr++)
	{
		scan2->pos.ang = orig_pos.ang + PASS3_A[a_corr];
		for(int x_corr = 0; x_corr < PASS3_NUM_X; x_corr++)
		{
			scan2->pos.x = orig_pos.x + PASS3_X[x_corr];
			for(int y_corr = 0; y_corr < PASS3_NUM_Y; y_corr++)
			{
				scan2->pos.y = orig_pos.y + PASS3_Y[y_corr];

				scan_to_2d(scan2, img2);

//				dev_send_jutsk(img1, 0);
//				dev_send_jutsk(img2, 1);

				int lvl = calc_match_lvl(img1, img2);
//				dev_send_hommel(scan1, scan2, lvl);

				if(lvl > biggest_lvl)
				{
					biggest_lvl = lvl;
					best_a = a_corr;
					best_x = x_corr;
					best_y = y_corr;
				}
			}
		}
	}

	if(biggest_lvl == 0)
	{
		COPY_POS(scan2->pos, scan2_very_orig_pos);
		return 4;
	}

	// Correct orig_pos to the best match.
	orig_pos.ang += PASS3_A[best_a];
	orig_pos.x   += PASS3_X[best_x];
	orig_pos.y   += PASS3_Y[best_y];
	corr->ang    += PASS3_A[best_a];
	corr->x      += PASS3_X[best_x];
	corr->y      += PASS3_Y[best_y];

//	COPY_POS(scan2->pos, orig_pos);
//	dev_send_hommel(scan1, scan2, biggest_lvl);
//	delay_ms(5000);

	COPY_POS(scan2->pos, scan2_very_orig_pos);
	return 0;
}


/*

State                :    0   1   2   0   1   2   0   1   2
Lidar store          :   111 222 333 111 222 333 111 222 333
Process img1 (before):           111 222 333 111 222 333 111
Process img2 (after) :           222 333 111 222 333 111 222
Apply correction to  :              2   3   1   2   3   1   2
Apply correction to  :              3   1   2   3   1   2   3
Apply corr to cur_pos:              x   x   x   x   x   x   x

*/

void apply_corr_to_livelidar(live_lidar_scan_t* lid)
{
	for(int i = 0; i < 90; i++)
	{
		lid->pos[i].ang += latest_corr.ang;
		lid->pos[i].x   += latest_corr.x;
		lid->pos[i].y   += latest_corr.y;
	}
}

live_lidar_scan_t lidlive1, lidlive2, lidlive3;
point_t l2dlive1[360], l2dlive2[360], l2dlive3[360];

live_lidar_scan_t* p_livelidar_store;
point_t*           p_livelid2d_store;

live_lidar_scan_t* p_livelidar_img1;
point_t*           p_livelid2d_img1;

live_lidar_scan_t* p_livelidar_img2;
point_t*           p_livelid2d_img2;

live_lidar_scan_t lid_skiphold;
point_t           l2d_skiphold[360];
int               num_samples_skiphold[6];

// For counting validness of data for lidar-based correction.
int num_samples1[6];
int num_samples2[6];
int num_samples3[6];

int* p_livelidar_num_samples_store;
int* p_livelidar_num_samples_img1;
int* p_livelidar_num_samples_img2;

int live_lidar_calc_req;
int skip;

volatile int calc_must_be_finished = 0;

void livelidar_storage_finished()
{
	static int state = 0;

	if(state == 0)
	{
		for(int i=0; i<6; i++) num_samples2[i] = 0;
		p_livelidar_store = &lidlive2;
		p_livelid2d_store = l2dlive2;
		p_livelidar_num_samples_store = num_samples2;
		p_livelidar_img1  = &lidlive3;
		p_livelid2d_img1  = l2dlive3;
		p_livelidar_num_samples_img1 = num_samples3;
		p_livelidar_img2  = &lidlive1;
		p_livelid2d_img2  = l2dlive1;
		p_livelidar_num_samples_img2 = num_samples1;
		state = 1;
	}
	else if(state == 1)
	{
		for(int i=0; i<6; i++) num_samples3[i] = 0;
		p_livelidar_store = &lidlive3;
		p_livelid2d_store = l2dlive3;
		p_livelidar_num_samples_store = num_samples3;
		p_livelidar_img1  = &lidlive1;
		p_livelid2d_img1  = l2dlive1;
		p_livelidar_num_samples_img1 = num_samples1;
		p_livelidar_img2  = &lidlive2;
		p_livelid2d_img2  = l2dlive2;
		p_livelidar_num_samples_img2 = num_samples2;
		state = 2;
	}
	else // state == 2
	{
		for(int i=0; i<6; i++) num_samples1[i] = 0;
		p_livelidar_store = &lidlive1;
		p_livelid2d_store = l2dlive1;
		p_livelidar_num_samples_store = num_samples1;
		p_livelidar_img1  = &lidlive2;
		p_livelid2d_img1  = l2dlive2;
		p_livelidar_num_samples_img1 = num_samples2;
		p_livelidar_img2  = &lidlive3;
		p_livelid2d_img2  = l2dlive3;
		p_livelidar_num_samples_img2 = num_samples3;
		state = 0;
	}

	if(skip)
	{
		p_livelidar_img1 = &lid_skiphold;
		p_livelid2d_img1 = l2d_skiphold;
		p_livelidar_num_samples_img1 = num_samples_skiphold;
	}

	calc_must_be_finished = 0;
	live_lidar_calc_req = 1;
}

int livelidar_skip()
{
	return skip;
}

#define LIVE_PASS1_NUM_A 3
static int LIVE_PASS1_A[LIVE_PASS1_NUM_A] =
{
	-1*ANG_1_DEG,
	0,
	1*ANG_1_DEG,
};
static int LIVE_PASS1_A_WEIGH[LIVE_PASS1_NUM_A] =
{
	1,
	1,
	1
};

#define LIVE_PASS1_NUM_X 5
static int LIVE_PASS1_X[LIVE_PASS1_NUM_X] =
{
	-60,
	-30,
	0,
	30,
	60
};
static int LIVE_PASS1_X_WEIGH[LIVE_PASS1_NUM_X] =
{
	5,
	6,
	6,
	6,
	5
};


#define LIVE_PASS2_NUM_A 5
static int LIVE_PASS2_A[LIVE_PASS2_NUM_A] =
{
	-2*ANG_0_5_DEG,
	-1*ANG_0_5_DEG,
	0,
	1*ANG_0_5_DEG,
	2*ANG_0_5_DEG
};

#define LIVE_PASS2_NUM_X 5
static int LIVE_PASS2_X[LIVE_PASS2_NUM_X] =
{
	-20,
	-10,
	0,
	10,
	20,
};


#define LIVE_PASS3_NUM_A 5
static int LIVE_PASS3_A[LIVE_PASS3_NUM_A] =
{
	-2*ANG_0_25_DEG,
	-1*ANG_0_25_DEG,
	0,
	1*ANG_0_25_DEG,
	2*ANG_0_25_DEG
};

#define LIVE_PASS3_NUM_X 5
static int LIVE_PASS3_X[LIVE_PASS3_NUM_X] =
{
	-10,
	-5,
	0,
	5,
	10
};

// Pass4 goes X,Y,A instead of (X,Y),A

#define LIVE_PASS4_NUM_A 9
static int LIVE_PASS4_A[LIVE_PASS4_NUM_A] =
{
	-4*ANG_0_05_DEG,
	-3*ANG_0_05_DEG,
	-2*ANG_0_05_DEG,
	-1*ANG_0_05_DEG,
	0,
	1*ANG_0_05_DEG,
	2*ANG_0_05_DEG,
	3*ANG_0_05_DEG,
	4*ANG_0_05_DEG
};

#define LIVE_PASS4_NUM_X 5
static int LIVE_PASS4_X[LIVE_PASS4_NUM_X] =
{
	-4,
	-2,
	0,
	2,
	4,
};


// Use the same tables for X & Y
#define LIVE_PASS1_NUM_Y LIVE_PASS1_NUM_X
#define LIVE_PASS1_Y LIVE_PASS1_X
#define LIVE_PASS1_Y_WEIGH LIVE_PASS1_X_WEIGH
#define LIVE_PASS2_NUM_Y LIVE_PASS2_NUM_X
#define LIVE_PASS2_Y LIVE_PASS2_X
#define LIVE_PASS3_NUM_Y LIVE_PASS3_NUM_X
#define LIVE_PASS3_Y LIVE_PASS3_X
#define LIVE_PASS4_NUM_Y LIVE_PASS4_NUM_X
#define LIVE_PASS4_Y LIVE_PASS4_X

void live_lidar_calc_must_be_finished()
{
	calc_must_be_finished = 1;
}

extern volatile int us100;

int do_livelidar_corr()
{
	latest_corr.ang = 0;
	latest_corr.x = 0;
	latest_corr.y = 0;
	// Require enough valid samples on at least four of six 60deg segments.
	int valid_segments_img1 = 0, valid_segments_img2 = 0;
	for(int i = 0; i < 6; i++)
	{
		if(p_livelidar_num_samples_img1[i] > 20)
			valid_segments_img1++;
		if(p_livelidar_num_samples_img2[i] > 20)
			valid_segments_img2++;
	}

	if(valid_segments_img1 < 4)
		return 1;

	if(valid_segments_img2 < 4)
		return 2;
	
	int32_t supposed_a_diff = p_livelidar_img2->pos[45].ang - p_livelidar_img1->pos[45].ang;
	int32_t supposed_x_diff = p_livelidar_img2->pos[45].x - p_livelidar_img1->pos[45].x;
	int32_t supposed_y_diff = p_livelidar_img2->pos[45].y - p_livelidar_img1->pos[45].y;


	int high_movement_mode = 0;
	int32_t (*p_calc_f)(point_t*, point_t*) = &calc_match_lvl_live;

	if(supposed_a_diff < -9*ANG_1_DEG || supposed_a_diff > 9*ANG_1_DEG ||
	   supposed_x_diff < -150 || supposed_x_diff > 150 ||
	   supposed_y_diff < -150 || supposed_y_diff > 150)
	{
		high_movement_mode = 1;
		p_calc_f = &calc_match_lvl_live_high_movement;
	}

	angle_optim = supposed_a_diff/-ANG_1_DEG;


	scan_to_2d_live(p_livelidar_img1, p_livelid2d_img1, 0, 0, 0);

	// Run PASS 1

	int biggest_lvl = 0;
	int best1_a, best1_x, best1_y;

	for(int x_corr = 0; x_corr < LIVE_PASS1_NUM_X; x_corr++)
	{
		for(int y_corr = 0; y_corr < LIVE_PASS1_NUM_Y; y_corr++)
		{
			int a = 0;
			int x = LIVE_PASS1_X[x_corr];
			int y = LIVE_PASS1_Y[y_corr];
			scan_to_2d_live(p_livelidar_img2, p_livelid2d_img2, a, x, y);
			int lvl = p_calc_f(p_livelid2d_img1, p_livelid2d_img2);
			lvl = lvl * LIVE_PASS1_X_WEIGH[x_corr] * LIVE_PASS1_Y_WEIGH[y_corr];

			if(lvl > biggest_lvl)
			{
				biggest_lvl = lvl;
				best1_x = x;
				best1_y = y;
			}
		}
		if(calc_must_be_finished) return 20;
	}

	if(biggest_lvl == 0)
	{
		return 3;
	}

	biggest_lvl = 0;

	for(int a_corr = 0; a_corr < LIVE_PASS1_NUM_A; a_corr++)
	{
		int a = LIVE_PASS1_A[a_corr];
		int x = best1_x;
		int y = best1_y;
		scan_to_2d_live(p_livelidar_img2, p_livelid2d_img2, a, x, y);
		int lvl = p_calc_f(p_livelid2d_img1, p_livelid2d_img2);
		lvl = lvl * LIVE_PASS1_A_WEIGH[a_corr];

		if(lvl > biggest_lvl)
		{
			biggest_lvl = lvl;
			best1_a = a;
		}

		if(calc_must_be_finished) return 21;
	}

	if(biggest_lvl == 0)
	{
		return 4;
	}

	// Run pass 2

	biggest_lvl = 0;

	int best2_a, best2_x, best2_y;
	for(int x_corr = 0; x_corr < LIVE_PASS2_NUM_X; x_corr++)
	{
		for(int y_corr = 0; y_corr < LIVE_PASS2_NUM_Y; y_corr++)
		{
			int a = best1_a;
			int x = best1_x + LIVE_PASS2_X[x_corr];
			int y = best1_y + LIVE_PASS2_Y[y_corr];
			scan_to_2d_live(p_livelidar_img2, p_livelid2d_img2, a, x, y);
			int lvl = p_calc_f(p_livelid2d_img1, p_livelid2d_img2);

			if(lvl > biggest_lvl)
			{
				biggest_lvl = lvl;
				best2_x = x;
				best2_y = y;
			}
		}
		if(calc_must_be_finished) return 22;
	}

	if(biggest_lvl == 0)
	{
		return 5;
	}

	biggest_lvl = 0;

	for(int a_corr = 0; a_corr < LIVE_PASS2_NUM_A; a_corr++)
	{
		int a = (uint32_t)best1_a + (uint32_t)LIVE_PASS2_A[a_corr];

		int x = best2_x;
		int y = best2_y;
		scan_to_2d_live(p_livelidar_img2, p_livelid2d_img2, a, x, y);
		int lvl = p_calc_f(p_livelid2d_img1, p_livelid2d_img2);

		if(lvl > biggest_lvl)
		{
			biggest_lvl = lvl;
			best2_a = a;
		}
		if(calc_must_be_finished) return 23;
	}

	if(biggest_lvl == 0)
	{
		return 6;
	}

	// Run pass 3
	biggest_lvl = 0;

	int best3_a, best3_x, best3_y;
	for(int x_corr = 0; x_corr < LIVE_PASS3_NUM_X; x_corr++)
	{
		for(int y_corr = 0; y_corr < LIVE_PASS3_NUM_Y; y_corr++)
		{
			int a = best2_a;
			int x = best2_x + LIVE_PASS3_X[x_corr];
			int y = best2_y + LIVE_PASS3_Y[y_corr];
			scan_to_2d_live(p_livelidar_img2, p_livelid2d_img2, a, x, y);
			int lvl = p_calc_f(p_livelid2d_img1, p_livelid2d_img2);

			if(lvl > biggest_lvl)
			{
				biggest_lvl = lvl;
				best3_x = x;
				best3_y = y;
			}
		}
		if(calc_must_be_finished) return 24;
	}

	if(biggest_lvl == 0)
	{
		return 7;
	}

	biggest_lvl = 0;

	for(int a_corr = 0; a_corr < LIVE_PASS3_NUM_A; a_corr++)
	{
		int a = best2_a + LIVE_PASS3_A[a_corr];
		int x = best3_x;
		int y = best3_y;
		scan_to_2d_live(p_livelidar_img2, p_livelid2d_img2, a, x, y);
		int lvl = p_calc_f(p_livelid2d_img1, p_livelid2d_img2);

		if(lvl > biggest_lvl)
		{
			biggest_lvl = lvl;
			best3_a = a;
		}
		if(calc_must_be_finished) return 25;
	}

	if(biggest_lvl == 0)
	{
		return 8;
	}

	// Run pass 4, which goes x,y,a instead of (x,y),a

	int best4_a, best4_x, best4_y;
	if(!high_movement_mode)
	{

		biggest_lvl = 0;

		for(int y_corr = 0; y_corr < LIVE_PASS4_NUM_Y; y_corr++)
		{
			int a = best3_a;
			int x = best3_x;
			int y = best3_y + LIVE_PASS4_Y[y_corr];
			scan_to_2d_live(p_livelidar_img2, p_livelid2d_img2, a, x, y);
			int lvl = p_calc_f(p_livelid2d_img1, p_livelid2d_img2);

			if(lvl > biggest_lvl)
			{
				biggest_lvl = lvl;
				best4_y = y;
			}
		}
		if(calc_must_be_finished) return 26;

		if(biggest_lvl == 0) return 9;
		biggest_lvl = 0;

		for(int x_corr = 0; x_corr < LIVE_PASS4_NUM_X; x_corr++)
		{
			int a = best3_a;
			int x = best3_x + LIVE_PASS4_X[x_corr];
			int y = best4_y;
			scan_to_2d_live(p_livelidar_img2, p_livelid2d_img2, a, x, y);
			int lvl = p_calc_f(p_livelid2d_img1, p_livelid2d_img2);

			if(lvl > biggest_lvl)
			{
				biggest_lvl = lvl;
				best4_x = x;
			}
		}
		if(calc_must_be_finished) return 27;

		if(biggest_lvl == 0) return 10;
		biggest_lvl = 0;


		for(int a_corr = 0; a_corr < LIVE_PASS4_NUM_A; a_corr++)
		{
			int a = best3_a + LIVE_PASS4_A[a_corr];
			int x = best4_x;
			int y = best4_y;
			scan_to_2d_live(p_livelidar_img2, p_livelid2d_img2, a, x, y);
			int lvl = p_calc_f(p_livelid2d_img1, p_livelid2d_img2);

			if(lvl > biggest_lvl)
			{
				biggest_lvl = lvl;
				best4_a = a;
			}
		}
		if(calc_must_be_finished) return 28;

		if(biggest_lvl == 0) return 11;
	}
	else
	{
		best4_a = best3_a;
		best4_x = best3_x;
		best4_y = best3_y;
	}

	latest_corr.ang = best4_a;
	latest_corr.x = best4_x;
	latest_corr.y = best4_y;	

	if(best4_a > -2*ANG_1_DEG && best4_a < 2*ANG_1_DEG &&
	   best4_x > -30  &&  best4_x < 30  &&
	   best4_y > -30  &&  best4_y < 30  &&
	   supposed_a_diff > -6*ANG_1_DEG && supposed_a_diff < 6*ANG_1_DEG &&
	   supposed_x_diff > -80 && supposed_x_diff < 80 &&
	   supposed_y_diff > -80 && supposed_y_diff < 80)
	{
		/*
			Based on both information provided by feedback.c, and our lidar_corr,
			robot has moved very little.

			As the lidar information is quantized (especially in angular 360-step resolution),
			error would accumulate. So, we instruct not to swap buffers, but overwrite img2,
			while keeping img1 the same.
		*/

		skip = 1;

		memcpy(&lid_skiphold, p_livelidar_img1, sizeof(live_lidar_scan_t));
		memcpy(l2d_skiphold, p_livelid2d_img1, 360*sizeof(point_t));
		memcpy(num_samples_skiphold, p_livelidar_num_samples_img1, 6*sizeof(int));
	}
	else
	{
		skip = 0;
	}

	// We apply the correction to img2, which will be img1 on the next round (unless we skip).
	// In addition, we still need to apply the correction to the new lidar scan just being finished,
	// and to the robot coordinates as well (just before starting a new lidar scan)
	apply_corr_to_livelidar(p_livelidar_img2);

	if(skip)
	{
		return 100;
	}

	return 0;
}

// Run this continuosly on the main thread.
// if allowed_to_send_lidar, and the time is right to send the previous lidar image, it's sent, and 1 is returned.
// if allowed_to_send_lidar == 0, sending is just skipped, and the image will be missed by the host.


int livelidar_fsm(int allowed_to_send_lidar)
{
	int ret = 0;
	if(live_lidar_calc_req)
	{
		live_lidar_calc_req = 0;
		// Before starting calculating the new images, copy the result of previous scan for sending, and send it.
		if(allowed_to_send_lidar && !uart_busy())
		{
			if(skip)
				send_2d_live_to_uart(p_livelidar_img2, p_livelid2d_img2, 0);
			else
				send_2d_live_to_uart(p_livelidar_img1, p_livelid2d_img1, 1);
			ret = 1;
		}
		// Start correcting img1,img2.
		latest_corr_ret = do_livelidar_corr();
	}

	return ret;
}

