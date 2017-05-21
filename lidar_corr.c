/*
	Lidar-based error correction
	Tries to remove any (ang,x,y) error between two adjacent robot positions with two lidar scans.

	Inputs:
	lidar_scan_t *before, lidar_scan_t *after, both include the assumed (ang,x,y) coordinates.

	Outputs:
	(ang,x,y) corrections
	
*/

#include <stdint.h>
#include "lidar_corr.h"
#include "sin_lut.h"

extern volatile int dbg[10];


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

/*
#define PASS1_NUM_X 11
static int PASS1_X[PASS1_NUM_X] =
{
	-200,
	-160,
	-120,
	-80,
	-40,
	0,
	40,
	80,
	120,
	160,
	200
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
	17,
	15,
	13,
	10
};
*/

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

// For moving lidar scans with 360 scan points, and 90 position points.
static void scan_to_2d_live(live_lidar_scan_t* in, point_t* out)
{
	for(int i = 0; i < 360; i++)
	{
		int pos_idx = i>>2;
		uint32_t angle = (uint32_t)in->pos[pos_idx].ang + (uint32_t)i*(uint32_t)ANG_1_DEG;
		int32_t y_idx = (angle)>>SIN_LUT_SHIFT;
		int32_t x_idx = (1073741824-angle)>>SIN_LUT_SHIFT;
		out[i].x = in->pos[pos_idx].x + (((int32_t)sin_lut[x_idx] * (int32_t)in->scan[i])>>15);
		out[i].y = in->pos[pos_idx].y + (((int32_t)sin_lut[y_idx] * (int32_t)in->scan[i])>>15);
	}
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


live_lidar_scan_t live1, live2;

live_lidar_scan_t* p_live_store;


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
	* turn 18 degrees at 4 sec/360deg.
	* go 33 cm at 6 km/h (1.67 m/s)

	So, I decided looking at points for +/- 24 deg is ok.
*/


int32_t calc_match_lvl_live(point_t* img1, point_t* img2)
{
	int32_t dist_sum = 0;
	for(int i = 0; i < 360; i++)
	{
		if(!img1[i].valid) continue;

		int smallest = 1000*1000;
		uint8_t odx = i-24;
		if(odx < 0) odx+=360;
		for(int o = 0; o < 48; o++)
		{
			odx++;
			if(odx>359) odx=0;
			if(!img2[odx].valid) continue;
			int dx = img2[odx].x - img1[i].x;
			int dy = img2[odx].y - img1[i].y;
			int dist = sq(dx) + sq(dy);
			if(dist < smallest)
			{
				smallest = dist;
			}
		}

		int32_t dist_scaled = (360*(200*200+400))/(smallest+400);
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


