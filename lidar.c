/*
A bit of development history:

First we used a Neato XV11 lidar; it seemed a good idea at the time to do some prototyping on. 
We also considered using it on early production models (small batches).

A few things happened:
- The XV11 lidar has a serious lifetime problem with the slip ring communication.
- It also has an availability problem; most units are used, and already starting to fail.
- We decided not to use 2D lidar on long term, since we are actively developing our own 3D vision system.

Scanse Sweep was not available at that time, but now it is, so we migrated to it.

We decided to break all compatibility to the old Neato lidar simply because of its poor availability and reliability;
Scanse, OTOH, is readily available.

Changing the lidars is not the case of only changing the low-level layer, since the two lidars work in different principle:
Neato lidar gave readings on fixed angular intervals of 1.0 degrees, whereas the Scance gives variable number of samples with
an angular resolution of 1/16th degree (5760th full circle).

Since the code designed for Neato only server historical purposes, the compatibility will be broken on purpose. You can always
look at the old code in the version control.

*/

/*
Scance Sweep

Sweep	Sample	Min	Max	Smp/rev
				min	max

1Hz	01	500Hz	600Hz	500	600
1Hz	02	750Hz	800Hz	750	800
1Hz	03	1000Hz	1075Hz	1000	1075

2Hz	01	500Hz	600Hz	250	300
2Hz	02	750Hz	800Hz	375	400
2Hz	03	1000Hz	1075Hz	500	538

3Hz	01	500Hz	600Hz	166	200
3Hz	02	750Hz	800Hz	250	267
3Hz	03	1000Hz	1075Hz	333	359

4Hz	01	500Hz	600Hz	125	150
4Hz	02	750Hz	800Hz	187	200
4Hz	03	1000Hz	1075Hz	250	269

We probably don't want to ever run at over 4Hz, since the angular resolution would be compromised too much.

For mapping large spaces, it's crucial to get enough samples from far-away (say, 10 meters) walls, when seen through
gaps created by nearby obstacles. The 360 degree resolution of the Neato lidar seemed bare minimum; even with its low 5 meter
range!

At the same time, we want to gather enough temporal data to ignore moving objects and fill in the wall data while the robot
moves and sees the world from different angles. To add to this compromise, we don't want to sacrifice measurement accuracy.

To begin, we'll be using 2Hz sweep with 750-800Hz sample rate, giving us 375 to 400 samples per 360 degrees.

Maximum number of readings per sweep is guaranteed 1075. We'll probably never do that.

Let's use an array[720] to hold the samples: we'll never try to write to the same cell twice (if we don't use 1Hz-02 or 1Hz-03 modes)

We fill the array in the interrupt handler receiving the packets: even though the array step is only 0.5 deg, full 1/16th deg resolution
is used to calculate the (x,y) coords, this calculation uses the most recent robot coordinates to map the lidar to the world coords.


*/



#include <stdint.h>
#include "ext_include/stm32f2xx.h"

#include "main.h"
#include "lidar.h"

extern int dbg[10];

extern void delay_us(uint32_t i);
extern void delay_ms(uint32_t i);

void lidar_reset_flags() 
{
	DMA2->LIFCR = 0b111101UL<<16;
}
void lidar_reset_complete_flag() 
{
	DMA2->LIFCR = 0b100000UL<<16;
}
void lidar_reset_half_flag() 
{
	DMA2->LIFCR = 0b010000UL<<16;
}

int lidar_is_complete()
{
	return DMA2->LISR & (1UL<<21);
}

int lidar_is_half()
{
	return DMA2->LISR & (1UL<<20);
}

uint8_t lidar_ignore[360];

const int lidar_ignore_len[32] =
#if defined(RN1P4) || defined(RN1P6) || defined(RN1P5)
{
100,
(100+170)/2,
170,
(170+200)/2,
200,
(200+220)/2,
220,
(220+200)/2,
200,
(200+250)/2,
250,
(250+300)/2,
300,
(300+390)/2,
390,
(390+350)/2,
350,
(350+390)/2,
390,
(390+300)/2,
300,
(300+250)/2,
250,
(250+200)/2,
200,
(200+220)/2,
220,
(220+200)/2,
200,
(200+170)/2,
170,
(170+100)/2
};
#endif

#ifdef PULU1
{
30,
(30+55)/2,
55,
(55+70)/2,
70,
(70+90)/2,
90,
(90+70)/2,
70,
(70+90)/2,
90,
(90+120)/2,
120,
(120+210)/2,
210,
(210+190)/2,
190,
(190+210)/2,
210,
(210+120)/2,
120,
(120+90)/2,
90,
(90+70)/2,
70,
(70+90)/2,
90,
(90+70)/2,
70,
(70+55)/2,
55,
(55+30)/2
};
#endif


extern live_lidar_scan_t* p_livelidar_store;
extern point_t* p_livelid2d_store;
extern int* p_livelidar_num_samples_store; // For counting validness of data for lidar-based correction.
/*
	Call lidar_fsm() at 1 kHz.

	As lidar produces full revolutions at 5Hz, packets are generated at 450Hz.
	Data is prepared and stored with location info (cur_pos from feedbacks.c) on each sample.
*/

int reset;
int cur_lidar_id;

void reset_livelidar_images(int id)
{
	reset = 3; // to skip doing anything with the image being acquired right now.
	if(id > -1 && id < 128)
	{
		cur_lidar_id = id;
	}
}

void lidar_mark_invalid()
{
	p_livelidar_store->status |= LIVELIDAR_INVALID;
}

void lidar_fsm()
{
	static int prev_cur_packet;
	int packets_left = DMA2_Stream2->NDTR/22;
	int cur_packet = 89-packets_left; if(cur_packet == -1) cur_packet = 0;

	if(cur_packet != prev_cur_packet)
	{
		int valid;
		int dist;
		int idx = prev_cur_packet; // We read from previous packet, since writing it is finished.
		int odx = 89-idx; // We write starting from the end to mirror the lidar image.
		int valid_tbl_odx = odx/15; // Validness table includes total counts of valid points divided in six 60 degree segments.
		int ignore_len_tbl_odx = (odx*32+16)/90;
		COPY_POS(p_livelidar_store->pos[odx], cur_pos);

		dist = lidar_full_rev[idx].d[0].flags_distance&0x3fff;
		p_livelidar_store->scan[odx*4+3] = dist;
		valid = !((lidar_ignore[idx*4+0]) || (lidar_full_rev[idx].d[0].flags_distance&(1<<15)) || dist < lidar_ignore_len[ignore_len_tbl_odx]);
		p_livelid2d_store[odx*4+3].valid = valid;
		if(valid) p_livelidar_num_samples_store[valid_tbl_odx]++;

		dist = lidar_full_rev[idx].d[1].flags_distance&0x3fff;
		p_livelidar_store->scan[odx*4+2] = dist;
		valid = !((lidar_ignore[idx*4+1]) || (lidar_full_rev[idx].d[1].flags_distance&(1<<15)) || dist < lidar_ignore_len[ignore_len_tbl_odx]);
		p_livelid2d_store[odx*4+2].valid = valid;
		if(valid) p_livelidar_num_samples_store[valid_tbl_odx]++;

		dist = lidar_full_rev[idx].d[2].flags_distance&0x3fff;
		p_livelidar_store->scan[odx*4+1] = dist;
		valid = !((lidar_ignore[idx*4+2]) || (lidar_full_rev[idx].d[2].flags_distance&(1<<15)) || dist < lidar_ignore_len[ignore_len_tbl_odx]);
		p_livelid2d_store[odx*4+1].valid = valid;
		if(valid) p_livelidar_num_samples_store[valid_tbl_odx]++;

		dist = lidar_full_rev[idx].d[3].flags_distance&0x3fff;
		p_livelidar_store->scan[odx*4+0] = dist;
		valid = !((lidar_ignore[idx*4+3]) || (lidar_full_rev[idx].d[3].flags_distance&(1<<15)) || dist < lidar_ignore_len[ignore_len_tbl_odx]);
		p_livelid2d_store[odx*4+0].valid = valid;
		if(valid) p_livelidar_num_samples_store[valid_tbl_odx]++;


		if(prev_cur_packet == 81)
		{
			// Time is running out: tell lidar_corr that calculation must be finished ASAP, or terminated.
			// It won't be terminated right away; the termination condition is not checked too frequently as
			// it would slow down the process.
			live_lidar_calc_must_be_finished();
		}

		if(prev_cur_packet == 89)
		{	
			// We just got the full round.

			if(reset == 0)
			{

				int skip = livelidar_skip();


				// Now processing the two previous lidar images is (must be) finished, and the correction
				// has already been applied to the latter one. We still need to apply the same correction
				// to this new image we just finished storing:

				if(!skip)
					apply_corr_to_livelidar(p_livelidar_store);

				p_livelidar_store->id = cur_lidar_id;

				// Now, correction has been applied to both images: the previous one, and the one just finished.
				// We can swap the buffers to start gathering the new image, and start processing the latest scan:
				livelidar_storage_finished(cur_lidar_id);

				// One more thing, we need to apply the correction to the robot coordinates right here,
				// so that we get the new coords applied to the new lidar scan from the start:
				extern pos_t latest_corr; // from lidar_corr.c
				if(!skip)
					correct_location_without_moving(latest_corr);
			}
			else
			{
				reset--;
				reset_lidar_corr_images();

				if(reset == 0)
					livelidar_storage_finished();
			}
		}
	}
	prev_cur_packet = cur_packet;
}

void generate_lidar_ignore()
{
	int i;
	for(i = 0; i < 360; i++) lidar_ignore[i] = 0;

	for(i = 0; i < 90; i++)
	{
		int o;
		for(o = 0; o < 4; o++)
		{
			if(!(lidar_full_rev[i].d[o].flags_distance&(1<<15)))
			{
				if((int)(lidar_full_rev[i].d[o].flags_distance&0x3fff) < ((i<12||i>=78)?LIDAR_IGNORE_LEN_FRONT:LIDAR_IGNORE_LEN))
				{
					int cur = i*4+o;
					int next = cur+1; if(next > 359) next = 0;
					int prev = cur-1; if(prev < 0) prev = 359;
					lidar_ignore[prev] = 1;
					lidar_ignore[cur] = 1;
					lidar_ignore[next] = 1;
				}
			}
		}
	}
}


void lidar_dma_inthandler()
{

}

void init_lidar()
{
	// USART1 (lidar) = APB2 = 60 MHz
	// 16x oversampling
	// 115200bps -> Baudrate register = 32.5625 = 32 9/16
	// USART1 RX is mapped to DMA2, Stream2, Ch4

	// Do not enable the DMA yet.

	DMA2_Stream2->PAR = (uint32_t)&(USART1->DR);
	DMA2_Stream2->M0AR = (uint32_t)(lidar_full_rev);

	USART1->BRR = 32UL<<4 | 9UL;
	USART1->CR1 = 1UL<<13 /*USART enable*/ | 1UL<<3 /*TX ena*/ | 1UL<<2 /*RX ena*/;

	lidar_initialized = 1;
}

