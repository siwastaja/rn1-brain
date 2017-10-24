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
an angular resolution of 1/16th degree (5760th full circle). Sampling time, and hence the angular sample interval, is _not_ constant.

Since the code designed for Neato only serves historical purposes, the compatibility will be broken on purpose. You can always
look at the old code in the version control.

*/

/*
Scance Sweep

Sweep	Sample	Min	Max	Samples/rev
rate	mode	samp.f	samp.f	min	max

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

We probably don't want to ever run at over 4Hz, since the angular resolution would be compromised too much. (Maybe we'll do it 
one day as a separate mode, when the environment is well mapped, with lots of easy visual clues, proven IMU reliability, so that
the localization dependence on good laser data is not crucial. Then we can run at higher speeds and avoid obstacles.

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

/*

Scan matching

We provide scan matching on low level. Benefits:

* Offloads some work from the mapping computer

* Reduces the amount of lidar data to send to the computer by combining scan points, an important point currently as we are still using 115200bps UART.

* Provides direct feedback to self-calibrate gyro & wheel measurements all the time. It's good to have them calibrated
  when we enter an area where visibility is poor so we can't scan match! This provides much less uncertainty for the mapping
  layer, so it can, for example, use fewer particles (if particle filter is used for SLAM)

Development history:

First version using NEATO lidar used low-level scan matching as well. It was a separate module working with two fully acquired scans.
There was one particular issue: on long, straight, smooth-walled corridors, it caused severe shift. This was due to the discrete points
in the lidar scans:


*         *



*         *   


*         *

*         *

*         *
*    O    *
*         *

*         *

*         *


*         *



*         * <-- think about scoring this point with the next scan, which looks exactly similar! Good scores are attained when
the scans are shifted on the top of each other, but the points do NOT represent the same point in the actual wall, so wrong data
association gets good scores. 

This was mitigated by two means:
* nonlinear scoring function, which gave rather small differences in scores for small alignment errors
* weighing of total scores, so that no correction was preferred over correction if the differences in scores was small
* requirement to have enough sample points in all segments of the images before running the algorithm at all, so that if no
perpendicular walls exist, correction is not done at all.

These mitigations were not enough: if adjusted not to produce unwanted errors, the algorithm simply practically never did
anything.


Instead, the right thing to do is:

*         *
|         |
|         |
|         |
*         *   
|         |
|         |
*         *
|         |
*         *
|         |
*         *
*    O    *
*         *
|         |
*         *
|         |
*         *
|         |
|         |
*         *
|         |
|         |
|         |    (where | is interpolated data, given the same value as the direct data points *)
*         *  <-- now, we still don't know what the right landmark association is, but scoring two of these images together results in
identical score for all linear shifts. Some weighing should easily take care of this uncertainty. And we can apply correction weighted by
the certainty of the results (basically forming a kalman filter).


What we need to do, is to match points in the latter scan to the _interpolated lines between points_ in the previous scan, instead of
just the points.


Why we do it here, instead of a separate module? Timing and performance.

By definition, the Scance lidar is producing a continuous data flow, with _nearly_ fixed data point interval. Take 2Hz turn rate for example.

Instead of buffering (load-calculate-store-load-calculate-store... in ISR) for 0.5 seconds, then processing (load-calculate-store-load-
calculate-store... outside the ISR) for another 0.5 seconds, while simultaneously buffering the next scan, why not just do the calculation 
right when the data arrives? It's already loaded in registers at that point, saving some cycles. Additionally, since we have to time everything
for the worst case anyway, we have the same fixed time to process each data point in any case.

So, since we need an interrupt handler processing one 7-byte lidar data packet to calculate X,Y coords for a point, we can as well calculate the coords
for several different potential robot poses, and while at it, score them.

Since we have a fairly good IMU + wheel estimation, and we are self-calibrating it all the time, we are only looking at small corrections
between the two images. We don't have time to look far, anyway. If the points diverge far away, they'll be removed or marked invalid, and the
mapping layer needs to figure it out.



Data Management

Prev img = corrected

Measurement at [ang] arrives
Loop through different potential robot poses
	Calculate (xp, yp) for the measured point at that robot pose
	Loop through prev img[ang-search_ang]..img[ang+search_ang] (in theory, the full image, but no time for that):
		Find the closest (x1,y1) and the second closest (x2,y2) points
	Calculate the shortest distance from (xp,yp) to the straight line defined by (x1,y1) and (x2,y2)


Distance (from Wikipedia):

abs((y2-y1)*xp - (x2-x1)*yp + x2*y1 - y2*x1)   /   sqrt(sq(y2-y1)+sq(x2-x1))

We don't need absolute distance, only a relative score, so we can as well square the whole thing:
sq((y2-y1)*xp - (x2-x1)*yp + x2*y1 - y2*x1)    /   (sq(y2-y1)+sq(x2-x1))

Or, why do we need the denominator) at all?
abs((y2-y1)*xp - (x2-x1)*yp + x2*y1 - y2*x1)




      *           *         *       *
                 X

           



Acq             11111111222222223333333344444444...
Match 1st                       11111   22222
Match 2nd                       22222   33333
Correct pose                        X       X
Replace old scan poses               3       4
Corr full scan                        2       3
Gen composite scan                    X       X
Send composite scan                    XXXX    XXXX




*/


#include <stdint.h>
#include "ext_include/stm32f2xx.h"

#include "own_std.h"
#include "uart.h" // temporary debug
#include "main.h"
#include "lidar.h"
#include "sin_lut.h"
#include "comm.h"

 
#define UART_DMA_NO() //do {USART1->CR3 = 0; USART1->SR = 0;} while(0)
#define UART_DMA_RX() //do {USART1->CR3 = 1UL<<6; USART1->SR = 0;} while(0)
#define UART_DMA_TX() //do {USART1->CR3 = 1UL<<7; USART1->SR = 0;} while(0)
#define UART_DMA_RXTX() //do {USART1->CR3 = 1UL<<6 | 1UL<<7; USART1->SR = 0;} while(0)

volatile int lidar_dbg1, lidar_dbg2;

void rx_dma_off()
{
	USART1->CR3 &= ~(1UL<<6);
	DMA2_Stream2->CR = 0;
	while(DMA2_Stream2->CR & 1UL)
		lidar_dbg2++;
	DMA2->LIFCR = 0b111101UL<<16;
	USART1->SR;
	USART1->DR;
	DMA2->LIFCR = 0b111101UL<<16;
}

void tx_dma_off()
{
	USART1->CR3 &= ~(1UL<<7);
	DMA2_Stream7->CR = 0;
	while(DMA2_Stream7->CR & 1UL)
		lidar_dbg2++;
	DMA2->HIFCR = 0b111101UL<<22;
}


void send_lidar_to_uart(lidar_scan_t* in, int significant_for_mapping)
{
	uint8_t* buf = txbuf;

	int a_mid = in->pos_at_start.ang>>16;
	int x_mid = in->pos_at_start.x;
	int y_mid = in->pos_at_start.y;

	*(buf++) = 0x84;
	*(buf++) = ((in->status&LIVELIDAR_INVALID)?4:0) | (significant_for_mapping&0b11);
	*(buf++) = in->id&0x7f;

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

	*(buf++) = 0;
	int tmp = 0>>16;
	*(buf++) = I16_MS(tmp);
	*(buf++) = I16_LS(tmp);
	tmp = 0<<2;
	*(buf++) = I16_MS(tmp);
	*(buf++) = I16_LS(tmp);
	tmp = 0<<2;
	*(buf++) = I16_MS(tmp);
	*(buf++) = I16_LS(tmp);


	for(int i = 0; i < 360; i++)
	{
		if(in->scan[i*2].valid)
		{
			int x = in->scan[i*2].x - x_mid;
			int y = in->scan[i*2].y - y_mid;

			if(x < -8000 || x > 8000 || y < -8000 || y > 8000)
			{
				x = 0;
				y = 0;
			}

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



extern int dbg[10];

extern void delay_us(uint32_t i);
extern void delay_ms(uint32_t i);

void lidar_mark_invalid()
{
}

point_t lidar_collision_avoidance[360];



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


/*
	Call lidar_fsm() at 1 kHz.
*/

int reset;
int cur_lidar_id;


/*
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
*/

lidar_state_t cur_lidar_state;

uint8_t lidar_rxbuf[2][16];
uint8_t lidar_txbuf[16];

uint8_t lidar_error_flags;

lidar_error_t lidar_error_code;

int sweep_idx;

int prev_lidar_scan_idx;

/*
Double buffer of processed lidar scans in the world coordinate frame.
ack_lidar_scan is actively written to all the time.
prev_lidar_scan points to the finished buffer. Take a local copy of the pointer: even if you start reading
it just before the buffer swap happens, you'll likely read it faster than the new data comes in, so it's ok :-).

The buffers are never zeroed out, but written over.
*/

lidar_scan_t lidar_scans[2];
lidar_scan_t *acq_lidar_scan;
lidar_scan_t *prev_lidar_scan;


void lidar_start_acq();
void lidar_send_cmd(int tx_len, int rx_len);
void lidar_rx_done_inthandler();

int lidar_fps = 2;
int lidar_smp = 2;

/*
	Controls the state machine to turn lidar on, stabilize, configure, and start acquiring.
	If called during acquisition, motor speed / sampling is changed on the fly.
*/
void lidar_on(int fps, int smp)
{
	if(fps < 1 || fps > 5 || smp < 1 || smp > 3)
		return;

	lidar_error_code = LIDAR_NO_ERROR;

	LIDAR_ENA();

	lidar_fps = fps;
	lidar_smp = smp;
	if(cur_lidar_state == S_LIDAR_RUNNING)
	{
		rx_dma_off();
		// Stop data acquisition first, then wait for the data to actually stop and flush the DMA state.
		lidar_txbuf[0] = 'D';
		lidar_txbuf[1] = 'X';
		lidar_txbuf[2] = 10;
		lidar_send_cmd(3, 0); // Don't expect any answer

		cur_lidar_state = S_LIDAR_RECONF;
	}
	else
		cur_lidar_state = S_LIDAR_WAITPOWERED; // This wait also makes sure the RX DMA is finished (or at least there's nothing we can do, anyway)
}

void lidar_off()
{
	rx_dma_off();
	tx_dma_off();
	LIDAR_DIS();
	cur_lidar_state = S_LIDAR_OFF;
	lidar_error_code = LIDAR_NO_ERROR;
}

int wait_ready_poll_cnt;

void lidar_fsm()
{
	static int powerwait_cnt;
	static int reconfwait_cnt;
	static int errorwait_cnt;

	if(cur_lidar_state != S_LIDAR_WAITPOWERED) powerwait_cnt = 0;
	if(cur_lidar_state != S_LIDAR_RECONF) reconfwait_cnt = 0;
	if(cur_lidar_state != S_LIDAR_ERROR) errorwait_cnt = 0;

	switch(cur_lidar_state)
	{
		case S_LIDAR_WAITPOWERED:
		{
			if(++powerwait_cnt > 2000)
			{
				cur_lidar_state = S_LIDAR_PRECONF_WAIT_READY;
				wait_ready_poll_cnt = 1000; // Try to first poll after 1 sec
			}
		}
		break;

		case S_LIDAR_PRECONF_WAIT_READY:
		case S_LIDAR_WAIT_READY:
		{
			// Just do the tx part of the polling here, the state is changed in ISR based on the reply.
			if(--wait_ready_poll_cnt == 0)
			{
				lidar_txbuf[0] = 'M';
				lidar_txbuf[1] = 'Z';
				lidar_txbuf[2] = 10;
				lidar_send_cmd(3, 5);
			}
		}
		break;

		case S_LIDAR_RECONF:
		{
			if(++reconfwait_cnt > 50 && !(DMA2_Stream2->CR & 1UL))
			{
				UART_DMA_TX();
				// Data flow has surely stopped, and RX DMA has been happily shut down - we can reconfigure whatever we want.
				// Configure everything again - since we were running just fine, the motor should be stable to accept the commands.
				lidar_txbuf[0] = 'M';
				lidar_txbuf[1] = 'I';
				lidar_txbuf[2] = 10;
				lidar_send_cmd(3, 5);
				cur_lidar_state = S_LIDAR_PRECONF_CHECK_SPEED;
			}

		}
		break;

		case S_LIDAR_ERROR:
		{
			LIDAR_DIS(); // Turn it off, back on later.
			errorwait_cnt++;

			if(errorwait_cnt == 5)
			{
				// Disable the DMAs, reset everything we can without affecting other things than lidar.
				rx_dma_off();
				tx_dma_off();
			}

			if(errorwait_cnt > 5000) // Let's decide that 5 seconds should reset any strange issues.
			{
				LIDAR_ENA();
				cur_lidar_state = S_LIDAR_WAITPOWERED;
			}
		}

		default:
		break;
	}

	static int watchdog;

	static lidar_state_t prev_lidar_state;

	if(prev_lidar_state == cur_lidar_state && cur_lidar_state != S_LIDAR_UNINIT && cur_lidar_state != S_LIDAR_OFF && cur_lidar_state != S_LIDAR_RUNNING)
	{
		if(++watchdog > 10000)
		{
			watchdog = 0;
			cur_lidar_state = S_LIDAR_ERROR;
			lidar_error_code = LIDAR_ERR_STATE_WATCHDOG;

/*
			char buffer[1000];
			char *p_buf = buffer;
			p_buf = o_str_append(p_buf, "\r\nWATCHDOG ERROR uart SR=");
			p_buf = o_utoa16(USART1->SR, p_buf);
			p_buf = o_str_append(p_buf, " CR3=");
			p_buf = o_utoa16(USART1->CR3, p_buf);
			p_buf = o_str_append(p_buf, " DMA2 LISR=");
			p_buf = o_utoa32(DMA2->LISR, p_buf);
			p_buf = o_str_append(p_buf, " HISR=");
			p_buf = o_utoa32(DMA2->HISR, p_buf);
			p_buf = o_str_append(p_buf, " St2CR=");
			p_buf = o_utoa32(DMA2_Stream2->CR, p_buf);
			p_buf = o_str_append(p_buf, " NDTR=");
			p_buf = o_utoa32(DMA2_Stream2->NDTR, p_buf);
			p_buf = o_str_append(p_buf, " St7CR=");
			p_buf = o_utoa32(DMA2_Stream7->CR, p_buf);
			p_buf = o_str_append(p_buf, " NDTR=");
			p_buf = o_utoa32(DMA2_Stream7->NDTR, p_buf);
			p_buf = o_str_append(p_buf, "\r\n");
			uart_print_string_blocking(buffer);
*/
		}
	}
	else
		watchdog = 0;

	prev_lidar_state = cur_lidar_state;

}

int lidar_cur_n_samples;
int dbg_prev_len;

// Undocumented bug in Scanse Sweep: while the motor is stabilizing / calibrating, it also ignores the "Adjust LiDAR Sample rate" command (completely, no reply).

void lidar_rx_done_inthandler()
{
	DMA2->LIFCR = 0b111101UL<<16; // Clear DMA interrupt flags
	USART1->SR = 0;

	lidar_dbg1++;
//	lidar_rxbuf[0][dbg_prev_len] = 0;

/*	char kakka[10] = "\r\n#RXxx|";
	kakka[5] = (cur_lidar_state>9)?('a'+cur_lidar_state):('0'+cur_lidar_state);
	kakka[6] = '0'+dbg_prev_len;
	uart_print_string_blocking(kakka);

	for(int i=0; i<dbg_prev_len; i++)
	{
		char buf[5];
		o_utoa8_fixed(lidar_rxbuf[0][i], buf);
		buf[3] = ' ';
		buf[4] = 0;
		uart_print_string_blocking(buf);
	}
*/
//	uart_print_string_blocking((const char *)lidar_rxbuf[0]);
//	uart_print_string_blocking("|\r\n");


	static int chk_err_cnt;
	switch(cur_lidar_state)
	{
		case S_LIDAR_PRECONF_WAIT_READY:
		{
			if(lidar_rxbuf[0][0] == 'M' && 
			   lidar_rxbuf[0][1] == 'Z' &&
			   lidar_rxbuf[0][2] == '0' &&
			   lidar_rxbuf[0][3] == '0')
			{
				// Motor speed is stabilized to whatever uninteresting default value: now we can start configuring the device.
				// Check the speed setting first:
				lidar_txbuf[0] = 'M';
				lidar_txbuf[1] = 'I';
				lidar_txbuf[2] = 10;
				lidar_send_cmd(3, 5);
				cur_lidar_state = S_LIDAR_PRECONF_CHECK_SPEED;
			}
			else
			{
			//	lidar_dbg1++;
				// Motor not stabilized yet. Poll again after a 100 ms pause (in lidar_fsm())
				wait_ready_poll_cnt = 100;
			}
		}
		break;

		case S_LIDAR_PRECONF_CHECK_SPEED:
		{
			if(lidar_rxbuf[0][0] == 'M' && 
			   lidar_rxbuf[0][1] == 'I' &&
			   lidar_rxbuf[0][2] == '0' &&
			   lidar_rxbuf[0][3] == '0'+lidar_fps)
			{
				// The speed is configured correctly already - don't conf it again, it would cause a recalibration wait.
				// Send "Adjust LiDAR Sample Rate" command.
				lidar_txbuf[0] = 'L';
				lidar_txbuf[1] = 'R';
				lidar_txbuf[2] = '0';
				lidar_txbuf[3] = '0'+lidar_smp;
				lidar_txbuf[4] = 10;
				cur_lidar_state = S_LIDAR_CONF_SAMPLING;
				lidar_send_cmd(5, 9);
			}
			else
			{
				// Speed is not configured correctly yet. Conf it now.
				lidar_txbuf[0] = 'M';
				lidar_txbuf[1] = 'S';
				lidar_txbuf[2] = '0';
				lidar_txbuf[3] = '0'+lidar_fps;
				lidar_txbuf[4] = 10;
				cur_lidar_state = S_LIDAR_CONF_SPEED;
				lidar_send_cmd(5, 9);
			}
		}
		break;


		case S_LIDAR_CONF_SPEED:
		{
			if(lidar_rxbuf[0][0] == 'M' && 
			   lidar_rxbuf[0][1] == 'S' &&
			   lidar_rxbuf[0][2] == '0' &&
			   lidar_rxbuf[0][3] == '0'+lidar_fps &&
			   lidar_rxbuf[0][4] == 10 &&
			   lidar_rxbuf[0][5] == '0' &&
			   lidar_rxbuf[0][6] == '0' &&
			   lidar_rxbuf[0][7] == 'P' &&
			   lidar_rxbuf[0][8] == 10)
			{
				// Change speed command succesful - start waiting for it to stabilize...
				cur_lidar_state = S_LIDAR_WAIT_READY;
				wait_ready_poll_cnt = 1000; // first poll after 1sec.
			}
			else
			{
				lidar_error_code = LIDAR_ERR_UNEXPECTED_REPLY_MOTORSPEED;
				cur_lidar_state = S_LIDAR_ERROR;
			}
		}
		break;

		case S_LIDAR_WAIT_READY:
		{
			if(lidar_rxbuf[0][0] == 'M' && 
			   lidar_rxbuf[0][1] == 'Z' &&
			   lidar_rxbuf[0][2] == '0' &&
			   lidar_rxbuf[0][3] == '0')
			{
				// Motor speed is stabilized.
				// Send "Adjust LiDAR Sample Rate" command.
				lidar_txbuf[0] = 'L';
				lidar_txbuf[1] = 'R';
				lidar_txbuf[2] = '0';
				lidar_txbuf[3] = '0'+lidar_smp;
				lidar_txbuf[4] = 10;
				cur_lidar_state = S_LIDAR_CONF_SAMPLING;
				lidar_send_cmd(5, 9);
			}
			else
			{
				// Motor not stabilized yet. Poll again after 100 ms pause (in lidar_fsm())
//				lidar_dbg1++;
				wait_ready_poll_cnt = 100;
			}
		}
		break;

		case S_LIDAR_CONF_SAMPLING:
		{
			if(lidar_rxbuf[0][0] == 'L' && 
			   lidar_rxbuf[0][1] == 'R' &&
			   lidar_rxbuf[0][2] == '0' &&
			   lidar_rxbuf[0][3] == '0'+lidar_smp &&
			   lidar_rxbuf[0][4] == 10 &&
			   lidar_rxbuf[0][5] == '0' &&
			   lidar_rxbuf[0][6] == '0' &&
			   lidar_rxbuf[0][7] == 'P' &&
			   lidar_rxbuf[0][8] == 10)
			{
				// Sample rate successfully set.
				// Send "Start data acquisition" command and start waiting for actual scan data.
				lidar_txbuf[0] = 'D';
				lidar_txbuf[1] = 'S';
				lidar_txbuf[2] = 10;
				cur_lidar_state = S_LIDAR_WAIT_START_ACK;
				lidar_send_cmd(3, 6);
			}
			else
			{
				lidar_error_code = LIDAR_ERR_UNEXPECTED_REPLY_SAMPLERATE;
				cur_lidar_state = S_LIDAR_ERROR;
			}
		}
		break;

		case S_LIDAR_WAIT_START_ACK:
		{
			if(lidar_rxbuf[0][0] == 'D' && 
			   lidar_rxbuf[0][1] == 'S' &&
			   lidar_rxbuf[0][2] == '0' &&
			   lidar_rxbuf[0][3] == '0' &&
			   lidar_rxbuf[0][4] == 'P') // The correct checksum from "00"
			{
				// Start data acquisition acknowledged OK. Reconfigure the DMA to circular doublebuffer without reconfig, to minimize time
				// spent in this ISR in the RUNNING state.
				lidar_start_acq();
				cur_lidar_state = S_LIDAR_RUNNING;
				chk_err_cnt = 0;
			}
			else
			{
				// This shouldn't happen, as we have polled to confirm that the motor is ready.
				lidar_error_code = LIDAR_ERR_DATASTART_ACK;
				cur_lidar_state = S_LIDAR_ERROR;
			}
		}
		break;

		case S_LIDAR_RUNNING:
		{
			int buf_idx = (DMA2_Stream2->CR&(1UL<<19))?0:1; // We want to read the previous buffer, not the one the DMA is now writing to.
			// Actual data packet is 7 bytes of binary instead of ASCII.
			int chk = (lidar_rxbuf[buf_idx][0]+lidar_rxbuf[buf_idx][1]+lidar_rxbuf[buf_idx][2]+
				  lidar_rxbuf[buf_idx][3]+lidar_rxbuf[buf_idx][4]+lidar_rxbuf[buf_idx][5]) % 255;
			if(chk != lidar_rxbuf[buf_idx][6] /*checksum fail*/ || (lidar_rxbuf[buf_idx][0]&0b11111110) /* any error bit*/)
			{
				chk_err_cnt+=20;

				if(chk_err_cnt > 100)
				{
					// In the long run, 1/20th of the data is allowed to fail the checksum / error flag tests.
					// In the short run, 5 successive samples are allowed to fail.
					cur_lidar_state = S_LIDAR_ERROR;
					lidar_error_code = LIDAR_ERR_CHKSUM_OR_ERRFLAGS;
					lidar_error_flags = lidar_rxbuf[buf_idx][0];
				}
				// Else: just ignore this data.

				break;
			}


			if(chk_err_cnt) chk_err_cnt--;
			if(lidar_rxbuf[buf_idx][0]) // non-zero = sync. (error flags have been handled already)
			{
				acq_lidar_scan->n_samples = lidar_cur_n_samples;
				COPY_POS(acq_lidar_scan->pos_at_end, cur_pos);
				lidar_scan_t* swptmp;
				swptmp = prev_lidar_scan;
				prev_lidar_scan = acq_lidar_scan;
				acq_lidar_scan = swptmp;
				COPY_POS(acq_lidar_scan->pos_at_start, cur_pos);
				lidar_cur_n_samples = 0;
				acq_lidar_scan->n_samples = lidar_cur_n_samples;
			}

			lidar_cur_n_samples++;

			// optimization todo: we are little endian like the sensor: align rxbuf properly and directly access as uint16
			int32_t degper16 = (lidar_rxbuf[buf_idx][2]<<8) | lidar_rxbuf[buf_idx][1];
			int32_t len      = (lidar_rxbuf[buf_idx][4]<<8) | lidar_rxbuf[buf_idx][3];
//			int snr      = lidar_rxbuf[buf_idx][5];

			unsigned int degper2 = degper16>>3;
			if(degper2 > 719)
			{
				chk_err_cnt+=20;
				break;
			}

			if(len < 2)
			{
				acq_lidar_scan->scan[degper2].valid = 0;
				break;
			}

			len *= 10; // cm --> mm

			uint32_t ang32 = (uint32_t)cur_pos.ang + degper16*ANG_1PER16_DEG;
			int32_t y_idx = (ang32)>>SIN_LUT_SHIFT;
			int32_t x_idx = (1073741824-ang32)>>SIN_LUT_SHIFT;

			acq_lidar_scan->scan[degper2].valid = 1;
			acq_lidar_scan->scan[degper2].x = cur_pos.x + (((int32_t)sin_lut[x_idx] * (int32_t)len)>>15);
			acq_lidar_scan->scan[degper2].y = cur_pos.y + (((int32_t)sin_lut[y_idx] * (int32_t)len)>>15);
			

				
		}
		break;


		default:
		break;

	}

}

/*
	Sends a command to the lidar using DMA; configures DMA to expect answer, which will give an interrupt after rx_len bytes received.
	To save a little bit of time, buffers are fixed (lidar_txbuf and lidar_rxbuf).

	If configuring the tx or rx DMA channel fails (due to it still being in use, no other reasons now), sets the error state.

*/
void lidar_send_cmd(int tx_len, int rx_len)
{
	if(rx_len && (DMA2_Stream2->CR & 1UL))
	{
		cur_lidar_state = S_LIDAR_ERROR;
		lidar_error_code = LIDAR_ERR_RX_DMA_BUSY;
		return;
	}

	if(tx_len && (DMA2_Stream7->CR & 1UL))
	{
		cur_lidar_state = S_LIDAR_ERROR;
		lidar_error_code = LIDAR_ERR_TX_DMA_BUSY;
		return;
	}

	if(rx_len)
	{
		rx_dma_off();
		// Configure for RX:
		DMA2_Stream2->CR = 4UL<<25 /*Channel*/ | 0b01UL<<16 /*med prio*/ | 0b00UL<<13 /*8-bit mem*/ | 0b00UL<<11 /*8-bit periph*/ |
			           1UL<<10 /*mem increment*/ | 0b00UL<<6 /*periph-to-memory*/ | 1UL<<4 /*transfer complete interrupt*/;
		DMA2_Stream2->NDTR = rx_len;

		USART1->SR;
		USART1->DR;
		USART1->CR3 |= (1UL<<6);

		dbg_prev_len = rx_len;

		DMA2->LIFCR = 0b111101UL<<16; // Clear DMA interrupt flags
		DMA2_Stream2->CR |= 1UL; // Enable RX DMA
	}

	if(tx_len)
	{
		tx_dma_off();
		// Configure for TX:
		DMA2_Stream7->CR = 4UL<<25 /*Channel*/ | 0b00UL<<16 /*low prio*/ | 0b00UL<<13 /*8-bit mem*/ | 0b00UL<<11 /*8-bit periph*/ |
			           1UL<<10 /*mem increment*/ | 0b01UL<<6 /*memory-to-periph*/;
		DMA2_Stream7->NDTR = tx_len;

		USART1->SR = 0;
		USART1->CR3 |= (1UL<<7);

		DMA2->HIFCR = 0b111101UL<<22; // Clear DMA interrupt flags
		DMA2_Stream7->CR |= 1UL; // Enable TX DMA
	}
}

/*
	Start receiving - configure the RX DMA for circular double buffering - we don't need to reconfigure DMA until we stop.
	Each new full packet causes an interrupt.
*/
void lidar_start_acq()
{
	DMA2_Stream2->CR = 4UL<<25 /*Channel*/ | 1UL<<18 /*Double Buf mode*/ | 0b01UL<<16 /*med prio*/ | 
			   0b00UL<<13 /*8-bit mem*/ | 0b00UL<<11 /*8-bit periph*/ |
	                   1UL<<10 /*mem increment*/ | 1UL<<8 /*circular*/ | 1UL<<4 /*transfer complete interrupt*/;  // Disable

	DMA2_Stream2->NDTR = 7;
	dbg_prev_len = 7;

	USART1->SR = 0;
	USART1->CR3 |= (1UL<<7);
	DMA2->LIFCR = 0b111101UL<<16; // Clear DMA interrupt flags
	DMA2_Stream2->CR |= 1UL; // Enable RX DMA
}

void init_lidar()
{
	acq_lidar_scan = &lidar_scans[0];
	prev_lidar_scan = &lidar_scans[1];
	// USART1 (lidar) = APB2 = 60 MHz
	// 16x oversampling
	// 115200bps -> Baudrate register = 32.5625 = 32 9/16
	// USART1 RX: DMA2 Stream2 Ch4
	// USART1 TX: DMA2 Stream7 Ch4

	// Preconfigure what we can on the DMA, don't enable yet.

	DMA2_Stream2->PAR = (uint32_t)&(USART1->DR);
	DMA2_Stream2->M0AR = (uint32_t)(lidar_rxbuf[0]);
	DMA2_Stream2->M1AR = (uint32_t)(lidar_rxbuf[1]);

	DMA2_Stream7->PAR = (uint32_t)&(USART1->DR);
	DMA2_Stream7->M0AR = (uint32_t)(lidar_txbuf);

	USART1->BRR = 32UL<<4 | 9UL;
	USART1->CR1 = 1UL<<13 /*USART enable*/ | 1UL<<3 /*TX ena*/ | 1UL<<2 /*RX ena*/;
	USART1->CR3 = 1UL<<6 | 1UL<<7;
	//UART_DMA_NO();

	cur_lidar_state = S_LIDAR_OFF;
	NVIC_SetPriority(DMA2_Stream2_IRQn, 0b0101);
	NVIC_EnableIRQ(DMA2_Stream2_IRQn);
	tx_dma_off();
	rx_dma_off();
}

