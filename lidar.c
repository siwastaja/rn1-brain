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
the localization dependence on good laser data is not crucial. Then we can run at higher speeds and avoid obstacles.)

For mapping large spaces, it's crucial to get enough samples from far-away (say, 10 meters) walls, when seen through
gaps created by nearby obstacles. The 360 degree resolution of the Neato lidar seemed bare minimum; even with its low 5 meter
range!

At the same time, we want to gather enough temporal data to ignore moving objects and fill in the wall data while the robot
moves and sees the world from different angles. To add to this compromise, we don't want to sacrifice measurement accuracy.

To begin, we'll be using 2Hz sweep with 750-800Hz sample rate, giving us 375 to 400 samples per 360 degrees.

Maximum number of readings per sweep is guaranteed 1075. We'll probably never do that.

We fill the array in the interrupt handler receiving the packets. Full 1/16th deg resolution is used to calculate the (x,y)
coords, this calculation uses the most recent robot coordinates to map the lidar to the world coords. The array is filled in
order, and only filled with valid readings, so the number of samples varies depending on scene and settings, and a certain
angle cannot be found at a certain index. After a (too) long consideration, I found this the best.

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

First version using NEATO lidar used low-level scan matching as well.

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

This was mitigated by:
* nonlinear scoring function, which gave rather small differences in scores for small alignment errors (also limiting the amount of correction in good environments!)
* weighing of total scores, so that zero correction was preferred if the differences in scores were small
* requirement to have enough sample points in most segments of the images before running the algorithm at all, so that if no
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
|         |    (where | is interpolated data, valued similarly to direct data points *)
*         *  <-- now, we still don't know what the right landmark association is, but scoring two of these images together results in
identical score for all linear shifts, which is the correct result; a human couldn't tell, either. Even minor weighing should easily take
care of this uncertainty and pick zero correction. And we can apply correction weighted by the certainty of the results (basically
forming a kalman filter).


What we need to do, is to match points in the latter scan to the _interpolated lines between points_ in the previous scan, instead of
just the points.


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

           

Timing:

Acq             11111111222222223333333344444444...
Match 1st                       11111   22222
Match 2nd                       22222   33333
Correct pose                        X       X
Replace old scan poses               3       4
Corr full scan                        2       3
Gen composite scan                    X       X
Send composite scan                    XXXX    XXXX


(More scan matching thoughts at the end of the file.)

*/


#include <stdint.h>
#include "ext_include/stm32f2xx.h"

#include "own_std.h"
#include "uart.h" // temporary debug
#include "main.h"
#include "lidar.h"
#include "sin_lut.h"
#include "comm.h"

#include "navig.h" // to inject points in micronavigation

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


extern int dbg[10];

extern void delay_us(uint32_t i);
extern void delay_ms(uint32_t i);

void lidar_mark_invalid()
{
}

point_t lidar_collision_avoidance[360];



/*
	Call lidar_fsm() at 1 kHz.
*/

int reset;
volatile int cur_lidar_id;

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

void set_lidar_id(int id)
{
	cur_lidar_id = id;
}


typedef struct  // These angles in 1/16th degrees!
{
	int32_t start;
	int32_t end;
} ignore_area_t;

#define IGN(mid, width) {(int32_t)(((mid)-((width)/2.0))*16.0), (int32_t)(((mid)+((width)/2.0))*16.0)}

// too many ignore areas could slow down the ISR too much... Sensible amount is 6.
#define N_IGNORE_AREAS 6
const ignore_area_t ignore_areas[N_IGNORE_AREAS] =
{
	IGN(58.80 , 6.80+5.0),
	IGN(121.20, 6.00+5.0),
	IGN(151.17, 3.46+5.0),
	IGN(208.83, 3.46+5.0),
	IGN(238.80, 6.00+10.0),  // cable hole
	IGN(301.20, 6.80+5.0)
};


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

volatile int lidar_near_filter_on = 1;
volatile int lidar_midlier_filter_on = 1;

volatile int lidar_scan_ready;

// Undocumented bug in Scanse Sweep: while the motor is stabilizing / calibrating, it also ignores the "Adjust LiDAR Sample rate" command (completely, no reply).

void lidar_rx_done_inthandler()
{
	int starttime = TIM6->CNT;

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
				dbg[6]++;
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

				extern volatile int dbg_sending_lidar;
				if(dbg_sending_lidar)
					dbg[5]++;


				acq_lidar_scan->n_points = lidar_cur_n_samples;
				dbg[9] = lidar_cur_n_samples;
				COPY_POS(acq_lidar_scan->pos_at_end, cur_pos);
				lidar_scan_t* swptmp;
				swptmp = prev_lidar_scan;
				prev_lidar_scan = acq_lidar_scan;
				acq_lidar_scan = swptmp;
				lidar_scan_ready = 1;
				COPY_POS(acq_lidar_scan->pos_at_start, cur_pos);
				// Right now, refxy is simply the robot pose at the start of the scan.
				acq_lidar_scan->refxy.x = cur_pos.x;
				acq_lidar_scan->refxy.y = cur_pos.y;
				lidar_cur_n_samples = 0;
				acq_lidar_scan->id = cur_lidar_id;
				acq_lidar_scan->status = 0;
				acq_lidar_scan->n_points = 0;
			}

			if(lidar_cur_n_samples > LIDAR_MAX_POINTS-1)
			{
				break; // Ignore excess data - wait for the sync data.
			}


			// optimization todo: we are little endian like the sensor: align rxbuf properly and directly access as uint16
			int32_t degper16 = (lidar_rxbuf[buf_idx][2]<<8) | lidar_rxbuf[buf_idx][1];
			int32_t len      = (lidar_rxbuf[buf_idx][4]<<8) | lidar_rxbuf[buf_idx][3];
			//int snr      = lidar_rxbuf[buf_idx][5];
			/*
				Filtering low-snr results was tested:
				Seems useless. When high-noise near-field results are looked at, snr thresholding seems to reduce
				the number of points quite a bit, but mostly in the middle; the worst case wrong points are still there!
				So, having a high number of noisy points is better. -> no snr-based filtering
			*/

			/*
				Remove "midliers", erroneous average points between two readings:

				#############################################

				                      <-- at least 25cm gap

				                 .    <-- midlier

				                      <-- at least 25cm gap

				##################


				                   O

			*/

			if(len < 20)
			{
				// "1 cm" signifies no signal. "0 cm" is undefined.
				// Points too near are garbage anyway, just ignore them and hope that a small obstacle near
				// the front is not unseen by this.
				break;
			}

			#define MIDLIER_LEN 25 // in cm
			if(lidar_midlier_filter_on)
			{

				static int midlier_prev3_len, midlier_prev2_len, midlier_prev_len;

				if(midlier_prev2_len > 1 && midlier_prev_len > 1 && len > 1)  //  -VxV       -=invalid, V=valid, x=potential midlier
				{
					if( (midlier_prev_len > midlier_prev2_len+MIDLIER_LEN && midlier_prev_len < len-MIDLIER_LEN) ||
					    (midlier_prev_len < midlier_prev2_len-MIDLIER_LEN && midlier_prev_len > len+MIDLIER_LEN))
					{
						// Remove (overwrite) the previous point as a midlier.
						if(lidar_cur_n_samples) lidar_cur_n_samples--;
					}
				}
				else if(midlier_prev3_len > 1 && midlier_prev_len > 1 && len > 1)  // V-xV
				{
					if( (midlier_prev_len > midlier_prev3_len+MIDLIER_LEN && midlier_prev_len < len-MIDLIER_LEN) ||
					    (midlier_prev_len < midlier_prev3_len-MIDLIER_LEN && midlier_prev_len > len+MIDLIER_LEN))
					{
						// Remove (overwrite) the previous point as a midlier.
						if(lidar_cur_n_samples) lidar_cur_n_samples--;
					}
				}
				else if(midlier_prev3_len > 1 && midlier_prev2_len > 1 && midlier_prev_len > 1) // VxV-
				{
					if( (midlier_prev2_len > midlier_prev3_len+MIDLIER_LEN && midlier_prev2_len < midlier_prev_len-MIDLIER_LEN) ||
					    (midlier_prev2_len < midlier_prev3_len-MIDLIER_LEN && midlier_prev2_len > midlier_prev_len+MIDLIER_LEN))
					{
						// Remove the point before the previous point as a midlier.
						if(lidar_cur_n_samples > 1)
						{
							acq_lidar_scan->scan[lidar_cur_n_samples-2].x = acq_lidar_scan->scan[lidar_cur_n_samples-1].x;
							acq_lidar_scan->scan[lidar_cur_n_samples-2].y = acq_lidar_scan->scan[lidar_cur_n_samples-1].y;
							lidar_cur_n_samples--;
						}
					}
				}
				else if(midlier_prev3_len > 1 && midlier_prev2_len > 1 && len > 1) // Vx-V
				{
					if( (midlier_prev2_len > midlier_prev3_len+MIDLIER_LEN && midlier_prev2_len < len-MIDLIER_LEN) ||
					    (midlier_prev2_len < midlier_prev3_len-MIDLIER_LEN && midlier_prev2_len > len+MIDLIER_LEN))
					{
						// Remove the point before the previous point as a midlier.
						if(lidar_cur_n_samples > 1)
						{
							acq_lidar_scan->scan[lidar_cur_n_samples-2].x = acq_lidar_scan->scan[lidar_cur_n_samples-1].x;
							acq_lidar_scan->scan[lidar_cur_n_samples-2].y = acq_lidar_scan->scan[lidar_cur_n_samples-1].y;
							lidar_cur_n_samples--;
						}
					}
				}

				midlier_prev3_len = midlier_prev2_len;
				midlier_prev2_len = midlier_prev_len;
				midlier_prev_len = len;

			}

			for(int i=0; i<N_IGNORE_AREAS; i++)
			{
				if(degper16 > ignore_areas[i].start && degper16 < ignore_areas[i].end)
				{
					goto IGNORE_SAMPLE;
				}
			}

			len *= 10; // cm --> mm

			/*
				Data of nearby points is very noisy, and gets noisier the nearer we see. In addition,
				we don't need so many points packed near each other -> average them together.
			*/

			int flt_len;
			static int prev_len, prev2_len, prev3_len, skip_averaging;

			if(lidar_near_filter_on && skip_averaging == 0)
			{
				if(len < 600 && (prev_len < 600 || prev2_len < 600 || prev3_len < 600))
				{
					// Average the four:
					flt_len = (len + prev_len + prev2_len + prev3_len)>>2;

					// Overwrite the previous, 4->1
					if(lidar_cur_n_samples > 2) lidar_cur_n_samples-=3;
					skip_averaging = 3;
				}
				else if(len < 800 && (prev_len < 800 || prev2_len < 800))
				{
					// Average the three:
					flt_len = (len + prev_len + prev2_len)/3;

					// Overwrite the previous, 3->1
					if(lidar_cur_n_samples > 1) lidar_cur_n_samples-=2;
					skip_averaging = 2;
				}
				else if(len < 1100)
				{
					// Average the two.
					flt_len = (len+prev_len)>>1;
					// 2->1
					if(lidar_cur_n_samples) lidar_cur_n_samples--;
					skip_averaging = 1;
				}
				else
				{
					flt_len = len;
					if(skip_averaging) skip_averaging--;
				}
			}
			else
			{
				flt_len = len;
				if(skip_averaging) skip_averaging--;
			}

			prev3_len = prev2_len;
			prev2_len = prev_len;
			prev_len = len;


			uint32_t ang32 = (uint32_t)cur_pos.ang - degper16*ANG_1PER16_DEG;
			int32_t y_idx = (ang32)>>SIN_LUT_SHIFT;
			int32_t x_idx = (1073741824-ang32)>>SIN_LUT_SHIFT;

			int32_t x = cur_pos.x + (((int32_t)sin_lut[x_idx] * (int32_t)flt_len)>>15) - acq_lidar_scan->refxy.x;
			int32_t y = cur_pos.y + (((int32_t)sin_lut[y_idx] * (int32_t)flt_len)>>15) - acq_lidar_scan->refxy.y;

			if(x < -30000 || x > 30000 || y < -30000 || y > 30000)
				break;

			acq_lidar_scan->scan[lidar_cur_n_samples].x = x;
			acq_lidar_scan->scan[lidar_cur_n_samples].y = y;
			
			lidar_cur_n_samples++;


			uint32_t ang32_robot_frame = -1*degper16*ANG_1PER16_DEG;
			int32_t y_idx_robot_frame = (ang32_robot_frame)>>SIN_LUT_SHIFT;
			int32_t x_idx_robot_frame = (1073741824-ang32_robot_frame)>>SIN_LUT_SHIFT;
			int32_t x_robot_frame =	((int32_t)sin_lut[x_idx_robot_frame] * (int32_t)flt_len)>>15;
			int32_t y_robot_frame =	((int32_t)sin_lut[y_idx_robot_frame] * (int32_t)flt_len)>>15;
			micronavi_point_in(x_robot_frame, y_robot_frame);


			IGNORE_SAMPLE: break;
		}
		break;


		default:
		break;

	}

	int tooktime = TIM6->CNT - starttime;
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


/*
Scan matching


Old MCU algorithm (called "livelider correction" in the old code)

Scoring function:
For each point (of all 360 points) in scan1, find the nearest one in scan2, by angularly scanning 20 points around the expected angle
(360*20 = 7200 operations, each operation:  dist = (x1-x2)^2 + (y1-y2)^2, if dist < smallest then smallest = dist)   <-- the runtime killer
Run smallest dist through non-linear shaping function (trivial, 1/(smallest+constant_offset), "only" 360 such operations)
Sum them up to form the score, bigger = better match

Iterations we used:

Pass 1:
First, (x,y) iterated together; then, with the winner, angle iterated separately

Ang:
-1.0 deg      
 0
+1.0 deg

X:
-60 mm
-30 mm
  0
+30 mm
+60 mm

Y: same as X

Run as:
First ang, then



Pass 2 (added on the top of the winner of pass1)
First, (x,y) iterated together; then, with the winner, angle iterated separately

Ang:
-1.0 deg      
-0.5 deg
 0
+0.5 deg
+1.0 deg

X:
-20 mm
-10 mm
  0
+10 mm
+20 mm

Y: same as X


Pass 3 (added on the top of the winner of pass2)
First, (x,y) iterated together; then, with the winner, angle iterated separately

Ang:
-0.50 deg      
-0.25 deg
 0
+0.25 deg
+0.50 deg

X:
-10 mm
 -5 mm
  0
 +5 mm
+10 mm

Y: same as X


Pass 4:
X, Y and angle all iterated separately

Ang:
-0.20 deg
-0.15 deg
-0.10 deg
-0.05 deg
 0
+0.05 deg
+0.10 deg
+0.15 deg
+0.20 deg


X:
-4 mm
-2 mm
 0
+2 mm
+4 mm




The new algorithm

* Admit that we'll only fix small drifts.
* The point in the gyro is that it's accurate short term!
* Angle is important, and needs to be iterated to high resolution, but not far away
* Understood that the X&Y are related, no need to stupidly iterate everything. As the angle is almost precise,
  most of the iteration needs to go in the "forward" direction only
* As the angle is iterated, sideway drift is expected to be related to the angular drift, as well
 --> much less iterations to do
 --> can spend the time to do them properly, i.e., nested


* Fix the biggest issue caused by discrete sampling points in featureless corridors
 --> scoring function complexity goes up, and it's the most important runtime killer :(
 --> stupid 360*20 two-dimensional search is not going to cut it anymore

* Now we have possibly more samples than 360 (prepare for 720) per scan
* At the same time, we have more time to process them, so it scales back somewhat
   * (But not quadratically, so we can't do a stupid 720*40 search)


* Correlate each sample in scan2 to an interpolated line of scan1, to fix the discretization issue.


By design:
* Limit scan range to 15 000 mm
* Limit search range (biggest error) to 0.75 deg, +/-100mm
-> Furthest away points will shift max 300 mm compared to zero correction (this info is useful in optimization)



* Pre-process step (for optimization)
For each point in scan2:




*/
