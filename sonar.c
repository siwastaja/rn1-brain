#include "ext_include/stm32f2xx.h"

#include "feedbacks.h" // for robot position
#include "sonar.h"
#include "sin_lut.h"

extern volatile int dbg[10];

typedef struct
{
	GPIO_TypeDef *trig_port;
	uint32_t trig_bit;
	GPIO_TypeDef *echo_port;
	uint32_t echo_bit;

/*

	##################
	##################
	##################
	################## -
	##############O### ^
	################## |  y_offs
	#################x v
	################## +
	##################
                     -<-->+  x_offs  


	            +
                    ^  
	 	O   |   side_angle


	up_angle: positive = looks up


*/
	int x_offs;
	int y_offs;
	int32_t side_angle;
	int32_t cos_up_angle_x32768; // 32768 = 1.0 = sensor points directly forward
} sonar_cfg_t;


#define SONAR_PULSE_ON(idx)  do{sonar_cfgs[(idx)].trig_port->BSRR = 1UL<<(sonar_cfgs[(idx)].trig_bit);}while(0);
#define SONAR_PULSE_OFF(idx) do{sonar_cfgs[(idx)].trig_port->BSRR = 1UL<<(sonar_cfgs[(idx)].trig_bit+16);}while(0);
#define SONAR_ECHO(idx) (sonar_cfgs[(idx)].echo_port->IDR & (1UL<<sonar_cfgs[(idx)].echo_bit))


sonar_cfg_t sonar_cfgs[NUM_SONARS] =
{
//	/* 0: (front view) left  : Trig IO1, Echo IO2 */ {GPIOE,  7,  GPIOE,  8},
//	/* 1: (front view) right : Trig IO7, Echo IO8 */ {GPIOE, 13,  GPIOE, 14},
	/* 2:         top middle : Trig IO3, Echo IO4 */ {GPIOE,  9,  GPIOE, 10,     140,   0,           0,   23170 /*45 deg = 0.707*/},
	/* 3:      bottom middle : Trig IO5, Echo IO6 */ {GPIOE, 11,  GPIOE, 12,     140,   0,           0,   32768 /*directly fwd  */}
};

void init_sonars()
{
	// Configure trigger & echo ports as outputs & inputs, respectively
	for(int i = 0; i < NUM_SONARS; i++)
	{
		sonar_cfgs[i].trig_port->MODER &= ~(0b11UL<<(sonar_cfgs[i].trig_bit*2));
		sonar_cfgs[i].trig_port->MODER |= 0b01UL<<(sonar_cfgs[i].trig_bit*2);
		sonar_cfgs[i].trig_port->OSPEEDR &= ~(0b11UL<<(sonar_cfgs[i].trig_bit*2));
		sonar_cfgs[i].trig_port->OSPEEDR |= 0b01UL<<(sonar_cfgs[i].trig_bit*2);
		sonar_cfgs[i].echo_port->MODER &= ~(0b11UL<<(sonar_cfgs[i].echo_bit*2));
	}
}

#define SONAR_FIFO_LEN 16 // 960ms worth of samples at 60 ms

xyc_t sonar_point_fifo[SONAR_FIFO_LEN];
int sonar_wr, sonar_rd;

xyc_t* get_sonar_point()
{
	if(sonar_wr == sonar_rd)
		return 0;

	xyc_t* ret = &sonar_point_fifo[sonar_rd];
	sonar_rd++; if(sonar_rd >= SONAR_FIFO_LEN) sonar_rd = 0;
	return ret;
}

void put_sonar_point(int32_t x, int32_t y, int8_t c)
{
	// overrun detection:
	// int next = sonar_wr+1; if(next >= SONAR_FIFO_LEN) next = 0;
	// if(next == sonar_rd)

	sonar_point_fifo[sonar_wr].x = x;
	sonar_point_fifo[sonar_wr].y = y;
	sonar_point_fifo[sonar_wr].c = c;

	dbg[0]++;
	dbg[1] = x;
	dbg[2] = y;

	sonar_wr++; if(sonar_wr >= SONAR_FIFO_LEN) sonar_wr = 0;
}

void process_sonar_point(int idx, int mm)
{
	mm = (mm * sonar_cfgs[idx].cos_up_angle_x32768) >> 15; // Distance to the obstacle projected to the floor plane.

	uint32_t angle = cur_pos.ang;
	int y_idx = (angle)>>SIN_LUT_SHIFT;
	int x_idx = (1073741824-angle)>>SIN_LUT_SHIFT;
	int sensor_x = cur_pos.x + (((int32_t)sin_lut[x_idx] * (int32_t)(sonar_cfgs[idx].x_offs))>>15);
	int sensor_y = cur_pos.y + (((int32_t)sin_lut[y_idx] * (int32_t)(sonar_cfgs[idx].y_offs))>>15);

	angle += sonar_cfgs[idx].side_angle;
	y_idx = (angle)>>SIN_LUT_SHIFT;
	x_idx = (1073741824-angle)>>SIN_LUT_SHIFT;
	int x = sensor_x + (((int32_t)sin_lut[x_idx] * (int32_t)mm)>>15);
	int y = sensor_y + (((int32_t)sin_lut[y_idx] * (int32_t)mm)>>15);

	// todo: classify based on approximated height (now 1)
	put_sonar_point(x, y, 1);
	// todo: give it to the obstacle avoidance
}



#define SONAR_INTERVAL 650  // unit: 0.1ms. 50ms provides 17.1m for unwanted reflections to die out - probably enough, given the "5m range" of the sensors.
                            // Minimum value limited by code: 350. Minimum suggested by the sonar datasheet: 600 (60 ms)

// Must be called at 10 kHz
void sonar_fsm_10k()
{
	static int cnt_sonar;
	static int echo_start_time;
	static int cur_sonar;
	cnt_sonar++;
	if(cnt_sonar == SONAR_INTERVAL-300)
	{       // Acquisition starts
		cur_sonar++; if(cur_sonar >= NUM_SONARS) cur_sonar = 0;
		SONAR_PULSE_ON(cur_sonar);
	}
	else if(cnt_sonar == SONAR_INTERVAL-300+1)
	{
		SONAR_PULSE_OFF(cur_sonar);  // a 100 us pulse is generated
		echo_start_time = 0;
	}
	else if(cnt_sonar > SONAR_INTERVAL+300) // 30000us pulse = 517 cm top limit
	{	// Acquisition ends.
		cnt_sonar = 0;
		//if(echo_start_time != -1) { we have no sample }
	}
	else if(cnt_sonar > SONAR_INTERVAL-300+1) // Wait for the echo signal
	{
		if(echo_start_time == 0 && SONAR_ECHO(cur_sonar))
			echo_start_time = cnt_sonar;
		else if(echo_start_time > 0 && !SONAR_ECHO(cur_sonar))
		{
			int mm = ((10000*(cnt_sonar-echo_start_time)))/583; // todo: temperature compensation
			echo_start_time = -1; // succesful reading
			process_sonar_point(cur_sonar, mm);
		}
	}
}


