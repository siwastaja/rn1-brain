#include "ext_include/stm32f2xx.h"

#include "feedbacks.h" // for robot position
#include "sonar.h"
#include "navig.h"
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

	z offset: positive up, 0 = floor level

	            +
                    ^  
	 	O   |   side_angle


	up_angle: positive = looks up


*/
	int x_offs;
	int y_offs;
	int z_offs;
	int32_t side_angle;
	int32_t up_angle;
} sonar_cfg_t;


#define SONAR_PULSE_ON(idx)  do{sonar_cfgs[(idx)].trig_port->BSRR = 1UL<<(sonar_cfgs[(idx)].trig_bit);}while(0);
#define SONAR_PULSE_OFF(idx) do{sonar_cfgs[(idx)].trig_port->BSRR = 1UL<<(sonar_cfgs[(idx)].trig_bit+16);}while(0);
#define SONAR_ECHO(idx) (sonar_cfgs[(idx)].echo_port->IDR & (1UL<<sonar_cfgs[(idx)].echo_bit))


sonar_cfg_t sonar_cfgs[NUM_SONARS] =
{                                                   //       TRIG      ECHO    offs:x     y     z      side_angle      up_angle
	/* 0: (front view) left  : Trig  A9, Echo A10 */ {GPIOC, 0,  GPIOC, 1,     140,  150,  135,    -5*ANG_1_DEG,   0           },
	/* 1: (front view) right : Trig I11, Echo I12 */ {GPIOA, 0,  GPIOA, 1,     140, -150,  135,     5*ANG_1_DEG,   0           },
	/* 2:         top middle : Trig IO3, Echo IO4 */ {GPIOE, 9,  GPIOE,10,     140,    0,  190,     0,             45*ANG_1_DEG},
	/* 3:      bottom middle : Trig IO5, Echo IO6 */ {GPIOE,11,  GPIOE,12,     140,    0,  120,     0,             0           }
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

sonar_xyz_t sonar_point_fifo[SONAR_FIFO_LEN];
int sonar_wr, sonar_rd;

sonar_xyz_t* get_sonar_point()
{
	if(sonar_wr == sonar_rd)
		return 0;

	sonar_xyz_t* ret = &sonar_point_fifo[sonar_rd];
	sonar_rd++; if(sonar_rd >= SONAR_FIFO_LEN) sonar_rd = 0;
	return ret;
}

void put_sonar_point(int32_t x, int32_t y, int16_t z, int8_t c)
{
	// overrun detection:
	// int next = sonar_wr+1; if(next >= SONAR_FIFO_LEN) next = 0;
	// if(next == sonar_rd)

	sonar_point_fifo[sonar_wr].x = x;
	sonar_point_fifo[sonar_wr].y = y;
	sonar_point_fifo[sonar_wr].z = z;
	sonar_point_fifo[sonar_wr].c = c;

	dbg[0]++;
	dbg[1] = x;
	dbg[2] = y;

	sonar_wr++; if(sonar_wr >= SONAR_FIFO_LEN) sonar_wr = 0;
}

void process_sonar_point(int idx, int mm)
{
	int sin_up_ang_idx = ((uint32_t)sonar_cfgs[idx].up_angle)>>SIN_LUT_SHIFT;
	int cos_up_ang_idx = (1073741824-(uint32_t)sonar_cfgs[idx].up_angle)>>SIN_LUT_SHIFT;

	uint32_t angle = cur_pos.ang;
	int y_idx = (angle)>>SIN_LUT_SHIFT;
	int x_idx = (1073741824-angle)>>SIN_LUT_SHIFT;
	int sensor_x = cur_pos.x + (((int32_t)sin_lut[x_idx] * (int32_t)(sonar_cfgs[idx].x_offs))>>15);
	int sensor_y = cur_pos.y + (((int32_t)sin_lut[y_idx] * (int32_t)(sonar_cfgs[idx].x_offs))>>15);
	uint32_t ang2 = angle + 90*ANG_1_DEG;
	y_idx = (ang2)>>SIN_LUT_SHIFT;
	x_idx = (1073741824-ang2)>>SIN_LUT_SHIFT;
	sensor_x += (((int32_t)sin_lut[x_idx] * (int32_t)(sonar_cfgs[idx].y_offs))>>15);
	sensor_y += (((int32_t)sin_lut[y_idx] * (int32_t)(sonar_cfgs[idx].y_offs))>>15);

	int sensor_z = sonar_cfgs[idx].z_offs;

	angle -= sonar_cfgs[idx].side_angle;
	y_idx = (angle)>>SIN_LUT_SHIFT;
	x_idx = (1073741824-angle)>>SIN_LUT_SHIFT;
	
	int x = sensor_x + (((int64_t)sin_lut[x_idx] * (int64_t)mm * (int64_t)sin_lut[cos_up_ang_idx])>>30);
	int y = sensor_y + (((int64_t)sin_lut[y_idx] * (int64_t)mm * (int64_t)sin_lut[cos_up_ang_idx])>>30);
	int z = sensor_z + (((int32_t)mm * (int32_t)sin_lut[sin_up_ang_idx])>>15);

	int8_t c;
	switch(idx)
	{
		case 0:
		case 1:
		case 3:
		{
			if(mm < 200) c = 3;
			else if(mm < 250) c = 2;
			else c = 1;
		}
		break;

		default:
			c = 3;
		break;
	}

	put_sonar_point(x, y, z, c);

	// Calculate in robot coord frame for micronavi:

	angle = sonar_cfgs[idx].side_angle;
	y_idx = (angle)>>SIN_LUT_SHIFT;
	x_idx = (1073741824-angle)>>SIN_LUT_SHIFT;
	
	x = sonar_cfgs[idx].x_offs + (((int64_t)sin_lut[x_idx] * (int64_t)mm * (int64_t)sin_lut[cos_up_ang_idx])>>30);
	y = sonar_cfgs[idx].y_offs + (((int64_t)sin_lut[y_idx] * (int64_t)mm * (int64_t)sin_lut[cos_up_ang_idx])>>30);
	z = sonar_cfgs[idx].z_offs + (((int32_t)mm * (int32_t)sin_lut[sin_up_ang_idx])>>15);

	micronavi_point_in(x, y, z, c-1);
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


