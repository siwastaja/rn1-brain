#include "ext_include/stm32f2xx.h"

#include "sonar.h"

extern int latest_sonars[MAX_NUM_SONARS]; // in cm

void sonar_fsm()
{
	int i;
	static int cnt_sonar;
	static int sonar_times[MAX_NUM_SONARS];
	cnt_sonar++;
	if(cnt_sonar == 1000) // Sonar with 100ms intervals
	{
		SONAR_PULSE_ON();
		for(i=0; i<NUM_SONARS; i++)
			sonar_times[i] = 0;
	}
	else if(cnt_sonar == 1001)
	{
		SONAR_PULSE_OFF();
	}
	else if(cnt_sonar > 1000+300) // 30000us pulse = 517 cm top limit
	{
		cnt_sonar = 0;
		for(i=0; i<NUM_SONARS; i++)
			if(sonar_times[i] != -1)
				latest_sonars[i] = 0;
	}
	else if(cnt_sonar > 1001) // Wait for signals
	{
		if(sonar_times[0] == 0 && SONAR1_ECHO())
			sonar_times[0] = cnt_sonar;
		else if(sonar_times[0] > 0 && !SONAR1_ECHO())
		{
			latest_sonars[0] = ((100*(cnt_sonar-sonar_times[0]))+29/*rounding*/)/58;
			sonar_times[0] = -1;
		}
		if(sonar_times[1] == 0 && SONAR2_ECHO())
			sonar_times[1] = cnt_sonar;
		else if(sonar_times[1] > 0 && !SONAR2_ECHO())
		{
			latest_sonars[1] = ((100*(cnt_sonar-sonar_times[1]))+29/*rounding*/)/58;
			sonar_times[1] = -1;
		}
		if(sonar_times[2] == 0 && SONAR3_ECHO())
			sonar_times[2] = cnt_sonar;
		else if(sonar_times[2] > 0 && !SONAR3_ECHO())
		{
			latest_sonars[2] = ((100*(cnt_sonar-sonar_times[2]))+29/*rounding*/)/58;
			sonar_times[2] = -1;
		}
/*
		if(sonar_times[3] == 0 && SONAR4_ECHO())
			sonar_times[3] = cnt_sonar;
		else if(sonar_times[3] > 0 && !SONAR4_ECHO())
		{
			latest_sonars[3] = ((100*(cnt_sonar-sonar_times[3]))+29)/58;
			sonar_times[3] = -1;
		}
*/
	}


}

