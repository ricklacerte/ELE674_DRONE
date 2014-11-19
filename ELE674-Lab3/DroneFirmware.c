/*
 ============================================================================
 Name        : ARDrone.c
 Author      : Bruno De Kelper
 Version     :
 Copyright   : Your copyright notice
 Description : Hello World in C, Ansi-style
 ============================================================================
 */

#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <sys/time.h>
#include <pthread.h>
#include <semaphore.h>
#include <math.h>
#include <poll.h>
#include <string.h>

#include "Sensor.h"
#include "Motor.h"
#include "Attitude.h"
#include "Control.h"
#include "Mavlink.h"

#define PERIODE_uS 	5000
#define PERIODE_nS 	5000000L
#define MAX_PERIOD	1000000000L
#define MAIN_PERIOD	20


SensorRawData	RawData[NUM_SENSOR][DATABUFSIZE];
SensorData	 	NavData[NUM_SENSOR][DATABUFSIZE];
AttitudeData	AttitudeDesire, AttitudeMesure;
MotorStruct		Motor;
MavlinkStruct	Mavlink;
SensorParam	 	ParamData[NUM_SENSOR]   = { ACCEL_PARAM, GYRO_PARAM, SONAR_PARAM, BAROM_PARAM, MAGNETO_PARAM };
SensorStruct	SensorTab[NUM_SENSOR]   = { ACCEL_INIT, GYRO_INIT, SONAR_INIT, BAROM_INIT, MAGNETO_INIT };
AttitudeStruct	AttitudeTab[NUM_SENSOR] = { ACCEL_ATT_INIT, GYRO_ATT_INIT, SONAR_ATT_INIT, BAROM_ATT_INIT, MAGNETO_ATT_INIT };
ControlStruct	Control = CONTROL_INIT;

struct itimerspec	NewTimer, OldTimer;
timer_t				TimerID;
struct sigaction  	TimerSig, old_TimerSig;

//TIMERS
sem_t 	MainTimerSem;
sem_t 	MotorTimerSem;
sem_t 	ControlTimerSem;
sem_t	MavlinkReceiveTimerSem;
sem_t	MavlinkStatusTimerSem;

//pthread_mutex_t AttFinishMutex;
//int NbData;

uint8_t		MotorActivated = 0;
uint8_t 	SensorsActivated =0;
uint8_t		MavlinkActivated = 0;
uint8_t		ControlActivated = 0;




void SigTimerHandler (int signo) {
	static uint32_t  Period = 0;

	//ControlTimer
	if (ControlActivated) {
		if ((Period % CONTROL_PERIOD) == 0)
			sem_post(&ControlTimerSem);
	}

	//MavlinkTimer
	if (MavlinkActivated) {
		if ((Period % MAVLINK_RECEIVE_PERIOD) == 0)
			sem_post(&MavlinkReceiveTimerSem);
		if ((Period % MAVLINK_STATUS_PERIOD) == 0)
			sem_post(&MavlinkStatusTimerSem);
	}

	//MotorTimer
	if (MotorActivated){
		if ((Period % MOTOR_PERIOD) == 0)
			sem_post (&MotorTimerSem);
	}

	//MainTimer
	if ((Period % MAIN_PERIOD) == 0)
		sem_post (&MainTimerSem);

	//Period = 5ms
	Period = (Period + 1) % MAX_PERIOD;
}


int StartTimer (void) {
	struct sigevent	 Sig;
	int				 retval = 0;

	memset (&TimerSig, 0, sizeof(struct sigaction));
	TimerSig.sa_handler = SigTimerHandler;
	if ((retval = sigaction(SIGRTMIN, &TimerSig, &old_TimerSig)) != 0) {
		printf("%s : Problème avec sigaction : retval = %d\n", __FUNCTION__, retval);
		return retval;
	}
	Sig.sigev_signo  = SIGRTMIN;
	Sig.sigev_notify = SIGEV_SIGNAL;

	timer_create(CLOCK_MONOTONIC, &Sig, &TimerID);
	NewTimer.it_value.tv_sec     = PERIODE_nS / 1000000000L;
	NewTimer.it_value.tv_nsec	 = PERIODE_nS % 1000000000L;
	NewTimer.it_interval.tv_sec  = PERIODE_nS / 1000000000L;
	NewTimer.it_interval.tv_nsec = PERIODE_nS % 1000000000L;
	timer_settime(TimerID, 0, &NewTimer, &OldTimer);

	return retval;
}


int StopTimer (void) {
	int	 retval = 0;

	timer_settime(TimerID, 0, &OldTimer, NULL);
	timer_delete(TimerID);
	sigaction(SIGRTMIN, &old_TimerSig, NULL);

	return retval;
}


#include <ctype.h>    /* For tolower() function */
int getchar_nonblock(void) {
	struct termios oldt, newt;
	int    ch=-1;

	tcgetattr( STDIN_FILENO, &oldt );
	memcpy ((void *) &newt, (void *) &oldt, sizeof(struct termios));
	newt.c_lflag &= ~( ICANON | ECHO );
	tcsetattr( STDIN_FILENO, TCSANOW, &newt );
	fcntl(STDIN_FILENO, F_SETFL, O_NONBLOCK | O_NDELAY);
	ch = getchar();
   	fcntl(STDIN_FILENO, F_SETFL, 0);
	tcsetattr( STDIN_FILENO, TCSANOW, &oldt );

	return ch;
}


int main(int argc, char *argv[]) {
	struct sched_param	param;
	char	IPAddress[20] = {TARGET_IP};
	int		retval = 0;
	int		ch = 0;

	//Main Init
	param.sched_priority = sched_get_priority_min(POLICY);
	pthread_setschedparam(pthread_self(), POLICY, &param);
	sem_init(&MainTimerSem, 0, 0);

	//INITIALISATIONS
	if ((retval = MotorInit(&Motor)) < 0){
		printf("%s Initialisation Motor Failed!\n",__FUNCTION__);
		return EXIT_FAILURE;
	}
	if ((retval = SensorsInit(SensorTab)) < 0){
		printf("%s Initialisation Sensors Failed!\n",__FUNCTION__);
		return EXIT_FAILURE;
	}
	if ((retval = AttitudeInit(AttitudeTab)) < 0){
		printf("%s Initialisation Attitude Failed!\n",__FUNCTION__);
		return EXIT_FAILURE;
	}
	if ((retval = MavlinkInit(&Mavlink, &AttitudeDesire, &AttitudeMesure, IPAddress)) < 0){
		printf("%s Initialisation MavlinkInit Failed!\n",__FUNCTION__);
		return EXIT_FAILURE;
	}
	if ((retval = ControlInit(&Control)) < 0){
		printf("%s Initialisation ControlInit Failed!\n",__FUNCTION__);
		return EXIT_FAILURE;
	}
	printf("%s Tout initialise\n", __FUNCTION__);

	//DÉMARRAGE
	MotorStart();
	SensorsStart();
	AttitudeStart();
	MavlinkStart();
	ControlStart();
	StartTimer();
	printf("%s Tout demarre\n", __FUNCTION__);

	//MainTASK
	ch = 0;
	while (ch != 'q') {
		sem_wait(&MainTimerSem);
		ch = tolower(getchar_nonblock());
	}

	//STOP
	StopTimer();
	ControlStop(&Control);
	MotorStop(&Motor);
	MavlinkStop(&Mavlink);
	SensorsStop(SensorTab);
	AttitudeStop(AttitudeTab);
	printf("%s Tout arret\n", __FUNCTION__);

	sem_destroy(&MainTimerSem);
	return EXIT_SUCCESS;
}
