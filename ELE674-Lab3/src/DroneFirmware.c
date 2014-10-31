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
#include <time.h>


//#include "Sensor.h"
#include "Motor.h"
//#include "Control.h"
//#include "Mavlink.h"

#define PERIODE_uS 	5000
#define PERIODE_nS 	5000000L

#define MAX_PERIOD	1000000000L
#define MAIN_PERIOD	20

//**************************************** VARIABLES GLOBALES **************************************************************
/*SensorRawData	RawData[NUM_SENSOR][DATABUFSIZE];
SensorData	 	NavData[NUM_SENSOR][DATABUFSIZE];
AttitudeData	AttitudeDesire, AttitudeMesure;
*/
MotorStruct		Motor;
/*MavlinkStruct	Mavlink;
SensorParam	 	ParamData[NUM_SENSOR]   = { ACCEL_PARAM, GYRO_PARAM, SONAR_PARAM, BAROM_PARAM, MAGNETO_PARAM };
SensorStruct	SensorTab[NUM_SENSOR]   = { ACCEL_INIT, GYRO_INIT, SONAR_INIT, BAROM_INIT, MAGNETO_INIT };
AttitudeStruct	AttitudeTab[NUM_SENSOR] = { ACCEL_ATT_INIT, GYRO_ATT_INIT, SONAR_ATT_INIT, BAROM_ATT_INIT, MAGNETO_ATT_INIT };
ControlStruct	Control = CONTROL_INIT;
*/
int		MavlinkActivated = 0;
int		MotorActivated = 0;
int		ControlActivated = 0;

struct itimerspec	NewTimer, OldTimer;
timer_t				TimerID;
struct sigaction  	TimerSig, old_TimerSig;           /* definition of signal action */

sem_t 	MainTimerSem;
sem_t 	MotorTimerSem;
sem_t 	ControlTimerSem;
sem_t	MavlinkReceiveTimerSem;
sem_t	MavlinkStatusTimerSem;

/* ******************************************* TIMER *********************************************************************/
void SigTimerHandler (int signo) {
/* Cette fonction est un "signal action" qui agit comme une interruption d'un timer.            */
/* Elle sert à créer un évènement périodique (5 ms) qui peut servir de base de temps            */
/* pour l'ensemble des Tâches du système.                                                       */

static uint32_t  Period = 0;

// Mavlink tempo
/*if (MavlinkActivated) {
		if ((Period % MAVLINK_RECEIVE_PERIOD) == 0)
			sem_post(&MavlinkReceiveTimerSem);
		if ((Period % MAVLINK_STATUS_PERIOD) == 0)
			sem_post(&MavlinkStatusTimerSem);
}*/

//Motor tempo
if (MotorActivated) {
	if ((Period % MOTOR_PERIOD) == 0)
		sem_post(&MotorTimerSem);
}

// Main tempo (pour les touches)
if ((Period % MAIN_PERIOD) == 0)
	sem_post (&MainTimerSem);

//Period = tranches de 5ms
Period = (Period + 1) % MAX_PERIOD;
}


int StartTimer (void) {
	struct 	sigevent Sig;
	int		retval = 0;

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

/************************************************** KEYBOARD ***************************************************************/
// pour le clavier
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

/******************************************************* TASK INIT *******************************************************/
// main peut recevoir des arguments
int main(int argc, char *argv[]) {

// variables locales
	struct sched_param	param;
//	int		minprio, maxprio;
//	char	IPAddress[20] = {TARGET_IP};
	//int		i, j,
	int retval = 0;
	int		ch = 0;

// options démarrages (par argc)
	printf("Usage :\n");
	printf("	DroneFirmware <Option>\n");
	printf("	   Option :	Pas d'option -> Par défaut (Pas de Log, IP = 192.168.1.2)\n");
	printf("	   		    LogAccel     -> Log de l'Accéléromètre\n");
	printf("	   		    LogGyro      -> Log du Gyroscope\n");
	printf("	   		    LogMagneto   -> Log du Magnétomètre\n");
	printf("	   		    LogSonar     -> Log du Sonar\n");
	printf("	   		    LogBarom     -> Log du Baromètre\n");
	printf("	   		    LogAll       -> Log de tous les capteurs\n");
	printf("	   		    IP=<Adresse>  (ex. : IP=192.168.1.3)\n"); // IP vu par Qgroundcontrol
	printf("	Note : Les options peuvent être cumulées et dans n'importe quel ordre.\n");
	printf("		   La syntaxe des options doit être strictement respectée.\n");

/*	if (argc > 1) {
		for (i = 1; i < argc; i++) {
			if (strcmp(argv[i],"LogAccel") == 0)
				SensorTab[ACCELEROMETRE].DoLog = 1;
			if (strcmp(argv[i],"LogGyro") == 0)
				SensorTab[GYROSCOPE].DoLog = 1;
			if (strcmp(argv[i],"LogSonar") == 0)
				SensorTab[SONAR].DoLog = 1;
			if (strcmp(argv[i],"LogBarom") == 0)
				SensorTab[BAROMETRE].DoLog = 1;
			if (strcmp(argv[i],"LogMagneto") == 0)
				SensorTab[MAGNETOMETRE].DoLog = 1;
			if (strcmp(argv[i],"LogAll") == 0)
				for (j = ACCELEROMETRE; j <= MAGNETOMETRE; j++)
					SensorTab[j].DoLog = 1;
			if (strncmp ( argv[i], "IP=", 3) == 0)
				strcpy( IPAddress, &(argv[i][3]));
		}
	}
	printf("%s : IP = %s\n", __FUNCTION__, IPAddress);
	printf("%s ça démarre !!!\n", __FUNCTION__);
*/
// scheduler setup : la tâche courante = la priorité minimale->TASK INIT :la plus prioritaire
	param.sched_priority = sched_get_priority_min(POLICY);
	pthread_setschedparam(pthread_self(), POLICY, &param);


// initialisation des verrous
	//temporisations
	sem_init(&MainTimerSem, 0, 0);

	// Spinlock : AttitudeDesire.AttitudeLock
	/*if ((retval = pthread_spin_init(&(AttitudeDesire.AttitudeLock), 1)) < 0) {
		printf("%s : Impossible d'initialiser le spinlock (AttitudeDesiree.AttitudeLock): retval = %d\n", __FUNCTION__, retval);
		return -1; // exit thread
	}*/
	// Spinlock : AttitudeMesure.AttitudeLock
	/*if ((retval = pthread_spin_init(&(AttitudeMesure.AttitudeLock), 1)) < 0) {
		printf("%s : Impossible d'initialiser le spinlock (AttitudeDesiree.AttitudeLock): retval = %d\n", __FUNCTION__, retval);
		return -1; // exit thread
	}*/


//initialisations 
	if ((retval = MotorInit(&Motor)) < 0)
		return EXIT_FAILURE;
	/*if ((retval = SensorsLogsInit(SensorTab)) < 0)
		return EXIT_FAILURE;
	if ((retval = SensorsInit(SensorTab)) < 0)
		return EXIT_FAILURE;
	if ((retval = AttitudeInit(AttitudeTab)) < 0)
		return EXIT_FAILURE;
	if ((retval = MavlinkInit(&Mavlink, &AttitudeDesire, &AttitudeMesure, IPAddress)) < 0)
		return EXIT_FAILURE;
	if ((retval = ControlInit(&Control)) < 0)
		return EXIT_FAILURE;
	*/printf("%s Tout initialisé\n", __FUNCTION__);

	StartTimer();

// Démarrage des tâches
	MotorStart();
	//SensorsStart();
	//AttitudeStart();

	//SensorsLogsStart();

	//MavlinkStart();
	//ControlStart();

	printf("%s Tout démarré\n", __FUNCTION__);

	ch = 0;

//attend une touche du clavier
	while (ch != 'q') {
		sem_wait(&MainTimerSem);
		ch = tolower(getchar_nonblock());
	}

//q au clavier-> quit:
	//MavlinkStop(&Mavlink);
	//pthread_spin_destroy(&(AttitudeDesire.AttitudeLock));
	//pthread_spin_destroy(&(AttitudeMesure.AttitudeLock));

	//ControlStop(&Control);

	MotorStop(&Motor);
	//SensorsLogsStop(SensorTab);
	//SensorsStop(SensorTab);
	//AttitudeStop(AttitudeTab);

	StopTimer();

	printf("%s Tout arrêté\n", __FUNCTION__);

	//Destructions des Verrous
	sem_destroy(&MainTimerSem);
	sem_destroy(&MotorTimerSem);

	return EXIT_SUCCESS;
}
