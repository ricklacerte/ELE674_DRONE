/*
 * Sensor.c
 *
 *  Created on: 12 sept. 2013
 *      Author: bruno
 */

#include "Sensor.h"

#define ABS(x) (((x) < 0.0) ? -(x) : (x))
#define MAX_TOT_SAMPLE 1000

extern uint8_t  SensorsActivated;
pthread_barrier_t   SensorStartBarrier;
pthread_barrier_t   LogStartBarrier;
pthread_mutex_t 	Log_Mutex;
uint8_t  LogActivated  	  	= 0;
uint8_t  numLogOutput 	  	= 0;

void *SensorTask ( void *ptr ) {

//INITIALISATION
	SensorStruct *Sensor = ptr;
	double temp_data[3]={0,0,0};
	uint32_t TimeStamp_new, TimeStamp_old, TimeDelay;

	//Ouverture du pilote
	if((Sensor->File=open(Sensor->DevName, O_RDONLY))<0)
		printf("%s: erreur ouverture du pilote: %s",__FUNCTION__, Sensor->Name );

	//Ready to start!
	pthread_barrier_wait(&SensorStartBarrier);
	printf("%s: %s started!\n",__FUNCTION__, Sensor->Name );

//EXECUTION
	while (SensorsActivated) {

		//Lecture RawData (pilote)
		read(Sensor->File, &Sensor->RawData[(Sensor->DataIdx + DATABUFSIZE +1) % DATABUFSIZE], sizeof(SensorRawData));

		//Exit
		if(!SensorsActivated){
			break;
		}

		pthread_mutex_lock(&Sensor->DataSampleMutex);

		//incrémentation DataIdx
		Sensor->DataIdx = (Sensor->DataIdx + DATABUFSIZE +1) % DATABUFSIZE;

		//Conversion de RAWDATA
		temp_data[0] = ((double)Sensor->RawData[Sensor->DataIdx].data[0]- Sensor->Param->centerVal) * Sensor->Param->Conversion;
		temp_data[1] = ((double)Sensor->RawData[Sensor->DataIdx].data[1]- Sensor->Param->centerVal) * Sensor->Param->Conversion;
		temp_data[2] = ((double)Sensor->RawData[Sensor->DataIdx].data[2]- Sensor->Param->centerVal) * Sensor->Param->Conversion;
		TimeStamp_old =TimeStamp_new;
		TimeStamp_new = Sensor->RawData[Sensor->DataIdx].timestamp_s*1000000000L + Sensor->RawData[Sensor->DataIdx].timestamp_n;
		TimeDelay = TimeStamp_new-TimeStamp_old;

		//Copie des conversions vers DATA
		pthread_spin_lock(&Sensor->DataLock);
			Sensor->Data[Sensor->DataIdx].Data[0] = temp_data[0];
			Sensor->Data[Sensor->DataIdx].Data[1] = temp_data[1];
			Sensor->Data[Sensor->DataIdx].Data[2] = temp_data[2];
			Sensor->Data[Sensor->DataIdx].TimeDelay = TimeDelay;
		pthread_spin_unlock(&Sensor->DataLock);

		//Réveil : AttitudeTask
		pthread_mutex_unlock(&Sensor->DataSampleMutex);
		pthread_cond_signal(&Sensor->DataNewSampleCondVar);
		}

		//EXIT
		printf("%s: %s stopped!\n",__FUNCTION__, Sensor->Name );
		pthread_exit(0);
}

int SensorsInit (SensorStruct SensorTab[NUM_SENSOR]) {

	// Variables locales
	int retval;
	int i;
	pthread_attr_t SensorTaskattr;
	struct sched_param SchedParam;
	//int PRIO[5]={0};
	int minprio,maxprio;

	//barrière initialisation Sensors tasks
	pthread_barrier_init(&SensorStartBarrier, NULL, NUM_SENSOR+1);

	// Création attr pour  les Tasks
	pthread_attr_init(&SensorTaskattr);
	pthread_attr_setdetachstate(&SensorTaskattr,PTHREAD_CREATE_JOINABLE);
	pthread_attr_setinheritsched(&SensorTaskattr,PTHREAD_EXPLICIT_SCHED);
	pthread_attr_setscope(&SensorTaskattr, PTHREAD_SCOPE_SYSTEM);
	pthread_attr_setstacksize(&SensorTaskattr,THREADSTACK);
	pthread_attr_setschedpolicy(&SensorTaskattr, POLICY);
	minprio = sched_get_priority_min(POLICY);
	maxprio = sched_get_priority_max(POLICY);
	SchedParam.sched_priority = minprio + (maxprio - minprio)/2;
	pthread_attr_setschedparam(&SensorTaskattr,&SchedParam);

	//Initialisation des verrous
	for(i=0;i<NUM_SENSOR;i++){

		//DataLock
		if ((retval = pthread_spin_init(&SensorTab[i].DataLock, 1)) < 0) {
			printf("%s : Errueur initialisation verrous retval = %d\n", __FUNCTION__, retval);
			return retval;
		}

		//DataSampleMutex
		if((retval=pthread_mutex_init(&SensorTab[i].DataSampleMutex,NULL))<0){
			printf("%s : Errueur initialisation verrous retval = %d\n", __FUNCTION__, retval);
			return retval;
		}

		//DataNewSampleCondVar
		if((retval=pthread_cond_init(&SensorTab[i].DataNewSampleCondVar,NULL))<0){
			printf("%s : Errueur initialisation verrous retval = %d\n", __FUNCTION__, retval);
			return retval;
		}

		//Création de la Task
		if ((retval = pthread_create(&SensorTab[i].SensorThread,&SensorTaskattr,SensorTask,&SensorTab[i]))<0){
			printf("%s : Task[%d] abort, res=%d\n",__FUNCTION__,i,retval);
			return retval;
		}

	}

	//Destruction attr pour SensorTask
	pthread_attr_destroy(&SensorTaskattr);
	return retval;
};


int SensorsStart (void) {
	SensorsActivated=1;
	pthread_barrier_wait(&SensorStartBarrier);
	pthread_barrier_destroy(&SensorStartBarrier);
	return 0;
}


int SensorsStop (SensorStruct SensorTab[NUM_SENSOR]) {
	int i=0;
	int err=0;

	//STOP
	SensorsActivated=0;
	for(i=0;i<NUM_SENSOR;i++){

		//EXIT
		err = pthread_join(SensorTab[i].SensorThread,NULL);
		if (err) {
			printf("pthread_join(Sensors [%d]) : Erreur\n",i);
			return err;
		}
	}
	return 0;
}

/* Le code ci-dessous est un CADEAU !!!	*/
/* Ce code permet d'afficher dans la console les valeurs reçues des capteurs.               */
/* Évidemment, celà suppose que les acquisitions sur les capteurs fonctionnent correctement. */
/* Donc, dans un premier temps, ce code peut vous servir d'exemple ou de source d'idées.     */
/* Et dans un deuxième temps, peut vous servir pour valider ou vérifier vos acquisitions.    */
/*                                                                                           */
/* NOTE : Ce code suppose que les échantillons des capteurs sont placés dans un tampon       */
/*        circulaire de taille DATABUFSIZE, tant pour les données brutes (RawData) que       */
/*        les données converties (NavData) (voir ci-dessous)                                 */
void *SensorLogTask ( void *ptr ) {
	SensorStruct	*Sensor    = (SensorStruct *) ptr;
	uint16_t		*Idx       = &(Sensor->DataIdx);
	uint16_t		LocalIdx   = DATABUFSIZE;
	SensorData	 	*NavData   = NULL;
	SensorRawData	*RawData   = NULL;
	SensorRawData   tpRaw;
	SensorData 	    tpNav;
	double			norm;

	printf("%s : Log de %s prêt à démarrer\n", __FUNCTION__, Sensor->Name);
	pthread_barrier_wait(&(LogStartBarrier));

	while (LogActivated) {
		pthread_mutex_lock(&(Sensor->DataSampleMutex));
		while (LocalIdx == *Idx)
			pthread_cond_wait(&(Sensor->DataNewSampleCondVar), &(Sensor->DataSampleMutex));
	    pthread_mutex_unlock(&(Sensor->DataSampleMutex));

	   	//pthread_spin_lock(&(Sensor->DataLock));

    	NavData   = &(Sensor->Data[LocalIdx]);
    	RawData   = &(Sensor->RawData[LocalIdx]);
		memcpy((void *) &tpRaw, (void *) RawData, sizeof(SensorRawData));
		memcpy((void *) &tpNav, (void *) NavData, sizeof(SensorData));
	   	//pthread_spin_unlock(&(Sensor->DataLock));

	   	pthread_mutex_lock(&Log_Mutex);
		if (numLogOutput == 0)
			printf("Sensor  :     TimeStamp      SampleDelay  Status  SampleNum   Raw Sample Data  =>        Converted Sample Data               Norme\n");
		else switch (tpRaw.type) {
				case ACCELEROMETRE :	norm = sqrt(tpNav.Data[0]*tpNav.Data[0]+tpNav.Data[1]*tpNav.Data[1]+tpNav.Data[2]*tpNav.Data[2]);
										printf("Accel   : (%5u.%09d)-(0.%09u)  %2d     %8u   %04X  %04X  %04X  =>  %10.5lf  %10.5lf  %10.5lf  =  %10.5lf\n", tpRaw.timestamp_s, tpRaw.timestamp_n, tpNav.TimeDelay, tpRaw.status, tpRaw.ech_num, (uint16_t) tpRaw.data[0], (uint16_t) tpRaw.data[1], (uint16_t) tpRaw.data[2], tpNav.Data[0], tpNav.Data[1], tpNav.Data[2], norm);
										break;
				case GYROSCOPE :		norm = sqrt(tpNav.Data[0]*tpNav.Data[0]+tpNav.Data[1]*tpNav.Data[1]+tpNav.Data[2]*tpNav.Data[2]);
										printf("Gyro    : (%5u.%09d)-(0.%09u)  %2d     %8u   %04X  %04X  %04X  =>  %10.5lf  %10.5lf  %10.5lf  =  %10.5lf\n", tpRaw.timestamp_s, tpRaw.timestamp_n, tpNav.TimeDelay, tpRaw.status, tpRaw.ech_num, (uint16_t) tpRaw.data[0], (uint16_t) tpRaw.data[1], (uint16_t) tpRaw.data[2], tpNav.Data[0], tpNav.Data[1], tpNav.Data[2], norm);
										break;
				case SONAR :			printf("Sonar   : (%5u.%09d)-(0.%09u)  %2d     %8u   %04X              =>  %10.5lf\n", tpRaw.timestamp_s, tpRaw.timestamp_n, tpNav.TimeDelay, tpRaw.status, tpRaw.ech_num, (uint16_t) tpRaw.data[0], tpNav.Data[0]);
										break;
				case BAROMETRE :		printf("Barom   : (%5u.%09d)-(0.%09u)  %2d     %8u   %04X              =>  %10.5lf\n", tpRaw.timestamp_s, tpRaw.timestamp_n, tpNav.TimeDelay, tpRaw.status, tpRaw.ech_num, (uint16_t) tpRaw.data[0], tpNav.Data[0]);
										break;
				case MAGNETOMETRE :		norm = sqrt(tpNav.Data[0]*tpNav.Data[0]+tpNav.Data[1]*tpNav.Data[1]+tpNav.Data[2]*tpNav.Data[2]);
										printf("Magneto : (%5u.%09d)-(0.%09u)  %2d     %8u   %04X  %04X  %04X  =>  %10.5lf  %10.5lf  %10.5lf  =  %10.5lf\n", tpRaw.timestamp_s, tpRaw.timestamp_n, tpNav.TimeDelay, tpRaw.status, tpRaw.ech_num, (uint16_t) tpRaw.data[0], (uint16_t) tpRaw.data[1], (uint16_t) tpRaw.data[2], tpNav.Data[0], tpNav.Data[1], tpNav.Data[2], norm);
										break;
			 }
		LocalIdx = *Idx;
		numLogOutput++;
		if (numLogOutput > 20)
			numLogOutput = 0;
		pthread_mutex_unlock(&Log_Mutex);
	}

	printf("%s : %s Terminé\n", __FUNCTION__, Sensor->Name);

	pthread_exit(0);
}


int InitSensorLog (SensorStruct *Sensor) {
	pthread_attr_t		attr;
	struct sched_param	param;
	int					retval;

	pthread_attr_init(&attr);
	pthread_attr_setinheritsched(&attr, PTHREAD_EXPLICIT_SCHED);
	pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_JOINABLE);
	pthread_attr_setscope(&attr, PTHREAD_SCOPE_PROCESS);
	pthread_attr_setschedpolicy(&attr, POLICY);
	param.sched_priority = sched_get_priority_min(POLICY);
	pthread_attr_setstacksize(&attr, THREADSTACK);
	pthread_attr_setschedparam(&attr, &param);

	printf("Creating Log thread : %s\n", Sensor->Name);
	if ((retval = pthread_create(&(Sensor->LogThread), &attr, SensorLogTask, (void *) Sensor)) != 0)
		printf("%s : Impossible de créer Tâche Log de %s => retval = %d\n", __FUNCTION__, Sensor->Name, retval);


	pthread_attr_destroy(&attr);

	return 0;
}


int SensorsLogsInit (SensorStruct SensorTab[]) {
	int16_t	  i, numLog = 0;
	int16_t	  retval = 0;

	for (i = 0; i < NUM_SENSOR; i++) {
		if (SensorTab[i].DoLog == 1) {
			if ((retval = InitSensorLog(&SensorTab[i])) < 0) {
				printf("%s : Impossible d'initialiser log de %s => retval = %d\n", __FUNCTION__, SensorTab[i].Name, retval);
				return -1;
			}
			numLog++;
		}
	}
	pthread_barrier_init(&LogStartBarrier, NULL, numLog+1);
	pthread_mutex_init(&Log_Mutex, NULL);

	return 0;
};


int SensorsLogsStart (void) {
	LogActivated = 1;
	pthread_barrier_wait(&(LogStartBarrier));
	pthread_barrier_destroy(&LogStartBarrier);
	printf("%s NavLog démarré\n", __FUNCTION__);

	return 0;
};


int SensorsLogsStop (SensorStruct SensorTab[]) {
	int16_t	i;

	LogActivated = 0;
	for (i = 0; i < NUM_SENSOR; i++) {
		if (SensorTab[i].DoLog == 1) {
			pthread_join(SensorTab[i].LogThread, NULL);
			SensorTab[i].DoLog = 0;
		}
	}
	pthread_mutex_destroy(&Log_Mutex);

	return 0;
};


