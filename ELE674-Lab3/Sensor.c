/*
 * Sensor.c
 *
 *  Created on: 12 sept. 2013
 *      Author: bruno
 */

#include "Sensor.h"

#define ABS(x) (((x) < 0.0) ? -(x) : (x))


#define MAX_TOT_SAMPLE 1000

//extern SensorStruct	SensorTab[NUM_SENSOR];

pthread_barrier_t   SensorStartBarrier;
pthread_barrier_t   LogStartBarrier;
pthread_barrier_t 	ConvStartBarrier;
pthread_mutex_t 	Log_Mutex;

extern uint8_t  SensorsActivated;
uint8_t ConvActivated;
uint8_t  LogActivated  	  	= 0;
uint8_t  numLogOutput 	  	= 0;



/*void *ReadTask ( void *ptr ) {
//INITIALISATION
	uint16_t LocalIdx=0;
	SensorStruct *Sensor = ptr;

	//Ouverture du pilote
	if((Sensor->File=open(Sensor->DevName, O_RDONLY))<0) // Read Bloquant ???, msg erreur = ??
		printf("%s: erreur ouverture du pilote: %s",__FUNCTION__, Sensor->Name );
	//Ready to start!
	pthread_barrier_wait(&SensorStartBarrier);

//EXECUTION
	while (SensorsActivated) {
		//tempo
		sem_wait(&Sensor->TimerSem);
		//printf("%s : %s\n",__FUNCTION__, Sensor->Name);

		LocalIdx=Sensor->DataIdx;

		//Lecture RawData (pilote)
		pthread_mutex_lock(&Sensor->DataSampleMutex);
		read(Sensor->File, &Sensor->RawData[LocalIdx], sizeof(SensorRawData));

		//log perso
		//printf("%s : %s =  %d  %d  %d \n",__FUNCTION__,Sensor->Name, Sensor->RawData[LocalIdx].data[0],Sensor->RawData[LocalIdx].data[1], Sensor->RawData[LocalIdx].data[2]);
		Sensor->DataIdx = (LocalIdx + DATABUFSIZE +1) % DATABUFSIZE;
		pthread_mutex_unlock(&Sensor->DataSampleMutex);

		//signal : nouvel échantillon arrivé
		pthread_cond_signal(&Sensor->DataNewSampleCondVar);
	}
//EXIT
	pthread_exit(0);
}*/


/*void *ConversionTask ( void *ptr ) {

//INITIALISATION
	SensorStruct *Sensor = ptr;
	uint16_t LocalIdx=0;
	int16_t temp_rawdata[3]={0,0,0};
	double temp_data[3]={0,0,0};

	//Ready to start!
	pthread_barrier_wait(&SensorStartBarrier);

//EXECUTION

	while (SensorsActivated) {

		//capture MUTEX (condition)
		pthread_mutex_lock(&Sensor->DataSampleMutex);

		//Vérifie sa condition (nouveau no_echantillon)

		while(!(Sensor->DataIdx != LocalIdx)){
			pthread_cond_wait(&Sensor->DataNewSampleCondVar,&Sensor->DataSampleMutex);
		}
		// Boucle if pour vérification: ((Sensor->RawData->type == Sensor->type) & (Sensor->RawData->status == NEW_SAMPLE))){ //& ((ech_num  == Sensor->RawData->ech_num-1) | (ech_num  == 0 )))){
		//printf("%s : %s\n",__FUNCTION__, Sensor->Name);
		pthread_spin_lock(&Sensor->DataLock);
			temp_rawdata[0]=Sensor->RawData[LocalIdx].data[0];
			temp_rawdata[1]=Sensor->RawData[LocalIdx].data[1];
			temp_rawdata[2]=Sensor->RawData[LocalIdx].data[2];
		pthread_spin_unlock(&Sensor->DataLock);

		temp_data[0] = ((double)temp_rawdata[0]- Sensor->Param->centerVal) * Sensor->Param->Conversion;
		temp_data[1] = ((double)temp_rawdata[1]- Sensor->Param->centerVal) * Sensor->Param->Conversion;
		temp_data[2] = ((double)temp_rawdata[2]- Sensor->Param->centerVal) * Sensor->Param->Conversion;

		pthread_spin_lock(&Sensor->DataLock);
			Sensor->Data[LocalIdx].Data[0] = temp_data[0];
			Sensor->Data[LocalIdx].Data[1] = temp_data[1];
			Sensor->Data[LocalIdx].Data[2] = temp_data[2];
		pthread_spin_unlock(&Sensor->DataLock);

		//calcul du temps entre
		Sensor->Data->TimeDelay = Sensor->RawData->timestamp_n - Sensor->RawData->timestamp_s;
		//fonction de log perso
		//printf("%s : %s = %d %10.5lf  %10.5lf  %10.5lf \n",__FUNCTION__,Sensor->Name, Sensor->Data[LocalIdx].TimeDelay, temp_data[0],temp_data[1],temp_data[2]);
		//incrémentation des buffers index
		LocalIdx=Sensor->DataIdx;

		//fin de sa conversion : libère MUTEX
		pthread_mutex_unlock(&Sensor->DataSampleMutex);
	}
//EXIT
	pthread_exit(0);
}*/

void *SensorTask ( void *ptr ) {

	//INITIALISATION
		//uint16_t LocalIdx=0;
		SensorStruct *Sensor = ptr;
		double temp_data[3]={0,0,0};

		//Ouverture du pilote
		if((Sensor->File=open(Sensor->DevName, O_RDONLY))<0)
			printf("%s: erreur ouverture du pilote: %s",__FUNCTION__, Sensor->Name );


		//Ready to start!
		pthread_barrier_wait(&SensorStartBarrier);

	//EXECUTION
		while (SensorsActivated) {
			//tempo
			sem_wait(&Sensor->TimerSem);

			printf("%s : %s\n",__FUNCTION__, Sensor->Name);

			//Lecture RawData (pilote)
			read(Sensor->File, &Sensor->RawData[Sensor->DataIdx], sizeof(SensorRawData));

			//log perso
			//printf("%s : %s = %d %d %d  %d  %d  \n",__FUNCTION__,Sensor->Name,Sensor->RawData[Sensor->DataIdx].ech_num,Sensor->RawData[Sensor->DataIdx].status, Sensor->RawData[Sensor->DataIdx].data[0],Sensor->RawData[Sensor->DataIdx].data[1], Sensor->RawData[Sensor->DataIdx].data[2]);

			// Boucle if pour vérification: ((Sensor->RawData->type == Sensor->type) & (Sensor->RawData->status == NEW_SAMPLE))){ //& ((ech_num  == Sensor->RawData->ech_num-1) | (ech_num  == 0 )))){
			//printf("%s : %s\n",__FUNCTION__, Sensor->Name);

			//Conversion de RAWDATA
			temp_data[0] = ((double)Sensor->RawData[Sensor->DataIdx].data[0]- Sensor->Param->centerVal) * Sensor->Param->Conversion;
			temp_data[1] = ((double)Sensor->RawData[Sensor->DataIdx].data[1]- Sensor->Param->centerVal) * Sensor->Param->Conversion;
			temp_data[2] = ((double)Sensor->RawData[Sensor->DataIdx].data[2]- Sensor->Param->centerVal) * Sensor->Param->Conversion;

			//Copie des conversions vers DATA
			pthread_spin_lock(&Sensor->DataLock);
				Sensor->Data[Sensor->DataIdx].Data[0] = temp_data[0];
				Sensor->Data[Sensor->DataIdx].Data[1] = temp_data[1];
				Sensor->Data[Sensor->DataIdx].Data[2] = temp_data[2];
				Sensor->Data->TimeDelay = Sensor->RawData->timestamp_n - Sensor->RawData->timestamp_s;
			pthread_spin_unlock(&Sensor->DataLock);

			//fonction de log perso
			//printf("%s : %s = %f %10.5lf  %10.5lf  %10.5lf \n",__FUNCTION__,Sensor->Name, Sensor->Data[Sensor->DataIdx].TimeDelay, temp_data[0],temp_data[1],temp_data[2]);

			//incrémentation DataIdx
			pthread_mutex_lock(&Sensor->DataSampleMutex);
				Sensor->DataIdx = (Sensor->DataIdx + DATABUFSIZE +1) % DATABUFSIZE;
			pthread_mutex_unlock(&Sensor->DataSampleMutex);

			//Réveil : AttitudeTask
			pthread_cond_signal(&Sensor->DataNewSampleCondVar);
			}

		//EXIT
			pthread_exit(0);

}

int SensorsInit (SensorStruct SensorTab[NUM_SENSOR]) {
/* si les priorités restent tous égales, on peut retirer PRIO[i]*/

	// Variables locales
	int retval;
	int i;
	pthread_attr_t SensorTaskattr;
	struct sched_param SchedParam;
	int PRIO[5]={0};

	//barrière initialisation Sensors tasks
	pthread_barrier_init(&SensorStartBarrier, NULL, NUM_SENSOR+1);

	//Définition : Priorité
	PRIO[ACCELEROMETRE]=30;
	PRIO[GYROSCOPE]=30;
	PRIO[SONAR]=30;
	PRIO[BAROMETRE]=30;
	PRIO[MAGNETOMETRE]=30;

	// Création attr pour  les Tasks
	pthread_attr_init(&SensorTaskattr);
	pthread_attr_setdetachstate(&SensorTaskattr,PTHREAD_CREATE_JOINABLE);
	pthread_attr_setinheritsched(&SensorTaskattr,PTHREAD_EXPLICIT_SCHED);
	pthread_attr_setscope(&SensorTaskattr, PTHREAD_SCOPE_SYSTEM);
	pthread_attr_setstacksize(&SensorTaskattr,THREADSTACK);
	pthread_attr_setschedpolicy(&SensorTaskattr, POLICY);

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

		//DataSem
		if((retval=sem_init(&SensorTab[i].DataSem,1,0))<0){
			printf("%s : Errueur initialisation verrous retval = %d\n", __FUNCTION__, retval);
			return retval;
		}

		//TimerSem
		if((retval=sem_init(&SensorTab[i].TimerSem,1,0))<0){
			printf("%s : Errueur initialisation verrous retval = %d\n", __FUNCTION__, retval);
			return retval;
		}

		//DataNewSampleCondVar
		if((retval=pthread_cond_init(&SensorTab[i].DataNewSampleCondVar,NULL))<0){
			printf("%s : Errueur initialisation verrous retval = %d\n", __FUNCTION__, retval);
			return retval;
		}

		//Création de la Task
		SchedParam.sched_priority=PRIO[i];
		pthread_attr_setschedparam(&SensorTaskattr,&SchedParam);
		if ((retval = pthread_create(&SensorTab[i].SensorThread,&SensorTaskattr,SensorTask,&SensorTab[i]))<0){
			printf("%s : Task[%d] abort, res=%d\n",__FUNCTION__,i,retval);
			return retval;
		}

	}



/*//ACCELERATION:
	SchedParam.sched_priority=PRIO[ACCELEROMETRE];
	pthread_attr_setschedparam(&SensorTaskattr,&SchedParam);
	if ((retval = pthread_create(&SensorTab[ACCELEROMETRE].SensorThread,&SensorTaskattr,SensorTask,&SensorTab[ACCELEROMETRE]))<0){
		printf("%s : AccelTask abort, res=%d\n",__FUNCTION__,retval);
		return retval;
	}


//GYROSCOPE
	SchedParam.sched_priority=PRIO[GYROSCOPE];
	pthread_attr_setschedparam(&SensorTaskattr,&SchedParam);
	if ((retval = pthread_create(&SensorTab[GYROSCOPE].SensorThread,&SensorTaskattr,SensorTask,&SensorTab[GYROSCOPE]))<0){
		printf("%s : GyroTask abort, res=%d\n",__FUNCTION__,retval);
		return retval;
	}

//SONAR
	SchedParam.sched_priority=PRIO[SONAR];
	pthread_attr_setschedparam(&SensorTaskattr,&SchedParam);
	if ((retval = pthread_create(&SensorTab[SONAR].SensorThread,&SensorTaskattr,SensorTask,&SensorTab[SONAR]))<0){
		printf("%s : SonarTask abort, res=%d\n",__FUNCTION__,retval);
		return retval;
	}

//BAROMETRE
	SchedParam.sched_priority=PRIO[BAROMETRE];
	pthread_attr_setschedparam(&SensorTaskattr,&SchedParam);
	if ((retval = pthread_create(&SensorTab[BAROMETRE].SensorThread,&SensorTaskattr,SensorTask,&SensorTab[BAROMETRE]))<0){
		printf("%s : BaretoTask abort, res=%d\n",__FUNCTION__,retval);
		return retval;
	}


//MAGNETOMETRE
	SchedParam.sched_priority=PRIO[MAGNETOMETRE];
	pthread_attr_setschedparam(&SensorTaskattr,&SchedParam);
	if ((retval = pthread_create(&SensorTab[MAGNETOMETRE].SensorThread,&SensorTaskattr,SensorTask,&SensorTab[MAGNETOMETRE]))<0){
		printf("%s : MagnetoTask abort, res=%d\n",__FUNCTION__,retval);
		return retval;
	}
*/
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

	SensorsActivated=0;
	for(i=0;i<NUM_SENSOR;i++){
		//attendre que la tâche -> exit
		pthread_join(SensorTab[i].SensorThread,NULL);

		//détruire les verrous
		pthread_spin_destroy(&SensorTab[i].DataLock);
		pthread_cond_destroy(&SensorTab[i].DataNewSampleCondVar);
		pthread_mutex_destroy(&SensorTab[i].DataSampleMutex);
		sem_destroy(&SensorTab[i].TimerSem);
		sem_destroy(&SensorTab[i].DataSem);
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

	   	pthread_spin_lock(&(Sensor->DataLock));
    	NavData   = &(Sensor->Data[LocalIdx]);
    	RawData   = &(Sensor->RawData[LocalIdx]);
		memcpy((void *) &tpRaw, (void *) RawData, sizeof(SensorRawData));
		memcpy((void *) &tpNav, (void *) NavData, sizeof(SensorData));
	   	pthread_spin_unlock(&(Sensor->DataLock));

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

	pthread_exit(0); /* exit thread */
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


