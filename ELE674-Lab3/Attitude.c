/*
 * Attitude.c
 *
 *  Created on: 12 sept. 2013
 *      Author: bruno
 */

#include "Attitude.h"
#include "Sensor.h"

pthread_barrier_t   	AttitudeStartBarrier;
uint8_t 			 	AttitudeActivated 	= 0;
extern AttitudeData 	AttitudeDesire, AttitudeMesure;


/*********************************************************************************/
/* La Tâche ci-dessous s'occupe de convertir les données des capteurs en valeurs */
/* d'attitude (angles d'Euler) qui servira ensuite dans la loi de commande.      */
/* Ce sont aussi les valeurs d'attitude qui sont transmises à Mavlink.           */
/*********************************************************************************/
void *AttitudeTask ( void *ptr ) {
// Variables Locales
	AttitudeStruct	*Attitude = (AttitudeStruct *) ptr;
	SensorStruct	*Sensor   = Attitude->Sensor;
	AttitudeData	*AttitudeMesure = Attitude->AttitudeMesure;
	AttData			LocData, LocSpeed;
	double			SensorData[2][3] = {{0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}};
	double			XYZ[2][3] = {{0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}};
	uint32_t		timestamp_s, timestamp_n;
	uint16_t		DataIdx[2] = {DATABUFSIZE, DATABUFSIZE-1};
	double 			Ts, Ka, Kb, Tau = Attitude->XYZTau;
	double			Mx, My;
	uint16_t		i;


	pthread_barrier_wait(&AttitudeStartBarrier);
	printf("%s: %s started!\n",__FUNCTION__, Sensor->Name );

//EXECUTON
	while (AttitudeActivated) {

		//NOTHING NEW -> SLEEP
		pthread_mutex_lock(&(Sensor->DataSampleMutex));
			while ((DataIdx[0] == Sensor->DataIdx)&&(AttitudeActivated != 0))
				pthread_cond_wait(&(Sensor->DataNewSampleCondVar), &(Sensor->DataSampleMutex));

			//exit
			if (AttitudeActivated == 0) {
				pthread_mutex_unlock(&(Sensor->DataSampleMutex));
			break;
			}

			//Copie DATA Sensor (from SensorTask): CURRENT et PRÉCÉDENT
			pthread_spin_lock(&(Sensor->DataLock));
				DataIdx[0]    = Sensor->DataIdx;
				DataIdx[1] = (DataIdx[0] + DATABUFSIZE - 1) % DATABUFSIZE;
				memcpy((void *) &(SensorData[0][0]), (void *) (Sensor->Data[DataIdx[0]].Data), 3*sizeof(double));
				memcpy((void *) &(SensorData[1][0]), (void *) (Sensor->Data[DataIdx[1]].Data), 3*sizeof(double));
				timestamp_s = Sensor->RawData[DataIdx[0]].timestamp_s;
				timestamp_n = Sensor->RawData[DataIdx[0]].timestamp_n;
				Ts	= ((double) (Sensor->Data[DataIdx[0]].TimeDelay))/1000000000.0;
			pthread_spin_unlock(&(Sensor->DataLock));
		pthread_mutex_unlock(&(Sensor->DataSampleMutex));

		//Copie AttitudeMesure
		pthread_spin_lock(&(AttitudeMesure->AttitudeLock));
			memcpy((void *) &LocData, (void *) &(Attitude->AttitudeMesure->Data), sizeof(AttData));
			memcpy((void *) &LocSpeed, (void *) &(Attitude->AttitudeMesure->Speed), sizeof(AttData));
		pthread_spin_unlock(&(AttitudeMesure->AttitudeLock));

		// Calcul de constantes
	  	Ka = ((2.0*Tau - Ts)/(2.0*Tau + Ts));
	  	Kb = (Ts/(2.0*Tau + Ts));

	  	memcpy((void *) &(XYZ[1][0]), (void *) &(XYZ[0][0]), 3*sizeof(double)); //??
	  	for (i = X_AXE; i <= Z_AXE; i++) {
	  		XYZ[0][i]	= Ka*XYZ[1][i] + Kb*(SensorData[0][i] + SensorData[1][i]);
	  		if ((Sensor->type == SONAR)||(Sensor->type == BAROMETRE))
	  			break;
	  	}

	  	//Calcul: attitude
		switch (Sensor->type) {
		  case ACCELEROMETRE :	LocData.Roll  = atan2(XYZ[0][Y_AXE], XYZ[0][Z_AXE]);
		  	  	  	  	  	  	LocData.Pitch = atan(XYZ[0][X_AXE]/(XYZ[0][Y_AXE]*sin(LocData.Roll)+XYZ[0][Z_AXE]*cos(LocData.Roll)));
			  	  	  	  	  	break;

		  case GYROSCOPE :		LocSpeed.Roll  = (XYZ[0][X_AXE]);
		  	  	  	  	  	  	LocSpeed.Pitch = -(XYZ[0][Y_AXE]);
		  	  	  	  	  	  	LocSpeed.Yaw   = -(XYZ[0][Z_AXE]);
	  	  	  	  				break;

		  case SONAR :			LocData.Elevation  = (XYZ[0][X_AXE]);
		  	  	  	  	  	  	LocSpeed.Elevation = ((XYZ[0][X_AXE])-(XYZ[1][X_AXE]))/Ts;
	  	  	  	  				break;

		  case BAROMETRE :		break;

		  case MAGNETOMETRE :	My = -XYZ[0][Z_AXE]*sin(LocData.Roll) + XYZ[0][X_AXE]*cos(LocData.Roll);
								Mx = -XYZ[0][Y_AXE]*cos(LocData.Pitch) - XYZ[0][X_AXE]*sin(LocData.Pitch)*sin(LocData.Roll) - XYZ[0][Z_AXE]*sin(LocData.Pitch)*cos(LocData.Roll);
								LocData.Yaw = atan2(My,Mx);
  								break;
		}

		//Copie: New values -> AttitudeMesure
		pthread_spin_lock(&(AttitudeMesure->AttitudeLock));
			memcpy((void *) &(Attitude->AttitudeMesure->Data), (void *) &LocData, sizeof(AttData));
			memcpy((void *) &(Attitude->AttitudeMesure->Speed), (void *) &LocSpeed, sizeof(AttData));
			AttitudeMesure->timestamp_s = timestamp_s;
			AttitudeMesure->timestamp_n = timestamp_n;
		pthread_spin_unlock(&(AttitudeMesure->AttitudeLock));
	}

	//EXIT
	printf("%s : %s Termine\n", __FUNCTION__, Sensor->Name);
	pthread_exit(0);
}



int AttitudeInit (AttitudeStruct AttitudeTab[NUM_SENSOR]) {
	pthread_attr_t		attr;
	struct sched_param	param;
	int					minprio, maxprio;
	int					i;
	int retval;

	pthread_barrier_init(&AttitudeStartBarrier, NULL, NUM_SENSOR+1);

	//TASKS
	pthread_attr_init(&attr);
	pthread_attr_setinheritsched(&attr, PTHREAD_EXPLICIT_SCHED);
	pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_JOINABLE);
	pthread_attr_setscope(&attr, PTHREAD_SCOPE_SYSTEM);
	minprio = sched_get_priority_min(POLICY);
	maxprio = sched_get_priority_max(POLICY);
	pthread_attr_setschedpolicy(&attr, POLICY);
	param.sched_priority = minprio + (maxprio - minprio)/2;
	pthread_attr_setstacksize(&attr, THREADSTACK);
	pthread_attr_setschedparam(&attr, &param);
	for (i = 0; i < NUM_SENSOR; i++) {
		pthread_create(&(AttitudeTab[i].AttitudeThread), &attr, AttitudeTask, (void *) &(AttitudeTab[i]));
	}
	pthread_attr_destroy(&attr);

	//Verrous : struct Attitude
	if ((retval = pthread_spin_init(&(AttitudeDesire.AttitudeLock), 1)) < 0) {
		printf("%s : Impossible d'initialiser le spinlock (AttitudeDesiree.AttitudeLock): retval = %d\n", __FUNCTION__, retval);
		return -1;
	}

	if ((retval = pthread_spin_init(&(AttitudeMesure.AttitudeLock), 1)) < 0) {
		printf("%s : Impossible d'initialiser le spinlock (AttitudeDesiree.AttitudeLock): retval = %d\n", __FUNCTION__, retval);
		return -1;
	}
	return 0;
}

int AttitudeStart (void) {
	AttitudeActivated = 1;
	pthread_barrier_wait(&(AttitudeStartBarrier));
	pthread_barrier_destroy(&AttitudeStartBarrier);
	printf("%s Attitude démarré\n", __FUNCTION__);
	return 0;
}


int AttitudeStop (AttitudeStruct AttitudeTab[NUM_SENSOR]) {
	SensorStruct *Sensor;
	int i;

	//STOP
	AttitudeActivated = 0;
	for (i = 0; i < NUM_SENSOR; i++) {
		Sensor = AttitudeTab[i].Sensor;
		pthread_cond_broadcast(&(Sensor->DataNewSampleCondVar));

		//EXIT
		pthread_join(AttitudeTab[i].AttitudeThread, NULL);

		//Destructions des Verrous
		pthread_cond_destroy(&(Sensor->DataNewSampleCondVar));
		pthread_mutex_destroy(&(Sensor->DataSampleMutex));

		//struct Data
		pthread_spin_destroy(&(Sensor->DataLock));
	}

	//struct AttitudeMesuree
	pthread_spin_destroy(&(AttitudeTab->AttitudeMesure->AttitudeLock));

	return 0;
}

