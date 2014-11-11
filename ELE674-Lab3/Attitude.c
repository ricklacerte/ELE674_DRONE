/*
 * Attitude.c
 *
 *  Created on: 12 sept. 2013
 *      Author: bruno
 */

#include "Attitude.h"
#include "Sensor.h"

pthread_barrier_t   AttitudeStartBarrier;

uint8_t  AttitudeActivated 	= 0;


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
	//modif*********** pour éviter partir Acceltask avant SensorTask
	//uint16_t		DataIdx[2] = {DATABUFSIZE, DATABUFSIZE-1};
	uint16_t		DataIdx[2] = {0, DATABUFSIZE};
	double 			Ts, Ka, Kb, Tau = Attitude->XYZTau;
	double			Mx, My;
	uint16_t		i;

	//printf("%s : %s prêt à démarrer\n", __FUNCTION__, Sensor->Name);
	pthread_barrier_wait(&AttitudeStartBarrier);
	//printf("%s : %s Démarrer\n", __FUNCTION__, Sensor->Name);

//EXECUTON
	while (AttitudeActivated) {

		//Si pas de NEW DATA -> SLEEP
		pthread_mutex_lock(&(Sensor->DataSampleMutex));

			while ((DataIdx[0] == Sensor->DataIdx)&&(AttitudeActivated != 0))
				pthread_cond_wait(&(Sensor->DataNewSampleCondVar), &(Sensor->DataSampleMutex));

			//exit
			if (AttitudeActivated == 0) {
				pthread_mutex_unlock(&(Sensor->DataSampleMutex));
			break;
			}
			//printf("%s: DataIdx[0]=%d, sensor->DataIdx=%d, AttitudeActivated=%d \n", __FUNCTION__,DataIdx[0],Sensor->DataIdx,AttitudeActivated);
			//printf("%s \n", __FUNCTION__);

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

		//Copie AttitudeMesure précédente ?
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
		  	  	  	  	  	  	printf("%s accel \n", __FUNCTION__);
			  	  	  	  	  	break;

		  case GYROSCOPE :		LocSpeed.Roll  = (XYZ[0][X_AXE]);
		  	  	  	  	  	  	LocSpeed.Pitch = -(XYZ[0][Y_AXE]);
		  	  	  	  	  	  	LocSpeed.Yaw   = -(XYZ[0][Z_AXE]);
		  	  	  	  	  	  	printf("%s gyro \n", __FUNCTION__);
	  	  	  	  				break;

		  case SONAR :			LocData.Elevation  = (XYZ[0][X_AXE]);
		  	  	  	  	  	  	LocSpeed.Elevation = ((XYZ[0][X_AXE])-(XYZ[1][X_AXE]))/Ts;
		  	  	  	  	  	  	printf("%s sonar \n", __FUNCTION__);
	  	  	  	  				break;

		  case BAROMETRE :		break;

		  case MAGNETOMETRE :	My = -XYZ[0][Z_AXE]*sin(LocData.Roll) + XYZ[0][X_AXE]*cos(LocData.Roll);
								Mx = -XYZ[0][Y_AXE]*cos(LocData.Pitch) - XYZ[0][X_AXE]*sin(LocData.Pitch)*sin(LocData.Roll) - XYZ[0][Z_AXE]*sin(LocData.Pitch)*cos(LocData.Roll);
								LocData.Yaw = atan2(My,Mx);
								printf("%s magnet \n", __FUNCTION__);
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
	printf("%s : %s Terminé\n", __FUNCTION__, Sensor->Name);
	pthread_exit(0); /* exit thread */
}



int AttitudeInit (AttitudeStruct AttitudeTab[NUM_SENSOR]) {
	pthread_attr_t		attr;
	struct sched_param	param;
	int					minprio, maxprio;
	int					i;

	pthread_barrier_init(&AttitudeStartBarrier, NULL, NUM_SENSOR+1);

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

	AttitudeActivated = 0;
	for (i = 0; i < NUM_SENSOR; i++) {
		Sensor = AttitudeTab[i].Sensor;
		pthread_cond_broadcast(&(Sensor->DataNewSampleCondVar));
		pthread_join(AttitudeTab[i].AttitudeThread, NULL);
	}

	return 0;
}

