/*
 * Motor.c
 *
 *  Created on: 12 sept. 2013
 *      Author: bruno
 */


#include "Motor.h"

#define TimeDelay 50000

extern sem_t		MotorTimerSem;
extern int			MotorActivated;

pthread_barrier_t 	MotorStartBarrier;


int gpio_set (int nr, int val)  {
	char cmdline[200];

	if (val < 0)
		sprintf(cmdline, "/usr/sbin/gpio %d -d i", nr);
	else if (val > 0)
		sprintf(cmdline, "/usr/sbin/gpio %d -d ho 1", nr);
	else
		sprintf(cmdline, "/usr/sbin/gpio %d -d ho 0", nr);

	return system(cmdline);
}


int motor_open(void) {
	struct termios config;
	int uart = open(MOTOR_UART, O_RDWR | O_NOCTTY | O_NDELAY);

	if (uart < 0) {
		printf("motor_open : impossible d'ouvrir le uart du moteur\n");
		return uart;
	}

	fcntl(uart, F_SETFL, 0); //read calls are non blocking

	//set port config
	tcgetattr(uart, &config);
	cfsetspeed(&config, B115200);
	config.c_cflag |= (CLOCAL | CREAD); //Enable the receiver and set local mode
	config.c_iflag = 0; //clear input config
	config.c_lflag = 0; //clear local config
	config.c_oflag &= ~OPOST; //clear output config (raw output)
	cfmakeraw(&config);
	tcsetattr(uart, TCSANOW, &config);
	return uart;
}

int motor_cmd(int file, uint8_t cmd, uint8_t *reply, int replylen) {
	int size;

	write(file, &cmd, 1);
	fsync(file);
	usleep(TimeDelay);
	size = read(file, reply, replylen);

	return size;
}

int MotorPortInit(MotorStruct *Motor) {
	uint8_t reply[256];
	int		i;//motor_cmd(Motor->file,)

	//open motor port
	Motor->file = motor_open();
	if (Motor->file < 0) {
		printf("motor_open: Impossible d'ouvrir le UART\n");
		return Motor->file;
	}
	//reset IRQ flipflop - this code resets GPIO_ERROR_READ to 0
	gpio_set(GPIO_ERROR_RESET, 0);
	usleep(2*TimeDelay);

	//all select lines inactive
	gpio_set(GPIO_M1, 0);
	gpio_set(GPIO_M2, 0);
	gpio_set(GPIO_M3, 0);
	gpio_set(GPIO_M4, 0);
	usleep(2*TimeDelay);

	//configure motors
	for (i = 0; i < 4; ++i) {
		gpio_set(GPIO_M1 + i, -1);
		usleep(2*TimeDelay);
		motor_cmd(Motor->file, 0xE0, reply, 2);
		motor_cmd(Motor->file, 0x91, reply, 121);
		motor_cmd(Motor->file, 0xA1, reply, 2);
		motor_cmd(Motor->file, i + 1, reply, 1);
		motor_cmd(Motor->file, 0x40, reply, 2);
		gpio_set(GPIO_M1 + i ,0);
		usleep(2*TimeDelay);
	}

	//all select lines active
	gpio_set(GPIO_M1, -1);
	gpio_set(GPIO_M2, -1);
	gpio_set(GPIO_M3, -1);
	gpio_set(GPIO_M4, -1);
	usleep(2*TimeDelay);

	gpio_set(GPIO_ERROR_READ, -1);
	usleep(2*TimeDelay);

	return 0;
}


void motor_send(MotorStruct *Motor, int SendMode){
/* Fonction utilitaire pour simplifier les transmissions aux moteurs */

	uint8_t 	trame_pwm[5]={0,0,0,0,0};
	uint16_t  	trame_led;
	uint8_t		trame_pwmled[7];
	int 		res;
	uint16_t	tmp_pwm[4];   //motor speed 0x00-0x1ff
	uint16_t	tmp_led[4];

	//Copie des valeurs PWM
	pthread_mutex_lock(&Motor->MotorMutex);
		tmp_pwm[0]=Motor->pwm[0];
		tmp_pwm[1]=Motor->pwm[1];
		tmp_pwm[2]=Motor->pwm[2];
		tmp_pwm[3]=Motor->pwm[3];
		tmp_led[0]=Motor->led[0];
		tmp_led[1]=Motor->led[1];
		tmp_led[2]=Motor->led[2];
		tmp_led[3]=Motor->led[3];
	pthread_mutex_unlock(&Motor->MotorMutex);

	//printf("%d %d %d %d \n",tmp_pwm[0],tmp_pwm[1],tmp_pwm[2],tmp_pwm[3]);

	switch (SendMode) {
	case MOTOR_NONE :
		break;

	case MOTOR_PWM_ONLY :

		//Construction de la trame PWM
		trame_pwm[0]=(		(CMD_PWM & 0x07) << 5 	| (tmp_pwm[0]&MASK_9BITS) >> 4);
		trame_pwm[1]=((tmp_pwm[0]&MASK_9BITS) << 4 	| (tmp_pwm[1]&MASK_9BITS) >> 5);
		trame_pwm[2]=((tmp_pwm[1]&MASK_9BITS) << 3 	| (tmp_pwm[2]&MASK_9BITS) >> 6);
		trame_pwm[3]=((tmp_pwm[2]&MASK_9BITS) << 2 	| (tmp_pwm[3]&MASK_9BITS) >> 7);
		trame_pwm[4]= (tmp_pwm[3]&MASK_9BITS) << 1;

		//envoie de la trame au moteur
			res=write(Motor->file,trame_pwm,5);
			fsync(Motor->file);


		if(res<0)
			printf("Erreur lors de l'envoi au pilote moteur(PWM)! res=%d \n",res);

		break;

	case MOTOR_LED_ONLY :
		//Construction de la trame LED
		trame_led=(CMD_LED << 13)	| (tmp_led[0])<<1 | (tmp_led[1]<<2) |	(tmp_led[2]<<3)	| (tmp_led[3]<<4);

		//envoie de la trame au moteur
		res=write(Motor->file,&trame_led,2);
		fsync(Motor->file);
		if(res<0)
			printf("Erreur lors de l'envoi au pilote moteur(LED)! res=%d \n",res);
		break;

	case MOTOR_PWM_LED :
		//Construction de la trame PWM_LED
		trame_pwmled[0]=(		(CMD_PWM & 0x07) << 5 	| (tmp_pwm[0]&MASK_9BITS) >> 4);
		trame_pwmled[1]=((tmp_pwm[0]&MASK_9BITS) << 4 	| (tmp_pwm[1]&MASK_9BITS) >> 5);
		trame_pwmled[2]=((tmp_pwm[1]&MASK_9BITS) << 3 	| (tmp_pwm[2]&MASK_9BITS) >> 6);
		trame_pwmled[3]=((tmp_pwm[2]&MASK_9BITS) << 2 	| (tmp_pwm[3]&MASK_9BITS) >> 7);
		trame_pwmled[4]= (tmp_pwm[3]&MASK_9BITS) << 1;
		trame_pwmled[5]=(CMD_LED << 5)	| (tmp_led[0]>>7) | (tmp_led[1]>>6) | (tmp_led[2]>>5)	| (tmp_led[3]>>4);
		trame_pwmled[6]=(tmp_led[0]<<1) | (tmp_led[1]<<2) |	(tmp_led[2]<<3)	| (tmp_led[3]<<4);

		//Envoi de la trame combinée
		res=write(Motor->file,trame_pwmled,7);
		fsync(Motor->file);

		if(res<0)
			printf("Erreur lors de l'envoi au pilote moteur(PWM)! res=%d \n",res);
		break;

	case MOTOR_STOP:
		tmp_pwm[0]=0;
		tmp_pwm[1]=0;
		tmp_pwm[2]=0;
		tmp_pwm[3]=0;

		//Construction de la trame PWM
		trame_pwm[0]=(		(CMD_PWM & 0x07) << 5 	| (tmp_pwm[0]&MASK_9BITS) >> 4);
		trame_pwm[1]=((tmp_pwm[0]&MASK_9BITS) << 4 	| (tmp_pwm[1]&MASK_9BITS) >> 5);
		trame_pwm[2]=((tmp_pwm[1]&MASK_9BITS) << 3 	| (tmp_pwm[2]&MASK_9BITS) >> 6);
		trame_pwm[3]=((tmp_pwm[2]&MASK_9BITS) << 2 	| (tmp_pwm[3]&MASK_9BITS) >> 7);
		trame_pwm[4]= (tmp_pwm[3]&MASK_9BITS) << 1;

		//envoie de la trame au moteur
		res=write(Motor->file,trame_pwm,5);
		fsync(Motor->file);
		if(res<0){
			printf("Erreur lors de l'envoi au pilote moteur(PWM)! res=%d \n",res);
		}

		break;
	}


}


void *MotorTask ( void * ptr ) {
	MotorStruct *Motor= (MotorStruct *)ptr;
	pthread_barrier_wait(&MotorStartBarrier);
	printf("%s start! \n",__FUNCTION__ );

	while (MotorActivated) {

		//tempo : 5ms
		sem_wait(&MotorTimerSem);

		//Exit
		if(!MotorActivated){
			break;
		}

		motor_send(Motor,MOTOR_PWM_LED);
	}

	//Destruction: MotorTask
	pthread_exit(NULL);
}


int MotorInit (MotorStruct *Motor) {


	// Variables locales
	int retval;
	pthread_attr_t MotorTaskattr;
	struct sched_param SchedParam;
	int					minprio, maxprio;

	//initialisation Pilote
	MotorPortInit(Motor);

	//initialisation de la barrière initialisation MotorTask
	pthread_barrier_init(&MotorStartBarrier, NULL, 2);

	// Création attr pour MotorTask
	pthread_attr_init(&MotorTaskattr);

	/* à déterminer ce qui est nécessaire à MotorTask*/
	pthread_attr_setdetachstate(&MotorTaskattr,PTHREAD_CREATE_JOINABLE);// task peut etre rejoignable(barriere)??
	//pthread_setname_np("");
	pthread_attr_setinheritsched(&MotorTaskattr,PTHREAD_EXPLICIT_SCHED); // héritage param. de la task qui la cree??
	pthread_attr_setscope(&MotorTaskattr, PTHREAD_SCOPE_SYSTEM); // Process vs task? tjrs ..._SYSTEM ne linux!
	pthread_attr_setstacksize(&MotorTaskattr,THREADSTACK);
	minprio = sched_get_priority_min(POLICY);
	maxprio = sched_get_priority_max(POLICY);
	pthread_attr_setschedpolicy(&MotorTaskattr, POLICY);
	SchedParam.sched_priority = minprio + (maxprio - minprio)/4;
	pthread_attr_setschedpolicy(&MotorTaskattr, POLICY);
	pthread_attr_setschedparam(&MotorTaskattr,&SchedParam);

	// Création de MotorTask
	if ((retval = pthread_create(&Motor->MotorTask,&MotorTaskattr,MotorTask,Motor))<0){
		printf("MotorInit : MotorTask Abort, res=%d\n",retval);
		return retval;
	}

	//Destruction attr pour MotorTask
	pthread_attr_destroy(&MotorTaskattr);

	//Initialisation des verrous
	if((retval=sem_init(&MotorTimerSem,1,0))<0){
		printf("%s : Impossible d'initialiser le spinlock (MotorTimerSem): retval = %d\n", __FUNCTION__, retval);
		return retval;
	}

	//MotorMutex
	if((retval=pthread_mutex_init(&Motor->MotorMutex,NULL))<0){
		printf("%s : Errueur initialisation verrous retval = %d\n", __FUNCTION__, retval);
		return retval;
	}
	return retval;
}



int MotorStart (void) {
	MotorActivated=1;
	pthread_barrier_wait(&MotorStartBarrier);
	pthread_barrier_destroy(&MotorStartBarrier);
	return SUCCESS;
}



int MotorStop (MotorStruct *Motor) {
	int res;

	//STOP
	MotorActivated=0;
	sem_post(&MotorTimerSem);
	motor_send(Motor,MOTOR_STOP);

	//TASK EXIT
	res = pthread_join(Motor->MotorTask,NULL);
	if (res) {
		printf("pthread_join(MotorTask) : Erreur\n");
		return res;
	}
	res=close(Motor->file);

	//Destructions des Verrrous
	sem_destroy(&MotorTimerSem);
	pthread_mutex_destroy(&Motor->MotorMutex); //Struct Motor

/* A faire! */
/* Ici, vous devriez arrêter les moteurs et fermer le Port des moteurs. */ 
	return SUCCESS;
}

