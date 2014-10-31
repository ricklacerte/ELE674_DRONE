/*
 * Motor.c
 *
 *  Created on: 12 sept. 2013
 *      Author: bruno
 */

#include "Motor.h"
#define TimeDelay 50000

extern sem_t	MotorTimerSem;
extern int		MotorActivated;


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
	/* return:   0 -> ouverture UART moteur = SUCCESS
	 * 			-1 -> ouverture UART moteur = FAIL
	 */
	struct termios config;

	// ouvrir le pilote UART du moteur
	int uart = open(MOTOR_UART, O_RDWR | O_NOCTTY | O_NDELAY);

	//erreur: ouverture du pilote UART
	if (uart < 0) {
		printf("motor_open : impossible d'ouvrir le uart du moteur\n");
		return uart;
	}

	//read calls are non blocking
	fcntl(uart, F_SETFL, 0);

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
	/*file 		: handler du moteur (UART)
	 * cmd 		: ??
	 * reply	: réponse du UART
	 * relpylen	: taille de la réponse
	 *
	 */
	int size;

	write(file, &cmd, 1);
	fsync(file);
	usleep(TimeDelay);
	size = read(file, reply, replylen);

	return size;
}


int MotorPortInit(MotorStruct *Motor) {
	/*return  	 0 : SUCCESS
	 * 			-1 : FAIL ( via motor_open() UART )
	 */

	uint8_t reply[256];
	int		i;

	//open motor port
	Motor->file = motor_open();

	//erreur: ouverture UART
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

	//SUCCESS
	return 0;
}


void motor_send(MotorStruct *Motor, int SendMode) {
/* Fonction utilitaire pour simplifier les transmissions aux moteurs envoyer une trame au moteur */
// implémenter un tableau, avec le code approprié, et le transmettre (série)  vers les moteurs
	switch (SendMode) {
	case MOTOR_NONE : 		break;
	case MOTOR_PWM_ONLY :	/* A faire! */
							break;
	case MOTOR_LED_ONLY :	/* A faire! */
							break;
	case MOTOR_PWM_LED :	/* A faire! */
							break;
	}
}


// MOTOR TASK 
void *MotorTask ( void *ptr ) {
/* A faire! */
/* Tache qui transmet les nouvelles valeurs de vitesse */
/* à chaque moteur à interval régulier (5 ms).         */


	while (MotorActivated) {
		//tempo : 5ms
		sem_wait(&MotorTimerSem);
		printf("fgndfkjssssssssssssslviyn8tmlwq");

//		DOSOMETHING();
	}

	//Destruction: MotorTask
	pthread_exit(0); /* exit thread */
}



/*MOTOR INIT:
 *Initialise le pilote de comm. des moteurs
 *Crée la tâche: MotorTask
 *return: 	0  -> SUCCESS
 *			- -> ERROR */

int MotorInit (MotorStruct *Motor) {


	// Variables locales
	int res;
	pthread_attr_t MotorTaskattr;
	struct sched_param SchedParam;

	//initialisation Pilote
	MotorPortInit(Motor);

	// Création attr pour MotorTask
	pthread_attr_init(&MotorTaskattr);

	/* à déterminer ce qui est nécessaire à MotorTask*/
	pthread_attr_setdetachstate(&MotorTaskattr,PTHREAD_CREATE_JOINABLE);// task peut etre rejoignable(barriere)??
	//pthread_setname_np("");
	pthread_attr_setinheritsched(&MotorTaskattr,PTHREAD_EXPLICIT_SCHED); // héritage param. de la task qui la cree??
	pthread_attr_setscope(&MotorTaskattr, PTHREAD_SCOPE_SYSTEM); // Process vs task? tjrs ..._SYSTEM ne linux!
	pthread_attr_setstacksize(&MotorTaskattr,THREADSTACK);
	SchedParam.sched_priority=MOTOR_TASK_PRIO;
	pthread_attr_setschedpolicy(&MotorTaskattr, POLICY);
	pthread_attr_setschedparam(&MotorTaskattr,&SchedParam);

	// Création de MotorTask
	res=pthread_create(&Motor->MotorThread,&MotorTaskattr,MotorTask,Motor); //est-ce que l'on envoie Motor ??

	if (!res)
		printf("MotorInit : MotorTask created, res=%d",res);

	//Destruction attr pour MotorTask
	pthread_attr_destroy(&MotorTaskattr);

	//Initialisation des verrous
	sem_init(&MotorTimerSem,0,0);

	return res;
}


//MOTOR START
int MotorStart (void) {
	MotorActivated=1;

/* A faire! */
/* Ici, vous devriez démarrer la mise à jour des moteurs (MotorTask).    */ 
/* Tout le système devrait être prêt à faire leur travail et il ne reste */
/* plus qu'à tout démarrer.                                              */
	return 0;
}


//MOTOR STOP
int MotorStop (MotorStruct *Motor) {
	MotorActivated=0;


/* A faire! (motor activated à 0)*/
/* Ici, vous devriez arrêter les moteurs et fermer le Port des moteurs. */ 
	return 0;
}

