/*
 * Motor.h
 *
 *  Created on: 12 sept. 2013
 *      Author: bruno
 */

#ifndef MOTOR_H_
#define MOTOR_H_

#include <stdio.h>   /* Standard input/output definitions */
#include <stdlib.h>
#include <stdint.h>   /* Standard input/output definitions */
#include <string.h>  /* String function definitions */
#include <unistd.h>  /* UNIX standard function definitions */
#include <signal.h>
#include <fcntl.h>   /* File control definitions */
#include <errno.h>   /* Error number definitions */
#include <termios.h> /* POSIX terminal control definitions */
#include <pthread.h>
#include <semaphore.h>
#include <sys/time.h>


#define POLICY SCHED_RR
#define THREADSTACK  65536


enum { MOTOR_NONE, MOTOR_PWM_ONLY, MOTOR_LED_ONLY, MOTOR_PWM_LED };

#define MOTOR_LEDOFF 0x0000
#define MOTOR_LEDRED 0x0100
#define MOTOR_LEDGREEN 0x0001
#define MOTOR_LEDORANGE 0x0101

#define MOTOR_UART "/dev/ttyO0"

#define GPIO_M1 171
#define GPIO_M2 172
#define GPIO_M3 173
#define GPIO_M4 174

#define GPIO_ERROR_READ 176
#define GPIO_ERROR_RESET 175

#define MOTOR_PERIOD	1	//period = 5ms
#define MOTOR_TASK_PRIO 30

#define CMD_PWM		0x01
#define CMD_LED		0x03
#define MASK_9BITS 	0x1FFF

typedef struct motor_struct {
	uint16_t	pwm[4];   //motor speed 0x00-0x1ff
	uint16_t	led[4];		// pourquoi 16 bits? -> 8 bits
	int			file;
	pthread_t 	MotorTask;
	pthread_spinlock_t 	MotorLock;
} MotorStruct;

int MotorInit (MotorStruct *Motor);
int MotorStart (void);
int MotorStop (MotorStruct *Motor);

#endif /* MOTOR_H_ */
