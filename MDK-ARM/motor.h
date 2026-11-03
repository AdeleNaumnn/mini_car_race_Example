#ifndef MOTOR_H
#define MOTOR_H

#include <stdio.h>
#include <stdint.h>
#include "stm32f1xx_hal.h"

#define integralLimit 20000

struct PID_Controller{
	int16_t  targetVal;
	int16_t currentError;
	int16_t preError;
	int32_t derivative;
	int32_t integral;
	float Kp;
	float Ki;
	float Kd;
	int32_t output;;
}PID;

void ComputePID(struct PID_Controller*pid,int16_t measureVal);
void MotorControl(struct PID_Controller*LeftPID,struct PID_Controller*RightPID);

extern struct PID_Controller LeftPID;
extern struct PID_Controller RightPID;
extern TIM_HandleTypeDef htim1;


#endif