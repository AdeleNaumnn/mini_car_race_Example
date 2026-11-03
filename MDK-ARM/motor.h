#ifndef MOTOR_H
#define MOTOR_H

#include <stdio.h>
#include <stdint.h>
#include "stm32f1xx_hal.h"
#include "main.h"

#define integralLimit 20000
#define pwmMax 3600
#define Base_Speed 30

struct PID_Controller{
	int16_t  targetVal;
	int16_t currentError;
	int16_t preError;
	int32_t derivative;
	int32_t integral;
	float Kp;
	float Ki;
	float Kd;
	int16_t output;
};

void ComputePID(struct PID_Controller*pid,int16_t measureVal);
void MotorControl(struct PID_Controller*LeftPID,struct PID_Controller*RightPID);
void Track(void);
void MotorInit(void);

extern struct PID_Controller LeftPID;
extern struct PID_Controller RightPID;
extern struct PID_Controller LinePID;
extern int16_t LinePosition;

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;


#endif