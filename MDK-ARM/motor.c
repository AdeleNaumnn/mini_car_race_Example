#include <stdio.h>
#include <stdint.h>
#include "motor.h"
#include "stm32f1xx_hal.h"
#include "stm32f1xx_hal_tim.h"

void ComputePID(struct PID_Controller*pid,int16_t measureVal){
	pid->currentError=pid->targetVal-measureVal;
	pid->derivative=pid->currentError-pid->preError;
	pid->integral+=pid->currentError;
	pid->preError=pid->currentError;
	
	if(pid->integral>integralLimit)
	{pid->integral=integralLimit;}
	else if(pid->integral<-integralLimit){
		pid->integral=integralLimit;}
	
		pid->output=pid->Kp*pid->currentError+pid->Ki*pid->integral+pid->Kd*pid->derivative;
	}

	void MotorControl(struct PID_Controller*LeftPID,struct PID_Controller*RightPID){
		int16_t leftpwm=LeftPID->output;
		int16_t rightpwm=RightPID->output;
		
		if(leftpwm>3600){
			leftpwm=3600;
	}
		else if(leftpwm<-3600){
			leftpwm=-3600;}
		
			if(leftpwm>0){
				__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,leftpwm);}
			if(leftpwm<0){
				__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,-leftpwm);}
			
		
		if(rightpwm>3600){
			rightpwm=3600;
	}
		else if(rightpwm<-3600){
			rightpwm=-3600;}
		
		if(rightpwm>0){
				__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,rightpwm);}
			if(rightpwm<0){
				__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,-rightpwm);}
			
	}
		
		
		

