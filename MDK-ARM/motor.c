#include <stdio.h>
#include <stdint.h>
#include "motor.h"
#include "stm32f1xx_hal.h"
#include "stm32f1xx_hal_tim.h"
#include "multiplexer.h"
#include "main.h"

struct PID_Controller LeftPID;
struct PID_Controller RightPID;
struct PID_Controller LinePID;
int16_t LinePosition;
int16_t encodeL;
int16_t encodeR;


	void MotorInit(void)
{
    
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
   
    HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);
    
    LinePID.targetVal    = 0;
    LinePID.Kp = 0.0f;    
    LinePID.Ki = 0.0f;
    LinePID.Kd = 0.0f;
    
    LeftPID.targetVal  = 0;
    LeftPID.Kp = 0.0f;
    LeftPID.Ki = 0.0f;
    LeftPID.Kd = 0.0f;
		
    RightPID.targetVal = 0;
    RightPID.Kp = 0.0f;
    RightPID.Ki = 0.0f;
    RightPID.Kd = 0.0f;
		
    LeftPID.output  = 0;
    RightPID.output = 0;
}   


void ComputePID(struct PID_Controller*pid,int16_t measureVal){
	pid->currentError=pid->targetVal-measureVal;
	pid->derivative=pid->currentError-pid->preError;
	pid->integral+=pid->currentError;
	pid->preError=pid->currentError;
	
	if(pid->integral>integralLimit)
	{pid->integral=integralLimit;}
	else if(pid->integral<-integralLimit){
		pid->integral=-integralLimit;}
	
		pid->output=pid->Kp*pid->currentError+pid->Ki*pid->integral+pid->Kd*pid->derivative;
		
		if(pid->output>pwmMax){
			pid->output=pwmMax;}
		else if(pid->output<-pwmMax){
			pid->output=-pwmMax;}
	}

	void Track(void){
		uint16_t mux_value;
		MUX_get_value(&mux_value);
    int count=0;
    int16_t sum=0;
    int16_t turn;		

		for (int i = 0; i <= 11; i++) {
    //printf("%d,", MUX_GET_CHANNEL(mux_value, i)); 
			if(MUX_GET_CHANNEL(mux_value,i)){
				sum+=2*i-11;
				count++;
		}
	}
		if(count>0)
		LinePosition=(sum*5)/count;
		ComputePID(&LinePID,LinePosition);
		turn = LinePID.output; 
		LeftPID.targetVal  = (int16_t)(Base_Speed - turn);
    RightPID.targetVal = (int16_t)(Base_Speed + turn);
}
		
		

	void MotorControl(struct PID_Controller*LeftPID,struct PID_Controller*RightPID){
		int16_t leftpwm=LeftPID->output;
		int16_t rightpwm=RightPID->output;
		
		if(leftpwm>3600){
			leftpwm=3600;
	}
		else if(leftpwm<-3600){
			leftpwm=-3600;}
		
			if(leftpwm>=0){
				HAL_GPIO_WritePin(L_DIR_GPIO_Port, L_DIR_Pin, GPIO_PIN_SET);
				__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,leftpwm);}
			else if(leftpwm<0){
				HAL_GPIO_WritePin(L_DIR_GPIO_Port, L_DIR_Pin, GPIO_PIN_RESET);
				__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,-leftpwm);}
			
			if(rightpwm>3600){
			rightpwm=3600;
	}
		else if(rightpwm<-3600){
			rightpwm=-3600;}
		
			if(rightpwm>=0){
				HAL_GPIO_WritePin(R_DIR_GPIO_Port, R_DIR_Pin, GPIO_PIN_SET);
				__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,leftpwm);}
			else if(rightpwm<0){
				HAL_GPIO_WritePin(R_DIR_GPIO_Port, R_DIR_Pin, GPIO_PIN_RESET);
				__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,-leftpwm);}
			
			
	}
		
		
		

