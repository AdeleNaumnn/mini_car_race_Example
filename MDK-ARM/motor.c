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
int16_t LinePosition = 0;

/* ============================================================
 *  Init: PWM + encoder + PID gains
 * ============================================================ */
void MotorInit(void)
{
    /* DWT: multiplexer.c 的微秒延时靠它，这里自己开，不依赖 IMU */
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);

    HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);

    /* ---- outer loop: line position -> speed difference ---- */
    LinePID.targetVal = 0;
    LinePID.preError  = 0;
    LinePID.integral  = 0;
    LinePID.output    = 0;
    LinePID.Kp = 1.0f;
    LinePID.Ki = 0.0f;
    LinePID.Kd = 1.5f;

    /* ---- inner loop: speed -> PWM ----
       速度环不要 Kd：编码器分辨率有限，微分会把量化噪声放大。
       之前 Kd = 5.0 会让 PWM 一直抽动。 */
    LeftPID.targetVal = 0;
    LeftPID.preError  = 0;
    LeftPID.integral  = 0;
    LeftPID.output    = 0;
    LeftPID.Kp = 60.0f;
    LeftPID.Ki = 3.0f;
    LeftPID.Kd = 0.0f;

    RightPID.targetVal = 0;
    RightPID.preError  = 0;
    RightPID.integral  = 0;
    RightPID.output    = 0;
    RightPID.Kp = 60.0f;
    RightPID.Ki = 3.0f;
    RightPID.Kd = 0.0f;
}

/* ============================================================
 *  PID
 * ============================================================ */
void ComputePID(struct PID_Controller*pid,int16_t measureVal)
{
    pid->currentError = pid->targetVal - measureVal;
    pid->derivative   = pid->currentError - pid->preError;
    pid->integral    += pid->currentError;
    pid->preError     = pid->currentError;

    if (pid->integral > integralLimit) {
        pid->integral = integralLimit;
    } else if (pid->integral < -integralLimit) {
        pid->integral = -integralLimit;
    }

    pid->output = pid->Kp * pid->currentError
                + pid->Ki * pid->integral
                + pid->Kd * pid->derivative;

    if (pid->output > pwmMax) {
        pid->output = pwmMax;
    } else if (pid->output < -pwmMax) {
        pid->output = -pwmMax;
    }
}

/* ============================================================
 *  Tracking: 每 10 ms 由 TIM2 中断调用一次
 * ============================================================ */
void Track(void)
{
    static uint16_t lostCnt = 0;                 /* 必须 static！ */
    const  uint16_t LOST_STOP_CNT = 200;         /* 200 x 10 ms = 2 s */
    uint16_t mux_value;
    int count = 0;
    int16_t sum = 0;
    int16_t turn;
    int i;

    MUX_get_value(&mux_value);

    for (i = 0; i <= 11; i++) {
        if (MUX_GET_CHANNEL(mux_value, i)) {
            sum += 2 * i - 11;
            count++;
        }
    }

    if (count > 0) {
        lostCnt = 0;                             /* 看到线，计时清零 */
        LinePosition = (int16_t)((sum * 5) / count);
    } else {
        /* 丢线：LinePosition 保持上一次的值 -> 继续朝原方向走 */
        if (lostCnt < LOST_STOP_CNT) {
            lostCnt++;
        } else {
            LeftPID.targetVal  = 0;              /* 超时：刹停 */
            RightPID.targetVal = 0;
            LeftPID.integral   = 0;
            RightPID.integral  = 0;
            return;
        }
    }

    ComputePID(&LinePID, LinePosition);
    turn = LinePID.output;

    LeftPID.targetVal  = (int16_t)(Base_Speed - turn);
    RightPID.targetVal = (int16_t)(Base_Speed + turn);
}

/* ============================================================
 *  PWM -> DIR + CCR (left and right)
 * ============================================================ */
void MotorControl(struct PID_Controller*LeftPID,struct PID_Controller*RightPID)
{
    int16_t leftpwm  = LeftPID->output;
    int16_t rightpwm = RightPID->output;

    if (leftpwm > pwmMax)   leftpwm  = pwmMax;
    if (leftpwm < -pwmMax)  leftpwm  = -pwmMax;
    if (rightpwm > pwmMax)  rightpwm = pwmMax;
    if (rightpwm < -pwmMax) rightpwm = -pwmMax;

    /* ---- left wheel ---- */
    if (leftpwm >= 0) {
        HAL_GPIO_WritePin(L_DIR_GPIO_Port, L_DIR_Pin, GPIO_PIN_SET);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, (uint32_t)leftpwm);
    } else {
        HAL_GPIO_WritePin(L_DIR_GPIO_Port, L_DIR_Pin, GPIO_PIN_RESET);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, (uint32_t)(-leftpwm));
    }

    /* ---- right wheel ---- */
    if (rightpwm >= 0) {
        HAL_GPIO_WritePin(R_DIR_GPIO_Port, R_DIR_Pin, GPIO_PIN_SET);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, (uint32_t)rightpwm);
    } else {
        HAL_GPIO_WritePin(R_DIR_GPIO_Port, R_DIR_Pin, GPIO_PIN_RESET);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, (uint32_t)(-rightpwm));
    }
}
