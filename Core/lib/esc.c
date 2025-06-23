/*
 *design by PhanHoangViet
 * */
/*
 * */
#include "esc.h"
#include "stm32f4xx_hal.h"
#include "tim.h"
#include "math.h"


void esc_init()
{
    HAL_TIM_PWM_Start_IT(&htim5, TIM_CHANNEL_4); // khởi tạo pwm esc
    HAL_Delay(100);
    calib_esc();
}

void calib_esc()
{
	__HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_4, 2000);
	HAL_Delay(2000);
	__HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_4, 1000);
	HAL_Delay(2000);
}

void MotorDriver (float Local_duty)
{
	if (Local_duty < 0)
	{
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET);  // chieu am
		__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, -Local_duty );
	}
	else
	{
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET);  // chieu duong
		__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, Local_duty );
	}
}
