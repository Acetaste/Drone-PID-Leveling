/*
 * hardware_control.c
 *
 *  Created on: Apr 30, 2026
 *      Author: matth
 */


#include "hardware_control.h"


/**
 * @brief function to set all 4 channels of Tim_Handle to  PWM_Value
 * @param PWM_Value the value the timer is set to
 * @param Tim_Handle the timer handle whose values are changed
 */
void set_all_motor_pwm(int PWM_Value, TIM_HandleTypeDef Tim_Handle)
{
	__HAL_TIM_SET_COMPARE(&Tim_Handle,TIM_CHANNEL_1, PWM_Value);
	__HAL_TIM_SET_COMPARE(&Tim_Handle,TIM_CHANNEL_2, PWM_Value);
	__HAL_TIM_SET_COMPARE(&Tim_Handle,TIM_CHANNEL_3, PWM_Value);
	__HAL_TIM_SET_COMPARE(&Tim_Handle,TIM_CHANNEL_4, PWM_Value);

}
