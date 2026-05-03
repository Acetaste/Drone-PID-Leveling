/*
 * hardware_control.c
 *
 *  Created on: Apr 30, 2026
 *      Author: matth
 */


#include "hardware_control.h"

void set_all_motor_pwm(int pwm_value, TIM_HandleTypeDef tim_handle)
{
	__HAL_TIM_SET_COMPARE(&tim_handle,TIM_CHANNEL_1, pwm_value);
	__HAL_TIM_SET_COMPARE(&tim_handle,TIM_CHANNEL_2, pwm_value);
	__HAL_TIM_SET_COMPARE(&tim_handle,TIM_CHANNEL_3, pwm_value);
	__HAL_TIM_SET_COMPARE(&tim_handle,TIM_CHANNEL_4, pwm_value);

}
