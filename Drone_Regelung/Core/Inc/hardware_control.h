/*
 * hardware_control.h
 *
 *  Created on: Apr 30, 2026
 *      Author: matth
 */

#ifndef INC_HARDWARE_CONTROL_H_
#define INC_HARDWARE_CONTROL_H_
#include "stm32l4xx_hal.h"

void set_all_motor_pwm(int PWM_Value, TIM_HandleTypeDef Tim_Handle);
#endif /* INC_HARDWARE_CONTROL_H_ */
