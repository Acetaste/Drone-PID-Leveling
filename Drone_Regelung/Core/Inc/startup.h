/*
 * startup.h
 *
 *  Created on: 31.1.12, 2026
 *      Author: matthias ellinger
 */

#ifndef INC_STARTUP_H_
#define INC_STARTUP_H_

#include "stm32l4xx_hal.h"
#include <stdio.h>
#include <string.h>

void timer_start(uint16_t* Timer_Val, TIM_HandleTypeDef* PWM_Timer_Handle, TIM_HandleTypeDef* IR_Remote_Duration_timer_Handle, TIM_HandleTypeDef* Flag_Setting_Timer_Handle);
void configure_imu(I2C_HandleTypeDef* I2C_Handle ,UART_HandleTypeDef* UART_Handle);

#endif /* INC_STARTUP_H_ */
