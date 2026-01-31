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

void timer_start(uint16_t* timer_val, TIM_HandleTypeDef* pwm_timer_handle, TIM_HandleTypeDef* IR_remote_duration_timer, TIM_HandleTypeDef* flag_setting_timer );
void configure_imu(I2C_HandleTypeDef* I2C_handle ,UART_HandleTypeDef* UART_handle);
void selftest_accel(I2C_HandleTypeDef* I2C_handle ,UART_HandleTypeDef* UART_handle);

#endif /* INC_STARTUP_H_ */
