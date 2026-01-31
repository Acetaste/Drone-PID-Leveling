/*
 * fernbedienung.h
 *
 *  Created on: Nov 2, 2025
 *      Author: matth
 */

#ifndef SRC_DECODE_H_
#define SRC_DECODE_H_

#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "stm32l4xx_hal.h"

char decode(int binary);
void interpret_flags(char* IR_Character, float* desired_Pitch, float* desired_Roll, uint8_t* stop_flag, TIM_HandleTypeDef* pwm_timer_handle, UART_HandleTypeDef* UART_handle, float* P_Pitch,float* P_Roll, float* P_Yaw);
#endif /* SRC_DECODE_H_ */
