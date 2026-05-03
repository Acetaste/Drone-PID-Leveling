/*
 * fernbedienung.h
 *
 *  Created on: Nov 2, 2025
 *      Author: matth
 */

#ifndef SRC_DECODE_H_
#define SRC_DECODE_H_


#include "stm32l4xx_hal.h"
#include "data_collection.h"

char decode(int binary);
void interpret_IR_Char(char IR_Character, struct data_header_struct* data_header, uint8_t* stop_flag , uint8_t* save_data_flag,uint8_t* send_data_flag,uint8_t* new_angle_flag, UART_HandleTypeDef* UART_handle, TIM_HandleTypeDef pwm_timer_handle);
void interpret_IR_char_tuning(char IR_Character, float* desired_Pitch, float* desired_Roll, uint8_t* stop_flag, TIM_HandleTypeDef pwm_timer_handle, UART_HandleTypeDef* UART_handle, float* P_Pitch,float* P_Roll, float* P_Yaw);
#endif /* SRC_DECODE_H_ */
