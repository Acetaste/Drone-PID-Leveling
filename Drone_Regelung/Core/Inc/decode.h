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

char decode(int Binary);
void interpret_IR_Char(char IR_Character, data_header_struct* Data_Header, uint8_t* Stop_Flag , uint8_t* Save_Data_Flag,uint8_t* Send_Data_Flag,uint8_t* New_Angle_Flag,uint8_t* Save_To_Flash_Flag, UART_HandleTypeDef* UART_Handle, TIM_HandleTypeDef PWM_Timer_Handle);
void interpret_IR_char_tuning(char IR_Character, float* Desired_Pitch, float* Desired_Roll, uint8_t* Stop_Flag, TIM_HandleTypeDef PWM_Timer_Handle, UART_HandleTypeDef* UART_Handle, float* P_Roll,float* P_Pitch, float* P_Yaw);
#endif /* SRC_DECODE_H_ */
