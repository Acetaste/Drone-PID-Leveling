/*
 * decode.c
 *
 *  Created on: Nov 2, 2025
 *      Author: matthias Ellinger
 */
#include "decode.h"
#include "hardware_control.h"
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>

#define max_numb(a, b) (((a) < (b)) ? (a) : (b))
#define P_Value_Increase 0.01

char decode(int binary)
{
	char result;
	switch(binary)
	{
	case 977665792:
		result = '1';
		break;
	case 960954112 :
		result = '2';
		break;
	case 944242432:
		result = '3';
		break;
	case 994377472:
		result = '4';
		break;
	case 1061224192:
		result = '5';
		break;
	case 1011089152:
		result = '6';
		break;
	case 2013789952:
		result = '7';
		break;
	case 1779826432:
		result = '8';
		break;
	case 1980366592:
		result = '9';
		break;
	case 1763114752:
		result = '*';
		break;
	case 1712979712:
		result = '0';
		break;
	case 1913519872 :
		result = '#';
		break;
	case  1729691392:
		result = 'U';
		break;
	case 760413952:
		result = 'D';
		break;
	case 1997078272:
		result = 'L';
		break;
	case 626720512:
		result = 'R';
		break;
	case 1662844672:
		result = 'O';
		break;
	}
	return result;
}

void interpret_IR_Char(char IR_Character, struct data_header_struct* data_header, uint8_t* stop_flag , uint8_t* save_data_flag,uint8_t* send_data_flag,uint8_t* new_angle_flag, UART_HandleTypeDef* UART_handle, TIM_HandleTypeDef pwm_timer_handle)
{
	static char setting_string[4];
	static uint8_t setting_string_index;
	const char clear_value[4] = { '\0','\0','\0','\0'};
	uint8_t uart_buffer[100];
	sprintf((char*) uart_buffer,"IR_Char: %c\n", IR_Character);
	HAL_UART_Transmit(UART_handle, uart_buffer, strlen((char*)uart_buffer), 100);
	switch(IR_Character)
	{

		case('*'):
			*stop_flag = 1;
			*send_data_flag =1;
			set_all_motor_pwm(0, pwm_timer_handle);
			sprintf((char*) uart_buffer,"Stop flag set\n");
			HAL_UART_Transmit(UART_handle, uart_buffer, strlen((char*)uart_buffer), 100);
			break;

		case('#'):
			strncpy(setting_string, clear_value, 4);
			setting_string_index 				= 	0;
			data_header->desired_pitch 			=	0;
			data_header->desired_roll 			=	0;
			data_header->desired_yaw 			=	0;
			*new_angle_flag						= 	1;


			sprintf((char*) uart_buffer,"Cleared setting string to %s and set Pitch:%d, Roll:%d, Yaw:%d\n", setting_string, data_header->desired_pitch, data_header->desired_roll, data_header->desired_yaw );
			HAL_UART_Transmit(UART_handle, uart_buffer, strlen((char*)uart_buffer), 100);

			break;

		case('L'):
			strncpy(setting_string, clear_value, 4);
			setting_string[0] = 'L';
			setting_string_index = 1;
			  break;

		case('R'):
			strncpy(setting_string, clear_value, 4);
			setting_string[0] = 'R';
			setting_string_index = 1;
			  break;

		case('U'):
			strncpy(setting_string, clear_value, 4);
			setting_string[0] = 'U';
			setting_string_index = 1;
			break;

		case('D'):
			strncpy(setting_string, clear_value, 4);
			setting_string[0] = 'D';
			setting_string_index = 1;
			break;

		case('O'):
			switch(setting_string[0])
			{
					case('L'):
							data_header->desired_roll 	= 		max_numb(atoi(&setting_string[1]),90);
							*new_angle_flag				= 		1;
							break;

					case('R'):
							data_header->desired_roll 	= (-1) *  max_numb(atoi(&setting_string[1]),90);
							*new_angle_flag				= 		1;
							break;

					case('U'):
							data_header->desired_pitch 	= 		max_numb(atoi(&setting_string[1]),90);
							*new_angle_flag				= 		1;
							break;

					case('D'):
							data_header->desired_pitch 	= (-1) *  max_numb(atoi(&setting_string[1]),90);
							*new_angle_flag				= 		1;
							break;

				default:
					sprintf((char*) uart_buffer,"Not a valid string, needs to start with direction\n");
					HAL_UART_Transmit(UART_handle, uart_buffer, strlen((char*)uart_buffer), 100);
			}

			sprintf((char*) uart_buffer,"Pitch set to: %d, Roll set to %d\n",data_header->desired_pitch,data_header->desired_roll);
			HAL_UART_Transmit(UART_handle, uart_buffer, strlen((char*)uart_buffer), 100);
			*stop_flag = 0;
			*save_data_flag =1;
			*new_angle_flag = 1;
			break;

		default:

			if(IR_Character == '\0')
			{
				strncpy(setting_string, clear_value, 4);
				sprintf((char*) uart_buffer,"interpret_flag invoked without setting the char\n");
				HAL_UART_Transmit(UART_handle, uart_buffer, strlen((char*)uart_buffer), 100);
			}
			else
			{
				if(setting_string_index > 2)
				{
					setting_string_index = 0;
					sprintf((char*) uart_buffer,"To many number Inputs reset index to 0 \n");
					HAL_UART_Transmit(UART_handle, uart_buffer, strlen((char*)uart_buffer), 100);
				}

				else if(isdigit(IR_Character) && (setting_string_index != 0))
				{

					setting_string[setting_string_index]= IR_Character;
					setting_string_index++;

				}

				else
				{
					setting_string_index = 0;
					strncpy(setting_string, clear_value, 4);
					sprintf((char*) uart_buffer,"In valid Input reset string and index \n");
					HAL_UART_Transmit(UART_handle, uart_buffer, strlen((char*)uart_buffer), 100);
				}
			}
	}


}

void interpret_IR_char_tuning(char IR_Character, float* desired_Pitch, float* desired_Roll, uint8_t* stop_flag, TIM_HandleTypeDef pwm_timer_handle, UART_HandleTypeDef* UART_handle, float* P_Pitch,float* P_Roll, float* P_Yaw)
{
	static int pwm_value = 0;
	uint8_t uart_buffer[50];
	switch(IR_Character)
	{
		case('L'):

				*desired_Pitch += 5;
			  	sprintf((char*) uart_buffer,"Desired Pitch: %d\n",(int) *desired_Pitch);
			  	HAL_UART_Transmit(UART_handle, uart_buffer, strlen((char*)uart_buffer), 100);
			  	break;

		case('R'):

				*desired_Pitch -= 5;
				sprintf((char*) uart_buffer,"Desired Pitch: %d\n",(int) *desired_Pitch);
			  	HAL_UART_Transmit(UART_handle, uart_buffer, strlen((char*)uart_buffer), 100);
			  	break;

		case('U'):

				*desired_Roll += 5;
				sprintf((char*) uart_buffer,"Desired Roll: %d\n",(int) *desired_Roll);
				HAL_UART_Transmit(UART_handle, uart_buffer, strlen((char*)uart_buffer), 100);
				break;

		case('D'):

				*desired_Roll -= 5;
				sprintf((char*) uart_buffer,"Desired Roll: %d\n",(int) *desired_Roll);
				HAL_UART_Transmit(UART_handle, uart_buffer, strlen((char*)uart_buffer), 100);
				break;

		case('#'):

				*desired_Roll 	= 0;
				*desired_Pitch 	= 0;
				*P_Pitch 		= 0;
				*P_Roll 		= 0;
				*P_Yaw 			= 0;
				sprintf((char*) uart_buffer,"Desired Roll: %d, Desired Pitch: %d\n",(int) *desired_Roll, (int) *desired_Pitch);
				HAL_UART_Transmit(UART_handle, uart_buffer, strlen((char*)uart_buffer), 100);
				break;

		case('*'):

				*stop_flag = 1;
				set_all_motor_pwm(0, pwm_timer_handle);
				sprintf((char*) uart_buffer,"P Pitch: %d.%d, P Roll: %d.%d, P Yaw: %d.%d\n",(int) *P_Pitch,(int) (*P_Pitch*10)%10,  (int) *P_Roll, (int) (*P_Roll*10)%10, (int) *P_Yaw, (int) (*P_Yaw*10)%10);
				HAL_UART_Transmit(UART_handle, uart_buffer, strlen((char*)uart_buffer), 100);
				break;

		case('1'):

				*stop_flag = 1;
				pwm_value-= 1;
				set_all_motor_pwm(pwm_value, pwm_timer_handle);
				break;

		case('2'):

				*stop_flag = 1;
				set_all_motor_pwm(10, pwm_timer_handle);
				break;

		case('3'):
				*stop_flag = 1;
				pwm_value+= 1;
				set_all_motor_pwm(pwm_value, pwm_timer_handle);
				break;

		case('4'):

				*P_Pitch += P_Value_Increase;
				sprintf((char*) uart_buffer,"P_Pitch: %d.%d\n",(int) *P_Pitch, (int) (*P_Pitch*10)%10);
				HAL_UART_Transmit(UART_handle, uart_buffer, strlen((char*)uart_buffer), 100);
				break;

		case('5'):

				*P_Roll += P_Value_Increase;
				sprintf((char*) uart_buffer,"P_Roll: %d.%d\n",(int) *P_Roll, (int) (*P_Roll*10)%10);
				HAL_UART_Transmit(UART_handle, uart_buffer, strlen((char*)uart_buffer), 100);
				break;

		case('6'):

				*P_Yaw += P_Value_Increase;
				sprintf((char*) uart_buffer,"P_Yaw: %d.%d\n",(int) *P_Yaw, (int) (*P_Yaw*10)%10);
				HAL_UART_Transmit(UART_handle, uart_buffer, strlen((char*)uart_buffer), 100);
				break;

		case('7'):

				*P_Pitch -= P_Value_Increase;
				sprintf((char*) uart_buffer,"P_Pitch: %d.%d\n",(int) *P_Pitch, (int) (*P_Pitch*10)%10);
				HAL_UART_Transmit(UART_handle, uart_buffer, strlen((char*)uart_buffer), 100);
				break;

		case('8'):

				*P_Roll -= P_Value_Increase;
				sprintf((char*) uart_buffer,"P_Roll: %d.%d\n",(int) *P_Roll, (int) (*P_Roll*10)%10);
				HAL_UART_Transmit(UART_handle, uart_buffer, strlen((char*)uart_buffer), 100);
				break;

		case('9'):

				*P_Yaw -= P_Value_Increase;
				sprintf((char*) uart_buffer,"P_Yaw: %d.%d\n",(int) *P_Yaw, (int) (*P_Yaw*10)%10);
				HAL_UART_Transmit(UART_handle, uart_buffer, strlen((char*)uart_buffer), 100);
				break;
		case('O'):

				*stop_flag = 0;
				sprintf((char*) uart_buffer,"stop flag cleared\n");
				HAL_UART_Transmit(UART_handle, uart_buffer, strlen((char*)uart_buffer), 100);
				break;
	}
}


