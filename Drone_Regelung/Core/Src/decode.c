/*
 * decode.c
 *
 *  Created on: Nov 2, 2025
 *      Author: matthias Ellinger
 */


#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>

#include "decode.h"
#include "user_constants.h"
#include "hardware_control.h"

/**
 * @brief function to decode the received value from the IR receiver
 * @param binary the value obtained by the decoding function of the IR receiver input
 * @return the decoded character equivalent to those on the IR remote
 */
char decode(int Binary)
{
	char result;
	switch(Binary)
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

/**
 * @brief function to interpret the characters received by the IR remote
 * @detail 	when pressing  '*' the stop flag is set and the PWM is cleared
 * 			when pressing  '#' the desired angles and the internal string used for storing the previous inputs are cleared
 * 			otherwise the function expects inputs in the form "XYY" 'OK' where X is a direction i.e. up, down, left, right and Y is a digit 0-9
 * 			after pressing 'OK' the newly entered desired angle is set and the data equistion is started for the number of iterations in user_constants.h
 * @param
 * @param Tim_Handle the timer handle whose values are changed
 */
void interpret_IR_Char(char IR_Character, Data_Header_struct* Data_Header, uint8_t* Stop_Flag , uint8_t* Save_Data_Flag,uint8_t* Send_Data_Flag,uint8_t* New_Angle_Flag,uint8_t* Save_To_Flash_Flag, UART_HandleTypeDef* UART_Handle, TIM_HandleTypeDef PWM_Timer_Handle)
{
	static char setting_string[4];
	static uint8_t setting_string_index;
	const char clear_value[4] = { '\0','\0','\0','\0'};
	uint8_t uart_buffer[100];
	sprintf((char*) uart_buffer,"IR_Char: %c\n", IR_Character);
	HAL_UART_Transmit(UART_Handle, uart_buffer, strlen((char*)uart_buffer), 100);
	switch(IR_Character)
	{

		case('*'):
			*Save_To_Flash_Flag = 1;
			*Stop_Flag = 1;
			set_all_motor_pwm(0, PWM_Timer_Handle);
			sprintf((char*) uart_buffer,"Stop flag set\n");
			HAL_UART_Transmit(UART_Handle, uart_buffer, strlen((char*)uart_buffer), 100);
			break;

		case('#'):
			strncpy(setting_string, clear_value, 4);
			setting_string_index 				= 	0;
			Data_Header->desired_pitch 			=	0;
			Data_Header->desired_roll 			=	0;
			Data_Header->desired_yaw 			=	0;
			*New_Angle_Flag						= 	1;


			sprintf((char*) uart_buffer,"Cleared setting string to %s and set Pitch:%d, Roll:%d, Yaw:%d\n", setting_string, Data_Header->desired_pitch, Data_Header->desired_roll, Data_Header->desired_yaw );
			HAL_UART_Transmit(UART_Handle, uart_buffer, strlen((char*)uart_buffer), 100);

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
							Data_Header->desired_pitch  	= 		MAX_CAP(atoi(&setting_string[1]),MAX_ANGLE);
							*New_Angle_Flag				= 		1;
							*Stop_Flag 					=		0;
							*Save_Data_Flag 			=		1;

							break;

					case('R'):
							Data_Header->desired_pitch 	= (-1) *  MAX_CAP(atoi(&setting_string[1]),MAX_ANGLE);
							*New_Angle_Flag				= 		1;
							*Stop_Flag 					=		0;
							*Save_Data_Flag 			=		1;

							break;

					case('U'):
							Data_Header->desired_roll 	= 		MAX_CAP(atoi(&setting_string[1]),MAX_ANGLE);
							*New_Angle_Flag				= 		1;
							*Stop_Flag 					=		0;
							*Save_Data_Flag 			=		1;
							break;

					case('D'):
							Data_Header->desired_roll 	= (-1) *  MAX_CAP(atoi(&setting_string[1]),MAX_ANGLE);
							*New_Angle_Flag				= 		1;
							*Stop_Flag 					=		0;
							*Save_Data_Flag 			=		1;

							break;
					case('0'):
							if(setting_string[1] == '0' && setting_string[2]== '0')
							{
								*Send_Data_Flag =1;
							}
							break;

				default:
					sprintf((char*) uart_buffer,"Not a valid string, needs to start with direction\n");
					HAL_UART_Transmit(UART_Handle, uart_buffer, strlen((char*)uart_buffer), 100);
			}

			sprintf((char*) uart_buffer,"Pitch set to: %d, Roll set to %d\n",Data_Header->desired_pitch,Data_Header->desired_roll);
			HAL_UART_Transmit(UART_Handle, uart_buffer, strlen((char*)uart_buffer), 100);

			break;

		default:

			if(IR_Character == '\0')
			{
				strncpy(setting_string, clear_value, 4);
				sprintf((char*) uart_buffer,"interpret_flag invoked without setting the char\n");
				HAL_UART_Transmit(UART_Handle, uart_buffer, strlen((char*)uart_buffer), 100);
			}
			else
			{
				if(setting_string_index > 2)
				{
					setting_string_index = 0;
					sprintf((char*) uart_buffer,"To many number Inputs reset index to 0 \n");
					HAL_UART_Transmit(UART_Handle, uart_buffer, strlen((char*)uart_buffer), 100);
				}

				else if(isdigit(IR_Character) && (setting_string_index != 0))
				{

					setting_string[setting_string_index]= IR_Character;
					setting_string_index++;

				}
				else if(IR_Character == '0' && setting_string_index == 0)
				{
					strncpy(setting_string, clear_value, 4);
					setting_string[0] = '0';
					setting_string_index = 1;
				}
				else
				{
					setting_string_index = 0;
					strncpy(setting_string, clear_value, 4);
					sprintf((char*) uart_buffer,"In valid Input reset string and index \n");
					HAL_UART_Transmit(UART_Handle, uart_buffer, strlen((char*)uart_buffer), 100);
				}
			}
	}


}

void interpret_IR_char_tuning(char IR_Character, float* desired_Pitch, float* desired_Roll, uint8_t* Stop_Flag, TIM_HandleTypeDef PWM_Timer_Handle, UART_HandleTypeDef* UART_Handle, float* P_Pitch,float* P_Roll, float* P_Yaw)
{
	static int pwm_value = 0;
	uint8_t uart_buffer[50];
	switch(IR_Character)
	{
		case('L'):

				*desired_Pitch += 5;
			  	sprintf((char*) uart_buffer,"Desired Pitch: %d\n",(int) *desired_Pitch);
			  	HAL_UART_Transmit(UART_Handle, uart_buffer, strlen((char*)uart_buffer), 100);
			  	break;

		case('R'):

				*desired_Pitch -= 5;
				sprintf((char*) uart_buffer,"Desired Pitch: %d\n",(int) *desired_Pitch);
			  	HAL_UART_Transmit(UART_Handle, uart_buffer, strlen((char*)uart_buffer), 100);
			  	break;

		case('U'):

				*desired_Roll += 5;
				sprintf((char*) uart_buffer,"Desired Roll: %d\n",(int) *desired_Roll);
				HAL_UART_Transmit(UART_Handle, uart_buffer, strlen((char*)uart_buffer), 100);
				break;

		case('D'):

				*desired_Roll -= 5;
				sprintf((char*) uart_buffer,"Desired Roll: %d\n",(int) *desired_Roll);
				HAL_UART_Transmit(UART_Handle, uart_buffer, strlen((char*)uart_buffer), 100);
				break;

		case('#'):

				*desired_Roll 	= 0;
				*desired_Pitch 	= 0;
				*P_Pitch 		= 0;
				*P_Roll 		= 0;
				*P_Yaw 			= 0;
				sprintf((char*) uart_buffer,"Desired Roll: %d, Desired Pitch: %d\n",(int) *desired_Roll, (int) *desired_Pitch);
				HAL_UART_Transmit(UART_Handle, uart_buffer, strlen((char*)uart_buffer), 100);
				break;

		case('*'):

				*Stop_Flag = 1;
				set_all_motor_pwm(0, PWM_Timer_Handle);
				sprintf((char*) uart_buffer,"P Pitch: %d.%d, P Roll: %d.%d, P Yaw: %d.%d\n",(int) *P_Pitch,(int) (*P_Pitch*10)%10,  (int) *P_Roll, (int) (*P_Roll*10)%10, (int) *P_Yaw, (int) (*P_Yaw*10)%10);
				HAL_UART_Transmit(UART_Handle, uart_buffer, strlen((char*)uart_buffer), 100);
				break;

		case('1'):

				*Stop_Flag = 1;
				pwm_value-= 1;
				set_all_motor_pwm(pwm_value, PWM_Timer_Handle);
				break;

		case('2'):

				*Stop_Flag = 1;
				set_all_motor_pwm(10, PWM_Timer_Handle);
				break;

		case('3'):
				*Stop_Flag = 1;
				pwm_value+= 1;
				set_all_motor_pwm(pwm_value, PWM_Timer_Handle);
				break;

		case('4'):

				*P_Pitch += P_VALUE_INCREMENT;
				sprintf((char*) uart_buffer,"P_Pitch: %d.%d\n",(int) *P_Pitch, (int) (*P_Pitch*10)%10);
				HAL_UART_Transmit(UART_Handle, uart_buffer, strlen((char*)uart_buffer), 100);
				break;

		case('5'):

				*P_Roll += P_VALUE_INCREMENT;
				sprintf((char*) uart_buffer,"P_Roll: %d.%d\n",(int) *P_Roll, (int) (*P_Roll*10)%10);
				HAL_UART_Transmit(UART_Handle, uart_buffer, strlen((char*)uart_buffer), 100);
				break;

		case('6'):

				*P_Yaw += P_VALUE_INCREMENT;
				sprintf((char*) uart_buffer,"P_Yaw: %d.%d\n",(int) *P_Yaw, (int) (*P_Yaw*10)%10);
				HAL_UART_Transmit(UART_Handle, uart_buffer, strlen((char*)uart_buffer), 100);
				break;

		case('7'):

				*P_Pitch -= P_VALUE_INCREMENT;
				sprintf((char*) uart_buffer,"P_Pitch: %d.%d\n",(int) *P_Pitch, (int) (*P_Pitch*10)%10);
				HAL_UART_Transmit(UART_Handle, uart_buffer, strlen((char*)uart_buffer), 100);
				break;

		case('8'):

				*P_Roll -= P_VALUE_INCREMENT;
				sprintf((char*) uart_buffer,"P_Roll: %d.%d\n",(int) *P_Roll, (int) (*P_Roll*10)%10);
				HAL_UART_Transmit(UART_Handle, uart_buffer, strlen((char*)uart_buffer), 100);
				break;

		case('9'):

				*P_Yaw -= P_VALUE_INCREMENT;
				sprintf((char*) uart_buffer,"P_Yaw: %d.%d\n",(int) *P_Yaw, (int) (*P_Yaw*10)%10);
				HAL_UART_Transmit(UART_Handle, uart_buffer, strlen((char*)uart_buffer), 100);
				break;
		case('O'):

				*Stop_Flag = 0;
				sprintf((char*) uart_buffer,"stop flag cleared\n");
				HAL_UART_Transmit(UART_Handle, uart_buffer, strlen((char*)uart_buffer), 100);
				break;
	}
}


