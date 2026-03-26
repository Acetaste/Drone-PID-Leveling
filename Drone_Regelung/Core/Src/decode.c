/*
 * decode.c
 *
 *  Created on: Nov 2, 2025
 *      Author: matthias Ellinger
 */
#include "decode.h"

#define max_numb(a, b) (((a) < (b)) ? (a) : (b))
#define min_numb(a, b) (((a) > (b)) ? (a) : (b))

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

void interpret_IR_Char(char IR_Character, float* desired_Pitch, float* desired_Roll, uint8_t* stop_flag , uint8_t* save_data_flag, UART_HandleTypeDef* UART_handle)
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
			sprintf((char*) uart_buffer,"Stop flag set\n");
			HAL_UART_Transmit(UART_handle, uart_buffer, strlen((char*)uart_buffer), 100);
			break;

		case('#'):
			strncpy(setting_string, clear_value, 4);
			sprintf((char*) uart_buffer,"Cleared angle setting to %s\n", setting_string);
			HAL_UART_Transmit(UART_handle, uart_buffer, strlen((char*)uart_buffer), 100);
			setting_string_index = 0;
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
					*desired_Pitch = (float) (max_numb(atoi(&setting_string[1]),90));
					break;

				case('R'):
					*desired_Pitch =(float) ((-1) *  min_numb(atoi(&setting_string[1]),-90));
					break;

				case('U'):
					*desired_Roll =(float) (max_numb(atoi(&setting_string[1]),90));
					break;

				case('D'):
					*desired_Roll =(float) ((-1) *  min_numb(atoi(&setting_string[1]),-90));
					break;

				default:
					sprintf((char*) uart_buffer,"Not a valid string, needs to start with direction\n");
					HAL_UART_Transmit(UART_handle, uart_buffer, strlen((char*)uart_buffer), 100);
			}

			sprintf((char*) uart_buffer,"Pitch set to: %d, Roll set to %d\n",(int) (*desired_Pitch),(int) (*desired_Roll));
			HAL_UART_Transmit(UART_handle, uart_buffer, strlen((char*)uart_buffer), 100);
			*stop_flag = 0;
			*save_data_flag =1;
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

void interpret_IR_char_tuning(char* IR_Character, float* desired_Pitch, float* desired_Roll, uint8_t* stop_flag, TIM_HandleTypeDef* pwm_timer_handle, UART_HandleTypeDef* UART_handle, float* P_Pitch,float* P_Roll, float* P_Yaw)
{
	uint8_t uart_buffer[50];
	switch(*IR_Character)
	{
		case('L'):
				*IR_Character = '\0';
				*desired_Pitch += 5;
			  	sprintf((char*) uart_buffer,"Desired Pitch: %d\n",(int) *desired_Pitch);
			  	HAL_UART_Transmit(UART_handle, uart_buffer, strlen((char*)uart_buffer), 100);
			  	break;

		case('R'):
				*IR_Character = '\0';
				*desired_Pitch -= 5;
				sprintf((char*) uart_buffer,"Desired Pitch: %d\n",(int) *desired_Pitch);
			  	HAL_UART_Transmit(UART_handle, uart_buffer, strlen((char*)uart_buffer), 100);
			  	break;

		case('U'):
				*IR_Character = '\0';
				*desired_Roll += 5;
				sprintf((char*) uart_buffer,"Desired Roll: %d\n",(int) *desired_Roll);
				HAL_UART_Transmit(UART_handle, uart_buffer, strlen((char*)uart_buffer), 100);
				break;

		case('D'):
				*IR_Character = '\0';
				*desired_Roll -= 5;
				sprintf((char*) uart_buffer,"Desired Roll: %d\n",(int) *desired_Roll);
				HAL_UART_Transmit(UART_handle, uart_buffer, strlen((char*)uart_buffer), 100);
				break;

		case('#'):
				*IR_Character = '\0';
				*desired_Roll = 0;
				*desired_Pitch = 0;
				sprintf((char*) uart_buffer,"Desired Roll: %d, Desired Pitch: %d\n",(int) *desired_Roll, (int) *desired_Pitch);
				HAL_UART_Transmit(UART_handle, uart_buffer, strlen((char*)uart_buffer), 100);
				break;

		case('*'):
				*IR_Character = '\0';
				*P_Pitch 	= 1;
				*P_Roll 	= 1;
				*P_Yaw 		= 1;
				sprintf((char*) uart_buffer,"P Pitch: %d, P Roll: %d, P Yaw: %d\n",(int) *P_Pitch, (int) *P_Roll,(int) *P_Yaw);
				HAL_UART_Transmit(UART_handle, uart_buffer, strlen((char*)uart_buffer), 100);
				break;

		case('2'):
				*IR_Character = '\0';
				*stop_flag = 1;
				__HAL_TIM_SET_COMPARE(pwm_timer_handle,TIM_CHANNEL_1, 30);
				__HAL_TIM_SET_COMPARE(pwm_timer_handle,TIM_CHANNEL_2, 30);
				__HAL_TIM_SET_COMPARE(pwm_timer_handle,TIM_CHANNEL_3, 30);
				__HAL_TIM_SET_COMPARE(pwm_timer_handle,TIM_CHANNEL_4, 30);
				break;

		case('4'):
				*IR_Character = '\0';
				*P_Pitch += 1;
				sprintf((char*) uart_buffer,"P_Pitch: %d\n",(int) *P_Pitch);
				HAL_UART_Transmit(UART_handle, uart_buffer, strlen((char*)uart_buffer), 100);
				break;

		case('5'):
				*IR_Character = '\0';
				*P_Roll += 1;
				sprintf((char*) uart_buffer,"P_Roll: %d\n",(int) *P_Roll);
				HAL_UART_Transmit(UART_handle, uart_buffer, strlen((char*)uart_buffer), 100);
				break;

		case('6'):
				*IR_Character = '\0';
				*P_Yaw += 1;
				sprintf((char*) uart_buffer,"P_Yaw: %d\n",(int) *P_Yaw);
				HAL_UART_Transmit(UART_handle, uart_buffer, strlen((char*)uart_buffer), 100);
				break;

		case('7'):
				*IR_Character = '\0';
				*P_Pitch -= 1;
				sprintf((char*) uart_buffer,"P_Pitch: %d\n",(int) *P_Pitch);
				HAL_UART_Transmit(UART_handle, uart_buffer, strlen((char*)uart_buffer), 100);
				break;

		case('8'):
				*IR_Character = '\0';
				*P_Roll -= 1;
				sprintf((char*) uart_buffer,"P_Roll: %d\n",(int) *P_Roll);
				HAL_UART_Transmit(UART_handle, uart_buffer, strlen((char*)uart_buffer), 100);
				break;

		case('9'):
				*IR_Character = '\0';
				*P_Yaw -= 1;
				sprintf((char*) uart_buffer,"P_Yaw: %d\n",(int) *P_Yaw);
				HAL_UART_Transmit(UART_handle, uart_buffer, strlen((char*)uart_buffer), 100);
				break;
		case('O'):
				*IR_Character = '\0';
				*stop_flag = 0;
				break;
	}
}


