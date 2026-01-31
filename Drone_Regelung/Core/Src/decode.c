/*
 * decode.c
 *
 *  Created on: Nov 2, 2025
 *      Author: matthias Ellinger
 */
#include "decode.h"

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

void interpret_flags(char* IR_Character, float* desired_Pitch, float* desired_Roll, uint8_t* stop_flag, TIM_HandleTypeDef* pwm_timer_handle, UART_HandleTypeDef* UART_handle, float* P_Pitch,float* P_Roll, float* P_Yaw)
{
	uint8_t word[50];
	switch(*IR_Character)
	{
		case('L'):
				*IR_Character = '\0';
				*desired_Pitch += 5;
			  	sprintf((char*) word,"Desired Pitch: %d\n",(int) *desired_Pitch);
			  	HAL_UART_Transmit(UART_handle, word, strlen((char*)word), 100);
			  	break;

		case('R'):
				*IR_Character = '\0';
				*desired_Pitch -= 5;
				sprintf((char*) word,"Desired Pitch: %d\n",(int) *desired_Pitch);
			  	HAL_UART_Transmit(UART_handle, word, strlen((char*)word), 100);
			  	break;

		case('U'):
				*IR_Character = '\0';
				*desired_Roll += 5;
				sprintf((char*) word,"Desired Roll: %d\n",(int) *desired_Roll);
				HAL_UART_Transmit(UART_handle, word, strlen((char*)word), 100);
				break;

		case('D'):
				*IR_Character = '\0';
				*desired_Roll -= 5;
				sprintf((char*) word,"Desired Roll: %d\n",(int) *desired_Roll);
				HAL_UART_Transmit(UART_handle, word, strlen((char*)word), 100);
				break;

		case('#'):
				*IR_Character = '\0';
				*desired_Roll = 0;
				*desired_Pitch = 0;
				sprintf((char*) word,"Desired Roll: %d, Desired Pitch: %d\n",(int) *desired_Roll, (int) *desired_Pitch);
				HAL_UART_Transmit(UART_handle, word, strlen((char*)word), 100);
				break;

		case('*'):
				*IR_Character = '\0';
				*P_Pitch 	= 1;
				*P_Roll 	= 1;
				*P_Yaw 		= 1;
				sprintf((char*) word,"P Pitch: %d, P Roll: %d, P Yaw: %d\n",(int) *P_Pitch, (int) *P_Roll,(int) *P_Yaw);
				HAL_UART_Transmit(UART_handle, word, strlen((char*)word), 100);
				break;

		case('2'):
				*IR_Character = '\0';
				*stop_flag = 1;
				__HAL_TIM_SET_COMPARE(pwm_timer_handle,TIM_CHANNEL_1, 30);
				__HAL_TIM_SET_COMPARE(pwm_timer_handle,TIM_CHANNEL_2, 30);
				__HAL_TIM_SET_COMPARE(pwm_timer_handle,TIM_CHANNEL_3, 30);
				__HAL_TIM_SET_COMPARE(pwm_timer_handle,TIM_CHANNEL_4, 30);
				break;
		case('O'):
				*IR_Character = '\0';
				*stop_flag = 0;
				break;

		case('4'):
				*IR_Character = '\0';
				*P_Pitch += 1;
				sprintf((char*) word,"P_Pitch: %d\n",(int) *P_Pitch);
				HAL_UART_Transmit(UART_handle, word, strlen((char*)word), 100);
				break;

		case('5'):
				*IR_Character = '\0';
				*P_Roll += 1;
				sprintf((char*) word,"P_Roll: %d\n",(int) *P_Roll);
				HAL_UART_Transmit(UART_handle, word, strlen((char*)word), 100);
				break;

		case('6'):
				*IR_Character = '\0';
				*P_Yaw += 1;
				sprintf((char*) word,"P_Yaw: %d\n",(int) *P_Yaw);
				HAL_UART_Transmit(UART_handle, word, strlen((char*)word), 100);
				break;

		case('7'):
				*IR_Character = '\0';
				*P_Pitch -= 1;
				sprintf((char*) word,"P_Pitch: %d\n",(int) *P_Pitch);
				HAL_UART_Transmit(UART_handle, word, strlen((char*)word), 100);
				break;

		case('8'):
				*IR_Character = '\0';
				*P_Roll -= 1;
				sprintf((char*) word,"P_Roll: %d\n",(int) *P_Roll);
				HAL_UART_Transmit(UART_handle, word, strlen((char*)word), 100);
				break;

		case('9'):
				*IR_Character = '\0';
				*P_Yaw -= 1;
				sprintf((char*) word,"P_Yaw: %d\n",(int) *P_Yaw);
				HAL_UART_Transmit(UART_handle, word, strlen((char*)word), 100);
				break;

}

}



