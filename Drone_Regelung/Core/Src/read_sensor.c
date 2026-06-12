/*
 * read_sensor.c
 *
 *  Created on: 31.1.12, 2026
 *      Author: matthias Ellinger
 */



#include <stdio.h>
#include <string.h>

#include "read_sensor.h"
#include "user_constants.h"



void get_gyr_data(float* gyro_rate, I2C_HandleTypeDef* I2C_handle ,UART_HandleTypeDef* UART_handle)
{
	HAL_StatusTypeDef ret;
	uint8_t data_buffer[50];
	int16_t twoByte;
	uint8_t byteLSB, byteMSB;
	data_buffer[0]= GYR_X_ADDR;
	ret= HAL_I2C_Master_Transmit(I2C_handle, GYR_ADDR, data_buffer, 1, 100);
	if(ret != HAL_OK)
	{
		strcpy((char*)data_buffer,"Error Gyr Transmit \n");
		HAL_UART_Transmit(UART_handle, data_buffer, strlen((char*)data_buffer), 100);
	}
	else
	{
		HAL_I2C_Master_Receive(I2C_handle, GYR_ADDR, data_buffer, 6, 100);
		if(ret != HAL_OK)
		{
			strcpy((char*)data_buffer,"Error Gyr Received \n");
			HAL_UART_Transmit(UART_handle, data_buffer, strlen((char*)data_buffer), 100);
		}
		else
		{
			for(int i= 0; i<3; i++)
			{
				byteLSB = data_buffer[i*2];
				byteMSB = data_buffer[i*2+1];
				twoByte = byteMSB<<8 |byteLSB;
				*(gyro_rate+i)= twoByte*GYRO_RANGE/32768;
			}
		}
	}
}


void get_acc_data(float* acc_rate,  int acc_range, I2C_HandleTypeDef* I2C_handle ,UART_HandleTypeDef* UART_handle)
{
	HAL_StatusTypeDef ret;
	uint8_t data_buffer[50];
	int16_t twoByte;
	uint8_t byteLSB, byteMSB;

	data_buffer[0]= ACC_X_ADDR;
	ret= HAL_I2C_Master_Transmit(I2C_handle, ACC_ADDR, data_buffer, 1, 100);
	if(ret != HAL_OK)
	{
		strcpy((char*)data_buffer,"Error ACC Transmit \n");
		HAL_UART_Transmit(UART_handle, data_buffer, strlen((char*)data_buffer), 100);
	}
	else
	{
		HAL_I2C_Master_Receive(I2C_handle, ACC_ADDR, data_buffer, 6, 100);
		if(ret != HAL_OK)
		{
			strcpy((char*)data_buffer,"Error ACC Received \n");
			HAL_UART_Transmit(UART_handle, data_buffer, strlen((char*)data_buffer), 100);
		}
		else
		{
			for(int i= 0; i<3; i++)
			{
				byteLSB = data_buffer[i*2];
				byteMSB = data_buffer[i*2+1];
				twoByte = byteMSB<<8 |byteLSB;
				*(acc_rate+i)= twoByte*acc_range*1000/32768;
			}

		}
	}
}
