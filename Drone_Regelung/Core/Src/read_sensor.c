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


/**
 * @brief function to read the gyroscopes X, Y, and Z axis data over I2C
 * @param Gyro_Rate a pointer to an array of 3 where the scaled gyroscope data is written to
 * @param I2C_Handle the handle of the I2C interface used
 * @param UART_Handle the handle of the UART interface used
 */
void get_gyr_data(float* Gyro_Rate, I2C_HandleTypeDef* I2C_Handle ,UART_HandleTypeDef* UART_Handle)
{
	HAL_StatusTypeDef ret;
	uint8_t data_buffer[50];
	int16_t twoByte;
	uint8_t byteLSB, byteMSB;
	data_buffer[0]= GYR_X_ADDR;
	ret= HAL_I2C_Master_Transmit(I2C_Handle, GYR_ADDR, data_buffer, 1, 100);
	if(ret != HAL_OK)
	{
		strcpy((char*)data_buffer,"Error Gyr Transmit \n");
		HAL_UART_Transmit(UART_Handle, data_buffer, strlen((char*)data_buffer), 100);
	}
	else
	{
		HAL_I2C_Master_Receive(I2C_Handle, GYR_ADDR, data_buffer, 6, 100);
		if(ret != HAL_OK)
		{
			strcpy((char*)data_buffer,"Error Gyr Received \n");
			HAL_UART_Transmit(UART_Handle, data_buffer, strlen((char*)data_buffer), 100);
		}
		else
		{
			for(int i= 0; i<3; i++)
			{
				byteLSB = data_buffer[i*2];
				byteMSB = data_buffer[i*2+1];
				twoByte = byteMSB<<8 |byteLSB;
				*(Gyro_Rate+i)= twoByte*GYRO_RANGE/32768;
			}
		}
	}
}

/**
 * @brief function to read the accelerometer X, Y, and Z axis data over I2C
 * @param Acc_Rate a pointer to an array of 3 where the scaled accelerometer data is written to
 * @param Acc_Range the value of the accelerometer range set in the sensor
 * @param I2C_Handle the handle of the I2C interface used
 * @param UART_Handle the handle of the UART interface used
 */
void get_acc_data(float* Acc_Rate,  int Acc_Range, I2C_HandleTypeDef* I2C_Handle ,UART_HandleTypeDef* UART_Handle)
{
	HAL_StatusTypeDef ret;
	uint8_t data_buffer[50];
	int16_t twoByte;
	uint8_t byteLSB, byteMSB;

	data_buffer[0]= ACC_X_ADDR;
	ret= HAL_I2C_Master_Transmit(I2C_Handle, ACC_ADDR, data_buffer, 1, 100);
	if(ret != HAL_OK)
	{
		strcpy((char*)data_buffer,"Error ACC Transmit \n");
		HAL_UART_Transmit(UART_Handle, data_buffer, strlen((char*)data_buffer), 100);
	}
	else
	{
		HAL_I2C_Master_Receive(I2C_Handle, ACC_ADDR, data_buffer, 6, 100);
		if(ret != HAL_OK)
		{
			strcpy((char*)data_buffer,"Error ACC Received \n");
			HAL_UART_Transmit(UART_Handle, data_buffer, strlen((char*)data_buffer), 100);
		}
		else
		{
			for(int i= 0; i<3; i++)
			{
				byteLSB = data_buffer[i*2];
				byteMSB = data_buffer[i*2+1];
				twoByte = byteMSB<<8 |byteLSB;
				*(Acc_Rate+i)= twoByte*Acc_Range*1000/32768;
			}

		}
	}
}
