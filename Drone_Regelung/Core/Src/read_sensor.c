/*
 * read_sensor.c
 *
 *  Created on: 31.1.12, 2026
 *      Author: matthias Ellinger
 */




#include "read_sensor.h"

#define GYR_ADDR (0x69u <<1)
#define ACC_ADDR (0x18u <<1)
#define GYR_X_ADDR 0x02u
#define ACC_X_ADDR 0x12u
#define gyro_range  2000


void get_gyr_data(float* gyro_rate, I2C_HandleTypeDef* I2C_handle ,UART_HandleTypeDef* UART_handle)
{
	HAL_StatusTypeDef ret;
	uint8_t word[50];
	int16_t twoByte;
	uint8_t byteLSB, byteMSB;
	word[0]= GYR_X_ADDR;
	ret= HAL_I2C_Master_Transmit(I2C_handle, GYR_ADDR, word, 1, 100);
	if(ret != HAL_OK)
	{
		strcpy((char*)word,"Error Gyr Transmit \n");
		HAL_UART_Transmit(UART_handle, word, strlen((char*)word), 100);
	}
	else
	{
		HAL_I2C_Master_Receive(I2C_handle, GYR_ADDR, word, 6, 100);
		if(ret != HAL_OK)
		{
			strcpy((char*)word,"Error Gyr Received \n");
			HAL_UART_Transmit(UART_handle, word, strlen((char*)word), 100);
		}
		else
		{
			for(int i= 0; i<3; i++)
			{
				byteLSB = word[i*2];
				byteMSB = word[i*2+1];
				twoByte = byteMSB<<8 |byteLSB;
				*(gyro_rate+i)= twoByte*gyro_range/32768;
			}
		}
	}
}


void get_acc_data(float* acc_rate,  int acc_range, I2C_HandleTypeDef* I2C_handle ,UART_HandleTypeDef* UART_handle)
{
	HAL_StatusTypeDef ret;
	uint8_t word[50];
	int16_t twoByte;
	uint8_t byteLSB, byteMSB;

	word[0]= ACC_X_ADDR;
	ret= HAL_I2C_Master_Transmit(I2C_handle, ACC_ADDR, word, 1, 100);
	if(ret != HAL_OK)
	{
		strcpy((char*)word,"Error ACC Transmit \n");
		HAL_UART_Transmit(UART_handle, word, strlen((char*)word), 100);
	}
	else
	{
		HAL_I2C_Master_Receive(I2C_handle, ACC_ADDR, word, 6, 100);
		if(ret != HAL_OK)
		{
			strcpy((char*)word,"Error ACC Received \n");
			HAL_UART_Transmit(UART_handle, word, strlen((char*)word), 100);
		}
		else
		{
			for(int i= 0; i<3; i++)
			{
				byteLSB = word[i*2];
				byteMSB = word[i*2+1];
				twoByte = byteMSB<<8 |byteLSB;
				*(acc_rate+i)= twoByte*acc_range*1000/32768;
			}
		}
	}
}
