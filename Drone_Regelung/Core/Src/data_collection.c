/*
 * data_collection.c
 *
 *  Created on: 25.3 2026
 *      Author: matthias Ellinger
 */




#include "data_collection.h"

void send_header(struct data_header_struct* data_header, UART_HandleTypeDef* UART_handle)
{
	uint8_t uart_buffer[50];
	sprintf((char*) uart_buffer,"HEADER:P%dR%dY%dT%d\n", data_header->desired_pitch,data_header->desired_roll,data_header->desired_yaw,data_header->loop_time);
	HAL_UART_Transmit(UART_handle, uart_buffer, strlen((char*)uart_buffer), 100);
}

void send_collected_data(int16_t* data_buff, int16_t numb_measurements, UART_HandleTypeDef* UART_handle)
{
	for(int i = 0;i < numb_measurements; i++)
	{
		uint8_t uart_buffer[50];
		sprintf((char*) uart_buffer,"%d%d%d%d\n", data_buff[(4*i)], data_buff[(4*i)+1], data_buff[(4*i)+2], data_buff[(4*i)+3]);
		HAL_UART_Transmit(UART_handle, uart_buffer, strlen((char*)uart_buffer), 100);
	}
}
