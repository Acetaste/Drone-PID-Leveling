/*
 * data_collection.h
 *
 *  Created on: 25.3, 2026
 *      Author: matthias ellinger
 */

#ifndef INC_DATA_COLLECTION_H_
#define INC_DATA_COLLECTION_H_

#include "stm32l4xx_hal.h"
#include <ctype.h>
#include <stdio.h>
#include <string.h>

struct data_header_struct
{
	int16_t desired_pitch;
	int16_t desired_roll;
	int16_t desired_yaw;
	int16_t loop_time;
	int16_t numb_measurements;


};

void send_header(struct data_header_struct* data_header, UART_HandleTypeDef* UART_handle);
void send_collected_data(int16_t* data_buff, int16_t numb_measurements, UART_HandleTypeDef* UART_handle);

#endif /* INC_DATA_COLLECTION_H_ */
