/*
 * read_sensor.h
 *
 *  Created on: 31.1.12, 2026
 *      Author: matthias ellinger
 */

#ifndef INC_READ_SENSOR_H_
#define INC_READ_SENSOR_H_

#include "stm32l4xx_hal.h"


void get_gyr_data(float* gyro_rate, I2C_HandleTypeDef* I2C_handle ,UART_HandleTypeDef* UART_handle);
void get_acc_data(float* acc_rate,  int acc_range, I2C_HandleTypeDef* I2C_handle ,UART_HandleTypeDef* UART_handle);

#endif /* INC_READ_SENSOR_H_ */
