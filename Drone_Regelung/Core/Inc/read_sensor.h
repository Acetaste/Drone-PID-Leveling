/*
 * read_sensor.h
 *
 *  Created on: 31.1.12, 2026
 *      Author: matthias ellinger
 */

#ifndef INC_READ_SENSOR_H_
#define INC_READ_SENSOR_H_

#include "stm32l4xx_hal.h"


void get_gyr_data(float* Gyro_Rate, I2C_HandleTypeDef* I2C_Handle ,UART_HandleTypeDef* UART_Handle);
void get_acc_data(float* Acc_Rate,  int Acc_Range, I2C_HandleTypeDef* I2C_Handle ,UART_HandleTypeDef* UART_Handle);

#endif /* INC_READ_SENSOR_H_ */
