/*
 * user_constants.h
 *
 *  Created on: May 4, 2026
 *      Author: matth
 */

#ifndef INC_USER_CONSTANTS_H_
#define INC_USER_CONSTANTS_H_

#include "stm32l4xx_hal.h"
//used in multiple places
#define LOOP_TIME_MS 3						//milliseconds between measurements
#define LOOP_TIME_S (LOOP_TIME_MS/1000.0f) 		//seconds between measurements
#define GYR_ADDR (0x69u <<1)				//i2c slave address
#define ACC_ADDR (0x18u <<1)				//i2c slave address
#define FLASH_REGION_START  0x08038000
//used in main.c
#define Numb_Measurements 3000				//Number of measurements to be acquired during a test run, used for buffer size allocation
#define P_outer 1
#define I_outer 0
#define D_outer 0
#define standard_acc_range  6

//used in calculations.c
#define PWM_CAP 160
#define I_CAP 100
#define PID_CAP 140
#define gyro_error 4
#define acc_error 2
#define rad_to_degree 57.29578f
#define degree_to_rad 0.017453f

//used in read_sensor.c

#define GYR_X_ADDR 0x02u
#define ACC_X_ADDR 0x12u
#define gyro_range  2000

//used in decode.c
#define P_Value_Increase 0.01
#define max_numb(a, b) (((a) < (b)) ? (a) : (b))

//used in startup.c
#define ACC_PWR_CONF_ADDR 0x7Cu
#define ACC_PWR_CNTRL_ADDR 0x7Du

#define ACC_RANGE_ADDR 0x41
#define ACC_CONF_ADDR 0x40
#define ACC_SELF_TEST_ADDR 0x6D

#define ACC_IT_MAP_ADDR 0x58u
#define GYR_IT_CNTRL_ADDR 0x15u
#define GYR_IT_MAP_ADDR 0x18u

#define ACC_POWER_ON 0x04u
#define ACC_ACTIVE_MODE 0x00u
#define ACC_IT_1_SELECT 0x04u
#define GYR_IT_POWER_ON 0x80u
#define GYR_IT_3_SELECT 0x01u

//used in data_collection.c
#define FLASH_NUMB_PAGES (FLASH_END-FLASH_REGION_START)/FLASH_PAGE_SIZE

#endif /* INC_USER_CONSTANTS_H_ */
