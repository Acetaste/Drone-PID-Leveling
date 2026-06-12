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
#define NUMB_MEASUREMENTS 3000				//Number of measurements to be acquired during a test run, used for buffer size allocation
#define P_OUTER 1
#define I_OUTER 0
#define D_OUTER 0
#define STANDARD_ACC_RANGE  6
#undef TUNING_MODE

#define P_ROLL_INNER 6
#define I_ROLL_INNER 2
#define D_ROLL_INNER 0.001

#define P_PITCH_INNER 2
#define I_PITCH_INNER 1
#define D_PITCH_INNER 0.001

#define P_YAW_INNER 4
#define I_YAW_INNER 4
#define D_YAW_INNER 0



//used in calculations.c
#define PWM_CAP 170
#define I_CAP 100
#define PID_CAP 150
#define GYRO_ERROR 4
#define ACC_ERROR 2
#define RAD_TO_DEGREE 57.29578f
#define DEGREE_TO_RAD 0.017453f

//used in read_sensor.c

#define GYR_X_ADDR 0x02u
#define ACC_X_ADDR 0x12u
#define GYRO_RANGE  2000

//used in decode.c
#define MAX_ANGLE 90
#define P_VALUE_INCREMENT 0.01
#define MAX_CAP(a, b) (((a) < (b)) ? (a) : (b))

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
