/*
 * startup.c
 *
 *  Created on: 31.1.12, 2026
 *      Author: matthias ellinger
 */
#define GYR_ADDR (0x69u <<1)
#define ACC_ADDR (0x18u <<1)

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


#include "startup.h"
#include "read_sensor.h"


void timer_start(uint16_t* timer_val, TIM_HandleTypeDef* pwm_timer_handle, TIM_HandleTypeDef* IR_remote_duration_timer_handle, TIM_HandleTypeDef* flag_setting_timer_handle )
{
	HAL_TIM_Base_Start(IR_remote_duration_timer_handle);
	(*timer_val) = __HAL_TIM_GET_COUNTER(IR_remote_duration_timer_handle);
	HAL_TIM_Base_Start_IT(flag_setting_timer_handle);
	HAL_TIM_PWM_Start(pwm_timer_handle, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(pwm_timer_handle, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(pwm_timer_handle, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(pwm_timer_handle, TIM_CHANNEL_4);
}


void configure_imu(I2C_HandleTypeDef* I2C_handle ,UART_HandleTypeDef* UART_handle)
{
	 HAL_StatusTypeDef ret;
	 uint8_t word[50];

	  //Setup Active Power Mode and Power on to Accelerometer
	 word[0] = ACC_ACTIVE_MODE;
	 ret = HAL_I2C_Mem_Write(I2C_handle, ACC_ADDR, ACC_PWR_CONF_ADDR, 1, word, 1, 100);
	 if(ret != HAL_OK)
	 {
		strcpy((char*)word,"Error Active Mode ACC\n");
		HAL_UART_Transmit(UART_handle, word, strlen((char*)word), HAL_MAX_DELAY);
	 }
	 word[0] = ACC_POWER_ON;
	 ret = HAL_I2C_Mem_Write(I2C_handle, ACC_ADDR, ACC_PWR_CNTRL_ADDR, 1, word, 1, 100);
	 if(ret != HAL_OK)
	 {
	  	strcpy((char*)word,"Error Power ON ACC\n");
	  	HAL_UART_Transmit(UART_handle, word, strlen((char*)word), HAL_MAX_DELAY);
	 }

}


// SELF TEST FEATURE NOT IMPLEMENTED YET
void selftest_accel(I2C_HandleTypeDef* I2C_handle ,UART_HandleTypeDef* UART_handle)
{
	uint8_t word[80];
	HAL_StatusTypeDef ret;
	float* acc_value =0;
	float diffrence_x, diffrence_y, diffrence_z;

	//Set Selftest range
	word[0] = 0x03;
	ret = HAL_I2C_Mem_Write(I2C_handle, ACC_ADDR, ACC_RANGE_ADDR, 1, word, 1, 100);
	if(ret != HAL_OK)
	{
		strcpy((char*)word,"Error setting Selftest Accelerometer range\n");
		HAL_UART_Transmit(UART_handle, word, strlen((char*)word), HAL_MAX_DELAY);
	}
	//Set Selftest filter
	word[0] = 0xA7;
	ret = HAL_I2C_Mem_Write(I2C_handle, ACC_ADDR, ACC_CONF_ADDR, 1, word, 1, 100);
	if(ret != HAL_OK)
	{
		strcpy((char*)word,"Error setting Selftest Accelerometer filter\n");
		HAL_UART_Transmit(UART_handle, word, strlen((char*)word), HAL_MAX_DELAY);
	}
	HAL_Delay(10);
	//Enable positive polarity selftest
	word[0] = 0x0D;
	ret = HAL_I2C_Mem_Write(I2C_handle, ACC_ADDR, ACC_SELF_TEST_ADDR, 1, word, 1, 100);
	if(ret != HAL_OK)
	{
		strcpy((char*)word,"Error enabling Accelerometer Selftest\n");
		HAL_UART_Transmit(UART_handle, word, strlen((char*)word), HAL_MAX_DELAY);
	}
	HAL_Delay(75);

	get_acc_data(acc_value, 24, I2C_handle, UART_handle);
	diffrence_x = *acc_value;
	diffrence_y = *(acc_value+1);
	diffrence_z = *(acc_value+2);

	//Enable negative polarity selftest
	word[0] = 0x09;
	ret = HAL_I2C_Mem_Write(I2C_handle, ACC_ADDR, ACC_SELF_TEST_ADDR, 1, word, 1, 100);
	if(ret != HAL_OK)
	{
		strcpy((char*)word,"Error enabling Accelerometer Selftest\n");
		HAL_UART_Transmit(UART_handle, word, strlen((char*)word), HAL_MAX_DELAY);
	}
	HAL_Delay(75);
	get_acc_data(acc_value, 24, I2C_handle, UART_handle);
	diffrence_x = diffrence_x - *(acc_value);
	diffrence_y = diffrence_y - *(acc_value+1);
	diffrence_z = diffrence_z - *(acc_value+2);

	//disable Selftest again
	word[0] = 0x00;
	ret = HAL_I2C_Mem_Write(I2C_handle, ACC_ADDR, ACC_SELF_TEST_ADDR, 1, word, 1, 100);
	if(ret != HAL_OK)
	{
		strcpy((char*)word,"Error  disabling Accelerometer Selftest\n");
		HAL_UART_Transmit(UART_handle, word, strlen((char*)word), HAL_MAX_DELAY);
	}

	//set range to standard 3g
	word[0] = 0x00;
	ret = HAL_I2C_Mem_Write(I2C_handle, ACC_ADDR, ACC_RANGE_ADDR, 1, word, 1, 100);
	if(ret != HAL_OK)
	{
		strcpy((char*)word,"Error setting Selftest Accelerometer range\n");
		HAL_UART_Transmit(UART_handle, word, strlen((char*)word), HAL_MAX_DELAY);
	}

	//print self test value
	sprintf((char*) word,"Selftest X value: %d, Selftest Y value: %d, Selftest Z value: %d\n",(int) diffrence_x,(int) diffrence_y, (int) diffrence_z);
	HAL_UART_Transmit(UART_handle, word, strlen((char*)word), 100);

}
