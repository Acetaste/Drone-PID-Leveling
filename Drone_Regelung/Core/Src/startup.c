/*
 * startup.c
 *
 *  Created on: 31.1.12, 2026
 *      Author: matthias ellinger
 */



#include "startup.h"
#include "read_sensor.h"
#include "user_constants.h"


/**
 * @brief function to start timers and set associated variables
 * @param Timer_Val a pointer to the variable keeping track of the time passed since the last IR interrupt
 * @param PWM_Timer_Handle the handle to the timer used for PWM generation
 * @param IR_Remote_Duration_timer_Handle the handle to the timer used for timing IR receiver interrupts
 * @param Flag_Setting_Timer_Handle the handle to the timer used for timing the main loop
 */
void timer_start(uint16_t* Timer_Val, TIM_HandleTypeDef* PWM_Timer_Handle, TIM_HandleTypeDef* IR_Remote_Duration_timer_Handle, TIM_HandleTypeDef* Flag_Setting_Timer_Handle)
{
	HAL_TIM_Base_Start(IR_Remote_Duration_timer_Handle);
	(*Timer_Val) = __HAL_TIM_GET_COUNTER(IR_Remote_Duration_timer_Handle);
	HAL_TIM_Base_Start_IT(Flag_Setting_Timer_Handle);
	HAL_TIM_PWM_Start(PWM_Timer_Handle, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(PWM_Timer_Handle, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(PWM_Timer_Handle, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(PWM_Timer_Handle, TIM_CHANNEL_4);
}

/**
 * @brief function to configure IMU on start up
 * @param I2C_Handle the handle of the I2C interface used
 * @param UART_Handle the handle of the UART interface used
 */
void configure_imu(I2C_HandleTypeDef* I2C_Handle ,UART_HandleTypeDef* UART_Handle)
{
	 HAL_StatusTypeDef ret;
	 uint8_t word[50];

	  //Setup Active Power Mode and Power on to Accelerometer
	 word[0] = ACC_ACTIVE_MODE;
	 ret = HAL_I2C_Mem_Write(I2C_Handle, ACC_ADDR, ACC_PWR_CONF_ADDR, 1, word, 1, 100);
	 if(ret != HAL_OK)
	 {
		strcpy((char*)word,"Error Active Mode ACC\n");
		HAL_UART_Transmit(UART_Handle, word, strlen((char*)word), HAL_MAX_DELAY);
	 }
	 word[0] = ACC_POWER_ON;
	 ret = HAL_I2C_Mem_Write(I2C_Handle, ACC_ADDR, ACC_PWR_CNTRL_ADDR, 1, word, 1, 100);
	 if(ret != HAL_OK)
	 {
	  	strcpy((char*)word,"Error Power ON ACC\n");
	  	HAL_UART_Transmit(UART_Handle, word, strlen((char*)word), HAL_MAX_DELAY);
	 }

}


