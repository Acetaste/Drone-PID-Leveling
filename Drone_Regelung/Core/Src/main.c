/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include "decode.h"
#include "calculations.h"
#include "startup.h"
#include "read_sensor.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define initial_uncertainty 2
#define Looptime 0.002 //seconds between measurements

#define P_outer 2
#define I_outer 0
#define D_outer 0

#define P_const 0.6
#define I_const 3.5
#define D_const 0.03

#define desired_Yaw 0

#define standard_acc_range  6
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c3;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim7;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C3_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM6_Init(void);
static void MX_TIM7_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


unsigned int binary =0;
static uint16_t timer_val;
static uint8_t stop_flag= 1;
static uint8_t transmission = 0;
static uint8_t data_index = 0;
static uint8_t get_data_flag = 0;
static char IR_Character = '\0';
static float KalmanRollAngle =0, KalmanPitchAngle = 0;
static float KalmanRollUncertainty = 2,KalmanPitchUncertainty = 2;

static float desired_Roll_Rate= 0, desired_Pitch_Rate= 0 , desired_Yaw_Rate= 0;
static float Error_Roll_Rate = 0, Error_Pitch_Rate= 0, Error_Yaw_Rate= 0;
static float Prev_Error_Roll_Rate = 0, Prev_Error_Pitch_Rate = 0, Prev_Error_Yaw_Rate = 0;
static float Prev_Int_Roll_Rate = 0, Prev_Int_Pitch_Rate = 0, Prev_Int_Yaw_Rate = 0;
static float Error_Roll_Ang= 0, Error_Pitch_Ang= 0, Error_Yaw_Ang= 0;
static float Prev_Error_Roll_Ang=0, Prev_Error_Pitch_Ang=0, Prev_Error_Yaw_Ang=0;
static float Prev_Int_Roll_Ang=0, Prev_Int_Pitch_Ang=0, Prev_Int_Yaw_Ang=0;


/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_I2C3_Init();
  MX_TIM1_Init();
  MX_TIM6_Init();
  MX_TIM7_Init();
  /* USER CODE BEGIN 2 */
  float KalmanOutput[2] = {0,0};
  int MotorInput[4] = {0,0,0,0};
  static float PID_Output[2] = {0,0};
  uint8_t word[50];
  float gyro_rate[3], acc_rate[3];
  float accel_pitch, accel_roll;
  int counter = 0;
  float InputPitch, InputRoll, InputYaw;
  float P_Pitch = 1;
  float P_Roll = 1;
  float P_Yaw = 1;
  float desired_Pitch 	= 0;
  float desired_Roll	= 0;
  float yaw = 0;


  configure_imu(&hi2c3, &huart2);
  timer_start(&timer_val,&htim1, &htim6, &htim7);



  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  interpret_flags(&IR_Character, &desired_Pitch, &desired_Roll, &stop_flag, &htim1, &huart2, &P_Pitch,&P_Roll, &P_Yaw);
	  if(get_data_flag ==1)
	  {
		  get_data_flag =0;
		  get_gyr_data(gyro_rate, &hi2c3, &huart2);
		  get_acc_data(acc_rate, standard_acc_range, &hi2c3, &huart2);

		  accel_pitch = acc_pitch((*(acc_rate+0)),(*(acc_rate+1)),(*(acc_rate+2)));
		  accel_roll = acc_roll((*(acc_rate+0)),(*(acc_rate+1)),(*(acc_rate+2)));
		  yaw += gyro_rate[2]* Looptime;
		  yaw = yaw_cap(yaw);

		  KalmanCalculation(KalmanRollAngle,KalmanRollUncertainty,gyro_rate[0], accel_roll, KalmanOutput);
		  KalmanRollAngle 			= KalmanOutput[0];
		  KalmanRollUncertainty 	= KalmanOutput[1];
		  KalmanCalculation(KalmanPitchAngle,KalmanPitchUncertainty,gyro_rate[1], accel_pitch, KalmanOutput);
		  KalmanPitchAngle 			= KalmanOutput[0];
		  KalmanPitchUncertainty 	= KalmanOutput[1];


		  Error_Roll_Ang 	= 	desired_Roll 		-KalmanRollAngle;
		  Error_Pitch_Ang 	= 	desired_Pitch		-KalmanPitchAngle;
		  Error_Yaw_Ang 	= 	desired_Yaw 		-yaw;

		  pid_equation(Error_Roll_Ang, Prev_Error_Roll_Ang, Prev_Int_Roll_Ang, P_outer, I_outer, D_outer, PID_Output);
		  Prev_Error_Roll_Ang 	= Error_Roll_Ang;
		  desired_Roll_Rate 	= PID_Output[0];
		  Prev_Int_Roll_Ang 	= PID_Output[1];

		  pid_equation(Error_Pitch_Ang, Prev_Error_Pitch_Ang, Prev_Int_Pitch_Ang, P_outer, I_outer, D_outer, PID_Output);
		  Prev_Error_Pitch_Ang 	= Error_Pitch_Ang;
		  desired_Pitch_Rate 	= PID_Output[0];
		  Prev_Int_Pitch_Ang 	= PID_Output[1];

		  pid_equation(Error_Yaw_Ang, Prev_Error_Yaw_Ang, Prev_Int_Yaw_Ang, P_outer, I_outer, D_outer, PID_Output);
		  Prev_Error_Yaw_Ang 	= Error_Yaw_Ang;
		  desired_Yaw_Rate		= PID_Output[0];
		  Prev_Int_Yaw_Ang		= PID_Output[1];

		  Error_Roll_Rate 	= desired_Roll_Rate 	- gyro_rate[0];
		  Error_Pitch_Rate 	= desired_Pitch_Rate 	- gyro_rate[1];
		  Error_Yaw_Rate 	= desired_Yaw_Rate 		- gyro_rate[2];

		  pid_equation(Error_Roll_Rate, Prev_Error_Roll_Rate, Prev_Int_Roll_Rate,  		1, 0, 0, PID_Output);
		  Prev_Error_Roll_Rate 	= Error_Roll_Rate;
		  InputRoll 			= PID_Output[0];
		  Prev_Int_Roll_Rate 	= PID_Output[1];

		  pid_equation(Error_Pitch_Rate, Prev_Error_Pitch_Rate, Prev_Int_Pitch_Rate, 	1, 0, 0, PID_Output);
		  Prev_Error_Pitch_Rate = Error_Pitch_Rate;
		  InputPitch 			= PID_Output[0];
		  Prev_Int_Pitch_Rate 	= PID_Output[1];

		  pid_equation(Error_Yaw_Rate, Prev_Error_Yaw_Rate, Prev_Int_Yaw_Rate, 			3, 0, 0, PID_Output);
		  Prev_Error_Yaw_Rate	= Error_Yaw_Rate;
		  InputYaw	 			= PID_Output[0];
		  Prev_Int_Yaw_Rate 	= PID_Output[1];

		  motor_inputs( InputRoll, InputPitch, InputYaw, MotorInput);
		  if (counter>= 99)
		  {
			  sprintf((char*) word,"Roll: %d Pitch: %d Yaw: %d\n",(int)KalmanRollAngle,(int)KalmanPitchAngle,(int) yaw);
			  HAL_UART_Transmit(&huart2, word, strlen((char*)word), 100);
			  sprintf((char*) word,"InputRoll: %d,InputPitch: %d,InputYaw: %d\n",(int) InputRoll,(int) InputPitch,(int) InputYaw);
			  HAL_UART_Transmit(&huart2, word, strlen((char*)word), 100);
			  counter = 0;
		  }
		  else
		  {
		  		counter ++;
		  }
		 /*sprintf((char*) word,"Motor1: %d,Motor2: %d,Motor3: %d,Motor4: %d,\n",(int)(*(MotorInput+0)),(int)(*(MotorInput+1)),(int)(*(MotorInput+2)),(int)(*(MotorInput+3)));
		  HAL_UART_Transmit(&huart2, word, strlen((char*)word), 100);*/

		  if(stop_flag ==0)
		  {
			  __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1, pwm_cap(*(MotorInput+0)));
			  __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2, pwm_cap(*(MotorInput+1)));
			  __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3, pwm_cap(*(MotorInput+2)));
			  __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_4, pwm_cap(*(MotorInput+3)));
		  }
	  }


    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSE|RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 16;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enable MSI Auto calibration
  */
  HAL_RCCEx_EnableMSIPLLMode();
}

/**
  * @brief I2C3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C3_Init(void)
{

  /* USER CODE BEGIN I2C3_Init 0 */

  /* USER CODE END I2C3_Init 0 */

  /* USER CODE BEGIN I2C3_Init 1 */

  /* USER CODE END I2C3_Init 1 */
  hi2c3.Instance = I2C3;
  hi2c3.Init.Timing = 0x00B07CB4;
  hi2c3.Init.OwnAddress1 = 0;
  hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c3.Init.OwnAddress2 = 0;
  hi2c3.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c3) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c3, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c3, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C3_Init 2 */

  /* USER CODE END I2C3_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 32-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 1000-1;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 32-1;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 65535;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief TIM7 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM7_Init(void)
{

  /* USER CODE BEGIN TIM7_Init 0 */

  /* USER CODE END TIM7_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM7_Init 1 */

  /* USER CODE END TIM7_Init 1 */
  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 32-1;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 2000-1;
  htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM7_Init 2 */

  /* USER CODE END TIM7_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : LD3_Pin */
  GPIO_InitStruct.Pin = LD3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD3_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : GYR_IT_Pin */
  GPIO_InitStruct.Pin = GYR_IT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GYR_IT_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	int duration;


	if(GPIO_Pin == GPIO_PIN_1)
	{
		duration = (__HAL_TIM_GET_COUNTER(&htim6)-timer_val);
		timer_val = __HAL_TIM_GET_COUNTER(&htim6);
		if(duration <0)
		{
			duration = duration + 65536;
		}
		get_data_flag=0;

		if(transmission == 1 )
		{

			if(duration>1700 && data_index != 0)
			{
				binary |= (1 << data_index);

			}

			data_index++;
			if(data_index>=31)
			{
				transmission = 0;
				IR_Character =decode(binary);
				HAL_NVIC_EnableIRQ(TIM7_IRQn);

				if( IR_Character == '1')
				{
					stop_flag = 1;
					__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1, 0);
					__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2, 0);
					__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3, 0);
					__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_4, 0);
				}
			}
		}
		if(transmission== 0)
		{
			if( (5500>duration)  && (duration> 4500))
			{
				transmission = 1;
				binary = 0;
				data_index = 0;
				binary = 0;
				get_data_flag= 0;
				HAL_NVIC_DisableIRQ(TIM7_IRQn);
			}
		}


	}

}
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{

	if(htim == &htim7)
	{
		if(transmission== 0 )
		{
			get_data_flag= 1;
		}

	}
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
