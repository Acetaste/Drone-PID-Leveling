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
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define GYR_ADDR (0x69u <<1)
#define ACC_ADDR (0x18u <<1)
#define GYR_X_ADDR 0x02u
#define ACC_X_ADDR 0x12u

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

#define gyro_range  2000
#define standard_acc_range  6
#define gyro_error 4
#define acc_error 3
#define initial_uncertainty 2
#define Looptime 0.002 //seconds between measurements
#define P_outer 2
#define P_const 0.6
#define I_const 3.5
#define D_const 0.03
#define desired_Yaw 0
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
void get_gyr_data(float* gyro_rate);
void get_acc_data(float* acc_rate,  int acc_range);
void configure_imu(void);
void timer_start(void);
void selftest_accel(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


unsigned int binary =0;
static uint16_t timer_val;
static uint8_t stop_flag= 1;
static uint8_t transmission = 0;
static uint8_t data_index = 0;
static uint8_t get_data_flag = 0;
static uint8_t up_flag = 0;
static uint8_t down_flag = 0;
static uint8_t left_flag = 0;
static uint8_t right_flag = 0;
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
  float P_Pitch = 15;
  float desired_Pitch 	= 0;
  float desired_Roll	= 0;
  float yaw = 0;


  configure_imu();
  timer_start();



  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  if(left_flag == 1)
	  {
		  left_flag = 0;
		  desired_Pitch += 5;
		  sprintf((char*) word,"Desired Pitch: %d\n",(int) desired_Pitch);
		  HAL_UART_Transmit(&huart2, word, strlen((char*)word), 100);
	  }
	  if(right_flag == 1)
	  {
		  right_flag = 0;
		  desired_Pitch -= 5;
		  sprintf((char*) word,"Desired Pitch: %d\n",(int) desired_Pitch);
		  HAL_UART_Transmit(&huart2, word, strlen((char*)word), 100);
	  }
	  if(up_flag == 1)
	  {
		  up_flag = 0;
		  desired_Roll += 5;
		  sprintf((char*) word,"Desired Roll: %d\n",(int) desired_Roll);
		  HAL_UART_Transmit(&huart2, word, strlen((char*)word), 100);
	  }
	  if(down_flag == 1)
	  {
		  down_flag = 0;
		  desired_Roll -= 5;
		  sprintf((char*) word,"Desired Roll: %d\n",(int) desired_Roll);
		  HAL_UART_Transmit(&huart2, word, strlen((char*)word), 100);
	  }

	  if(get_data_flag ==1)
	  {
		  get_data_flag =0;
		  get_gyr_data(gyro_rate);
		  get_acc_data(acc_rate, standard_acc_range);

		  accel_pitch = acc_pitch((*(acc_rate+0)),(*(acc_rate+1)),(*(acc_rate+2)));
		  accel_roll = acc_roll((*(acc_rate+0)),(*(acc_rate+1)),(*(acc_rate+2)));
		  yaw += gyro_rate[2]* Looptime;

		  KalmanCalculation(KalmanRollAngle,KalmanRollUncertainty,gyro_rate[0], accel_roll, KalmanOutput);
		  KalmanRollAngle 			= KalmanOutput[0];
		  KalmanRollUncertainty 	= KalmanOutput[1];
		  KalmanCalculation(KalmanPitchAngle,KalmanPitchUncertainty,gyro_rate[1], accel_pitch, KalmanOutput);
		  KalmanPitchAngle 			= KalmanOutput[0];
		  KalmanPitchUncertainty 	= KalmanOutput[1];


		  Error_Roll_Ang 	= 	desired_Roll 		-KalmanRollAngle;
		  Error_Pitch_Ang 	= 	desired_Pitch		-KalmanPitchAngle;
		  Error_Yaw_Ang 	= 	desired_Yaw 		-yaw;

		  pid_equation(Error_Roll_Ang, Prev_Error_Roll_Ang, Prev_Int_Roll_Ang, P_outer, 0, 0, PID_Output);
		  Prev_Error_Roll_Ang 	= Error_Roll_Ang;
		  desired_Roll_Rate 	= PID_Output[0];
		  Prev_Int_Roll_Ang 	= PID_Output[1];

		  pid_equation(Error_Pitch_Ang, Prev_Error_Pitch_Ang, Prev_Int_Pitch_Ang, P_outer, 0, 0, PID_Output);
		  Prev_Error_Pitch_Ang 	= Error_Pitch_Ang;
		  desired_Pitch_Rate 	= PID_Output[0];
		  Prev_Int_Pitch_Ang 	= PID_Output[1];

		  pid_equation(Error_Yaw_Ang, Prev_Error_Yaw_Ang, Prev_Int_Yaw_Ang, P_outer, 0, 0, PID_Output);
		  Prev_Error_Yaw_Ang 	= Error_Yaw_Ang;
		  desired_Yaw_Rate		= PID_Output[0];
		  Prev_Int_Yaw_Ang		= PID_Output[1];

		  Error_Roll_Rate 	= desired_Roll_Rate 	- gyro_rate[0];
		  Error_Pitch_Rate 	= desired_Pitch_Rate 	- gyro_rate[1];
		  Error_Yaw_Rate 	= desired_Yaw_Rate 		- gyro_rate[2];

		  pid_equation(Error_Roll_Rate, Prev_Error_Roll_Rate, Prev_Int_Roll_Rate,  		10, 0, 0, PID_Output);
		  Prev_Error_Roll_Rate 	= Error_Roll_Rate;
		  InputRoll 			= PID_Output[0];
		  Prev_Int_Roll_Rate 	= PID_Output[1];

		  pid_equation(Error_Pitch_Rate, Prev_Error_Pitch_Rate, Prev_Int_Pitch_Rate, 	1, 0, 0, PID_Output);
		  Prev_Error_Pitch_Rate = Error_Pitch_Rate;
		  InputPitch 			= PID_Output[0];
		  Prev_Int_Pitch_Rate 	= PID_Output[1];

		  pid_equation(Error_Yaw_Rate, Prev_Error_Yaw_Rate, Prev_Int_Yaw_Rate, 			1, 0, 0, PID_Output);
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
	char character;

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
				character =decode(binary);
				HAL_NVIC_EnableIRQ(TIM7_IRQn);
				switch (character)
				{
				case('1'):
					stop_flag = 1;
					__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1, 0);
					__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2, 0);
					__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3, 0);
					__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_4, 0);
					break;

				case('2'):
					stop_flag = 1;
					__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1, 30);
					__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2, 30);
					__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3, 30);
					__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_4, 30);
					break;

				case('O'):
					stop_flag = 0;
					break;

				case('U'):
					up_flag = 1;
					break;

				case('D'):
					down_flag = 1;
					break;

				case('L'):
					left_flag = 1;
					break;

				case('R'):
					right_flag = 1;
					break;
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
void get_gyr_data(float* gyro_rate)
{
	HAL_StatusTypeDef ret;
	uint8_t word[50];
	int16_t twoByte;
	uint8_t byteLSB, byteMSB;
	word[0]= GYR_X_ADDR;
					ret= HAL_I2C_Master_Transmit(&hi2c3, GYR_ADDR, word, 1, 100);
					if(ret != HAL_OK)
					{
						strcpy((char*)word,"Error Gyr Transmit \n");
						HAL_UART_Transmit(&huart2, word, strlen((char*)word), 100);
					}
					else
					{
						HAL_I2C_Master_Receive(&hi2c3, GYR_ADDR, word, 6, 100);
						if(ret != HAL_OK)
						{
							strcpy((char*)word,"Error Gyr Received \n");
							HAL_UART_Transmit(&huart2, word, strlen((char*)word), 100);
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
							/*sprintf((char*) word,"Gyro X: %d\nGyro Y: %d\nGyro Z: %d\n\n", (signed int)(*gyro_rate),(signed int)(*(gyro_rate+1)),(signed int)*(gyro_rate+2));
							HAL_UART_Transmit(&huart2, word, strlen((char*)word), 100);*/
						}
					}
}
void get_acc_data(float* acc_rate,  int acc_range)
{
	HAL_StatusTypeDef ret;
	uint8_t word[50];
	int16_t twoByte;
	uint8_t byteLSB, byteMSB;

	word[0]= ACC_X_ADDR;
	ret= HAL_I2C_Master_Transmit(&hi2c3, ACC_ADDR, word, 1, 100);
	if(ret != HAL_OK)
	{
		strcpy((char*)word,"Error ACC Transmit \n");
		HAL_UART_Transmit(&huart2, word, strlen((char*)word), 100);
	}
	else
	{
		HAL_I2C_Master_Receive(&hi2c3, ACC_ADDR, word, 6, 100);
		if(ret != HAL_OK)
		{
			strcpy((char*)word,"Error ACC Received \n");
			HAL_UART_Transmit(&huart2, word, strlen((char*)word), 100);
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


								/*sprintf((char*)word,"Acc X: %d\nAcc Y: %d\nAcc Z: %d\n\n", (int)(*(acc_rate+0)),(int)(*(acc_rate+1)),(int)(*(acc_rate+2)) );
								HAL_UART_Transmit(&huart2, word, strlen((char*)word), 100);*/


						 }
				}


}


void timer_start(void)
{
	HAL_TIM_Base_Start_IT(&htim7);
	HAL_TIM_Base_Start(&htim6);
	timer_val = __HAL_TIM_GET_COUNTER(&htim6);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
}
void configure_imu(void)
{
	 HAL_StatusTypeDef ret;
	 uint8_t word[50];

	  //Setup Active Power Mode and Power on to Accelerometer
	 word[0] = ACC_ACTIVE_MODE;
	 ret = HAL_I2C_Mem_Write(&hi2c3, ACC_ADDR, ACC_PWR_CONF_ADDR, 1, word, 1, 100);
	 if(ret != HAL_OK)
	 {
		strcpy((char*)word,"Error Active Mode ACC\n");
		HAL_UART_Transmit(&huart2, word, strlen((char*)word), HAL_MAX_DELAY);
	 }
	 word[0] = ACC_POWER_ON;
	 ret = HAL_I2C_Mem_Write(&hi2c3, ACC_ADDR, ACC_PWR_CNTRL_ADDR, 1, word, 1, 100);
	 if(ret != HAL_OK)
	 {
	  	strcpy((char*)word,"Error Power ON ACC\n");
	  	HAL_UART_Transmit(&huart2, word, strlen((char*)word), HAL_MAX_DELAY);
	 }


	  	  //Setup Interrupt for Gyroscope
	 word[0] = GYR_IT_POWER_ON;
	 ret = HAL_I2C_Mem_Write(&hi2c3, GYR_ADDR, GYR_IT_CNTRL_ADDR, 1, word, 1, 100);
	 if(ret != HAL_OK)
	 {
		strcpy((char*)word,"Error IT ON GYR\n");
	  	HAL_UART_Transmit(&huart2, word, strlen((char*)word), HAL_MAX_DELAY);
	 }

	  	  // Setup IT PIN 1 and 3 respectively
	 word[0] = GYR_IT_3_SELECT;
	 ret = HAL_I2C_Mem_Write(&hi2c3, GYR_ADDR, GYR_IT_MAP_ADDR, 1, word, 1, 100);
	 if(ret != HAL_OK)
	 {
	  	strcpy((char*)word,"Error IT 3 SELECT GYR\n");
	  	HAL_UART_Transmit(&huart2, word, strlen((char*)word), HAL_MAX_DELAY);
	 }
	 word[0] = ACC_IT_1_SELECT;
	 ret = HAL_I2C_Mem_Write(&hi2c3, GYR_ADDR, ACC_IT_MAP_ADDR, 1, word, 1, 100);
	 if(ret != HAL_OK)
	 {
	  	strcpy((char*)word,"Error IT 1 SELECT ACC\n");
	  	HAL_UART_Transmit(&huart2, word, strlen((char*)word), HAL_MAX_DELAY);
	 }

}



void selftest_accel(void)
{
	uint8_t word[80];
	HAL_StatusTypeDef ret;
	float* acc_value =0;
	float diffrence_x, diffrence_y, diffrence_z;

	//Set Selftest range
	word[0] = 0x03;
	ret = HAL_I2C_Mem_Write(&hi2c3, ACC_ADDR, ACC_RANGE_ADDR, 1, word, 1, 100);
	if(ret != HAL_OK)
	{
		strcpy((char*)word,"Error setting Selftest Accelerometer range\n");
		HAL_UART_Transmit(&huart2, word, strlen((char*)word), HAL_MAX_DELAY);
	}
	//Set Selftest filter
	word[0] = 0xA7;
	ret = HAL_I2C_Mem_Write(&hi2c3, ACC_ADDR, ACC_CONF_ADDR, 1, word, 1, 100);
	if(ret != HAL_OK)
	{
		strcpy((char*)word,"Error setting Selftest Accelerometer filter\n");
		HAL_UART_Transmit(&huart2, word, strlen((char*)word), HAL_MAX_DELAY);
	}
	HAL_Delay(10);
	//Enable positive polarity selftest
	word[0] = 0x0D;
	ret = HAL_I2C_Mem_Write(&hi2c3, ACC_ADDR, ACC_SELF_TEST_ADDR, 1, word, 1, 100);
	if(ret != HAL_OK)
	{
		strcpy((char*)word,"Error enabling Accelerometer Selftest\n");
		HAL_UART_Transmit(&huart2, word, strlen((char*)word), HAL_MAX_DELAY);
	}
	HAL_Delay(75);

	get_acc_data(acc_value, 24);
	diffrence_x = *acc_value;
	diffrence_y = *(acc_value+1);
	diffrence_z = *(acc_value+2);

	//Enable negative polarity selftest
	word[0] = 0x09;
	ret = HAL_I2C_Mem_Write(&hi2c3, ACC_ADDR, ACC_SELF_TEST_ADDR, 1, word, 1, 100);
	if(ret != HAL_OK)
	{
		strcpy((char*)word,"Error enabling Accelerometer Selftest\n");
		HAL_UART_Transmit(&huart2, word, strlen((char*)word), HAL_MAX_DELAY);
	}
	HAL_Delay(75);
	get_acc_data(acc_value, 24);
	diffrence_x = diffrence_x - *(acc_value);
	diffrence_y = diffrence_y - *(acc_value+1);
	diffrence_z = diffrence_z - *(acc_value+2);

	//disable Selftest again
	word[0] = 0x00;
	ret = HAL_I2C_Mem_Write(&hi2c3, ACC_ADDR, ACC_SELF_TEST_ADDR, 1, word, 1, 100);
	if(ret != HAL_OK)
	{
		strcpy((char*)word,"Error  disabling Accelerometer Selftest\n");
		HAL_UART_Transmit(&huart2, word, strlen((char*)word), HAL_MAX_DELAY);
	}

	//set range to standard 3g
	word[0] = 0x00;
	ret = HAL_I2C_Mem_Write(&hi2c3, ACC_ADDR, ACC_RANGE_ADDR, 1, word, 1, 100);
	if(ret != HAL_OK)
	{
		strcpy((char*)word,"Error setting Selftest Accelerometer range\n");
		HAL_UART_Transmit(&huart2, word, strlen((char*)word), HAL_MAX_DELAY);
	}

	//print self test value
	sprintf((char*) word,"Selftest X value: %d, Selftest Y value: %d, Selftest Z value: %d\n",(int) diffrence_x,(int) diffrence_y, (int) diffrence_z);
	HAL_UART_Transmit(&huart2, word, strlen((char*)word), 100);

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
