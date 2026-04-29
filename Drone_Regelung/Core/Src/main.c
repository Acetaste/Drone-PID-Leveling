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
#include "data_collection.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define Capture_Duration 4000 //milliseconds of capture time
#define Loop_Time 5 //milliseconds between measurements
#define Numb_Measurements Capture_Duration/Loop_Time



#define P_outer 3
#define I_outer 0
#define D_outer 0




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



static uint16_t timer_val;

static uint8_t 	stop_flag= 1;
static uint8_t 	loop_increment_flag = 0;
static uint8_t 	transmission = 0;

static int16_t 	data_collection_buffer[4*Numb_Measurements];

static char 	ir_character = '\0';




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

  uint8_t save_data_flag 				= 0;
  uint8_t send_data_flag 				= 0;
  uint8_t new_angle_flag 				= 0;


  int 	current_measurement_index 		= 0;

  struct data_header_struct data_header = {
		.desired_pitch 					= 0,
		.desired_roll 					= 0,
		.desired_yaw 					= 0,
		.loop_time 						= Loop_Time,
		.numb_measurements 				= Numb_Measurements};


  float kalman_output[2] 				= {0,0};
  int	motor_input[4] 					= {0,0,0,0};
  float pid_output[2] 					= {0,0};

  float kalman_roll_angle 				= 0;
  float kalman_pitch_angle 				= 0;
  float kalman_roll_uncertainty 		= 2;
  float kalman_pitch_uncertainty 		= 2;


  float gyro_body_rate[3] 				= {0,0,0};
  float gyro_rate[3] 					= {0,0,0};
  float acc_rate[3] 					= {0,0,0};
  float yaw 							= 0;
  float accel_pitch, accel_roll;


  float desired_pitch 					= 0;
  float desired_roll					= 0;
  float desired_yaw						= 0;


  float input_pitch, 					input_roll, 					input_yaw;

  float desired_roll_rate		= 0, 	desired_pitch_rate		= 0, 	desired_yaw_rate	= 0;
  float error_roll_rate			= 0, 	error_pitch_rate		= 0, 	error_yaw_rate		= 0;
  float prev_error_roll_rate 	= 0, 	prev_error_pitch_rate 	= 0, 	prev_error_yaw_rate = 0;
  float prev_int_roll_rate 		= 0, 	prev_int_pitch_rate 	= 0, 	prev_int_yaw_rate 	= 0;

  float error_roll_ang			= 0, 	error_pitch_ang			= 0, 	error_yaw_ang		= 0;
  float prev_error_roll_ang		= 0, 	prev_error_pitch_ang	= 0, 	prev_error_yaw_ang	= 0;
  float prev_int_roll_ang		= 0, 	prev_int_pitch_ang		= 0, 	prev_int_yaw_ang	= 0;

  uint8_t uart_buffer[150];

  float p_pitch 						= 0;
  float p_roll 							= 0;
  float p_yaw 							= 0;
  int counter 							= 0;

  configure_imu(&hi2c3, &huart2);
  timer_start(&timer_val,&htim1, &htim6, &htim7);
  interpret_IR_Char('#', &data_header, &stop_flag, &save_data_flag, &send_data_flag, &new_angle_flag, &huart2);





  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	  if(ir_character != '\0')
	  {
		  //interpret_IR_Char(ir_character, &data_header, &stop_flag, &save_data_flag, &send_data_flag, &new_angle_flag, &huart2);
		  interpret_IR_char_tuning(&ir_character, &desired_pitch, &desired_roll, &stop_flag, &htim1, &huart2, &p_pitch, &p_roll, &p_yaw);
		  ir_character = '\0';
	  }


	  // main function is run only when loop_increment_flag is periodically set from timer interrupt
	  if(0 ==1)
	  {
		  loop_increment_flag =0;


		  //update desired angle if changed through IR remote, new_data_flag set in interpret_IR_Char()
		  if(new_angle_flag ==1)
		  {
			  new_angle_flag = 0;
			  desired_pitch 	= (float) data_header.desired_pitch;
			  desired_roll 		= (float) data_header.desired_roll;
			  desired_yaw		= (float) data_header.desired_yaw;
		  }

		  //gettin data from imu
		  get_gyr_data(gyro_body_rate, &hi2c3, &huart2);
		  get_acc_data(acc_rate, standard_acc_range, &hi2c3, &huart2);



		  //calculating roll an pitch from accelerometer data
		  accel_pitch = acc_pitch((*(acc_rate+0)),(*(acc_rate+1)),(*(acc_rate+2)));
		  accel_roll = acc_roll((*(acc_rate+0)),(*(acc_rate+1)),(*(acc_rate+2)));

		  //converting body gyrorate to fixed gyro rate
		  body_rate_to_fixed_rate(gyro_body_rate, accel_roll,accel_pitch, gyro_rate);



		  //Combining accelerometer and gyroscope data with kalman filter and integrating yaw from gyroscope

		  KalmanCalculation(kalman_pitch_angle,kalman_pitch_uncertainty,gyro_rate[1], accel_pitch, kalman_output);
		  kalman_pitch_angle 		= kalman_output[0];
		  kalman_pitch_uncertainty 	= kalman_output[1];

		  KalmanCalculation(kalman_roll_angle,kalman_roll_uncertainty,gyro_rate[0], accel_roll, kalman_output);
		  kalman_roll_angle 		= kalman_output[0];
		  kalman_roll_uncertainty 	= kalman_output[1];



		  yaw += gyro_rate[2]* Loop_Time/1000;
		  yaw = yaw_cap(yaw);


		  if(counter == 100)
		  		  		  {
			  	  	  	  	  sprintf((char*) uart_buffer," accel_roll: %d, accel_Pitch:%d\n",(int) accel_roll	, (int) accel_pitch);
	  			  		  	  HAL_UART_Transmit(&huart2, uart_buffer, strlen((char*)uart_buffer), 100);
		  			  	  	  sprintf((char*) uart_buffer," kalmanRoll: %d, kalmanPitch:%d, kalmanYaw: %d\n",(int) kalman_roll_angle	, (int) kalman_pitch_angle,(int) yaw);
		  			  		  HAL_UART_Transmit(&huart2, uart_buffer, strlen((char*)uart_buffer), 100);
		  		  			  sprintf((char*) uart_buffer," body rate Roll: %d, body rate Pitch: %d, body rate Yaw: %d\n",(int) gyro_body_rate[0], (int) gyro_body_rate[1],(int) gyro_body_rate[2]);
		  		  			  HAL_UART_Transmit(&huart2, uart_buffer, strlen((char*)uart_buffer), 100);
		  		  			  sprintf((char*) uart_buffer," fixed rate Roll: %d, fixed rate Pitch: %d, fixed rate Yaw: %d\n\n",(int) gyro_rate[0], (int) gyro_rate[1],(int) gyro_rate[2]);
		  		  			  HAL_UART_Transmit(&huart2, uart_buffer, strlen((char*)uart_buffer), 100);
		  		  			  counter =0 ;
		  		  		  }
		  		  		  else
		  		  		  {
		  		  			  counter++;
		  		  		  }


		  //outer loop PID controllers
		  error_pitch_ang 	= 	desired_pitch		-kalman_pitch_angle;
		  error_roll_ang 	= 	desired_roll 		-kalman_roll_angle;
		  error_yaw_ang 	= 	desired_yaw 		-yaw;



		  pid_equation(error_pitch_ang, prev_error_pitch_ang, prev_int_pitch_ang, P_outer, I_outer, D_outer, pid_output);
		  prev_error_pitch_ang 	= error_pitch_ang;
		  desired_pitch_rate 	= pid_output[0];
		  prev_int_pitch_ang 	= pid_output[1];



		  pid_equation(error_roll_ang, prev_error_roll_ang, prev_int_roll_ang, P_outer, I_outer, D_outer, pid_output);
		  prev_error_roll_ang 	= error_roll_ang;
		  desired_roll_rate 	= pid_output[0];
		  prev_int_roll_ang 	= pid_output[1];



		  pid_equation(error_yaw_ang, prev_error_yaw_ang, prev_int_yaw_ang, P_outer, I_outer, D_outer, pid_output);
		  prev_error_yaw_ang 	= error_yaw_ang;
		  desired_yaw_rate		= pid_output[0];
		  prev_int_yaw_ang		= pid_output[1];



		  //inner loop PID controllers
		  error_pitch_rate 	= desired_pitch_rate 	- gyro_rate[0];
		  error_roll_rate 	= desired_roll_rate 	- gyro_rate[1];
		  error_yaw_rate 	= desired_yaw_rate 		- gyro_rate[2];



		  pid_equation(error_pitch_rate, prev_error_pitch_rate, prev_int_pitch_rate, 	1, 0, 0, pid_output);
		  prev_error_pitch_rate = error_pitch_rate;
		  input_pitch 			= pid_output[0];
		  prev_int_pitch_rate 	= pid_output[1];

		  pid_equation(error_roll_rate, prev_error_roll_rate, prev_int_roll_rate,  		p_roll, 0, 0, pid_output);
		  prev_error_roll_rate 	= error_roll_rate;
		  input_roll 			= pid_output[0];
		  prev_int_roll_rate 	= pid_output[1];

		  pid_equation(error_yaw_rate, prev_error_yaw_rate, prev_int_yaw_rate, 			p_yaw, 0, 0, pid_output);
		  prev_error_yaw_rate	= error_yaw_rate;
		  input_yaw	 			= pid_output[0];
		  prev_int_yaw_rate 	= pid_output[1];



		  //translate the change in angles to motor inputs
		  motor_inputs(input_pitch, input_roll,input_yaw, motor_input);




		  //set pwm channels
		  if(stop_flag ==0)
		  {
			  __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1, pwm_cap(*(motor_input+0)));
			  __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2, pwm_cap(*(motor_input+1)));
			  __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3, pwm_cap(*(motor_input+2)));
			  __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_4, pwm_cap(*(motor_input+3)));
		  }



		  //save data to data collection buffer as Measurement Index/Pitch/Roll/Yaw
		  if(save_data_flag == 1)
		  {
			  if(current_measurement_index >= data_header.numb_measurements)
			  {
				  save_data_flag = 0;
				  current_measurement_index = 0;
			  }
			  else
			  {
				  data_collection_buffer[(current_measurement_index*4)] 	= current_measurement_index;
				  data_collection_buffer[(current_measurement_index*4)+1] 	= (int)(kalman_pitch_angle/180*32768);
				  data_collection_buffer[(current_measurement_index*4)+2] 	= (int)(kalman_roll_angle/180*32768);
				  data_collection_buffer[(current_measurement_index*4)+3] 	= (int)(yaw/180*32768);
				  current_measurement_index+=1;
			  }
		  }



		  //send data header and data collection buffer over UART
		  if(send_data_flag == 1)
		  {
			  send_data_flag = 0;
			  send_header(&data_header, &huart2);
			  send_collected_data(data_collection_buffer, data_header.numb_measurements, &huart2);
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
  htim1.Init.Prescaler = 1-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 256;
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
  htim7.Init.Period = 5000-1;
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
  GPIO_InitStruct.Pull = GPIO_NOPULL;
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
	static unsigned int 	binary =0;
	static uint8_t 			data_index = 0;
	int 					duration;
	uint8_t					uart_buffer[50];


	if(GPIO_Pin == GPIO_PIN_1)
	{
		duration = (__HAL_TIM_GET_COUNTER(&htim6)-timer_val);
		timer_val = __HAL_TIM_GET_COUNTER(&htim6);
		if(duration <0)
		{
			duration = duration + 65536;
		}
		loop_increment_flag=0;

		if(transmission == 1 )
		{

			if(duration>1600 && data_index != 0)
			{
				binary |= (1 << data_index);

			}

			data_index++;
			if(data_index>=31)
			{
				transmission = 0;
				ir_character =decode(binary);
				HAL_NVIC_EnableIRQ(TIM7_IRQn);
				sprintf((char*) uart_buffer," 1: \n");
				HAL_UART_Transmit(&huart2, uart_buffer, strlen((char*)uart_buffer), 100);

				if( ir_character == '*')
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
				loop_increment_flag= 0;
				HAL_NVIC_DisableIRQ(TIM7_IRQn);

			}
		}


	}

}
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{

	if(htim == &htim7)
	{
		if(transmission == 0 )
		{
			loop_increment_flag= 0;
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
