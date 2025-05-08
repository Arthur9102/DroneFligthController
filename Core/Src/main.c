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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "bmp280.h"
#include "mpu6050.h"
#include <stdio.h>
#include <string.h>
#include "pid.h"
#include "config.h"
#include <math.h>
#include "kalman.h"
#include "FS_IA10B.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c3;

TIM_HandleTypeDef htim2;

/* Definitions for motorTask */
osThreadId_t motorTaskHandle;
const osThreadAttr_t motorTask_attributes = {
  .name = "motorTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for pidTask */
osThreadId_t pidTaskHandle;
const osThreadAttr_t pidTask_attributes = {
  .name = "pidTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for sensorTask */
osThreadId_t sensorTaskHandle;
const osThreadAttr_t sensorTask_attributes = {
  .name = "sensorTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for rcTask */
osThreadId_t rcTaskHandle;
const osThreadAttr_t rcTask_attributes = {
  .name = "rcTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for stateTask05 */
osThreadId_t stateTask05Handle;
const osThreadAttr_t stateTask05_attributes = {
  .name = "stateTask05",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for stateQueue */
osMessageQueueId_t stateQueueHandle;
const osMessageQueueAttr_t stateQueue_attributes = {
  .name = "stateQueue"
};
/* Definitions for controlQueue */
osMessageQueueId_t controlQueueHandle;
const osMessageQueueAttr_t controlQueue_attributes = {
  .name = "controlQueue"
};
/* USER CODE BEGIN PV */
/*========================= PID Pareameter define ================================*/
double _kp_roll;
double _ki_roll;
double _kd_roll;
double _kp_pitch;
double _ki_pitch;
double _kd_pitch;
double _kp_z;
double _ki_z;
double _kd_z;
double _roll_set = 0;
double _pitch_set = 0;
double _z_set = 100;  //cm
double _output_roll,_output_pitch,_output_z;
double _roll_measured;
double _pitch_measured;
double _altitude; //cm
PID_PARA _pid_roll;
PID_PARA _pid_pitch;
PID_PARA _pid_z;
/*========================= RC Pareameter define ================================*/
extern uint8_t usart2_rx_data;
extern uint8_t usart2_rx_flag;

extern uint8_t ibus_rx_buf[32];
extern uint8_t ibus_rx_cplt_flag;



/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM5_Init(void);
static void MX_I2C3_Init(void);
static void MX_TIM2_Init(void);
void MotorTask(void *argument);
void PidTask(void *argument);
void SensorTask(void *argument);
void RCTask(void *argument);
void StateEstimatorTask(void *argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
//BMP280_HandleTypedef bmp280;
//float temperature = 0.0f;
//float pressure = 0.0f;
//uint8_t _flag = 0;
//MPU6050_RawData sensor_data;
uint8_t _status = 0;
extern uint8_t _flag;
//float accel_x =0.0f;
//float accel_y =0.0f;
//float accel_z =0.0f;
//float gyro_x =0.0f;
//float gyro_y =0.0f;
//float gyro_z =0.0f;
osMessageQueueId_t sensorQueueHandle;
const osMessageQueueAttr_t sensorQueue_attributes = {
  .name = "sensorQueue"
};

float gyro_x_bias = 0.0f, gyro_y_bias = 0.0f, gyro_z_bias = 0.0f;
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
  PID_INIT(&_pid_roll,&_roll_set,&_output_roll, &_roll_measured,
    10, 90.0, 0.0, &_kp_roll, &_ki_roll, &_kd_roll);
  PID_INIT(&_pid_pitch,&_pitch_set,&_output_pitch, &_pitch_measured,
    10, 90.0, 0.0, &_kp_pitch, &_ki_pitch, &_kd_pitch);
  PID_INIT(&_pid_z,&_z_set,&_output_z, &_altitude,
      10, 90.0, 0.0, &_kp_z, &_ki_z, &_kd_z);
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  MX_TIM5_Init();
  MX_I2C3_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  LL_USART_EnableIT_RXNE(USART1);
  LL_USART_EnableIT_RXNE(USART2);


  LL_TIM_EnableCounter(TIM5);
  LL_TIM_CC_EnableChannel(TIM5, LL_TIM_CHANNEL_CH1);
  LL_TIM_CC_EnableChannel(TIM5, LL_TIM_CHANNEL_CH2);
  LL_TIM_CC_EnableChannel(TIM5, LL_TIM_CHANNEL_CH3);
  LL_TIM_CC_EnableChannel(TIM5, LL_TIM_CHANNEL_CH4);

  LL_TIM_EnableCounter(TIM2);
  LL_TIM_CC_EnableChannel(TIM2, LL_TIM_CHANNEL_CH1);
  // ESC Calibration
  TIM2 -> CCR1 = 2000;
  TIM5 -> CCR2 = 12500;
  TIM5 -> CCR3 = 12500;
  TIM5 -> CCR4 = 12500;

//  TIM5 -> CCR1 = 200;
//  TIM5 -> CCR2 = 200;
//  TIM5 -> CCR3 = 200;
//  TIM5 -> CCR4 = 200;
  HAL_Delay(7000);

  TIM2 -> CCR1 = 1000;
  TIM5 -> CCR2 = 6250;
  TIM5 -> CCR3 = 6250;
  TIM5 -> CCR4 = 6250;

//  TIM5 -> CCR1 = 100;
//  TIM5 -> CCR2 = 100;
//  TIM5 -> CCR3 = 100;
//  TIM5 -> CCR4 = 100;
  HAL_Delay(8000);
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of stateQueue */
  stateQueueHandle = osMessageQueueNew (16, sizeof(uint16_t), &stateQueue_attributes);

  /* creation of controlQueue */
  controlQueueHandle = osMessageQueueNew (16, sizeof(uint16_t), &controlQueue_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  sensorQueueHandle = osMessageQueueNew (16, sizeof(MPU6050_ConvertedData), &sensorQueue_attributes);
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of motorTask */
  motorTaskHandle = osThreadNew(MotorTask, NULL, &motorTask_attributes);

  /* creation of pidTask */
  pidTaskHandle = osThreadNew(PidTask, NULL, &pidTask_attributes);

  /* creation of sensorTask */
  sensorTaskHandle = osThreadNew(SensorTask, NULL, &sensorTask_attributes);

  /* creation of rcTask */
  rcTaskHandle = osThreadNew(RCTask, NULL, &rcTask_attributes);

  /* creation of stateTask05 */
  stateTask05Handle = osThreadNew(StateEstimatorTask, NULL, &stateTask05_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 64;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV8;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

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
  hi2c3.Init.ClockSpeed = 100000;
  hi2c3.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c3.Init.OwnAddress1 = 0;
  hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c3.Init.OwnAddress2 = 0;
  hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C3_Init 2 */

  /* USER CODE END I2C3_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 7;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 19999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  LL_TIM_InitTypeDef TIM_InitStruct = {0};
  LL_TIM_OC_InitTypeDef TIM_OC_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM5);

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  TIM_InitStruct.Prescaler = 0;
  TIM_InitStruct.CounterMode = LL_TIM_COUNTERMODE_UP;
  TIM_InitStruct.Autoreload = 24999;
  TIM_InitStruct.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
  LL_TIM_Init(TIM5, &TIM_InitStruct);
  LL_TIM_EnableARRPreload(TIM5);
  LL_TIM_OC_EnablePreload(TIM5, LL_TIM_CHANNEL_CH2);
  TIM_OC_InitStruct.OCMode = LL_TIM_OCMODE_PWM1;
  TIM_OC_InitStruct.OCState = LL_TIM_OCSTATE_DISABLE;
  TIM_OC_InitStruct.OCNState = LL_TIM_OCSTATE_DISABLE;
  TIM_OC_InitStruct.CompareValue = 0;
  TIM_OC_InitStruct.OCPolarity = LL_TIM_OCPOLARITY_HIGH;
  LL_TIM_OC_Init(TIM5, LL_TIM_CHANNEL_CH2, &TIM_OC_InitStruct);
  LL_TIM_OC_DisableFast(TIM5, LL_TIM_CHANNEL_CH2);
  LL_TIM_OC_EnablePreload(TIM5, LL_TIM_CHANNEL_CH3);
  LL_TIM_OC_Init(TIM5, LL_TIM_CHANNEL_CH3, &TIM_OC_InitStruct);
  LL_TIM_OC_DisableFast(TIM5, LL_TIM_CHANNEL_CH3);
  LL_TIM_OC_EnablePreload(TIM5, LL_TIM_CHANNEL_CH4);
  LL_TIM_OC_Init(TIM5, LL_TIM_CHANNEL_CH4, &TIM_OC_InitStruct);
  LL_TIM_OC_DisableFast(TIM5, LL_TIM_CHANNEL_CH4);
  LL_TIM_SetTriggerOutput(TIM5, LL_TIM_TRGO_RESET);
  LL_TIM_DisableMasterSlaveMode(TIM5);
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
  /**TIM5 GPIO Configuration
  PA1   ------> TIM5_CH2
  PA2   ------> TIM5_CH3
  PA3   ------> TIM5_CH4
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_1|LL_GPIO_PIN_2|LL_GPIO_PIN_3;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_2;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  LL_USART_InitTypeDef USART_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_USART1);

  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
  /**USART1 GPIO Configuration
  PA9   ------> USART1_TX
  PA10   ------> USART1_RX
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_9|LL_GPIO_PIN_10;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_7;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USART1 interrupt Init */
  NVIC_SetPriority(USART1_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),5, 0));
  NVIC_EnableIRQ(USART1_IRQn);

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  USART_InitStruct.BaudRate = 115200;
  USART_InitStruct.DataWidth = LL_USART_DATAWIDTH_8B;
  USART_InitStruct.StopBits = LL_USART_STOPBITS_1;
  USART_InitStruct.Parity = LL_USART_PARITY_NONE;
  USART_InitStruct.TransferDirection = LL_USART_DIRECTION_TX_RX;
  USART_InitStruct.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
  USART_InitStruct.OverSampling = LL_USART_OVERSAMPLING_16;
  LL_USART_Init(USART1, &USART_InitStruct);
  LL_USART_ConfigAsyncMode(USART1);
  LL_USART_Enable(USART1);
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5|GPIO_PIN_6, GPIO_PIN_SET);

  /*Configure GPIO pins : PA5 PA6 */
  GPIO_InitStruct.Pin = GPIO_PIN_5|GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_MotorTask */
/**
  * @brief  Function implementing the motorTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_MotorTask */
void MotorTask(void *argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
    osDelay(10);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_PidTask */
/**
* @brief Function implementing the pidTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_PidTask */
void PidTask(void *argument)
{
  /* USER CODE BEGIN PidTask */

  /* Infinite loop */
  for(;;)
  {
    PID_CONTROLLER(&_pid_roll);
    PID_CONTROLLER(&_pid_pitch);
    PID_CONTROLLER(&_pid_z);
    osDelay(10);
  }
  /* USER CODE END PidTask */
}

/* USER CODE BEGIN Header_SensorTask */
/**
* @brief Function implementing the sensorTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_SensorTask */
void SensorTask(void *argument)
{
  /* USER CODE BEGIN SensorTask */
//  SensorData sensor_data = {0};
	  mpu6050.hi2c = &hi2c1;
	  mpu6050.dev_addr = MPU6050_ADDRESS; // 0x68 (AD0 = GND)
	  mpu6050.timeout = 100; // 100ms timeout
	  if (MPU6050_Init(&mpu6050)) {
		  _status |= 0x02; // Bit 1: MPU6050 OK
	  } else {
		  _status |= 0x20; // Bit 5: MPU6050 error
	  }
	  if (_status & 0x02) {
	      MPU6050_RawData raw_data;
	      int32_t gyro_x_sum = 0, gyro_y_sum = 0, gyro_z_sum = 0;
	      const int samples = 200; // Thu thập 200 mẫu
	      for (int i = 0; i < samples; i++) {
	        if (MPU6050_ReadRawData(&mpu6050, &raw_data)) {
	          gyro_x_sum += raw_data.gyro_x;
	          gyro_y_sum += raw_data.gyro_y;
	          gyro_z_sum += raw_data.gyro_z;
	        }
	        osDelay(10);
	      }
	      gyro_x_bias = (float)(gyro_x_sum / samples) * (1.0f / 16.4f) * M_PI / 180.0f; // Chuyển LSB sang rad/s
	      gyro_y_bias = (float)(gyro_y_sum / samples) * (1.0f / 16.4f) * M_PI / 180.0f;
	      gyro_z_bias = (float)(gyro_z_sum / samples) * (1.0f / 16.4f) * M_PI / 180.0f;
	    }
	  /* Infinite loop */
	  for(;;)
	  {
	    if (_status & 0x02) {
	      MPU6050_RawData raw_data;
	      MPU6050_ConvertedData conv_data;
	      if (MPU6050_ReadRawData(&mpu6050, &raw_data)) {
	          MPU6050_ConvertData(&mpu6050, &raw_data, &conv_data);
//	          accel_x = conv_data.accel_x ;
//	          accel_y = conv_data.accel_y ;
//	          accel_z = conv_data.accel_z ;
//	          gyro_x = conv_data.gyro_x * (M_PI / 180.0f) - gyro_x_bias; //rad/s
//	          gyro_y = conv_data.gyro_y * (M_PI / 180.0f) - gyro_y_bias;
//	          gyro_z = conv_data.gyro_z * (M_PI / 180.0f) - gyro_z_bias;
	          if (osMessageQueuePut(sensorQueueHandle, &conv_data, 0, 0) != osOK) {
	             _status |= 0x80; // Bit 7: Queue error
	          }
	      }
	  }
  osDelay(10);
  }
  /* USER CODE END SensorTask */
}

/* USER CODE BEGIN Header_RCTask */
/**
* @brief Function implementing the rcTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_RCTask */
void RCTask(void *argument)
{
  /* USER CODE BEGIN RCTask */
  /* Infinite loop */
  for(;;)
  {
      if(ibus_rx_cplt_flag == 1){
          ibus_rx_cplt_flag = 0;

          if(ibus_check_CS(&ibus_rx_buf[0], 32) == 1)
          {
              HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, 1);
              HAL_Delay(100);
            //	  				HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
              ibus_message(&ibus_rx_buf[0], &ibus);

              if(ibus_active_failsafe(&ibus) == 1)
              {
                HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, 1);
                //HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, 1);
              }
              else
              {
                HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, 0);
                //HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, 0);
              }
          }
          else
          {
              HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, 0);
          }
      }
      _z_set = map_ibus_to_altitude(ibus.left_horizontal);
  	  TIM2 -> CCR1 = 1000 + (ibus.left_horizontal - 1000 + 15);
  	  TIM5 -> CCR2 = 6250 + (ibus.left_horizontal - 1000) * 6.25;
  	  TIM5 -> CCR3 = 6250 + (ibus.left_horizontal - 1000) * 6.25;
  	  TIM5 -> CCR4 = 6250 + (ibus.left_horizontal - 1000) * 6.25;
    osDelay(10);
  }
  /* USER CODE END RCTask */
}

/* USER CODE BEGIN Header_StateEstimatorTask */
/**
* @brief Function implementing the stateTask05 thread.
* @param argument: Not used
* @retval None
*/
// float pitch_kal;
// float roll_kal;
/* USER CODE END Header_StateEstimatorTask */
void StateEstimatorTask(void *argument)
{
  /* USER CODE BEGIN StateEstimatorTask */
  MPU6050_ConvertedData conv_data;
  Kalman_t kf;
  Kalman_Init(&kf);
  float dt = 0.01f;
  /* Infinite loop */
   for (;;) {
       if (osMessageQueueGet(sensorQueueHandle, &conv_data, NULL, osWaitForever) == osOK) {
           // Compute measured angles
          float roll = atan2f(conv_data.accel_y,
                                sqrtf(conv_data.accel_x * conv_data.accel_x +
                                       conv_data.accel_z * conv_data.accel_z));
          float pitch = atan2f(-conv_data.accel_x,
                                sqrtf(conv_data.accel_y * conv_data.accel_y +
                                        conv_data.accel_z * conv_data.accel_z));
//           // Compute acceleration magnitude
//           float acc_magnitude = sqrtf(conv_data.accel_x * conv_data.accel_x +
//                                      conv_data.accel_y * conv_data.accel_y +
//                                      conv_data.accel_z * conv_data.accel_z) * 9.81f; // Convert g to m/s^2
           // Use pre-converted and bias-corrected gyro data
           float p = conv_data.gyro_x * (M_PI / 180.0f) - gyro_x_bias;; // Already in rad/s
           float q = conv_data.gyro_y * (M_PI / 180.0f) - gyro_x_bias;;
           //float r = conv_data.gyro_x * (M_PI / 180.0f) - gyro_x_bias;;
           // Validate inputs
           if (!isnan(roll) && !isnan(pitch) &&
               !isnan(p) && !isnan(q)) {
                _roll_measured = Kalman_Update(&kf, roll , q , dt);
                _pitch_measured = Kalman_Update(&kf, pitch, q, dt);
           } else {
               _status |= 0x10; // Bit 4: Invalid sensor data
           }
       }
       osDelay(10); // 100Hz
  }
  /* USER CODE END StateEstimatorTask */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

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

#ifdef  USE_FULL_ASSERT
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
