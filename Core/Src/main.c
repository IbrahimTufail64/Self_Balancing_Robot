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
#include "MPU9250.h"
#include "Kalman_filter.h"
#include "string.h"
#include "stdio.h"
//
//#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "queue.h"
#include "semphr.h"
#include "event_groups.h"

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

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart2;

/* Definitions for Task1 */
osThreadId_t Task1Handle;
const osThreadAttr_t Task1_attributes = {
  .name = "Task1",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for myTask02 */
osThreadId_t myTask02Handle;
const osThreadAttr_t myTask02_attributes = {
  .name = "myTask02",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for myTask03 */
osThreadId_t myTask03Handle;
const osThreadAttr_t myTask03_attributes = {
  .name = "myTask03",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal1,
};
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM4_Init(void);
void SensorAquisition(void *argument);
void PID_Loop(void *argument);
void PWM_Driver(void *argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

typedef struct {
    float kp;
    float ki;
    float kd;
    float prev_err;
    float setpoint;
    float integral;
    float interval;
} PID;

PID myPID;

float bias_gyro_x = 0,bias_gyro_y = 0,bias_gyro_z = 0;

float roll = 0;
float pitch = 0;

raw_readings readings;

//SemaphoreHandle_t Counting_Sem;
int Resources[] = {111,222,333};
int indx = 0;
char debug_buffer[150];

volatile uint32_t previous_counter_value = 0;
volatile uint32_t period_ticks = 0;
SemaphoreHandle_t mySemaphore;
// notification for ISR
static TaskHandle_t xMyTaskToNotify = NULL;

//Queue handler
QueueHandle_t queue_sensor_value;
QueueHandle_t queue_pid_output;
/* Counter to track ISR ticks */
#define ISR_TICKS_PER_TASK 1

//xSemaphoreHandle sem;
//osSemaphoreId_t xBinarySemaphore;

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

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
  MX_I2C1_Init();
  MX_TIM2_Init();
  MX_USART2_UART_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */

  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);


  //IMU code
  init_MPU(&hi2c1);

//  Counting_Sem = xSemaphoreCreateCounting(3,0);
  HAL_TIM_Base_Start_IT(&htim4);
//  xBinarySemaphore = xSemaphoreCreateBinary();



  myPID.kp = 60;
  myPID.ki = 0.01;
  myPID.kd = 0;
  myPID.integral = 0;
  myPID.setpoint = 0;
  myPID.prev_err = 0;
  myPID.interval = 0.001;



    // bias correction startup code
    int attempts = 0;
    int i = 0;
    int total_samples = 500;


    while( i < total_samples && attempts < 1000){
    	raw_readings readings_temp;

        get_values_MPU(&hi2c1,&readings_temp);

        if(  readings_temp.x_gyro_degree != 0 || readings_temp.y_gyro_degree != 0 || readings_temp.z_gyro_degree != 0 ){
            bias_gyro_x += readings_temp.x_gyro_degree;
            bias_gyro_y += readings_temp.y_gyro_degree;
            bias_gyro_z += readings_temp.z_gyro_degree;
            i++;
        }

        HAL_Delay(5);  // give time to settle
        attempts++;
    }

      bias_gyro_x /= (float)total_samples;
      bias_gyro_y /= (float)total_samples;
      bias_gyro_z /= (float)total_samples;



//     timer_val = __HAL_TIM_GET_COUNTER(&htim4);

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  mySemaphore = xSemaphoreCreateBinary();
  if(mySemaphore == NULL)
  {
      // Handle error: not enough heap
	  	  char msg[] = "Err! Ran out of Heap for semaphore\r\n";
	  	  HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
  }

  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  queue_sensor_value = xQueueCreate(1, sizeof(uint32_t));
  queue_pid_output = xQueueCreate(1, sizeof(uint32_t));
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of Task1 */
  Task1Handle = osThreadNew(SensorAquisition, NULL, &Task1_attributes);

  /* creation of myTask02 */
  myTask02Handle = osThreadNew(PID_Loop, NULL, &myTask02_attributes);

  /* creation of myTask03 */
  myTask03Handle = osThreadNew(PWM_Driver, NULL, &myTask03_attributes);

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


//	  __HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_1,255);
//	  __HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_4,255);
//		  HAL_Delay(100);
//	  HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_4);
//	  strcpy(debug_buffer,"This is Highest Priority: SensorAquisition\n");
//	  char msg[] = "Hello Arduino!\r\n";
//	  HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);

//	  char msg[] = "Sensor Aquisition Task 1\r\n";
//	  HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 50);

//	  	readings.x_acc_g = 0;
//	  	readings.y_acc_g = 0;
//	  	readings.z_acc_g = 0;
//	  	readings.x_gyro_degree = 0;
//	  	readings.y_gyro_degree = 0;
//	  	readings.z_gyro_degree = 0;
//	  	readings.x_mag = 0;
//	  	readings.y_mag = 0;
//	  	readings.z_mag = 0;
//	  	  get_values_MPU(&hi2c1,&readings);
//
//	  	readings.x_gyro_degree -= bias_gyro_x;
//	  	readings.y_gyro_degree -= bias_gyro_y;
//	  	readings.z_gyro_degree -= bias_gyro_z;
//
//
//	  	Complementary_filter(&readings,&pitch,&roll);

//
//	  	//PID control loop
//	  	float error = myPID.setpoint - pitch;
//
//	  	        // Integral term
//	  	myPID.integral += error * myPID.interval;
//
//	  	        // Derivative term
//	  	float derivative = (error - myPID.prev_err) / myPID.interval;
//
//	  	        // PID output
//	  	float output = myPID.kp * error + myPID.ki * myPID.integral + myPID.kd * derivative;
//
//	  	        // Save current error for next derivative calculation
//	  	myPID.prev_err = error;
//	  	//use output in degrees to calibrate pwm
//
//	  	float scale = 80.0f; // adjust to taste
//	  	int pwm = (int)fabs(output) * scale;
//	  	if (pwm > 255) pwm = 255;
//
//	  	if (output < 0) {
//	  	    // Reverse
//	  	    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, pwm);
//	  	    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, pwm);
//	  	    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 0);
//	  	    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 0);
//	  	} else {
//	  	    // Forward
//	  	    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, pwm);
//	  	    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, pwm);
//	  	    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0);
//	  	    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, 0);
//	  	}




		  // Transmit data arduino debug



//	  		  char buffer2[32];
//	  		  snprintf(buffer2, sizeof(buffer2), "Pitch,Roll:%f,%f\n", pitch, roll);
//	  		 HAL_I2C_Master_Transmit(&hi2c1, 0x48 << 1, (uint8_t*)buffer2, sizeof(buffer2), 100);



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

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 60;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
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
  hi2c1.Init.Timing = 0x307075B1;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 255;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
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
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 8000-1;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 30-1;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

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
  huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart2, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart2, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart2) != HAL_OK)
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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pin : PE3 */
  GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : PA4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PD5 */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
// ISR for PID


/* USER CODE END 4 */

/* USER CODE BEGIN Header_SensorAquisition */
/**
  * @brief  Function implementing the Task1 thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_SensorAquisition */
void SensorAquisition(void *argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
//	  	  char msg[] = "Task1!\r\n";
//	  	  HAL_UART_Transmit_DMA(&huart2, (uint8_t*)msg, strlen(msg));

	  		  	readings.x_acc_g = 0;
	  		  	readings.y_acc_g = 0;
	  		  	readings.z_acc_g = 0;
	  		  	readings.x_gyro_degree = 0;
	  		  	readings.y_gyro_degree = 0;
	  		  	readings.z_gyro_degree = 0;
	  		  	readings.x_mag = 0;
	  		  	readings.y_mag = 0;
	  		  	readings.z_mag = 0;
	  		  	  get_values_MPU(&hi2c1,&readings);

	  		  	readings.x_gyro_degree -= bias_gyro_x;
	  		  	readings.y_gyro_degree -= bias_gyro_y;
	  		  	readings.z_gyro_degree -= bias_gyro_z;


	  		  	Complementary_filter(&readings,&pitch,&roll);
//	  		  	char msg[50] ;
//	  		  	sprintf(msg,"Sensor: %f\r\n", &pitch);
//	  		  	HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg),40);
	  		  	xQueueOverwrite(queue_sensor_value, &pitch);
    osDelay(1);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_PID_Loop */
/**
* @brief Function implementing the myTask02 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_PID_Loop */
void PID_Loop(void *argument)
{
  /* USER CODE BEGIN PID_Loop */
	 xMyTaskToNotify = xTaskGetCurrentTaskHandle();


  /* Infinite loop */
  for(;;)
  {
//	  xSemaphoreTake(mySemaphore, portMAX_DELAY);
	  ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
//  	  char msg[] = "Semaphore taken for Task2\r\n";
//  	  HAL_UART_Transmit_DMA(&huart2, (uint8_t*)msg, strlen(msg));
	  static float sensor_val = 0;
	  xQueueReceive(queue_sensor_value, &sensor_val,portMAX_DELAY);
	   HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_4);
	  	//PID control loop
	  	float error = myPID.setpoint - sensor_val;

	  	        // Integral term
	  	myPID.integral += error * myPID.interval;

	  	        // Derivative term
	  	float derivative = (error - myPID.prev_err) / myPID.interval;

	  	        // PID output
	  	float output = myPID.kp * error + myPID.ki * myPID.integral + myPID.kd * derivative;
//	  	if (fabs(error) < 1.0) output = 0;

//	  	char msg[70] ;
//	  	sprintf(msg,"Val:%f\r\n", output);
//	  	HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg),40);

	  	        // Save current error for next derivative calculation
	  	myPID.prev_err = error;
	  	// for signaling pwm queue
	  	xQueueOverwrite(queue_pid_output, &output);
	  	//use output in degrees to calibrate pwm
//  	osDelay(1);
//  	processEvent();
  }
  /* USER CODE END PID_Loop */
}

/* USER CODE BEGIN Header_PWM_Driver */
/**
* @brief Function implementing the myTask03 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_PWM_Driver */
void PWM_Driver(void *argument)
{
  /* USER CODE BEGIN PWM_Driver */
  /* Infinite loop */
  for(;;)
  {
	  float pwm = 0;
	  xQueueReceive(queue_pid_output, &pwm,portMAX_DELAY);



	  int drive_pwm = (int)fabs(pwm);
	  if(drive_pwm > 255) drive_pwm = 255;

	  	  	if (pwm > 0) {
	  	  	    // Reverse
	  	  	    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, drive_pwm);
	  	  	    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, drive_pwm);
	  	  	    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 0);
	  	  	    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 0);
	  	  	} else {
	  	  	    // Forward
	  	  	    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, drive_pwm);
	  	  	    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, drive_pwm);
	  	  	    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0);
	  	  	    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, 0);
	  	  	}

    osDelay(1);
  }
  /* USER CODE END PWM_Driver */
}

 /* MPU Configuration */

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x0;
  MPU_InitStruct.Size = MPU_REGION_SIZE_4GB;
  MPU_InitStruct.SubRegionDisable = 0x87;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */
	  if (htim->Instance == TIM4)
	  {
	        BaseType_t xHigherPriorityTaskWoken = pdFALSE;

	        if(xMyTaskToNotify != NULL)
	        {
	            vTaskNotifyGiveFromISR(xMyTaskToNotify, &xHigherPriorityTaskWoken);
	            portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
	        }

	  }

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6)
  {
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
