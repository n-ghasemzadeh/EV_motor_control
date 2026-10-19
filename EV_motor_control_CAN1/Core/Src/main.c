/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention

  * Copyright (c) 2026 STMicroelectronics.
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
#include <stdio.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MOTOR_MAX_RPM             3000U
#define CAN_COMMAND_TIMEOUT_MS     500U
#define CAN_STATUS_PERIOD_MS       100U

/* Project example only: choose this for your actual sensor/motor. */
#define MOTOR_OVERTEMP_LIMIT_C      90.0f

#define FAULT_NONE                  0U
#define FAULT_CAN_TIMEOUT           1U
#define FAULT_OVERTEMPERATURE       2U
#define FAULT_INVALID_COMMAND       3U
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/

/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

ADC_HandleTypeDef hadc1;

CAN_HandleTypeDef hcan1;

TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart2;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for PedalTask */
osThreadId_t PedalTaskHandle;
const osThreadAttr_t PedalTask_attributes = {
  .name = "PedalTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal2,
};
/* Definitions for TemperatureTask */
osThreadId_t TemperatureTaskHandle;
const osThreadAttr_t TemperatureTask_attributes = {
  .name = "TemperatureTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for ControlTask */
osThreadId_t ControlTaskHandle;
const osThreadAttr_t ControlTask_attributes = {
  .name = "ControlTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for PWMTask */
osThreadId_t PWMTaskHandle;
const osThreadAttr_t PWMTask_attributes = {
  .name = "PWMTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for CanTask */
osThreadId_t CanTaskHandle;
const osThreadAttr_t CanTask_attributes = {
  .name = "CanTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* USER CODE BEGIN PV */
typedef struct
{
    uint32_t adcValue;
    float pedalPercent;
} PedalData_t;

typedef struct
{
    uint32_t adcValue;
    float temperatureC;
} TemperatureData_t;

typedef struct
{
    float desiredSpeedRPM;
} MotorCommand_t;

osMessageQueueId_t PedalQueueHandle;
osMessageQueueId_t TemperatureQueueHandle;
osMessageQueueId_t MotorCommandQueueHandle;
osMutexId_t ADC1MutexHandle;

/* Shared between CanTask and ControlTask. */
volatile uint8_t  canCommandSeen = 0U;
volatile uint8_t  canEnableRequest = 0U;
volatile uint16_t canMaxRPM = 0U;
volatile uint32_t lastCanCommandTick = 0U;

volatile uint8_t  canFaultCode = FAULT_CAN_TIMEOUT;
volatile uint16_t requestedRPMStatus = 0U;
volatile float    systemTemperatureC = 0.0f;

volatile uint32_t commandRxMs = 0;
volatile uint32_t commandAppliedMs = 0;
volatile uint8_t timingPending = 0;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM3_Init(void);
static void MX_CAN1_Init(void);
void StartDefaultTask(void *argument);
void StartPedalTask(void *argument);
void StartTemperatureTask(void *argument);
void StartControlTask(void *argument);
void StartPWMTask(void *argument);
void StartCanTask(void *argument);
void CAN_TransmitStatus(void);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static void CAN_ConfigFilter(void)
{
    CAN_FilterTypeDef filter = {0};

    filter.FilterBank = 0;
    filter.FilterMode = CAN_FILTERMODE_IDMASK;
    filter.FilterScale = CAN_FILTERSCALE_32BIT;

    // Accept standard CAN IDs 0x100 and 0x200
    // Mask the upper 10 bits of the 11-bit standard ID.
    // For simplicity, accept all standard IDs in this first test.
    filter.FilterIdHigh = 0x0000;
    filter.FilterIdLow = 0x0000;
    filter.FilterMaskIdHigh = 0x0000;
    filter.FilterMaskIdLow = 0x0000;

    filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
    filter.FilterActivation = ENABLE;
    filter.SlaveStartFilterBank = 14;

    if (HAL_CAN_ConfigFilter(&hcan1, &filter) != HAL_OK)
    {
        Error_Handler();
    }
}
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
  MX_ADC1_Init();
  MX_USART2_UART_Init();
  MX_TIM3_Init();
  MX_CAN1_Init();




  /* USER CODE BEGIN 2 */

    CAN_ConfigFilter();

  if (HAL_CAN_Start(&hcan1) != HAL_OK)
  {
      Error_Handler();
  }

  /* USER CODE END 2 */


  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  ADC1MutexHandle = osMutexNew(NULL);
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  PedalQueueHandle = osMessageQueueNew(
        1,
        sizeof(PedalData_t),
        NULL
    );
    TemperatureQueueHandle = osMessageQueueNew(
        1,
        sizeof(TemperatureData_t),
        NULL
    );
    MotorCommandQueueHandle = osMessageQueueNew(
        1,
        sizeof(MotorCommand_t),
        NULL
    );
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of PedalTask */
  PedalTaskHandle = osThreadNew(StartPedalTask, NULL, &PedalTask_attributes);

  /* creation of TemperatureTask */
  TemperatureTaskHandle = osThreadNew(StartTemperatureTask, NULL, &TemperatureTask_attributes);

  /* creation of ControlTask */
  ControlTaskHandle = osThreadNew(StartControlTask, NULL, &ControlTask_attributes);

  /* creation of PWMTask */
  PWMTaskHandle = osThreadNew(StartPWMTask, NULL, &PWMTask_attributes);

  /* creation of CanTask */
  CanTaskHandle = osThreadNew(StartCanTask, NULL, &CanTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Initialize leds */
  BSP_LED_Init(LED2);

  /* Initialize USER push-button, will be used to trigger an interrupt each time it's pressed.*/
  BSP_PB_Init(BUTTON_USER, BUTTON_MODE_EXTI);

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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_84CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief CAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 6;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_11TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */

  /* USER CODE END CAN1_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 83;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV2;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

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
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void CAN_TransmitStatus(void)
{
	    CAN_TxHeaderTypeDef txHeader = {0};
	    uint8_t txData[8] = {0};
	    uint32_t txMailbox;

	    uint16_t requestedRPM = requestedRPMStatus;
	    float tempC = systemTemperatureC;

	    if (requestedRPM > MOTOR_MAX_RPM)
	        requestedRPM = MOTOR_MAX_RPM;

	    /* Encode temperature in tenths of a degree C. */
	    if (tempC < 0.0f)
	        tempC = 0.0f;

	    if (tempC > 6553.5f)
	        tempC = 6553.5f;

	    uint16_t tempTenths = (uint16_t)(tempC * 10.0f);

	    txHeader.StdId = 0x100;
	    txHeader.ExtId = 0;
	    txHeader.IDE = CAN_ID_STD;
	    txHeader.RTR = CAN_RTR_DATA;
	    txHeader.DLC = 8;
	    txHeader.TransmitGlobalTime = DISABLE;

	    /* Bytes 0-1: requested RPM. */
	    txData[0] = (uint8_t)(requestedRPM >> 8);
	    txData[1] = (uint8_t)(requestedRPM & 0xFF);

	    /* Bytes 2-3: measured RPM unavailable without a speed sensor. */
	    txData[2] = 0xFF;
	    txData[3] = 0xFF;

	    /* Bytes 4-5: temperature in 0.1 C units. */
	    txData[4] = (uint8_t)(tempTenths >> 8);
	    txData[5] = (uint8_t)(tempTenths & 0xFF);

	    /* Byte 6: status flags. */
	    txData[6] = 0U;

	    if (canEnableRequest != 0U)
	        txData[6] |= (1U << 0);

	    if (canFaultCode == FAULT_CAN_TIMEOUT)
	        txData[6] |= (1U << 1);

	    if (canFaultCode == FAULT_OVERTEMPERATURE)
	        txData[6] |= (1U << 2);

	    /* Byte 7: fault code. */
	    txData[7] = canFaultCode;

	    if (HAL_CAN_AddTxMessage(
	            &hcan1,
	            &txHeader,
	            txData,
	            &txMailbox) != HAL_OK)
	    {
	        /* Do not print on every failed transmission. */
	        canFaultCode = FAULT_CAN_TIMEOUT;
	        canEnableRequest = 0U;
	    }

}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartPedalTask */
/**
* @brief Function implementing the PedalTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartPedalTask */
void StartPedalTask(void *argument)
{
  /* USER CODE BEGIN StartPedalTask */
    PedalData_t pedal;
    ADC_ChannelConfTypeDef sConfig = {0};

    for (;;)
    {
        osMutexAcquire(ADC1MutexHandle, osWaitForever);

        sConfig.Channel = ADC_CHANNEL_0;
        sConfig.Rank = 1;
        sConfig.SamplingTime = ADC_SAMPLETIME_84CYCLES;
        HAL_ADC_ConfigChannel(&hadc1, &sConfig);

        HAL_ADC_Start(&hadc1);
        HAL_ADC_PollForConversion(&hadc1, 100);
        pedal.adcValue = HAL_ADC_GetValue(&hadc1);
        HAL_ADC_Stop(&hadc1);

        osMutexRelease(ADC1MutexHandle);

        pedal.pedalPercent = ((float)pedal.adcValue * 100.0f) / 4095.0f;

        osMessageQueuePut(PedalQueueHandle, &pedal, 0U, 0U);
        osDelay(10);
    }
  /* USER CODE END StartPedalTask */
}

/* USER CODE BEGIN Header_StartTemperatureTask */
/**
* @brief Function implementing the TemperatureTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTemperatureTask */
void StartTemperatureTask(void *argument)
{
  /* USER CODE BEGIN StartTemperatureTask */
    TemperatureData_t temperature;
    ADC_ChannelConfTypeDef sConfig = {0};

    for (;;)
    {
        osMutexAcquire(ADC1MutexHandle, osWaitForever);

        sConfig.Channel = ADC_CHANNEL_1;
        sConfig.Rank = 1;
        sConfig.SamplingTime = ADC_SAMPLETIME_84CYCLES;
        HAL_ADC_ConfigChannel(&hadc1, &sConfig);

        HAL_ADC_Start(&hadc1);
        HAL_ADC_PollForConversion(&hadc1, 100);
        temperature.adcValue = HAL_ADC_GetValue(&hadc1);
        HAL_ADC_Stop(&hadc1);

        osMutexRelease(ADC1MutexHandle);

        temperature.temperatureC =
            ((float)temperature.adcValue * 120.0f) / 4095.0f;

        osMessageQueuePut(TemperatureQueueHandle, &temperature, 0U, 0U);
        osDelay(100);
    }
  /* USER CODE END StartTemperatureTask */
}

/* USER CODE BEGIN Header_StartControlTask */
/**
* @brief Function implementing the ControlTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartControlTask */
void StartControlTask(void *argument)
{
  /* USER CODE BEGIN StartControlTask */
	    PedalData_t pedal;
	    TemperatureData_t temperature;
	    MotorCommand_t motorCommand;

	    float latestPedalPercent = 0.0f;
	    float latestTemperatureC = 0.0f;

	    const uint32_t tickFreq = osKernelGetTickFreq();
	    const uint32_t timeoutTicks =
	        (CAN_COMMAND_TIMEOUT_MS * tickFreq) / 1000U;

	    for (;;)
	    {
	        if (osMessageQueueGet(
	                PedalQueueHandle, &pedal, NULL, 0U) == osOK)
	        {
	            latestPedalPercent = pedal.pedalPercent;
	        }

	        if (osMessageQueueGet(
	                TemperatureQueueHandle,
	                &temperature, NULL, 0U) == osOK)
	        {
	            latestTemperatureC = temperature.temperatureC;
	            systemTemperatureC = latestTemperatureC;
	        }

	        uint32_t now = osKernelGetTickCount();

	        uint8_t commOK =
	            (canCommandSeen != 0U) &&
	            ((uint32_t)(now - lastCanCommandTick) <= timeoutTicks);

	        uint8_t overTemperature =
	            (latestTemperatureC >= MOTOR_OVERTEMP_LIMIT_C);

	        if (overTemperature)
	        {
	            canEnableRequest = 0U;
	            canFaultCode = FAULT_OVERTEMPERATURE;
	        }
	        else if (!commOK)
	        {
	            canEnableRequest = 0U;
	            canFaultCode = FAULT_CAN_TIMEOUT;
	        }

	        float pedalRPM = 0.0f;

	        /* Validate the pedal percentage before using it. */
	        if (latestPedalPercent < 0.0f)
	            latestPedalPercent = 0.0f;

	        if (latestPedalPercent > 100.0f)
	            latestPedalPercent = 100.0f;

	        pedalRPM =
	            (latestPedalPercent / 100.0f) * (float)MOTOR_MAX_RPM;

	        float finalRPM = 0.0f;

	        if ((canEnableRequest != 0U) &&
	            (canFaultCode == FAULT_NONE) &&
	            commOK &&
	            !overTemperature)
	        {
	            finalRPM = pedalRPM;

	            if (finalRPM > (float)canMaxRPM)
	                finalRPM = (float)canMaxRPM;
	        }

	        motorCommand.desiredSpeedRPM = finalRPM;
	        requestedRPMStatus = (uint16_t)finalRPM;

	        /*
	         * The queue has length 1 and this is a latest-value command.
	         * Replace a pending value rather than silently losing updates
	         * when the queue is full.
	         */
	        osMessageQueueReset(MotorCommandQueueHandle);
	        osMessageQueuePut(
	            MotorCommandQueueHandle,
	            &motorCommand,
	            0U,
	            0U);

	        char buffer[128];
	        int len = snprintf(
	            buffer, sizeof(buffer),
	            "Pedal=%.1f%% Temp=%.1fC Limit=%u RPM=%.0f Fault=%u\r\n",
	            latestPedalPercent,
	            latestTemperatureC,
	            (unsigned int)canMaxRPM,
	            finalRPM,
	            (unsigned int)canFaultCode);

	        if (len > 0)
	        {
	            HAL_UART_Transmit(
	                &huart2, (uint8_t *)buffer,
	                (uint16_t)len, 20U);
	        }

	        osDelay(20U);
	    }

  /* USER CODE END StartControlTask */
}

/* USER CODE BEGIN Header_StartPWMTask */
/**
* @brief Function implementing the PWMTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartPWMTask */
void StartPWMTask(void *argument)
{
  /* USER CODE BEGIN StartPWMTask */
	    MotorCommand_t motorCommand;

	    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0U);
	    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);

	    for (;;)
	    {
	        if (osMessageQueueGet(
	                MotorCommandQueueHandle,
	                &motorCommand,
	                NULL,
	                portMAX_DELAY) == osOK)
	        {
	            float rpm = motorCommand.desiredSpeedRPM;

	            if (rpm < 0.0f)
	                rpm = 0.0f;

	            if (rpm > (float)MOTOR_MAX_RPM)
	                rpm = (float)MOTOR_MAX_RPM;

	            uint32_t arr = __HAL_TIM_GET_AUTORELOAD(&htim3);

	            uint32_t ccr =
	                (uint32_t)((rpm / (float)MOTOR_MAX_RPM) * arr);

	            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, ccr);
	            if (timingPending != 0U)
	            {
	                commandAppliedMs = HAL_GetTick();

	                uint32_t responseMs =
	                    commandAppliedMs - commandRxMs;

	                char msg[100];

	                int len = snprintf(
	                    msg,
	                    sizeof(msg),
	                    "CAN-to-PWM response: %lu ms, RPM=%.0f\r\n",
	                    (unsigned long)responseMs,
	                    (double)rpm);

	                if (len > 0 && len < (int)sizeof(msg))
	                {
	                    HAL_UART_Transmit(
	                        &huart2,
	                        (uint8_t *)msg,
	                        (uint16_t)len,
	                        100U);
	                }

	                timingPending = 0U;
	            }

	        }
	    }

  /* USER CODE END StartPWMTask */
}

/* USER CODE BEGIN Header_StartCanTask */
/**
* @brief Function implementing the CanTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartCanTask */
void StartCanTask(void *argument)
{
  /* USER CODE BEGIN StartCanTask*/
	    CAN_RxHeaderTypeDef rxHeader;
	    uint8_t rxData[8];

	    uint32_t lastStatusTick = osKernelGetTickCount();
	    const uint32_t tickFreq = osKernelGetTickFreq();

	    const uint32_t timeoutTicks =
	        (CAN_COMMAND_TIMEOUT_MS * tickFreq) / 1000U;

	    const uint32_t statusTicks =
	        (CAN_STATUS_PERIOD_MS * tickFreq) / 1000U;

	    for (;;)
	    {
	        /* Process all currently queued receive frames. */
	        while (HAL_CAN_GetRxFifoFillLevel(
	                   &hcan1, CAN_RX_FIFO0) > 0U)
	        {
	            if (HAL_CAN_GetRxMessage(
	                    &hcan1,
	                    CAN_RX_FIFO0,
	                    &rxHeader,
	                    rxData) != HAL_OK)
	            {
	                canEnableRequest = 0U;
	                canFaultCode = FAULT_INVALID_COMMAND;
	                break;
	            }

	            /* Accept standard data frames with ID 0x200 only. */
	            if ((rxHeader.IDE != CAN_ID_STD) ||
	                (rxHeader.RTR != CAN_RTR_DATA) ||
	                (rxHeader.StdId != 0x200U))
	            {
	                continue;
	            }

	            /* Require exactly 8 bytes. */
	            if (rxHeader.DLC != 8U)
	            {
	                canEnableRequest = 0U;
	                canMaxRPM = 0U;
	                canFaultCode = FAULT_INVALID_COMMAND;
	                continue;
	            }

	            uint8_t enable = rxData[0];

	            uint16_t rpmLimit =
	                ((uint16_t)rxData[1] << 8) | rxData[2];

	            uint8_t mode = rxData[3];

	            uint8_t valid = 1U;

	            if (enable > 1U)
	                valid = 0U;

	            if (rpmLimit > MOTOR_MAX_RPM)
	                valid = 0U;

	            if (mode != 0U)
	                valid = 0U;

	            /* Reserved bytes must be zero. */
	            if ((rxData[4] != 0U) ||
	                (rxData[5] != 0U) ||
	                (rxData[6] != 0U) ||
	                (rxData[7] != 0U))
	            {
	                valid = 0U;
	            }

	            if (valid == 0U)
	            {
	                canEnableRequest = 0U;
	                canMaxRPM = 0U;
	                canFaultCode = FAULT_INVALID_COMMAND;
	                continue;
	            }

	            /*
	             * Record the timestamp for this valid command.
	             * Do this only after validating the complete frame.
	             */
	            commandRxMs = HAL_GetTick();
	            timingPending = 1U;

	            /* Accept the valid command. */
	            canMaxRPM = rpmLimit;
	            lastCanCommandTick = osKernelGetTickCount();
	            canCommandSeen = 1U;

	            /* Preserve an active overtemperature fault. */
	            if (systemTemperatureC >= MOTOR_OVERTEMP_LIMIT_C)
	            {
	                canEnableRequest = 0U;
	                canFaultCode = FAULT_OVERTEMPERATURE;
	            }
	            else
	            {
	                canEnableRequest = enable;
	                canFaultCode = FAULT_NONE;
	            }

	            /* UART diagnostic for the received command. */
	            char msg[80];

	            int len = snprintf(
	                msg,
	                sizeof(msg),
	                "CAN cmd: enable=%u maxRPM=%u\r\n",
	                (unsigned int)enable,
	                (unsigned int)rpmLimit);

	            if (len > 0 && len < (int)sizeof(msg))
	            {
	                HAL_UART_Transmit(
	                    &huart2,
	                    (uint8_t *)msg,
	                    (uint16_t)len,
	                    20U);
	            }
	        }

	        uint32_t now = osKernelGetTickCount();

	        /* CAN communication timeout. */
	        if ((canCommandSeen == 0U) ||
	            ((uint32_t)(now - lastCanCommandTick) > timeoutTicks))
	        {
	            canEnableRequest = 0U;
	            canFaultCode = FAULT_CAN_TIMEOUT;
	        }

	        /* Periodic STM32 -> PC status transmission. */
	        if ((uint32_t)(now - lastStatusTick) >= statusTicks)
	        {
	            CAN_TransmitStatus();
	            lastStatusTick = now;
	        }

	        osDelay(2U);

	}



  /* USER CODE END StartCanTask */
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
