/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "fatfs.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "stdbool.h"
#include "string.h"
#include "stdio.h"
#include "math.h"

#include "uartDMA.h"
#include "ssd1306.h"
#include "inverter.h"
#include "vcu_can.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

typedef enum
{
    TS_INACTIVE,
    TS_ACTIVE,
    R2D_TRANSITION,
    R2D_MODE,
} VcuState;

typedef struct
{
    bool APPS1_SCS;
    bool APPS2_SCS;
    bool APPS_PB;

    bool BPS_SCS;
    bool BPS_PB;
} VcuErrors;

typedef struct
{
    bool SDC_Disconnected;
} VcuCritErrors;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ADC_BUF_LEN (1024 * 3)

VcuErrors vcuError = {0};
VcuCritErrors vcuCritError = {0};

volatile VcuState currVcuState = TS_INACTIVE;
volatile VcuState prevVcuState = TS_INACTIVE;

uint16_t adc1Buff[ADC_BUF_LEN]; // buffer to store values read from adc1
uint16_t volatile apps1Avg = 0;
uint16_t volatile apps2Avg = 0;
uint16_t volatile bpsAvg = 0;

float volatile apps1Scaled = 0;
float volatile apps2Scaled = 0;
float volatile appsPosition = 0;

float volatile appsDiff = 0;

volatile bool isFaultNormal   = false;
volatile bool isFaultCritical = false;

volatile bool isBrakePressed = false;

bool isCardDetected = false;

// 4.5V -> 3686
// 0.5V -> 410

const uint16_t ADC_MAX = 4095;

const uint16_t APPS1_MAX = 3180;
const uint16_t APPS1_MIN = 420;
const uint16_t APPS2_MAX = 3520;
const uint16_t APPS2_MIN = 845;
const uint16_t APPS_SCS_OFFSET = 0.1 / 5.0 * ADC_MAX;   // Volts offset to be considered SCS Fault

const uint16_t BPS_THRESH = ADC_MAX * 0.5;
const uint16_t BPS_SCS_UPPER = 4.75 / 5.0 * ADC_MAX;
const uint16_t BPS_SCS_LOWER = 0.25 / 5.0 * ADC_MAX;

const bool     CMD_SEL_ERPM = true;         // TRUE: ERPM, FALSE: CURRENT
const uint32_t CMD_ERPM_MAX = 50000;        // ERPM = Motor RPM * number of the motor pole pairs
const float    CMD_DC_CURRENT_MAX = 10;     // Amps
const float    CMD_AC_CURRENT_MAX = 10;     // Amps


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

CAN_HandleTypeDef hcan1;
CAN_HandleTypeDef hcan2;

I2C_HandleTypeDef hi2c1;
DMA_HandleTypeDef hdma_i2c1_tx;

SD_HandleTypeDef hsd;
DMA_HandleTypeDef hdma_sdio_rx;
DMA_HandleTypeDef hdma_sdio_tx;

UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_tx;

/* Definitions for t_main */
osThreadId_t t_mainHandle;
const osThreadAttr_t t_main_attributes = {
  .name = "t_main",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for t_faultHandler */
osThreadId_t t_faultHandlerHandle;
const osThreadAttr_t t_faultHandler_attributes = {
  .name = "t_faultHandler",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for t_uart */
osThreadId_t t_uartHandle;
const osThreadAttr_t t_uart_attributes = {
  .name = "t_uart",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for t_logging */
osThreadId_t t_loggingHandle;
const osThreadAttr_t t_logging_attributes = {
  .name = "t_logging",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for t_can */
osThreadId_t t_canHandle;
const osThreadAttr_t t_can_attributes = {
  .name = "t_can",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for q_printf */
osMessageQueueId_t q_printfHandle;
const osMessageQueueAttr_t q_printf_attributes = {
  .name = "q_printf"
};
/* Definitions for q_can1Tx */
osMessageQueueId_t q_can1TxHandle;
const osMessageQueueAttr_t q_can1Tx_attributes = {
  .name = "q_can1Tx"
};
/* Definitions for q_can2Tx */
osMessageQueueId_t q_can2TxHandle;
const osMessageQueueAttr_t q_can2Tx_attributes = {
  .name = "q_can2Tx"
};
/* Definitions for tim_invrtrCmd */
osTimerId_t tim_invrtrCmdHandle;
const osTimerAttr_t tim_invrtrCmd_attributes = {
  .name = "tim_invrtrCmd"
};
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_CAN1_Init(void);
static void MX_CAN2_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_SDIO_SD_Init(void);
void t_main_func(void *argument);
void t_faultHandler_func(void *argument);
extern void t_uart_func(void *argument);
void t_logging_func(void *argument);
extern void t_can_func(void *argument);
void invrtrCmd_Callback(void *argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


uint8_t BSP_SD_IsDetected(void)
{
  __IO uint8_t status = SD_PRESENT;

  if(HAL_GPIO_ReadPin(SD_DETECT_GPIO_PORT, SD_DETECT_PIN) != GPIO_PIN_SET)
  {
      status = SD_NOT_PRESENT;
  }

  return status;
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

  HAL_Delay(1000);          // Boot Delay

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_CAN1_Init();
  MX_CAN2_Init();
  MX_I2C1_Init();
  MX_USART2_UART_Init();
  MX_SDIO_SD_Init();
  MX_FATFS_Init();
  /* USER CODE BEGIN 2 */


  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* Create the timer(s) */
  /* creation of tim_invrtrCmd */
  tim_invrtrCmdHandle = osTimerNew(invrtrCmd_Callback, osTimerPeriodic, NULL, &tim_invrtrCmd_attributes);

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of q_printf */
  q_printfHandle = osMessageQueueNew (16, sizeof(printfString_t), &q_printf_attributes);

  /* creation of q_can1Tx */
  q_can1TxHandle = osMessageQueueNew (16, sizeof(CanTxMsg), &q_can1Tx_attributes);

  /* creation of q_can2Tx */
  q_can2TxHandle = osMessageQueueNew (16, sizeof(CanTxMsg), &q_can2Tx_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of t_main */
  t_mainHandle = osThreadNew(t_main_func, NULL, &t_main_attributes);

  /* creation of t_faultHandler */
  t_faultHandlerHandle = osThreadNew(t_faultHandler_func, NULL, &t_faultHandler_attributes);

  /* creation of t_uart */
  t_uartHandle = osThreadNew(t_uart_func, NULL, &t_uart_attributes);

  /* creation of t_logging */
  t_loggingHandle = osThreadNew(t_logging_func, NULL, &t_logging_attributes);

  /* creation of t_can */
  t_canHandle = osThreadNew(t_can_func, NULL, &t_can_attributes);

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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
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
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV8;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 3;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_56CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = 3;
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
  hcan1.Init.Prescaler = 12;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_11TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = ENABLE;
  hcan1.Init.AutoWakeUp = ENABLE;
  hcan1.Init.AutoRetransmission = ENABLE;
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
  * @brief CAN2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN2_Init(void)
{

  /* USER CODE BEGIN CAN2_Init 0 */

  /* USER CODE END CAN2_Init 0 */

  /* USER CODE BEGIN CAN2_Init 1 */

  /* USER CODE END CAN2_Init 1 */
  hcan2.Instance = CAN2;
  hcan2.Init.Prescaler = 6;
  hcan2.Init.Mode = CAN_MODE_NORMAL;
  hcan2.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan2.Init.TimeSeg1 = CAN_BS1_11TQ;
  hcan2.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan2.Init.TimeTriggeredMode = DISABLE;
  hcan2.Init.AutoBusOff = ENABLE;
  hcan2.Init.AutoWakeUp = ENABLE;
  hcan2.Init.AutoRetransmission = ENABLE;
  hcan2.Init.ReceiveFifoLocked = DISABLE;
  hcan2.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN2_Init 2 */

  /* USER CODE END CAN2_Init 2 */

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
  hi2c1.Init.ClockSpeed = 400000;
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
  * @brief SDIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_SDIO_SD_Init(void)
{

  /* USER CODE BEGIN SDIO_Init 0 */

  /* USER CODE END SDIO_Init 0 */

  /* USER CODE BEGIN SDIO_Init 1 */
  
    hsd.Instance = SDIO;
    hsd.Init.ClockEdge = SDIO_CLOCK_EDGE_RISING;
    hsd.Init.ClockBypass = SDIO_CLOCK_BYPASS_DISABLE;
    hsd.Init.ClockPowerSave = SDIO_CLOCK_POWER_SAVE_DISABLE;
    // Corrected to 1B for initialisation: https://community.st.com/t5/stm32cubemx-mcus/sdio-interface-not-working-in-4bits-with-stm32f4-firmware/m-p/625603#M26901
    // SDIO always init at 1B bus width
    hsd.Init.BusWide = SDIO_BUS_WIDE_1B;
    hsd.Init.HardwareFlowControl = SDIO_HARDWARE_FLOW_CONTROL_DISABLE;
    hsd.Init.ClockDiv = 0;


#define ONLY_USER_INIT // Tired of editing the code every time you change, the line "hsd.Init.BusWide = SDIO_BUS_WIDE_4B;"
#ifndef ONLY_USER_INIT

  /* USER CODE END SDIO_Init 1 */
  hsd.Instance = SDIO;
  hsd.Init.ClockEdge = SDIO_CLOCK_EDGE_RISING;
  hsd.Init.ClockBypass = SDIO_CLOCK_BYPASS_DISABLE;
  hsd.Init.ClockPowerSave = SDIO_CLOCK_POWER_SAVE_DISABLE;
  hsd.Init.BusWide = SDIO_BUS_WIDE_4B;
  hsd.Init.HardwareFlowControl = SDIO_HARDWARE_FLOW_CONTROL_DISABLE;
  hsd.Init.ClockDiv = 0;
  /* USER CODE BEGIN SDIO_Init 2 */

#endif

  if (BSP_SD_IsDetected())
  {
      if (HAL_SD_Init(&hsd) != HAL_OK)
      {
          Error_Handler();
      }
      if (HAL_SD_ConfigWideBusOperation(&hsd, SDIO_BUS_WIDE_4B) != HAL_OK)
      {
          Error_Handler();
      }
      isCardDetected = true;
  }
  else
  {
      isCardDetected = false;
      HAL_GPIO_WritePin(DEBUG_LED_GPIO_Port, DEBUG_LED_Pin, GPIO_PIN_SET);
  }

  /* USER CODE END SDIO_Init 2 */

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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);
  /* DMA1_Stream7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream7_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream7_IRQn);
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
  /* DMA2_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream3_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream3_IRQn);
  /* DMA2_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream6_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream6_IRQn);

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(DEBUG_LED_GPIO_Port, DEBUG_LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, DO_R2D_SOUND_Pin|DO_R2D_LIGHT_Pin|DO_SC_LIGHT_Pin|DO_SC_RELAY_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : DEBUG_LED_Pin */
  GPIO_InitStruct.Pin = DEBUG_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(DEBUG_LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : IT_TS_BUTTON_Pin */
  GPIO_InitStruct.Pin = IT_TS_BUTTON_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(IT_TS_BUTTON_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : IT_R2D_BUTTON_Pin */
  GPIO_InitStruct.Pin = IT_R2D_BUTTON_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(IT_R2D_BUTTON_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : DO_R2D_SOUND_Pin DO_R2D_LIGHT_Pin DO_SC_LIGHT_Pin DO_SC_RELAY_Pin */
  GPIO_InitStruct.Pin = DO_R2D_SOUND_Pin|DO_R2D_LIGHT_Pin|DO_SC_LIGHT_Pin|DO_SC_RELAY_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : SD_CARD_DETECT_Pin */
  GPIO_InitStruct.Pin = SD_CARD_DETECT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(SD_CARD_DETECT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SC_IN_Pin */
  GPIO_InitStruct.Pin = SC_IN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(SC_IN_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI4_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

bool buttonPressed_TS = false;
bool buttonPressed_R2D = false;
int interruptCount = 0;

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    uint32_t currentTime = HAL_GetTick();

    const uint32_t DEBOUNCE_DELAY = 200;
    static uint32_t lastDebounceTime_TS = 0;
    static uint32_t lastDebounceTime_R2D = 0;

    switch (GPIO_Pin)
    {
        case IT_TS_BUTTON_Pin:
            // Debounce check
            if (currentTime - lastDebounceTime_TS < DEBOUNCE_DELAY) break;
            lastDebounceTime_TS = currentTime;
            buttonPressed_TS = true;
            printfDma("TS Button pressed\n");
            break;

        case IT_R2D_BUTTON_Pin:
            // Debounce check
            if (currentTime - lastDebounceTime_R2D < DEBOUNCE_DELAY) break;
            lastDebounceTime_R2D = currentTime;
            buttonPressed_R2D = true;
            printfDma("R2D Button pressed\n");
            break;

        default:
            break;
    }

    interruptCount++;
}

uint32_t prevTime = 0;
uint32_t timeDiff = 0;
uint32_t currTime = 0;
uint32_t setErpm = 0;

void ssd1306_RasterIntCallback(uint8_t r)
{
    currTime = HAL_GetTick();
    timeDiff = currTime - prevTime;
    prevTime = currTime;

    ssd1306_SetColor(Black);
    ssd1306_Fill();

    ssd1306_SetColor(White);
    ssd1306_DrawRect(0,  0, apps1Avg * 128 / ADC_MAX, 8);
    ssd1306_DrawRect(0,  0, apps2Avg * 128 / ADC_MAX, 8);

    char msg[32];
    char *stateStr;
    switch (currVcuState)
    {
        case TS_INACTIVE:
            stateStr = "IDLE";
            break;
        case TS_ACTIVE:
            stateStr = "TS";
            break;
        case R2D_TRANSITION:
            stateStr = "BUZ";
            break;
        case R2D_MODE:
            stateStr = "R2D";
            break;
        default:
            stateStr = "ERR";
            break;
    }

//    snprintf(msg, 64, "FPS: %.0lf (%ld ms)", 1 / ((float)timeDiff/1000.0), timeDiff);
    snprintf(msg, 64, "S%d%d%d P%d%d X%d (%s)",
             vcuError.APPS1_SCS,
             vcuError.APPS2_SCS,
             vcuError.BPS_SCS,
             vcuError.APPS_PB,
             vcuError.BPS_PB,
             vcuCritError.SDC_Disconnected,
             stateStr
             );
//    snprintf(msg, 64, "R ERPM:%ld V:%d", invrtr.erpm, invrtr.inputVoltage);
    ssd1306_SetCursor(0, 11);
    ssd1306_WriteString(msg, Font_7x10);

    snprintf(msg, 64, "S ERPM:%ld", invrtr.setErpm);
    ssd1306_SetCursor(0, 21);
    ssd1306_WriteString(msg, Font_7x10);
}


bool checkSCS(uint16_t value, uint16_t min, uint16_t max)
{
    if (value > max || value < min)
    {
        return true;
    }
    return false;
}


float mapValue(float value, float min_val, float max_val)
{
    if (max_val == min_val) return 0.0f; // avoid division by zero

    if (value > max_val)
    {
        value = max_val;
    }
    else if (value < min_val)
    {
        value = min_val;
    }

    return ((value - min_val) / (max_val - min_val));
}


// Called when first half of buffer is filled
void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* hadc)
{
    uint32_t sum1 = 0;
    uint32_t sum2 = 0;
    uint32_t sum3 = 0;

    // Calculate the average of the first half of the buffer
    if (hadc == &hadc1)
    {
        for (int i = 0; i < ADC_BUF_LEN/2; i += 3)
        {
            sum1 += adc1Buff[i];
            sum2 += adc1Buff[i+1];
            sum3 += adc1Buff[i+2];
        }

        // Invert if needed
        uint32_t rawBpsAvg   = (sum1 / (ADC_BUF_LEN/6));
        uint32_t rawApps2Avg = (sum2 / (ADC_BUF_LEN/6));
        uint32_t rawApps1Avg = (sum3 / (ADC_BUF_LEN/6));

        // basic Low pass filter
        float a = 0.9;
        float b = 0.1;
        bpsAvg   = bpsAvg   * a + rawBpsAvg   * b;
        apps2Avg = apps2Avg * a + rawApps2Avg * b;
        apps1Avg = apps1Avg * a + rawApps1Avg * b;
    }

    vcuError.APPS1_SCS = checkSCS(apps1Avg, APPS1_MIN - APPS_SCS_OFFSET, APPS1_MAX + APPS_SCS_OFFSET);
    vcuError.APPS2_SCS = checkSCS(apps2Avg, APPS2_MIN - APPS_SCS_OFFSET, APPS2_MAX + APPS_SCS_OFFSET);
    apps1Scaled =  1.0 - mapValue(apps1Avg, APPS1_MIN, APPS1_MAX);
    apps2Scaled =  1.0 - mapValue(apps2Avg, APPS2_MIN, APPS2_MAX);
    appsPosition = (apps1Scaled + apps2Scaled) / 2;          // Get the average of position from both sensors
    appsDiff = apps1Scaled - apps2Scaled;

    vcuError.BPS_SCS = checkSCS(bpsAvg, BPS_SCS_LOWER, BPS_SCS_UPPER);
    isBrakePressed = (bool)(bpsAvg > BPS_THRESH);
}

// Called when buffer is completely filled
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
    uint32_t sum1 = 0;
    uint32_t sum2 = 0;
    uint32_t sum3 = 0;

    // Calculate the average of the second half of the buffer
    if (hadc == &hadc1)
    {
        for (int i = ADC_BUF_LEN/2; i < ADC_BUF_LEN; i += 3)
        {
            sum1 += adc1Buff[i];
            sum2 += adc1Buff[i+1];
            sum3 += adc1Buff[i+2];
        }

        // Invert if needed
        uint32_t rawBpsAvg   = (sum1 / (ADC_BUF_LEN/6));
        uint32_t rawApps2Avg = (sum2 / (ADC_BUF_LEN/6));
        uint32_t rawApps1Avg = (sum3 / (ADC_BUF_LEN/6));

        // basic Low pass filter
        float a = 0.9;
        float b = 0.1;
        bpsAvg   = bpsAvg   * a + rawBpsAvg   * b;
        apps2Avg = apps2Avg * a + rawApps2Avg * b;
        apps1Avg = apps1Avg * a + rawApps1Avg * b;
    }

    vcuError.APPS1_SCS = checkSCS(apps1Avg, APPS1_MIN - APPS_SCS_OFFSET, APPS1_MAX + APPS_SCS_OFFSET);
    vcuError.APPS2_SCS = checkSCS(apps2Avg, APPS2_MIN - APPS_SCS_OFFSET, APPS2_MAX + APPS_SCS_OFFSET);
    apps1Scaled =  1.0 - mapValue(apps1Avg, APPS1_MIN, APPS1_MAX);
    apps2Scaled =  1.0 - mapValue(apps2Avg, APPS2_MIN, APPS2_MAX);
    appsPosition = (apps1Scaled + apps2Scaled) / 2;          // Get the average of position from both sensors
    appsDiff = apps1Scaled - apps2Scaled;

    vcuError.BPS_SCS = checkSCS(bpsAvg, BPS_SCS_LOWER, BPS_SCS_UPPER);
    isBrakePressed = (bool)(bpsAvg > BPS_THRESH);
}


/* USER CODE END 4 */

/* USER CODE BEGIN Header_t_main_func */
/**
  * @brief  Function implementing the t_main thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_t_main_func */
void t_main_func(void *argument)
{
  /* USER CODE BEGIN 5 */
    uint32_t r2dTimer = 0;
    const uint32_t R2D_PERIOD = 2000;
    const uint32_t INVRTR_PERIOD = 20;

    /* Infinite loop */
    for(;;)
    {
        if (isFaultCritical)          // Force to inactive state if crit fault detected
        {
            currVcuState = TS_INACTIVE;
        }

        // State transition checks
        switch (currVcuState)
        {
        case TS_INACTIVE:
            if (buttonPressed_TS && !isFaultCritical)
            {
                currVcuState = TS_ACTIVE;
            }
            break;

        case TS_ACTIVE:
            if (buttonPressed_TS)
            {
                currVcuState = TS_INACTIVE;
                break;
            }
            if (buttonPressed_R2D && !isFaultNormal && (appsPosition < 0.1))
            {
                currVcuState = R2D_TRANSITION;
                break;
            }
            break;

        case R2D_TRANSITION:
            if (buttonPressed_R2D)
            {
                currVcuState = TS_ACTIVE;
                break;
            }
            if ((HAL_GetTick() - r2dTimer) > R2D_PERIOD)
            {
                currVcuState = R2D_MODE;
            }
            break;

        case R2D_MODE:
            if (buttonPressed_R2D)
            {
                currVcuState = TS_ACTIVE;
                break;
            }
            break;

        default:
            break;
        }

        // State transitions
        if (currVcuState != prevVcuState)
        {
            switch (currVcuState)
            {
            case TS_INACTIVE:
                printfDma("VCU State Changed: TS Inactive \n");
                HAL_GPIO_WritePin(DO_SC_RELAY_GPIO_Port,  DO_SC_RELAY_Pin,  0);
                HAL_GPIO_WritePin(DO_R2D_LIGHT_GPIO_Port, DO_R2D_LIGHT_Pin, 0);
                HAL_GPIO_WritePin(DO_R2D_SOUND_GPIO_Port, DO_R2D_SOUND_Pin, 0);
                osTimerStop(tim_invrtrCmdHandle);
                break;

            case TS_ACTIVE:
                printfDma("VCU State Changed: TS Active \n");
                HAL_GPIO_WritePin(DO_SC_RELAY_GPIO_Port,  DO_SC_RELAY_Pin,  1);
                HAL_GPIO_WritePin(DO_R2D_LIGHT_GPIO_Port, DO_R2D_LIGHT_Pin, 0);
                HAL_GPIO_WritePin(DO_R2D_SOUND_GPIO_Port, DO_R2D_SOUND_Pin, 0);
                osTimerStop(tim_invrtrCmdHandle);
                break;

            case R2D_TRANSITION:
                printfDma("VCU State Changed: R2D Transition \n");
                HAL_GPIO_WritePin(DO_SC_RELAY_GPIO_Port,  DO_SC_RELAY_Pin,  1);
                HAL_GPIO_WritePin(DO_R2D_LIGHT_GPIO_Port, DO_R2D_LIGHT_Pin, 1);
                HAL_GPIO_WritePin(DO_R2D_SOUND_GPIO_Port, DO_R2D_SOUND_Pin, 1);
                osTimerStop(tim_invrtrCmdHandle);
                r2dTimer = HAL_GetTick();
                break;

            case R2D_MODE:
                printfDma("VCU State Changed: R2D Mode \n");
                HAL_GPIO_WritePin(DO_SC_RELAY_GPIO_Port,  DO_SC_RELAY_Pin,  1);
                HAL_GPIO_WritePin(DO_R2D_LIGHT_GPIO_Port, DO_R2D_LIGHT_Pin, 1);
                HAL_GPIO_WritePin(DO_R2D_SOUND_GPIO_Port, DO_R2D_SOUND_Pin, 0);

                // Set current limits
                if (!CMD_SEL_ERPM)
                {
                    inverter_setMaxDCCurrent(CMD_DC_CURRENT_MAX);
                    inverter_setMaxACCurrent(CMD_AC_CURRENT_MAX);
                }

                osTimerStart(tim_invrtrCmdHandle, INVRTR_PERIOD);
                break;

            default:
                break;
            }
        }

        buttonPressed_R2D = false;
        buttonPressed_TS = false;

        prevVcuState = currVcuState;
        osDelay(10);
    }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_t_faultHandler_func */
/**
* @brief Function implementing the t_faultHandler thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_t_faultHandler_func */
void t_faultHandler_func(void *argument)
{
  /* USER CODE BEGIN t_faultHandler_func */
    const uint32_t APPS_PB_PERIOD_MS = 100;
    static uint32_t lastAppsPbTime = 0;
    static bool appsPbDetected = false;

    const uint32_t BPS_PB_PERIOD_MS = 500;
    static uint32_t lastBpsPbTime = 0;
    static bool bpsPbDetected = false;

    for(;;)
    {
        uint32_t currentTime = HAL_GetTick();

        // Shutdown circuit input detection
        // Considered as a critical error
        if (HAL_GPIO_ReadPin(SC_IN_GPIO_Port, SC_IN_Pin))
        {
            HAL_GPIO_WritePin(DO_SC_LIGHT_GPIO_Port, DO_SC_LIGHT_Pin, 0);
            vcuCritError.SDC_Disconnected = false;
        }
        else
        {
            HAL_GPIO_WritePin(DO_SC_LIGHT_GPIO_Port, DO_SC_LIGHT_Pin, 1);
            vcuCritError.SDC_Disconnected = true;
        }

        // APPS Plausibility check
        if ((fabsf(appsDiff) > 0.1)) // NOT OK
        {
            if (!appsPbDetected)
            {
                lastAppsPbTime = currentTime;
                appsPbDetected = true;
            }
            else if (currentTime - lastAppsPbTime > APPS_PB_PERIOD_MS)
            {
                vcuError.APPS_PB = true;
            }
        }
        else // OK
        {
            appsPbDetected = false;
            vcuError.APPS_PB = false;
        }

        // APPS BPS Plausibility check
        if (isBrakePressed && appsPosition > 0.25) // NOT OK
        {
            if (!bpsPbDetected)
            {
                lastBpsPbTime = currentTime;
                bpsPbDetected = true;
            }
            else if (currentTime - lastBpsPbTime > BPS_PB_PERIOD_MS)
            {
                vcuError.BPS_PB = true;
            }
        }
        else if (appsPosition < 0.05)       // only clear the flag if apps returns below 5%
        {
            bpsPbDetected = false;
            vcuError.BPS_PB = false;
        }


        // Update the fault flags
        isFaultNormal = (
                vcuError.APPS1_SCS ||
                vcuError.APPS2_SCS ||
                vcuError.APPS_PB ||
                vcuError.BPS_SCS ||
                vcuError.BPS_PB
            );

        isFaultCritical = (
                vcuCritError.SDC_Disconnected
            );

        static uint8_t ledCounter = 0;
        if (ledCounter++ > 50)
        {
            HAL_GPIO_TogglePin(DEBUG_LED_GPIO_Port, DEBUG_LED_Pin);
            ledCounter = 0;
        }
        osDelay(5);
    }

  /* USER CODE END t_faultHandler_func */
}

/* USER CODE BEGIN Header_t_logging_func */
/**
* @brief Function implementing the t_logging thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_t_logging_func */
void t_logging_func(void *argument)
{
  /* USER CODE BEGIN t_logging_func */

    osDelay(500); // Startup delay

    // Start ADC Conversion
    HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc1Buff, ADC_BUF_LEN);

    // Start CAN
    can_setup();

    /* OLED SCREEN STUFF */
    ssd1306_Init();
    ssd1306_SetRasterInt(1);        // Enable interrupt
//  ssd1306_FlipScreenVertically();

    /* SD card stuff */
    if (isCardDetected)
    {
        printfDma("Block size   : %lu\n", hsd.SdCard.BlockSize);
        printfDma("Block nbmr   : %lu\n", hsd.SdCard.BlockNbr);
        printfDma("Card size    : %lu\n", (hsd.SdCard.BlockSize * hsd.SdCard.BlockNbr) / 1000);
        printfDma("Card ver     : %lu\n", hsd.SdCard.CardVersion);

        if (f_mount(&SDFatFS, (TCHAR const*) SDPath, 0) != FR_OK)
        {
            printfDma("Unable to mount SD Card \n");
            Error_Handler();
        }

        FIL fil;
        UINT bytesRead;
        uint8_t buff[32];

        /* Open a text file (w+ mode)*/
        if (f_open(&fil, "message.txt", FA_WRITE | FA_READ) != FR_OK)
        {
            Error_Handler(); // Error accessing file
        }

        if (f_read(&fil, buff, 32, &bytesRead))
        {
            Error_Handler();
        }

        int val = atoi((char *)buff);
        f_rewind(&fil);

        // Fails if return negative
        if (f_printf(&fil, "%d \n\nSD CARD TEST SUCCESS \n", ++val) < 0)
        {
            Error_Handler();
        }

        printfDma("Runtime Cycle: %d \n", val);

        /* Close the file */
        f_close(&fil);
    }

//    float fr = 0;


  /* Infinite loop */
  for(;;)
  {
        osDelay(10);

//        HAL_GPIO_TogglePin(DEBUG_LED_GPIO_Port, DEBUG_LED_Pin);
//        HAL_GPIO_TogglePin(DO_SC_LIGHT_GPIO_Port,  DO_SC_LIGHT_Pin);

//        fr = fr + 1;

//        printfDma("%f %f %f \n", fr, fr, fr);
//        printfDma("                           \n");
//        char *msg = "Test s\n";
//        HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);

//        HAL_GPIO_TogglePin(DO_R2D_LIGHT_GPIO_Port, DO_R2D_LIGHT_Pin);
//        osDelay(350);
//        HAL_GPIO_TogglePin(DO_R2D_SOUND_GPIO_Port, DO_R2D_SOUND_Pin);
//        osDelay(241);
//        HAL_GPIO_TogglePin(DO_SC_LIGHT_GPIO_Port,  DO_SC_LIGHT_Pin);
//        osDelay(111);
  }
//        _LED_GPIO_Port, DEBUG_LED_Pin);
    //        HAL_GPIO_TogglePin(DO_SC_LIGHT_GPIO_Port,  DO_SC_LIGHT_Pin);

    //        fr = fr + 1;
  /* USER CODE END t_logging_func */
}

/* invrtrCmd_Callback function */
void invrtrCmd_Callback(void *argument)
{
  /* USER CODE BEGIN invrtrCmd_Callback */
    const float MIN_POS = 0.05;
    const float MAX_POS = 0.95;
    float responseMult  = mapValue(appsPosition, MIN_POS, MAX_POS);

    if (isFaultNormal)
    {
        invrtr.setErpm = 0;
        invrtr.setRelativeCurrent = 0;
    }
    else
    {
        invrtr.setErpm = (uint32_t) (responseMult * CMD_ERPM_MAX);
        invrtr.setRelativeCurrent = responseMult;
    }

    if (CMD_SEL_ERPM)
    {
        inverter_setERPM((uint32_t) invrtr.setErpm);
    }
    else
    {
        inverter_setRelativeCurrent(invrtr.setRelativeCurrent);
    }
    inverter_setDriveEnable(1);

    const uint32_t cmdID = 0x20;
    uint32_t id = ((uint32_t)cmdID << 8) | 0x20;
    bool isExtId = true;

    static int canCounter = 0;
    canCounter++;
    uint8_t data[8] = {0};
    data[6] = canCounter >> 8;
    data[7] = canCounter;
    // invrtr.inputVoltage = ((int16_t)data[6] << 8) | data[7];
    can2_sendMsg(id, isExtId, data, sizeof(data));
  /* USER CODE END invrtrCmd_Callback */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM4 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM4)
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
