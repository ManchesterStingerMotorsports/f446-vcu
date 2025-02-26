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
#include "string.h"
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

//SCALE = 10 except for ERPM and DriveEnable which is 1 #######################

#define INVERTER_NODE_ID             0x22  // Example Node ID from CAN Manual
#define CMD_SET_AC_CURRENT           0x01  // Set AC Current
#define CMD_SET_BRAKE_CURRENT        0x02  // Set Brake Current
#define CMD_SET_ERPM                 0x03  // Set ERPM
#define CMD_SET_POSITION             0x04  // Set Position
#define CMD_SET_RELATIVE_CURRENT     0x05  // Set Relative Current
#define CMD_SET_RELATIVE_BRAKE_CUR   0x06  // Set Relative Brake Current
#define CMD_SET_DIGITAL_OUTPUT       0x07  // Set Digital Output
#define CMD_SET_MAX_AC_CURRENT       0x08  // Set Maximum AC Current
#define CMD_SET_MAX_BRAKE_AC_CURRENT 0x09  // Set Maximum AC Brake Current
#define CMD_SET_MAX_DC_CURRENT       0x0A  // Set Maximum DC Current
#define CMD_SET_MAX_DC_BRAKE_CURRENT 0x0B  // Set Maximum DC Brake Current
#define CMD_DRIVE_ENABLE             0x0C  // Drive Enable Command
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_CAN1_Init(void);
/* USER CODE BEGIN PFP */

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan);

// COMMANDS TO INVERTER
void sendSetACCurrent(int16_t acCurrent);
void sendSetBrakeCurrent(int16_t brakeCurrent);
void sendSetERPM(int32_t erpm);
void sendSetPosition(int16_t position);
void sendSetRelativeCurrent(int16_t relativeCurrent);
void sendSetRelativeBrakeCurrent(int16_t relativeBrakeCurrent);
void sendSetDigitalOutput(uint8_t digitalOut);
void sendSetMaxACCurrent(int16_t maxACCurrent);
void sendSetMaxBrakeACCurrent(int16_t maxBrakeACCurrent);
void sendSetMaxDCCurrent(int16_t maxDCCurrent);
void sendSetMaxDCBrakeCurrent(int16_t maxDCBrakeCurrent);
void sendDriveEnable(uint8_t enable);


// DATA FROM INVERTER
void decodeMessage0x20(uint8_t *data);
void decodeMessage0x21(uint8_t *data);
void decodeMessage0x22(uint8_t *data);
void decodeMessage0x23(uint8_t *data);
void decodeMessage0x24(uint8_t *data);


/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

CAN_TxHeaderTypeDef TxHeader;
CAN_RxHeaderTypeDef RxHeader;

uint8_t TxData[8];
uint8_t RxData[8];

uint32_t TxMailbox;

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
  MX_CAN1_Init();
  /* USER CODE BEGIN 2 */

  HAL_CAN_Start(&hcan1);

  HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);


//  // TESTING
//  TxHeader.DLC = 2;
//  TxHeader.IDE = CAN_ID_STD;
//  TxHeader.RTR = CAN_RTR_DATA;
//  TxHeader.StdId = 0x123;
//
//  TxData[0] = 5;
//  TxData[1] = 5;
//
//  if (HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox) != HAL_OK) {
//      Error_Handler();  // Transmission failed
//  }

  char msg[] = "CAN Started\r\n";
  HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);



//	sendDriveEnable(1);
//	sendSetACCurrent(100);  // e.g., 10 A -> 100 (scaled)
//	sendSetERPM(500); // 500 RPM


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	// testing
	sendSetMaxBrakeACCurrent(-1000);
	HAL_Delay(500);

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

  CAN_FilterTypeDef sf;
  sf.FilterIdHigh = 0x0000;
  sf.FilterIdLow = 0x0000;
  sf.FilterMaskIdHigh = 0x0000;
  sf.FilterMaskIdLow = 0x0000;
  sf.FilterFIFOAssignment = CAN_FILTER_FIFO0;
  sf.FilterBank = 0;
  sf.FilterMode = CAN_FILTERMODE_IDMASK;
  sf.FilterScale = CAN_FILTERSCALE_32BIT;
  sf.FilterActivation = CAN_FILTER_ENABLE;

  if (HAL_CAN_ConfigFilter(&hcan1, &sf) != HAL_OK)
  {
	  Error_Handler();
  }

  /* USER CODE END CAN1_Init 2 */

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
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  CAN_RxHeaderTypeDef RxHeader;
  uint8_t RxData[8];
  if(hcan->Instance == CAN1)
  {
    if(HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData) != HAL_OK)
    {
      Error_Handler();
    }
    if(RxHeader.IDE == CAN_ID_EXT)
    {
		HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
		uint32_t extId = RxHeader.ExtId;
		uint8_t packetId = (uint8_t)(extId >> 8);   // Packet ID is the high byte
		uint8_t nodeId = (uint8_t)(extId & 0xFF);     // Node ID is the low byte
		char msg[100];
		sprintf(msg, "Received: PacketID=0x%02X, NodeID=0x%02X, DLC=%ld, Data=",
			  packetId, nodeId, RxHeader.DLC);
		for(int i = 0; i < RxHeader.DLC; i++)
		{
		char byteStr[6];
		sprintf(byteStr, "%02X ", RxData[i]);
		strcat(msg, byteStr);
		}
		strcat(msg, "\r\n");
		HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);

		switch(packetId)
		{
			case 0x20:
				decodeMessage0x20(RxData);
				break;
			case 0x21:
				decodeMessage0x21(RxData);
				break;
			case 0x22:
				decodeMessage0x22(RxData);
				break;
			case 0x23:
				decodeMessage0x23(RxData);
				break;
			case 0x24:
				decodeMessage0x24(RxData);
				break;
			default:
				// Handle unknown messages as needed.
				break;
		}

		HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
    }
  }
}

/*

#############  COMMANDS TO INVERTER ######################

*/


/* Generic function to send a command over CAN.
   Constructs the extended CAN ID as: (cmdID << 8) | INVERTER_NODE_ID.
   (See DTI CAN Manual V2.4, e.g., Table 11 for Set AC Current command.) */
HAL_StatusTypeDef CAN_SendCommand(uint32_t cmdID, uint8_t* data, uint8_t dlc)
{
  CAN_TxHeaderTypeDef TxHeader;
  uint32_t TxMailbox;
  TxHeader.IDE = CAN_ID_EXT;
  TxHeader.ExtId = ((uint32_t)cmdID << 8) | INVERTER_NODE_ID;
  TxHeader.RTR = CAN_RTR_DATA;
  TxHeader.DLC = dlc;
  return HAL_CAN_AddTxMessage(&hcan1, &TxHeader, data, &TxMailbox);
}

/* Command: Set AC Current (0x01)
   As per Table 11 in the manual, the first 2 bytes contain the target AC current
   (in Apk, scaled by 10) in Big Endian order. */
void sendSetACCurrent(int16_t acCurrent)
{
  uint8_t payload[2];
  payload[0] = (uint8_t)((uint16_t)acCurrent >> 8);
  payload[1] = (uint8_t)(acCurrent & 0xFF);
  if (CAN_SendCommand(CMD_SET_AC_CURRENT, payload, 2) != HAL_OK)
  {
    Error_Handler();
  }
}

void sendSetBrakeCurrent(int16_t brkCurrent)
{
	uint8_t payload[2];
	payload[0] = (uint8_t)((uint16_t)brkCurrent >> 8);
	payload[1] = (uint8_t)(brkCurrent & 0xFF);
	if (CAN_SendCommand(CMD_SET_BRAKE_CURRENT, payload, 2) != HAL_OK)
	{
	Error_Handler();
	}
}

/* Command: Set ERPM (0x03)
   According to Table 13, a 4-byte signed value (Big Endian) is sent. */
void sendSetERPM(int32_t erpm)
{
  uint8_t payload[4];
  payload[0] = (uint8_t)((uint32_t)erpm >> 24);
  payload[1] = (uint8_t)((uint32_t)erpm >> 16);
  payload[2] = (uint8_t)((uint32_t)erpm >> 8);
  payload[3] = (uint8_t)(erpm & 0xFF);
  if (CAN_SendCommand(CMD_SET_ERPM, payload, 4) != HAL_OK)
  {
    Error_Handler();
  }
}


// Command: Set Position (0x04)
// Payload: 2 bytes, target position (in degrees multiplied by 10)
void sendSetPosition(int16_t position)
{
    uint8_t payload[2];
    payload[0] = (uint8_t)((uint16_t)position >> 8);
    payload[1] = (uint8_t)(position & 0xFF);
    if (CAN_SendCommand(CMD_SET_POSITION, payload, 2) != HAL_OK)
    {
        Error_Handler();
    }
}

// Command: Set Relative Current (0x05)
// Payload: 2 bytes, target relative current percentage (multiplied by 10)
void sendSetRelativeCurrent(int16_t relativeCurrent)
{
    uint8_t payload[2];
    payload[0] = (uint8_t)((uint16_t)relativeCurrent >> 8);
    payload[1] = (uint8_t)(relativeCurrent & 0xFF);
    if (CAN_SendCommand(CMD_SET_RELATIVE_CURRENT, payload, 2) != HAL_OK)
    {
        Error_Handler();
    }
}

// Command: Set Relative Brake Current (0x06)
// Payload: 2 bytes, target relative brake current percentage (multiplied by 10)
void sendSetRelativeBrakeCurrent(int16_t relativeBrakeCurrent)
{
    uint8_t payload[2];
    payload[0] = (uint8_t)((uint16_t)relativeBrakeCurrent >> 8);
    payload[1] = (uint8_t)(relativeBrakeCurrent & 0xFF);
    if (CAN_SendCommand(CMD_SET_RELATIVE_BRAKE_CUR, payload, 2) != HAL_OK)
    {
        Error_Handler();
    }
}

// Command: Set Digital Output (0x07)
// Payload: 8 bytes. Only the first byte is used to set digital outputs;
// the remaining bytes are filled with 0xFF (as "not used").
void sendSetDigitalOutput(uint8_t digitalOut)
{
    uint8_t payload[8];
    payload[0] = digitalOut;  // Lower 4 bits could represent outputs 1 to 4.
    for (int i = 1; i < 8; i++) {
        payload[i] = 0xFF;
    }
    if (CAN_SendCommand(CMD_SET_DIGITAL_OUTPUT, payload, 8) != HAL_OK)
    {
        Error_Handler();
    }
}

// Command: Set Maximum AC Current (0x08)
// Payload: 2 bytes, maximum AC current (multiplied by 10)
void sendSetMaxACCurrent(int16_t maxACCurrent)
{
    uint8_t payload[2];
    payload[0] = (uint8_t)((uint16_t)maxACCurrent >> 8);
    payload[1] = (uint8_t)(maxACCurrent & 0xFF);
    if (CAN_SendCommand(CMD_SET_MAX_AC_CURRENT, payload, 2) != HAL_OK)
    {
        Error_Handler();
    }
}

// Command: Set Maximum AC Brake Current (0x09)
// Payload: 2 bytes, maximum AC brake current (multiplied by 10; negative value expected)
void sendSetMaxBrakeACCurrent(int16_t maxBrakeACCurrent)
{
    uint8_t payload[2];
    payload[0] = (uint8_t)((uint16_t)maxBrakeACCurrent >> 8);
    payload[1] = (uint8_t)(maxBrakeACCurrent & 0xFF);
    if (CAN_SendCommand(CMD_SET_MAX_BRAKE_AC_CURRENT, payload, 2) != HAL_OK)
    {
        Error_Handler();
    }
}

// Command: Set Maximum DC Current (0x0A)
// Payload: 2 bytes, maximum DC current limit (multiplied by 10)
void sendSetMaxDCCurrent(int16_t maxDCCurrent)
{
    uint8_t payload[2];
    payload[0] = (uint8_t)((uint16_t)maxDCCurrent >> 8);
    payload[1] = (uint8_t)(maxDCCurrent & 0xFF);
    if (CAN_SendCommand(CMD_SET_MAX_DC_CURRENT, payload, 2) != HAL_OK)
    {
        Error_Handler();
    }
}

// Command: Set Maximum DC Brake Current (0x0B)
// Payload: 2 bytes, maximum DC brake current (multiplied by 10; negative value expected)
void sendSetMaxDCBrakeCurrent(int16_t maxDCBrakeCurrent)
{
    uint8_t payload[2];
    payload[0] = (uint8_t)((uint16_t)maxDCBrakeCurrent >> 8);
    payload[1] = (uint8_t)(maxDCBrakeCurrent & 0xFF);
    if (CAN_SendCommand(CMD_SET_MAX_DC_BRAKE_CURRENT, payload, 2) != HAL_OK)
    {
        Error_Handler();
    }
}

/* Command: Drive Enable (0x0C)
   As per Table 21, a 1-byte command where 1 enables drive and 0 disables it. */
void sendDriveEnable(uint8_t enable)
{
  uint8_t payload[1];
  payload[0] = enable;
  if (CAN_SendCommand(CMD_DRIVE_ENABLE, payload, 1) != HAL_OK)
  {
    Error_Handler();
  }
}


/*

#############  DATA FROM INVERTER ######################

*/

/**
  * @brief  Decode status message (Packet ID 0x20).
  *         Payload format:
  *           Bytes 0–3: ERPM (32‐bit signed, scale = 1)
  *           Bytes 4–5: Duty cycle (16‐bit signed, scale factor = 10)
  *           Bytes 6–7: Input Voltage (16‐bit signed, scale = 1)
  */
void decodeMessage0x20(uint8_t *data)
{
    int32_t erpm = ((int32_t)data[0] << 24) | ((int32_t)data[1] << 16) |
                   ((int32_t)data[2] << 8)  | data[3];
    int16_t dutyRaw = ((int16_t)data[4] << 8) | data[5];
    float duty = dutyRaw / 10.0f;
    int16_t inputVoltage = ((int16_t)data[6] << 8) | data[7];

    char buffer[100];
    sprintf(buffer, "0x20: ERPM = %ld RPM, Duty = %.1f%%, Input Voltage = %d V\r\n", erpm, duty, inputVoltage);
    HAL_UART_Transmit(&huart2, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);
}

/**
  * @brief  Decode AC and DC current message (Packet ID 0x21).
  *         Payload format:
  *           Bytes 0–1: AC current (16‐bit signed, scale factor = 10)
  *           Bytes 2–3: DC current (16‐bit signed, scale factor = 10)
  *           Bytes 4–7: Reserved (filled with 0xFF)
  */
void decodeMessage0x21(uint8_t *data)
{
    int16_t acCurrentRaw = ((int16_t)data[0] << 8) | data[1];
    int16_t dcCurrentRaw = ((int16_t)data[2] << 8) | data[3];
    float acCurrent = acCurrentRaw / 10.0f;
    float dcCurrent = dcCurrentRaw / 10.0f;

    char buffer[100];
    sprintf(buffer, "0x21: AC Current = %.1f A, DC Current = %.1f A\r\n",
            acCurrent, dcCurrent);
    HAL_UART_Transmit(&huart2, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);
}

/**
  * @brief  Decode temperature and fault code message (Packet ID 0x22).
  *         Payload format:
  *           Bytes 0–1: Controller Temperature (16‐bit signed, scale factor = 10)
  *           Bytes 2–3: Motor Temperature (16‐bit signed, scale factor = 10)
  *           Byte 4: Fault Code (unsigned, scale = 1)
  *           Bytes 5–7: Reserved (filled with 0xFF)
  */
void decodeMessage0x22(uint8_t *data)
{
    int16_t ctrlTempRaw = ((int16_t)data[0] << 8) | data[1];
    int16_t motorTempRaw = ((int16_t)data[2] << 8) | data[3];
    uint8_t faultCode = data[4];
    float ctrlTemp = ctrlTempRaw / 10.0f;
    float motorTemp = motorTempRaw / 10.0f;

    char buffer[100];
    sprintf(buffer, "0x22: Ctrl Temp = %.1f°C, Motor Temp = %.1f°C, Fault Code = 0x%02X\r\n",
            ctrlTemp, motorTemp, faultCode);
    HAL_UART_Transmit(&huart2, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);
}

/**
  * @brief  Decode FOC current components (Packet ID 0x23).
  *         Payload format:
  *           Bytes 0–3: Id (FOC algorithm component current, 32‐bit signed, scale factor = 100)
  *           Bytes 4–7: Iq (FOC algorithm component current, 32‐bit signed, scale factor = 100)
  */
void decodeMessage0x23(uint8_t *data)
{
    int32_t idRaw = ((int32_t)data[0] << 24) | ((int32_t)data[1] << 16) |
                    ((int32_t)data[2] << 8)  | data[3];
    int32_t iqRaw = ((int32_t)data[4] << 24) | ((int32_t)data[5] << 16) |
                    ((int32_t)data[6] << 8)  | data[7];
    float idValue = idRaw / 100.0f;
    float iqValue = iqRaw / 100.0f;

    char buffer[100];
    sprintf(buffer, "0x23: Id = %.2f, Iq = %.2f\r\n", idValue, iqValue);
    HAL_UART_Transmit(&huart2, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);
}

/**
  * @brief  Decode miscellaneous data message (Packet ID 0x24).
  *         Payload format (example interpretation):
  *           Byte 0: Throttle signal (int8_t, scale = 1, in %)
  *           Byte 1: Brake signal (int8_t, scale = 1, in %)
  *           Byte 2: Digital inputs (each bit represents an input)
  *           Byte 3: Digital outputs (each bit represents an output)
  *           Bytes 4–6: Various limit flags (each bit represents a limit status)
  *           Byte 7: CAN map version (unsigned)
  *
  *         (Depending on your integration needs, you may decode individual bits further.)
  */
void decodeMessage0x24(uint8_t *data)
{
    int8_t throttle = (int8_t)data[0];
    int8_t brake = (int8_t)data[1];
    uint8_t digitalInputs = data[2];   // Bits 0-3: digital inputs 1-4
    uint8_t digitalOutputs = data[3];  // Bits 0-3: digital outputs 1-4
    uint8_t canMapVersion = data[7];

    /* Extract individual digital input bits */
    uint8_t din1 = digitalInputs & 0x01;
    uint8_t din2 = (digitalInputs >> 1) & 0x01;
    uint8_t din3 = (digitalInputs >> 2) & 0x01;
    uint8_t din4 = (digitalInputs >> 3) & 0x01;

    /* Extract individual digital output bits */
    uint8_t dout1 = digitalOutputs & 0x01;
    uint8_t dout2 = (digitalOutputs >> 1) & 0x01;
    uint8_t dout3 = (digitalOutputs >> 2) & 0x01;
    uint8_t dout4 = (digitalOutputs >> 3) & 0x01;

    char buffer[150];
    sprintf(buffer,
            "0x24: Throttle = %d%%, Brake = %d%%, DIN = %d %d %d %d, DOUT = %d %d %d %d, CAN Map ver = %d\r\n",
            throttle, brake, din1, din2, din3, din4, dout1, dout2, dout3, dout4, canMapVersion);
    HAL_UART_Transmit(&huart2, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);
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
