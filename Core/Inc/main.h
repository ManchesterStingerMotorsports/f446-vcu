/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdbool.h>
#include "cmsis_os.h"

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;
extern osMessageQueueId_t q_can1TxHandle;
extern osMessageQueueId_t q_can2TxHandle;
extern osThreadId_t t_canHandle;

extern UART_HandleTypeDef huart2;
extern osMessageQueueId_t q_printfHandle;
extern osThreadId_t t_uartHandle;

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define DEBUG_LED_Pin GPIO_PIN_13
#define DEBUG_LED_GPIO_Port GPIOC
#define USART2_TX_Pin GPIO_PIN_2
#define USART2_TX_GPIO_Port GPIOA
#define USART2_RX_Pin GPIO_PIN_3
#define USART2_RX_GPIO_Port GPIOA
#define ADC_BPS_Pin GPIO_PIN_4
#define ADC_BPS_GPIO_Port GPIOA
#define ADC_APPS2_Pin GPIO_PIN_5
#define ADC_APPS2_GPIO_Port GPIOA
#define ADC_APPS1_Pin GPIO_PIN_6
#define ADC_APPS1_GPIO_Port GPIOA
#define IT_TS_BUTTON_Pin GPIO_PIN_7
#define IT_TS_BUTTON_GPIO_Port GPIOA
#define IT_TS_BUTTON_EXTI_IRQn EXTI9_5_IRQn
#define IT_R2D_BUTTON_Pin GPIO_PIN_4
#define IT_R2D_BUTTON_GPIO_Port GPIOC
#define IT_R2D_BUTTON_EXTI_IRQn EXTI4_IRQn
#define DO_R2D_SOUND_Pin GPIO_PIN_0
#define DO_R2D_SOUND_GPIO_Port GPIOB
#define DO_R2D_LIGHT_Pin GPIO_PIN_1
#define DO_R2D_LIGHT_GPIO_Port GPIOB
#define DO_SC_LIGHT_Pin GPIO_PIN_2
#define DO_SC_LIGHT_GPIO_Port GPIOB
#define SD_CARD_DETECT_Pin GPIO_PIN_7
#define SD_CARD_DETECT_GPIO_Port GPIOC
#define SWDIO_Pin GPIO_PIN_13
#define SWDIO_GPIO_Port GPIOA
#define SWCLK_Pin GPIO_PIN_14
#define SWCLK_GPIO_Port GPIOA
#define SWO_Pin GPIO_PIN_3
#define SWO_GPIO_Port GPIOB
#define SC_IN_Pin GPIO_PIN_6
#define SC_IN_GPIO_Port GPIOB
#define DO_SC_RELAY_Pin GPIO_PIN_7
#define DO_SC_RELAY_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
