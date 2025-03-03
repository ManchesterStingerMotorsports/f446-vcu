/*
 * uartDMA.c
 *
 *  Created on: Jan 25, 2025
 *      Author: amrlxyz
 */

#include "uartDMA.h"

#include "stdio.h"
#include "stdarg.h"
#include "stdbool.h"

#include "main.h"
//#include "cmsis_os.h"


#define uartHandle huart2

#define FLAG_UART_INACTIVE 0x0001
#define FLAG_TASK_ACTIVE   0x0002


void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    osThreadFlagsSet(t_uartHandle, FLAG_UART_INACTIVE);
}


int printfDma(const char *format, ...)
{
    // Check if queue is full
    if (!osMessageQueueGetSpace(q_printfHandle))
    {
        Error_Handler();
    }

    printfString_t prtStr;

    va_list args;
    va_start(args, format);
    prtStr.len = vsnprintf(prtStr.str, BUFF_SIZE, format, args);
    va_end(args);

    if (prtStr.len < 0) // Error in formatting
    {
        return -1;
    }
    else if (prtStr.len > BUFF_SIZE) // Over limit of buffer size
    {
        Error_Handler();
    }

    // Add buffer to queue
    if (osMessageQueuePut(q_printfHandle, &prtStr, 0, 0) != osOK)
    {
        Error_Handler();
    }

    // Notify using flag to start transmit if not already
    osThreadFlagsSet(t_uartHandle, FLAG_TASK_ACTIVE);

    return prtStr.len;
}


void t_uart_func(void *argument)
{
    printfString_t prtStr;

    for(;;)
    {
        osThreadFlagsWait(FLAG_TASK_ACTIVE, osFlagsWaitAny, osWaitForever);

        while (osMessageQueueGetCount(q_printfHandle))
        {
            if (osMessageQueueGet(q_printfHandle, &prtStr, NULL, 0) == osOK)
            {
                if (HAL_UART_Transmit_DMA(&uartHandle, (uint8_t*)prtStr.str, prtStr.len) != HAL_OK)
                {
                    // UART tx error
                    Error_Handler();
                }
            }
            else
            {
                // Queue Empty
                Error_Handler();
            }

            // Wait until UART tx is done
            osThreadFlagsWait(FLAG_UART_INACTIVE, osFlagsWaitAny, osWaitForever);
        }

        // Clear the flag if it was set during uart transmission,
        // so it can detect new printf requests
        osThreadFlagsClear(FLAG_TASK_ACTIVE);
    }
}



