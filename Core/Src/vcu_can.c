/*
 * vcu_can.c
 *
 *  Created on: Mar 1, 2025
 *      Author: amrlxyz
 */

#include "main.h"
#include "vcu_can.h"
#include "uartDMA.h"
#include "inverter.h"

#include <stdio.h>
#include <string.h>


void can_setup(void)
{
    const uint8_t can1FilterBankAmount = 14; // The amount of master can filter bank allocated
    CAN_FilterTypeDef sf;

    /* --- DEFAULT CAN FILTER CONFIGURATION --- */

    /*
     * The current config defaults messages that dont match any filter
     * goes to FIFO1. Message that matches a filter goes to FIFO0
     */

    // Setup all bank to allow all messages to pass by default to FIFO1
    sf.FilterIdHigh = 0x0000;
    sf.FilterIdLow = 0x0000;
    sf.FilterMaskIdHigh = 0x0000;
    sf.FilterMaskIdLow = 0x0000;
    sf.FilterFIFOAssignment = CAN_FILTER_FIFO1;
    sf.FilterMode = CAN_FILTERMODE_IDMASK;
    sf.FilterScale = CAN_FILTERSCALE_32BIT;
    sf.FilterActivation = CAN_FILTER_ENABLE;
    sf.SlaveStartFilterBank = can1FilterBankAmount;

    for (int i = 0; i < can1FilterBankAmount; i++)
    {
        sf.FilterBank = i;
        if (HAL_CAN_ConfigFilter(&hcan1, &sf) != HAL_OK)
        {
            Error_Handler();
        }
    }

    // Technically hcan2 here is unnecessary as both share the same can config filters
    for (int i = can1FilterBankAmount; i < 27; i++)
    {
        sf.FilterBank = i;
        if (HAL_CAN_ConfigFilter(&hcan2, &sf) != HAL_OK)
        {
            Error_Handler();
        }
    }


    /* --- SPECIFIC CAN FILTER CONFIGURATION --- */

    // Config filter for Inverter packet IDs (0x20 to 0x24)
    // Filter is set to filter (0x2000 to 0x27FF) ext ID to FIFO1
    uint32_t idFilter   = 0x00002000;
    uint32_t idMask     = 0xFFFFFFFF ^ (0x07FF);      // Dont care's for inverter IDs and the 5 packet Ids
    sf.FilterIdHigh     = (idFilter >> 13) & 0xFFFF;  // 29 - 16 = 13 bits (the higher bits of the filter)
    sf.FilterIdLow      = (idFilter << 3 ) & 0xFFF8;  // 3 bits for {IDE}, {RTR}, {0}
    sf.FilterMaskIdHigh = (idMask   >> 13) & 0xFFFF;
    sf.FilterMaskIdLow  = (idMask   << 3 ) & 0xFFF8;
    sf.FilterFIFOAssignment = CAN_FILTER_FIFO0;
    sf.FilterBank = 0;                              // Lower the number higher the priority
    sf.FilterMode = CAN_FILTERMODE_IDMASK;
    sf.FilterScale = CAN_FILTERSCALE_32BIT;
    sf.FilterActivation = CAN_FILTER_ENABLE;
    sf.SlaveStartFilterBank = 14;
    if (HAL_CAN_ConfigFilter(&hcan1, &sf) != HAL_OK)
    {
        Error_Handler();
    }

    // Config filter for other stuff
    idFilter   = 0x0000FF00;
    idMask     = 0xFFFFFFFF;
    sf.FilterIdHigh     = (idFilter >> 13) & 0xFFFF;  // 29 - 16 = 13 bits (the higher bits of the filter)
    sf.FilterIdLow      = (idFilter << 3 ) & 0xFFF8;  // 3 bits for {IDE}, {RTR}, {0}
    sf.FilterMaskIdHigh = (idMask   >> 13) & 0xFFFF;
    sf.FilterMaskIdLow  = (idMask   << 3 ) & 0xFFF8;
    sf.FilterFIFOAssignment = CAN_FILTER_FIFO0;
    sf.FilterBank = 1;
    if (HAL_CAN_ConfigFilter(&hcan1, &sf) != HAL_OK)
    {
        Error_Handler();
    }


    /* --- CAN FUNCTIONALITY START --- */

    // Start both CAN peripheral
    if (HAL_CAN_Start(&hcan1) != HAL_OK)
    {
        Error_Handler();
    }
    if (HAL_CAN_Start(&hcan2) != HAL_OK)
    {
        Error_Handler();
    }

    // Enable Interrupt callback when a CAN message is recieved
    if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
    {
        Error_Handler();
    }
    if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO1_MSG_PENDING) != HAL_OK)
    {
        Error_Handler();
    }
    if (HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
    {
        Error_Handler();
    }
    if (HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO1_MSG_PENDING) != HAL_OK)
    {
        Error_Handler();
    }
}


void can_uartHexDump(CAN_RxHeaderTypeDef *rxHeader, uint8_t rxData[static 1])
{
    char txBuff[128];
    char *cursor = txBuff;
    uint32_t id;

    // Format ID section
    if (rxHeader->IDE == CAN_ID_STD)
    {
        id = rxHeader->StdId;
        cursor += sprintf(cursor, "ID: 0x%04lX, DATA:", id);
    }
    else
    {
        id = rxHeader->ExtId;
        cursor += sprintf(cursor, "ID: 0x%08lX, DATA:", id);
    }

    // Format data bytes with spaces between them
    for (int i = 0; i < rxHeader->DLC; i++)
    {
        cursor += sprintf(cursor, " %02X", rxData[i]);
    }

    // Add newline and calculate total length
    cursor += sprintf(cursor, "\n");

    // Send formatted message
    printfDma("%s", txBuff);
}


// ISR Callback to process recieved can messages:
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef rxHeader;
    uint8_t rxData[8];

    // Get CAN message
    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rxHeader, rxData) != HAL_OK)
    {
        Error_Handler();
    }

//    printfDma("FIFO0: %d, ", rxHeader.FilterMatchIndex);
//    can_uartHexDump(&rxHeader, rxData); // Dumps the raw CAN message to UART

    if (rxHeader.FilterMatchIndex == 0) // If filter matches inverter CAN IDs
    {
        uint32_t extId;
        uint8_t  packetId;
//        uint8_t  nodeId;

        if (rxHeader.IDE == CAN_ID_STD)
        {
            extId    = rxHeader.StdId;
            packetId = (uint8_t)(extId >> 5);
//            nodeId   = (uint8_t)(extId & 0x1F);
        }
        else
        {
            extId    = rxHeader.ExtId;
            packetId = (uint8_t)(extId >> 8);       // Packet ID is the high byte
//            nodeId   = (uint8_t)(extId & 0xFF);     // Node ID is the low byte
        }

        inverter_decode(packetId, rxData);
    }
}


// ISR Callback to process recieved can messages:
void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef rxHeader;
    uint8_t rxData[8];

    // Get CAN message
    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO1, &rxHeader, rxData) != HAL_OK)
    {
        Error_Handler();
    }

//    printfDma("FIFO1: %d, ", rxHeader.FilterMatchIndex);
//    can_uartHexDump(&rxHeader, rxData); // Dumps the raw CAN message to UART

    // Data packet that dont match any filter goes here
    if (hcan == &hcan1)
    {

    }
}


HAL_StatusTypeDef can_sendMsg(CAN_HandleTypeDef *hcan, uint32_t id, bool isExtId, uint8_t* data, uint8_t dlc)
{
    CAN_TxHeaderTypeDef txHeader;
//    uint32_t txMailbox;             // Stores which Tx Mailbox is used

    txHeader.RTR = CAN_RTR_DATA;
    txHeader.DLC = dlc;

    if (isExtId)
    {
        txHeader.IDE    = CAN_ID_EXT;
        txHeader.ExtId  = id;
    }
    else
    {
        txHeader.IDE    = CAN_ID_STD;
        txHeader.StdId  = id;
    }

    HAL_StatusTypeDef status;

    CanTxMsg canMsg;
    canMsg.header = txHeader;
    memcpy(canMsg.data, data, txHeader.DLC);


    if (hcan == &hcan1)
    {
        if (osMessageQueuePut(q_can1TxHandle, &canMsg, 0, 0) != osOK)
        {
            status = HAL_BUSY;
        }
    }
    else if (hcan == &hcan2)
    {
        if (osMessageQueuePut(q_can2TxHandle, &canMsg, 0, 0) != osOK)
        {
            status = HAL_BUSY;
        }
    }
    else
    {
        // Invalid can peripheral
        Error_Handler();
    }


    return status;
}


HAL_StatusTypeDef can1_sendMsg(uint32_t id, bool isExtId, uint8_t* data, uint8_t dlc)
{
    HAL_StatusTypeDef status = can_sendMsg(&hcan1, id, isExtId, data, dlc);

    if (status == HAL_BUSY) // If txMailbox full
    {
        // TODO: TX Full Handling
//        printfDma("ERR: TxMailbox1 Full (Data Loss) \n");
    }

    return status;
}

HAL_StatusTypeDef can2_sendMsg(uint32_t id, bool isExtId, uint8_t* data, uint8_t dlc)
{
    HAL_StatusTypeDef status = can_sendMsg(&hcan2, id, isExtId, data, dlc);

    if (status == HAL_BUSY) // If txMailbox full
    {
        // TODO: TX Full Handling
//        printfDma("ERR: TxMailbox2 Full (Data Loss) \n");
    }

    return status;
}


void t_can_func(void *argument)
{
    CanTxMsg canMsg;

    for(;;)
    {

        uint32_t mb1;                // Stores last Tx Mailbox used
        uint32_t mb2;                // Stores last Tx Mailbox used

        while (osMessageQueueGetCount(q_can1TxHandle) && HAL_CAN_GetTxMailboxesFreeLevel(&hcan1))
        {
            if (osMessageQueueGet(q_can1TxHandle, &canMsg, NULL, 0) == osOK)
            {
                if (HAL_CAN_AddTxMessage(&hcan1, &canMsg.header, canMsg.data, &mb1) != HAL_OK)
                {
                    // CAN peripheral tx error
                    Error_Handler();
                }
            }
            else
            {
                // Queue Empty
                Error_Handler();
            }
        }

        while (osMessageQueueGetCount(q_can2TxHandle) && HAL_CAN_GetTxMailboxesFreeLevel(&hcan2))
        {
            if (osMessageQueueGet(q_can2TxHandle, &canMsg, NULL, 0) == osOK)
            {
                if (HAL_CAN_AddTxMessage(&hcan2, &canMsg.header, canMsg.data, &mb2) != HAL_OK)
                {
                    // CAN peripheral tx error
                    Error_Handler();
                }
            }
            else
            {
                // Queue Empty
                Error_Handler();
            }
        }

        osDelay(2);
    }
}



