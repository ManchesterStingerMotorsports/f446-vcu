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


void can_setup(void)
{
    // Setup the lowest priority filter bank to allow all messages to pass
    CAN_FilterTypeDef sf;
    sf.FilterIdHigh = 0x0000;
    sf.FilterIdLow = 0x0000;
    sf.FilterMaskIdHigh = 0x0000;
    sf.FilterMaskIdLow = 0x0000;
    sf.FilterFIFOAssignment = CAN_FILTER_FIFO0;
    sf.FilterBank = 13;
    sf.FilterMode = CAN_FILTERMODE_IDMASK;
    sf.FilterScale = CAN_FILTERSCALE_32BIT;
    sf.FilterActivation = CAN_FILTER_ENABLE;
    sf.SlaveStartFilterBank = 14;

    if (HAL_CAN_ConfigFilter(&hcan1, &sf) != HAL_OK)
    {
        Error_Handler();
    }

    sf.FilterBank = 23;
    if (HAL_CAN_ConfigFilter(&hcan2, &sf) != HAL_OK)
    {
        Error_Handler();
    }

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
    sf.FilterFIFOAssignment = CAN_FILTER_FIFO1;
    sf.FilterBank = 1;
    if (HAL_CAN_ConfigFilter(&hcan1, &sf) != HAL_OK)
    {
        Error_Handler();
    }

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

    if (rxHeader->IDE == CAN_ID_STD) { id = rxHeader->StdId; }
    else                             { id = rxHeader->ExtId; }

    // Format ID section
    cursor += sprintf(cursor, "ID: 0x%03X, DATA:", (uint16_t)id);

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

    printfDma("FIFO0: %d, ", rxHeader.FilterMatchIndex);
    can_uartHexDump(&rxHeader, rxData); // Dumps the raw CAN message to UART

    if (hcan == &hcan1)
    {
        // TODO: Find a way to distinguish packet from inverter

        // If CAN packet is from inverter:
        uint32_t extId;
        uint8_t  packetId;
        uint8_t  nodeId;

        if (rxHeader.IDE == CAN_ID_STD)
        {
            extId    = rxHeader.StdId;
            packetId = (uint8_t)(extId >> 5);
            nodeId   = (uint8_t)(extId & 0x1F);
        }
        else
        {
            extId    = rxHeader.ExtId;
            packetId = (uint8_t)(extId >> 8);       // Packet ID is the high byte
            nodeId   = (uint8_t)(extId & 0xFF);     // Node ID is the low byte
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

    printfDma("FIFO1: %d, ", rxHeader.FilterMatchIndex);
    can_uartHexDump(&rxHeader, rxData); // Dumps the raw CAN message to UART

    if (hcan == &hcan1)
    {
        // TODO: Find a way to distinguish packet from inverter

        // If CAN packet is from inverter:
        uint32_t extId;
        uint8_t  packetId;
        uint8_t  nodeId;

        if (rxHeader.IDE == CAN_ID_STD)
        {
            extId    = rxHeader.StdId;
            packetId = (uint8_t)(extId >> 5);
            nodeId   = (uint8_t)(extId & 0x1F);
        }
        else
        {
            extId    = rxHeader.ExtId;
            packetId = (uint8_t)(extId >> 8);       // Packet ID is the high byte
            nodeId   = (uint8_t)(extId & 0xFF);     // Node ID is the low byte
        }

        inverter_decode(packetId, rxData);
    }

}

HAL_StatusTypeDef can_sendMsg(CAN_HandleTypeDef *hcan, uint32_t id, bool isExtId, uint8_t* data, uint8_t dlc)
{
    CAN_TxHeaderTypeDef txHeader;
    uint32_t txMailbox;             // Stores which Tx Mailbox is used

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

    if (HAL_CAN_GetTxMailboxesFreeLevel(hcan))
    {
        status = HAL_CAN_AddTxMessage(hcan, &txHeader, data, &txMailbox);

        if (status != HAL_OK)
        {
            // CAN peripheral tx error
            Error_Handler();
        }
    }
    else
    {
        status = HAL_BUSY;
    }

    return status;
}


HAL_StatusTypeDef can1_sendMsg(uint32_t id, bool isExtId, uint8_t* data, uint8_t dlc)
{
    return can_sendMsg(&hcan1, id, isExtId, data, dlc);
}

HAL_StatusTypeDef can2_sendMsg(uint32_t id, bool isExtId, uint8_t* data, uint8_t dlc)
{
    return can_sendMsg(&hcan2, id, isExtId, data, dlc);
}



