/*
 * vcu_can.h
 *
 *  Created on: Mar 1, 2025
 *      Author: amrlxyz
 */

#ifndef INC_VCU_CAN_H_
#define INC_VCU_CAN_H_

#include "main.h"

void can_setup(void);

void can_uartHexDump(CAN_RxHeaderTypeDef *rxHeader, uint8_t rxData[static 1]);

//void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan);


HAL_StatusTypeDef can1_sendMsg(uint32_t id, bool isExtId, uint8_t* data, uint8_t dlc);

HAL_StatusTypeDef can2_sendMsg(uint32_t id, bool isExtId, uint8_t* data, uint8_t dlc);

#endif /* INC_VCU_CAN_H_ */
