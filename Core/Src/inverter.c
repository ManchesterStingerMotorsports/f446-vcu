
#include "inverter.h"
#include "main.h"
#include "vcu_can.h"

#include <string.h>
#include <stdio.h>

InverterData invrtr;


// SCALE = 10 except for ERPM and DriveEnable which is 1 #######################
#define USE_EXT_ID                   true
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

// CAN_HandleTypeDef hcan1;
// UART_HandleTypeDef huart2;

// CAN_TxHeaderTypeDef TxHeader;
// CAN_RxHeaderTypeDef RxHeader;

// uint8_t TxData[8];
// uint8_t RxData[8];

// uint32_t TxMailbox;

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

  // char msg[] = "CAN Started\r\n";
  // HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);

//	sendDriveEnable(1);
//	sendSetACCurrent(100);  // e.g., 10 A -> 100 (scaled)
//	sendSetERPM(500); // 500 RPM

	// sendSetMaxBrakeACCurrent(-1000);
	// HAL_Delay(500);



void decodeMessage0x20(uint8_t *data);
void decodeMessage0x21(uint8_t *data);
void decodeMessage0x22(uint8_t *data);
void decodeMessage0x23(uint8_t *data);
void decodeMessage0x24(uint8_t *data);


void inverter_decode(uint32_t packetId, uint8_t rxData[static 8])
{
    switch(packetId)
    {
    case 0x20:
        decodeMessage0x20(rxData);
        break;
    case 0x21:
        decodeMessage0x21(rxData);
        break;
    case 0x22:
        decodeMessage0x22(rxData);
        break;
    case 0x23:
        decodeMessage0x23(rxData);
        break;
    case 0x24:
        decodeMessage0x24(rxData);
        break;
    default:
        // Handle unknown messages as needed.
        break;
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
    if (USE_EXT_ID)
    {
        uint32_t id = ((uint32_t)cmdID << 8) | INVERTER_NODE_ID;
        return can1_sendMsg(id, true, data, dlc);
    }
    else
    {
        uint32_t id = ((uint32_t)cmdID << 5) | INVERTER_NODE_ID;
        return can1_sendMsg(id, false, data, dlc);
    }
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
    invrtr.erpm = ((int32_t)data[0] << 24) | ((int32_t)data[1] << 16) |
                        ((int32_t)data[2] << 8)  | data[3];
    invrtr.dutyRaw = ((int16_t)data[4] << 8) | data[5];
    invrtr.duty = invrtr.dutyRaw / 10.0f;
    invrtr.inputVoltage = ((int16_t)data[6] << 8) | data[7];

//    char buffer[100];
//    sprintf(buffer, "0x20: ERPM = %ld RPM, Duty = %.1f%%, Input Voltage = %d V\r\n", erpm, duty, inputVoltage);
//    HAL_UART_Transmit(&huart2, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);
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
    invrtr.acCurrentRaw = ((int16_t)data[0] << 8) | data[1];
    invrtr.dcCurrentRaw = ((int16_t)data[2] << 8) | data[3];
    invrtr.acCurrent = invrtr.acCurrentRaw / 10.0f;
    invrtr.dcCurrent = invrtr.dcCurrentRaw / 10.0f;

//    char buffer[100];
//    sprintf(buffer, "0x21: AC Current = %.1f A, DC Current = %.1f A\r\n",
//            acCurrent, dcCurrent);
//    HAL_UART_Transmit(&huart2, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);
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
    invrtr.ctrlTempRaw = ((int16_t)data[0] << 8) | data[1];
    invrtr.motorTempRaw = ((int16_t)data[2] << 8) | data[3];
    invrtr.faultCode = data[4];
    invrtr.ctrlTemp = invrtr.ctrlTempRaw / 10.0f;
    invrtr.motorTemp = invrtr.motorTempRaw / 10.0f;

//    char buffer[100];
//    sprintf(buffer, "0x22: Ctrl Temp = %.1f°C, Motor Temp = %.1f°C, Fault Code = 0x%02X\r\n",
//            ctrlTemp, motorTemp, faultCode);
//    HAL_UART_Transmit(&huart2, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);
}

/**
  * @brief  Decode FOC current components (Packet ID 0x23).
  *         Payload format:
  *           Bytes 0–3: Id (FOC algorithm component current, 32‐bit signed, scale factor = 100)
  *           Bytes 4–7: Iq (FOC algorithm component current, 32‐bit signed, scale factor = 100)
  */
void decodeMessage0x23(uint8_t *data)
{
    invrtr.idRaw = ((int32_t)data[0] << 24) | ((int32_t)data[1] << 16) |
                         ((int32_t)data[2] << 8)  | data[3];
    invrtr.iqRaw = ((int32_t)data[4] << 24) | ((int32_t)data[5] << 16) |
                         ((int32_t)data[6] << 8)  | data[7];
    invrtr.idValue = invrtr.idRaw / 100.0f;
    invrtr.iqValue = invrtr.iqRaw / 100.0f;

//    char buffer[100];
//    sprintf(buffer, "0x23: Id = %.2f, Iq = %.2f\r\n", idValue, iqValue);
//    HAL_UART_Transmit(&huart2, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);
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
    invrtr.throttle = (int8_t)data[0];
    invrtr.brake = (int8_t)data[1];
    uint8_t digitalInputs = data[2];   // Bits 0-3: digital inputs 1-4
    uint8_t digitalOutputs = data[3];  // Bits 0-3: digital outputs 1-4
    invrtr.canMapVersion = data[7];

    /* Extract individual digital input bits */
    invrtr.din1 = digitalInputs & 0x01;
    invrtr.din2 = (digitalInputs >> 1) & 0x01;
    invrtr.din3 = (digitalInputs >> 2) & 0x01;
    invrtr.din4 = (digitalInputs >> 3) & 0x01;

    /* Extract individual digital output bits */
    invrtr.dout1 = digitalOutputs & 0x01;
    invrtr.dout2 = (digitalOutputs >> 1) & 0x01;
    invrtr.dout3 = (digitalOutputs >> 2) & 0x01;
    invrtr.dout4 = (digitalOutputs >> 3) & 0x01;

//    char buffer[150];
//    sprintf(buffer,
//            "0x24: Throttle = %d%%, Brake = %d%%, DIN = %d %d %d %d, DOUT = %d %d %d %d, CAN Map ver = %d\r\n",
//            throttle, brake, din1, din2, din3, din4, dout1, dout2, dout3, dout4, canMapVersion);
//    HAL_UART_Transmit(&huart2, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);
}





