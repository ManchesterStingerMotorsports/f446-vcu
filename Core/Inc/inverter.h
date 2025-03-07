

#ifndef INC_INVERTER_H_
#define INC_INVERTER_H_

#include <stdint.h>

typedef enum {
    NO_FAULTS               = 0x00, // No faults detected.
    OVERVOLTAGE             = 0x01, // Overvoltage - The input voltage is higher than the set maximum.
    UNDERVOLTAGE            = 0x02, // Undervoltage - The input voltage is lower than the set minimum.
    DRV_FAULT               = 0x03, // DRV - Transistor or transistor drive error.
    ABS_OVERCURRENT         = 0x04, // ABS. Overcurrent - The AC current is higher than the set absolute maximum current.
    CTLR_OVERTEMP           = 0x05, // CTLR Overtemp. - The controller temperature is higher than the set maximum.
    MOTOR_OVERTEMP          = 0x06, // Motor Overtemp. - The motor temperature is higher than the set maximum.
    SENSOR_WIRE_FAULT       = 0x07, // Sensor wire fault - Something went wrong with the sensor differential signals.
    SENSOR_GENERAL_FAULT    = 0x08, // Sensor general fault - An error occurred while processing the sensor signals.
    CAN_COMMAND_ERROR       = 0x09, // CAN Command error - CAN message received contains parameter out of boundaries.
    ANALOG_INPUT_ERROR      = 0x0A  // Analog input error – Redundant output out of range.
} FaultCode;


typedef struct
{
    // --- ID: 0x20 --- //
    int32_t erpm;
    int16_t dutyRaw;            // Scale: 10
    float duty;
    int16_t inputVoltage;

    // --- ID: 0x21 --- //
    int16_t acCurrentRaw;       // Scale: 10
    int16_t dcCurrentRaw;       // Scale: 10
    float acCurrent;
    float dcCurrent;

    // --- ID: 0x22 --- //
    int16_t ctrlTempRaw;        // Scale: 10, Celcius
    int16_t motorTempRaw;       // Scale: 10, Celcius
    FaultCode faultCode;
    float ctrlTemp;
    float motorTemp;

    // --- ID: 0x23 --- //
    int32_t idRaw;              // Scale: 100
    int32_t iqRaw;              // Scale: 100
    float idValue;
    float iqValue;

    // --- ID: 0x24 --- //
    int8_t throttle;
    int8_t brake;
    uint8_t canMapVersion;

    uint8_t din1;
    uint8_t din2;
    uint8_t din3;
    uint8_t din4;

    uint8_t dout1;
    uint8_t dout2;
    uint8_t dout3;
    uint8_t dout4;

    // --- Command Variables --- //
    int32_t setErpm;

} InverterData;

extern InverterData invrtr;





// COMMANDS TO INVERTER
void inverter_setACCurrent(int16_t acCurrent);
void inverter_setBrakeCurrent(int16_t brakeCurrent);
void inverter_setERPM(int32_t erpm);
void inverter_setPosition(int16_t position);
void inverter_setRelativeCurrent(int16_t relativeCurrent);
void inverter_setRelativeBrakeCurrent(int16_t relativeBrakeCurrent);
void inverter_setDigitalOutput(uint8_t digitalOut);
void inverter_setMaxACCurrent(int16_t maxACCurrent);
void inverter_setMaxBrakeACCurrent(int16_t maxBrakeACCurrent);
void inverter_setMaxDCCurrent(int16_t maxDCCurrent);
void inverter_setMaxDCBrakeCurrent(int16_t maxDCBrakeCurrent);
void inverter_setDriveEnable(uint8_t enable);

// DATA FROM INVERTER
void inverter_decode(uint32_t packetId, uint8_t rxData[static 8]);



#endif /* INC_INVERTER_H_ */
