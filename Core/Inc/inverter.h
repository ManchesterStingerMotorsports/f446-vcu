






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