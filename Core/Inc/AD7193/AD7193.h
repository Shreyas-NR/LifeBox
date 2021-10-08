/*
 * AD7193.h
 *
 *  Created on: Feb 8, 2021
 *      Author: shreyasnr
 */

#ifndef SRC_AD7193_AD7193_H_
#define SRC_AD7193_AD7193_H_


#include "main.h"


//#define AD7193_RDY_STATE  MISO   // pin to watch for data ready state

/* AD7193 Register Map */
#define AD7193_REG_COMM         0 // Communications Register (WO, 8-bit)
#define AD7193_REG_STAT         0 // Status Register         (RO, 8-bit)
#define AD7193_REG_MODE         1 // Mode Register           (RW, 24-bit
#define AD7193_REG_CONF         2 // Configuration Register  (RW, 24-bit)
#define AD7193_REG_DATA         3 // Data Register           (RO, 24/32-bit)
#define AD7193_REG_ID           4 // ID Register             (RO, 8-bit)
#define AD7193_REG_GPOCON       5 // GPOCON Register         (RW, 8-bit)
#define AD7193_REG_OFFSET       6 // Offset Register         (RW, 24-bit
#define AD7193_REG_FULLSCALE    7 // Full-Scale Register     (RW, 24-bit)

/* Communications Register Bit Designations (AD7193_REG_COMM) */
#define AD7193_COMM_WEN         (1 << 7)           // Write Enable.
#define AD7193_COMM_WRITE       (0 << 6)           // Write Operation.
#define AD7193_COMM_READ        (1 << 6)           // Read Operation.
#define AD7193_COMM_ADDR(x)     (((x) & 0x7) << 3) // Register Address.
#define AD7193_COMM_CREAD       (1 << 2)           // Continuous Read of Data Register.







	void AD7193Reset(void);
	void AD7193Calibrate(void);

	void AD7193SetPGAGain(int gain);
	void AD7193SetAveraging(int filterRate);
	void AD7193SetChannel(int channel);

	void AD7193SetPsuedoDifferentialInputs(void);
	void AD7193AppendStatusValuetoData(void);

	unsigned long AD7193ReadADCChannel(int channel);
	unsigned long AD7193ReadADCData(void);
	void AD7193IntitiateSingleConversion(void);
	void AD7193WaitForADC(void);


	float AD7193DataToVoltage(long rawData);
	float AD7193TempSensorDataToDegC(unsigned long rawData);


	unsigned long AD7193GetRegisterValue(unsigned char registerAddress,
                                       unsigned char bytesNumber,
                                       unsigned char modifyCS);
	void AD7193SetRegisterValue(unsigned char registerAddress,
                              unsigned long registerValue,
                              unsigned char bytesNumber,
                              unsigned char modifyCS);


	void AD7193ReadRegisterMap(void);

	//void SetChannelSelect(unsigned long wordValue);
	//void ChannelEnable(int channel);

	//void SingleConversionAndReadADC(long unsigned int *ADCDataByChannel);  // this needs to be a pointer
	//void DisplayADCData(long unsigned int ADCDataByChannel[]);
	//void WriteAllRegisters(void);

	typedef struct{
	int16_t InputMin;
	int16_t InputMax;
	int16_t OutputMin;
	int16_t OutputMax;
	} Pressure_t;

	float CalculatePressure(Pressure_t* PressureVal, float PressureSensorInput);
#endif /* SRC_AD7193_AD7193_H_ */
