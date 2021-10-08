/*
 * PressureSensor.c
 *
 *  Created on: Feb 11, 2021
 *      Author: shreyasnr
 */
#include "Pressure Sensor/PressureSensor.h"

Pressure_t PressureLimits;
float ReadPressureSensor()
{
	unsigned long ch1Data;
	float ch1Voltage, Pressure;
	// Read channel measurement data
	ch1Data = (AD7193ReadADCChannel(1) >> 8);

	// Convert to voltage
	ch1Voltage = AD7193DataToVoltage(ch1Data);
	ch1Voltage = ch1Voltage * -1;
	ch1Voltage = ch1Voltage * 1000000;

	Pressure = CalculatePressure(&PressureLimits,ch1Voltage);
	#if DEBUG_PERFUSION
	memset(usr_msg1, 0, sizeof(usr_msg1));
	sprintf(usr_msg1, "\n\t\tChannel 1 Data: %lX", ch1Data);
	SendToUSB((uint8_t*)usr_msg1);
	memset(usr_msg1, 0, sizeof(usr_msg1));
	sprintf(usr_msg1, "\n\t\tChannel 1 Voltage Measurement: %f", ch1Voltage);
		SendToUSB((uint8_t*)usr_msg1);
	memset(usr_msg1, 0, sizeof(usr_msg1));
	sprintf(usr_msg1, "\n\t\tChannel 1 Pressure Measurement: %f mmHg", Pressure);
		SendToUSB((uint8_t*)usr_msg1);
	#endif
//		HAL_Delay(500);
		if(Pressure < 0)
			Pressure = 0;
	return Pressure;
}
