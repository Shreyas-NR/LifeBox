/*
 * i2c.c
 *
 *  Created on: 11-Aug-2020
 *      Author: shreyasnr
 */

#include "I2C/i2c.h"

void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
	if(hi2c->Instance == I2C1)
	{
		if(TemperatureSensorOne == 1)
		{
			TempOneRegValue  = (AmbTempOneRegValue[0]<<8) | AmbTempOneRegValue[1];
			if (TempOneRegValue != 0xFFFF)
			{
				TemperatureOne = TempOneRegValue & 0x0FFF;
				TemperatureOne /= 16.0;
				if (TempOneRegValue & 0x1000)
				  TemperatureOne -= 256;
			}
		}
		else if(TemperatureSensorTwo == 1)
		{
			TempTwoRegValue  = (AmbTempTwoRegValue[0]<<8) | AmbTempTwoRegValue[1];
			if (TempTwoRegValue != 0xFFFF)
			{
				TemperatureTwo = TempTwoRegValue & 0x0FFF;
				TemperatureTwo /= 16.0;
				if (TempTwoRegValue & 0x1000)
					TemperatureTwo -= 256;
			}
		}
	}
	else if(hi2c->Instance == I2C2)
	{
		HeartTempRegValue = (ObjTempRegValue[1] <<8 | ObjTempRegValue[0]);

		HeartTemperature = HeartTempRegValue * 0.02;
		HeartTemperature -= 273.15;
	}
	else if(hi2c->Instance == I2C3)
	{

	}
}
