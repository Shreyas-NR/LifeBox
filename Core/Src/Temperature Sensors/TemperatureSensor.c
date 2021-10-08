/*
 * TemperatureSensor.c
 *
 *  Created on: 07-Aug-2020
 *      Author: shreyasnr
 */


#include "Temperature Sensors/TemperatureSensor.h"
#include "I2C/i2c.h"

uint8_t TemperatureSensorOne = 0;
uint8_t TemperatureSensorTwo = 0;
uint8_t AmbTempOneRegValue[2] = {0,0};
uint8_t AmbTempTwoRegValue[2] = {0,0};
uint8_t ObjTempRegValue[2] = {0,0};
uint16_t TempOneRegValue = 0;
uint16_t TempTwoRegValue = 0;
uint16_t HeartTempRegValue = 0;
float TemperatureOne = 0;
float TemperatureTwo = 0;
float HeartTemperature = 0;
float TemperatureDifference = 0;
uint32_t iTemperatureDifference = 0;
uint32_t aTemperatureDifference[200];
uint32_t TemperauteDifferenceAvg = 0;
char usr_msg[50];

void SetTempSensorResolution(uint8_t SensorNum, uint8_t ResolutionBits)
{
	ResolutionBits = ResolutionBits & 0x03;
	switch(SensorNum)
	{
		case 1:
			if(HAL_I2C_Mem_Write_IT(&hi2c1, MCP9808_I2CADDR_1, RESOLUTION_REG_ADDR, (sizeof(RESOLUTION_REG_ADDR)/sizeof(RESOLUTION_REG_ADDR)), &ResolutionBits, sizeof(ResolutionBits))!= HAL_OK)
			{
			  Error_Handler();
			}
			break;
		case 2:
			if(HAL_I2C_Mem_Write_IT(&hi2c1, MCP9808_I2CADDR_2, RESOLUTION_REG_ADDR, (sizeof(RESOLUTION_REG_ADDR)/sizeof(RESOLUTION_REG_ADDR)), &ResolutionBits, sizeof(ResolutionBits))!= HAL_OK)
			{
			  Error_Handler();
			}
			break;
	}
}
void ReadTemperatureSensorOne()
{
	TemperatureSensorOne = 1;
	TemperatureSensorTwo = 0;
	if(HAL_I2C_Mem_Read_IT(&hi2c1, MCP9808_I2CADDR_1, AMBIENT_TEMP_REG_ADDR, (sizeof(AMBIENT_TEMP_REG_ADDR)/sizeof(AMBIENT_TEMP_REG_ADDR)), AmbTempOneRegValue, sizeof(AmbTempOneRegValue))!= HAL_OK)
	{
	  Error_Handler();
	}
	HAL_Delay(65);
//	TempOneRegValue  = (AmbTempOneRegValue[0]<<8) | AmbTempOneRegValue[1];
//	if (TempOneRegValue != 0xFFFF)
//	{
//		TemperatureOne = TempOneRegValue & 0x0FFF;
//		TemperatureOne /= 16.0;
//		if (TempOneRegValue & 0x1000)
//		  TemperatureOne -= 256;
//	}
//	sprintf(usr_msg, "\n\tTemperature Sensor One Value =  %f *C\n\r", (TemperatureOne));
//	if(HAL_UART_Transmit(&huart2, (uint8_t*)usr_msg, sizeof(usr_msg), HAL_MAX_DELAY)!= HAL_OK)
//	{
//		Error_Handler();
//	}
	if(POWER_ON_SELF_TEST == 1)
		MenuDisplay();
}
void ReadTemperatureSensorTwo()
{
	TemperatureSensorOne = 0;
	TemperatureSensorTwo = 1;
	if(HAL_I2C_Mem_Read_IT(&hi2c1, MCP9808_I2CADDR_2, AMBIENT_TEMP_REG_ADDR, (sizeof(AMBIENT_TEMP_REG_ADDR)/sizeof(AMBIENT_TEMP_REG_ADDR)), AmbTempTwoRegValue, sizeof(AmbTempTwoRegValue))!= HAL_OK)
	{
	  Error_Handler();
	}
	HAL_Delay(65);
//	TempTwoRegValue  = (AmbTempTwoRegValue[0]<<8) | AmbTempTwoRegValue[1];
//	if (TempTwoRegValue != 0xFFFF)
//	{
//		TemperatureTwo = TempTwoRegValue & 0x0FFF;
//		TemperatureTwo /= 16.0;
//		if (TempTwoRegValue & 0x1000)
//			TemperatureTwo -= 256;
//	}
//	sprintf(usr_msg, "\n\tTemperature Sensor Two Value =  %f *C\n\r", (TemperatureTwo));
//	if(HAL_UART_Transmit(&huart2, (uint8_t*)usr_msg, sizeof(usr_msg), HAL_MAX_DELAY)!= HAL_OK)
//	{
//		Error_Handler();
//	}
	if(POWER_ON_SELF_TEST == 1)
		MenuDisplay();
}
void ReadHeartTemperatureSensor()
{
	if(HAL_I2C_Mem_Read_IT(&hi2c2, MLX90614_I2CADDR, MLX90614_TOBJ1, 1, ObjTempRegValue, sizeof(ObjTempRegValue))!= HAL_OK)
	{
	  Error_Handler();
	}
	HAL_Delay(65);
//	sprintf(usr_msg, "\n\tHeart Temperature Sensor Value =  %f *C\n\r", (HeartTemperature));
//	if(HAL_UART_Transmit(&huart2, (uint8_t*)usr_msg, sizeof(usr_msg), HAL_MAX_DELAY)!= HAL_OK)
//	{
//		Error_Handler();
//	}
	if(POWER_ON_SELF_TEST == 1)
		MenuDisplay();
}

void ReadHumidtySensor()
{
	sprintf(usr_msg, "\n\tWORK IN PROGRESS\n\r");
	if(HAL_UART_Transmit(&huart2, (uint8_t*)usr_msg, sizeof(usr_msg), HAL_MAX_DELAY)!= HAL_OK)
	{
		Error_Handler();
	}
	if(POWER_ON_SELF_TEST == 1)
		MenuDisplay();
}

void CalculateTempAverage()
{
	/*Weighted Average Calculation*/
	if(TemperatureOne >= TemperatureTwo)
	{
	  Temperature = (((TemperatureOne * 0.25) + (TemperatureTwo * 0.75)) );
	}
	else
	{
	  Temperature = (((TemperatureOne * 0.75) + (TemperatureTwo * 0.25)) );
	}
	Temperature = (((TemperatureOne * 0.5) + (TemperatureTwo * 0.5)) );
	if(TemperatureOne >= TemperatureTwo)
	{
	  TemperatureDifference = (TemperatureOne - TemperatureTwo);
	//		  iTemperatureDifference = (uint32_t)TemperatureDifference;
	  iTemperatureDifference = F_I(TemperatureDifference);
	}
	else
	{
	  TemperatureDifference = (TemperatureTwo - TemperatureOne);
	//		  iTemperatureDifference = (uint32_t)TemperatureDifference;
	  iTemperatureDifference = F_I(TemperatureDifference);
	}
}

