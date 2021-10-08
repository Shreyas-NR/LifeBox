/*
 * Perfusion.c
 *
 *  Created on: Jan 11, 2021
 *      Author: shreyasnr
 */

#include "Perfusion/Perfusion.h"
#include "roboClaw/roboClaw.h"
PumpSpeed_t MainPumpSpeed;
float Pump1Slope = 1.1107;
float Pump1Constant = 28.9139;

void GetPerfusionData()
{
	switch(PerfusionDataBuf[0])
	{
	  case '0':
		  PerfusionFlowRate = 0;
		  break;
	  case '1':
		  PerfusionFlowRate = 100;
		  break;
	  case '2':
		  PerfusionFlowRate = 150;
		  break;
	  case '3':
		  PerfusionFlowRate = 200;
		  break;
	  case '4':
		  PerfusionFlowRate = 250;
		  break;
	  case '5':
		  PerfusionFlowRate = 300;
		  break;
	  default:
		  PerfusionFlowRate = 0;
		  break;
	}
	switch(PerfusionDataBuf[1])
	{
	  case '0':
		  PerfusionTimeInterval = 0;
		  break;
	  case '1':
		  PerfusionTimeInterval = 3;
		  break;
	  case '2':
		  PerfusionTimeInterval = 10;
		  break;
	  case '3':
		  PerfusionTimeInterval = 15;
		  break;
	  case '4':
		  PerfusionTimeInterval = 20;
		  break;
	  case '5':
		  PerfusionTimeInterval = 25;
		  break;
	  case '6':
		  PerfusionTimeInterval = 30;
		  break;
	  case '7':
		  PerfusionTimeInterval = 35;
		  break;
	  case '8':
		  PerfusionTimeInterval = 40;
		  break;
	  default:
		  PerfusionTimeInterval = 0;
		  break;
	}
	switch(PerfusionDataBuf[2])
	{
	  case '0':
		  PerfusionTime = 0;
		  break;
	  case '1':
		  PerfusionTime = 30;
		  break;
	  case '2':
		  PerfusionTime = 45;
		  break;
	  case '3':
		  PerfusionTime = 60;
		  break;
	  case '4':
		  PerfusionTime = 75;
		  break;
	  case '5':
		  PerfusionTime = 90;
		  break;
	  case '6':
		  PerfusionTime = 105;
		  break;
	  case '7':
		  PerfusionTime = 120;
		  break;
	  default:
		  PerfusionTime = 0;
		  break;
	}
	MainPumpSpeed.InputMax = 1000;
	MainPumpSpeed.InputMin = 0;
	MainPumpSpeed.OutputMax = 127;
	MainPumpSpeed.OutputMin = 0;
	MainPumpSpeedValue = MapFlowRate(&MainPumpSpeed, PerfusionFlowRate);
//	PerfusionTimeInterval = 3;
}
void PerfusionIntervalTimeCalc()
{
	SecondCount++;
	GsecCounter++;
	GminCounter = (GsecCounter/60);
	MinuteCount = (SecondCount/60);
	if(MinuteCount == PerfusionTimeInterval)
	{
		SecondCount = 0;
		MinuteCount = 0;
		StopPumpOneMotor = 0;
		StartPumpOneMotor = 1;
		StopPumpTwoMotor = 1;
		StartPumpTwoMotor = 0;
		NumOfPerfusionCycles++;
	}
	if(SecondCount >= PerfusionTime)
	{
		StopPumpOneMotor = 1;
		StartPumpOneMotor = 0;
	}
	if(SecondCount >= PerfusionTime + 60)
	{
		StopPumpTwoMotor = 0;
		StartPumpTwoMotor = 1;
	}
}

void StartPerfusion()
{
	HAL_TIM_Base_Start_IT(&htim6);//Start the timer for the perfusion interval counter (1s)
//	ForwardM1(&hroboclaw_mc1,MainPumpSpeedValue);//32
//	ForwardM2(&hroboclaw_mc1,64);// always 64-should not exceed 64 since the operating voltage of the pump is 12v
	StartPumpOneMotor = 1;
	StopPumpOneMotor = 0;
}

void StopPerfusion()
{
	HAL_TIM_Base_Stop_IT(&htim6);//Start the timer for the perfusion interval counter (1s)
	StopPumpOneMotor = 1;
	StartPumpOneMotor = 0;
	StopPumpTwoMotor = 1;
	StartPumpTwoMotor = 0;
}

uint8_t MapFlowRate(PumpSpeed_t* MainPumpSpeed, float FlowRate)
{
	int16_t InMax = MainPumpSpeed->InputMax;
	int8_t InMin = MainPumpSpeed->InputMin;
	int8_t OutMax = MainPumpSpeed->OutputMax;
	int8_t OutMin = MainPumpSpeed->OutputMin;
	uint16_t Temp1 = 0;
	uint8_t Temp2 = 0;
	Temp1 = (Pump1Slope * FlowRate) + Pump1Constant;
	Temp2 = (((Temp1 - InMin)*(OutMax - OutMin)/(InMax - InMin)+OutMin));
	return Temp2;
}

void ResetPerfusionData()
{
	  SecondCount = 0;
	  GsecCounter = 0;
	  GminCounter = 0;
	  MinuteCount = 0;
	  NumOfPerfusionCycles = 0;
	  PerfusionFlowRate = 0;
	  PerfusionTimeInterval = 0;
	  PerfusionTime = 0;
	  for(uint8_t i = 0; i<5; i++)
		  PerfusionDataBuf[i] = 0;
}
