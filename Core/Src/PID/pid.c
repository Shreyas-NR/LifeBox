/*
 * pid.c
 *
 *  Created on: Aug 24, 2020
 *      Author: shreyasnr
 */

#include "PID/pid.h"

//helper function to compare two floats within epsilon accuracy
//return true if floats are the same
static uint8_t Compare(float a, float b, float epsilon);
PID_Properties_t BlowerPID;
PID_PWM_t BlowerPWM;
PID_TIMER_t BlowerTime;
float kp,ki,kd;
float *TemperatureSetPoint;
float *TemperatureFeedback;
float *PIDTemperatureOutput;
char msg[500];
float SP = 0;
float PidOutput = 0;
uint8_t counter = 0;
static uint8_t IntervalCount = 0;
void PIDTest()
{
	/*Start the Timer 5 here and necessary code for the PID*/
	/*Initialize all the PID parameters*/
//	kp = 0.5;
//	ki = 0;
//	kd = 0;
	SP = 6;
	kp = 3.75;
	ki = 1;//1;
	kd = 4;//6
	TemperatureSetPoint = &SP;
	TemperatureFeedback = &Temperature;
//	TemperatureFeedback = &HeartTemperature;
//	if((Temperature < 8) && (Temperature > 4))
//	{
//		TemperatureFeedback = &HeartTemperature;
//	}
//	else
//	{
//		TemperatureFeedback = &Temperature;
//	}
	BlowerPID.period = 8000;// Time Interval between every PID Loop execution
	BlowerPID.error = *(TemperatureSetPoint) - *(TemperatureFeedback);
	BlowerPID.negIntegralLimit = -30;//-50;
	BlowerPID.negOutputLimit = -30;//-30;//-50
	BlowerPID.posIntegralLimit = 0;//50;
	BlowerPID.posOutputLimit = 0;//0;//50

	PidSetParams(&BlowerPID,kp,ki,kd);

	BlowerPWM.InputMax = 0;//50
	BlowerPWM.InputMin = -30;//-30;//-50
	BlowerPWM.OutputMax = 100;
	BlowerPWM.OutputMin = 0;

	BlowerTime.InputMax = 0;
	BlowerTime.InputMin = -30;
	BlowerTime.OutputMax = 8;
	BlowerTime.OutputMin = 0;

	PID(&BlowerPID, TemperatureSetPoint, TemperatureFeedback, PIDTemperatureOutput, derivativeOnError, noReverse);
}



uint8_t PID(PID_Properties_t* PID_Properties, float* pSetpoint, float* pFeedback, float* pOutput, derivative_t derivativeType, PID_Reverse_t pidReverse){

//  if(PID_Properties == NULL || pSetpoint == NULL || pFeedback == NULL || pOutput == NULL) return 0;

  float feedback = *pFeedback;
  float error = *pSetpoint - feedback;
  float derivativeOutput = 0.0f;
  float output;

  //proportional part
  float proportionalOutput = PID_Properties->kp * error;
  //integral part
	if(IntervalCount == 5)
	{
		PID_Properties->integralSum = 0;
		IntervalCount = 0;
	}
    PID_Properties->integralSum += (PID_Properties->ki * error);
    //anti wind-up
    if (PID_Properties->integralSum > PID_Properties->posIntegralLimit)
    	PID_Properties->integralSum = PID_Properties->posIntegralLimit;
    else if (PID_Properties->integralSum < PID_Properties->negIntegralLimit)
    	PID_Properties->integralSum = PID_Properties->negIntegralLimit;

    IntervalCount++;


  //derivative part
  switch(derivativeType){
    case 1:
      derivativeOutput = PID_Properties->kd * (-1.0f) * (feedback - PID_Properties->lastFeedback);
    break;

    case 2:
      derivativeOutput = PID_Properties->kd * (error - PID_Properties->lastError);
    break;

    default:
    break;
  }

  output = proportionalOutput + PID_Properties->integralSum + derivativeOutput;
   if(pidReverse) output *= -1.0f;

  //check if output is within bounds
  if(output > PID_Properties->posOutputLimit)
	  output = PID_Properties->posOutputLimit;
  else if(output < PID_Properties->negOutputLimit)
	  output = PID_Properties->negOutputLimit;

  *pOutput = output;

  PID_Properties->lastFeedback = feedback;
  PID_Properties->lastError = error;

  PidOutput = output;
/*  PWM = CalculatePWM(&BlowerPWM,output);Mapping the PID Output to PWM duty cycle 0-100

  if(PWM != LastPWM)
  {
	BlowerVaryingPIDSpeed();
  }
  LastPWM = PWM;*/
  BlowerFunctionalTime = CalculateTime(&BlowerTime,output);
  return 1;
}


uint8_t PidSetParams(PID_Properties_t* PID_Properties, float _kp, float _ki, float _kd)
{
  if(_kp < 0 || _ki < 0 || _kd < 0 || PID_Properties == NULL)
	  return 0;

  //check if PID_Properties are different
  if( ! Compare(PID_Properties->kp, _kp, 1E-4))
  {
	  PID_Properties->kp = _kp;
  }

  if( ! Compare(PID_Properties->ki, _ki, 1E-4))
  {
	  PID_Properties->ki = _ki * ((float)PID_Properties->period / 1000.0f);
  }

  if( ! Compare(PID_Properties->kd, _kd, 1E-4))
  {
	  if(PID_Properties->period > 0)
      //PID_Properties->kd = _kd / ((float)PID_Properties->period / 1000.0f); //some matlab problems? this might need to be commented out
      PID_Properties->kd = _kd;
  }
  return 1;
}

static uint8_t Compare(float a, float b, float epsilon){
  return fabs(a - b) < epsilon;
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	/*This loop executes every 1 s*/
	if(htim->Instance == TIM5)
	{
		if(BlowerFunctionalTime > 0)
		{
			BlowerFunctionalTime--;
		}
		else if(BlowerFunctionalTime == 0)
		{
			StopBlowerPID();
			PwmZero = 1;
//			ExhaustGateOpen();
//			ExhaustOn();
		}
		counter++;
		if(counter == 8)
		{
			counter = 0;
			PIDTest();
	//		if(PWM == 0)
			if(BlowerFunctionalTime == 0)
			{
				StopBlowerPID();
				PwmZero = 1;
	//			ExhaustGateOpen();
	//			ExhaustOn();
			}
	//		else if((PwmZero == 1)&&(PWM > 0)){
			else if((PwmZero == 1)&&(BlowerFunctionalTime > 0)){
	//			ExhaustGateClose();
	//			ExhaustOff();
				BlowerOn();
	//			BlowerVaryingPIDSpeed();
				PwmZero = 0;
			}
		}

		if(iTemperatureDifference >= 4)
		{
			ExhaustOn();
			ExhaustOnOff = 1;
		}
		else
		{
			ExhaustOff();
			ExhaustOnOff = 0;
		}
	}
	if(htim->Instance == TIM6)
	{
		/*This loop executes every 1Sec
		 * To be used for Perfusion Interval count
		 * call the function in the Perfusion.c file*/
		PerfusionIntervalTimeCalc();
	}
}

uint8_t CalculatePWM(PID_PWM_t* PID_PWM, float PID_Output)
{
	int8_t InMax = PID_PWM->InputMax;
	int8_t InMin = PID_PWM->InputMin;
	int8_t OutMax = PID_PWM->OutputMax;
	int8_t OutMin = PID_PWM->OutputMin;
	uint8_t PWMOutput = 0;
	PWMOutput = (((PID_Output - InMax)*(OutMax - OutMin)/(InMax - InMin)+OutMin)*-1);
//	PWMOutput = ((((PID_Output - InMin)*(OutMax - OutMin)/(InMax - InMin)+OutMin)-100)*-1);
	return PWMOutput;
}

uint8_t CalculateTime(PID_TIMER_t* PID_TIME, float PID_Output)
{
	int8_t InMax = PID_TIME->InputMax;
	int8_t InMin = PID_TIME->InputMin;
	int8_t OutMax = PID_TIME->OutputMax;
	int8_t OutMin = PID_TIME->OutputMin;
	uint8_t TimeOutput = 0;
	TimeOutput = (((PID_Output - InMax)*(OutMax - OutMin)/(InMax - InMin)+OutMin)*-1);
	return TimeOutput;
}
