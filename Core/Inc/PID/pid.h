/*
 * pid.h
 *
 *  Created on: Aug 24, 2020
 *      Author: shreyasnr
 */

#ifndef INC_PID_PID_H_
#define INC_PID_PID_H_

#include "main.h"
#include "math.h"

void PIDTest();

typedef struct{
  uint16_t period;              //interval between PID computations
  float error;
  float integralSum;
  float kp;                     //do not change Kp Ki Kd manually, use PidSetParams();
  float ki;
  float kd;
  float posIntegralLimit;       //integral anti-windup
  float negIntegralLimit;       //integral anti-windup
  float lastError;
  float lastFeedback;
  float posOutputLimit;         //bound the output
  float negOutputLimit;
} PID_Properties_t;

//derivative type calculation
typedef enum{
  noDerivative,
  derivativeOnFeedback,
  derivativeOnError
} derivative_t;

//reverse output of the PID
typedef enum{
  noReverse,
  reverse,
} PID_Reverse_t;

//compute PID
//return 0 if null pointers
//return 1 if ok
uint8_t PID(PID_Properties_t* PID_Properties, float* pSetpoint, float* pFeedback, float* pOutput, derivative_t derivativeType, PID_Reverse_t pidReverse);

//set PID tunings
//return 0 if parameters are invalid (<0) or null pointer
//return 1 if set went ok
uint8_t PidSetParams(PID_Properties_t* PID_Properties, float _kp, float _ki, float _kd);

typedef struct{
int8_t InputMin;
int8_t InputMax;
int8_t OutputMin;
int8_t OutputMax;
} PID_PWM_t,PID_TIMER_t ;

uint8_t CalculatePWM(PID_PWM_t* PID_PWM, float PID_Output);
uint8_t CalculateTime(PID_TIMER_t* PID_TIME, float PID_Output);

#endif /* INC_PID_PID_H_ */
