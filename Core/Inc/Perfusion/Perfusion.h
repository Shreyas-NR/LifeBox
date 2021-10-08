/*
 * Perfusion.h
 *
 *  Created on: Jan 11, 2021
 *      Author: shreyasnr
 */

#ifndef INC_PERFUSION_PERFUSION_H_
#define INC_PERFUSION_PERFUSION_H_

#include "main.h"
typedef struct{
int8_t InputMin;
int16_t InputMax;
int8_t OutputMin;
int8_t OutputMax;
} PumpSpeed_t;

void GetPerfusionData();
void PerfusionIntervalTimeCalc();
void StartPerfusion();
void StopPerfusion();
uint8_t MapFlowRate(PumpSpeed_t* MainPumpSpeed, float FlowRate);
void ResetPerfusionData();


#endif /* INC_PERFUSION_PERFUSION_H_ */
