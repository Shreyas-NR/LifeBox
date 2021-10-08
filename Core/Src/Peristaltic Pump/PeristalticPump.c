/*
 * PeristalticPump.c
 *
 *  Created on: 13-Aug-2020
 *      Author: shreyasnr
 */

#include "Peristaltic Pump/PeristalticPump.h"
#include "roboClaw/roboClaw.h"
char usr_msg[50];
uint8_t PumpSpeedValue = 0;
void StartPumpOne()
{
	BlowerStartMode = 1;
	MainMenuMode = 0;
	ExhaustFanStartMode = 0;
	PumpOneSettingsDisplay();
	switch(RxBlowerOption[0]){
	case '1':
		PumpSpeedValue = 12;
		break;
	case '2':
		PumpSpeedValue = 127;
		break;
	}
	ForwardM1(&hroboclaw_mc1,PumpSpeedValue);
	if(POWER_ON_SELF_TEST == 1)
		MenuDisplay();
}

void StopPumpOne()
{
	BlowerStartMode = 0;
	ForwardM1(&hroboclaw_mc1,0);
	if(POWER_ON_SELF_TEST == 1)
		MenuDisplay();
}
void StartPumpTwo()
{
	sprintf(usr_msg, "\n\tWORK IN PROGRESS\n\r");
	if(HAL_UART_Transmit(&huart2, (uint8_t*)usr_msg, sizeof(usr_msg), HAL_MAX_DELAY)!= HAL_OK)
	{
		Error_Handler();
	}
	if(POWER_ON_SELF_TEST == 1)
		MenuDisplay();
}

void StopPumpTwo()
{
	sprintf(usr_msg, "\n\tWORK IN PROGRESS\n\r");
	if(HAL_UART_Transmit(&huart2, (uint8_t*)usr_msg, sizeof(usr_msg), HAL_MAX_DELAY)!= HAL_OK)
	{
		Error_Handler();
	}
	if(POWER_ON_SELF_TEST == 1)
		MenuDisplay();
}

void ReadEncoder()
{
	sprintf(usr_msg, "\n\tEncoder Value = %lu\n\r", ((TIM1->CNT)>>2));
	if(HAL_UART_Transmit(&huart2, (uint8_t*)usr_msg, sizeof(usr_msg), HAL_MAX_DELAY)!= HAL_OK)
	{
	  Error_Handler();
	}
	if(POWER_ON_SELF_TEST == 1)
		MenuDisplay();
}
