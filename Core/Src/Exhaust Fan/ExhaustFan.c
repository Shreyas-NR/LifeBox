/*
 * ExhaustFan.c
 *
 *  Created on: 07-Aug-2020
 *      Author: shreyasnr
 */

#include "Exhaust Fan/ExhaustFan.h"

uint8_t StartExhaustFanOTF = 0,StopExhaustFanOTF = 0;
uint8_t ExhaustFanPWMValue = 0;
void StartExhaustFan(){
//	if(StartExhaustFanOTF == 0)
//	{
		ExhaustFanStartMode = 1;
		BlowerStartMode = 0;
		MainMenuMode = 0;
		ExhaustFanSettingsDisplay();
		switch(RxExhaustFanOption[0]){
		case '1':
			ExhaustFanPWMValue = 25;
			break;
		case '2':
			ExhaustFanPWMValue = 50;
			break;
		case '3':
			ExhaustFanPWMValue = 75;
			break;
		case '4':
			ExhaustFanPWMValue = 100;
			break;
		case '5':
			ExhaustFanPWMValue = 1;
			ExhaustFanLoop = 1;
			break;
		default:
			ExhaustFanPWMValue = 10;
			break;
		}
		TIM2_Init(ExhaustFanPWMValue,EXHAUST);
//		HAL_TIM_PWM_Start_IT(&htim2, TIM_CHANNEL_2);
		HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
		HAL_UART_Transmit(&huart2, (uint8_t*)CurrentExhaustFanOption[0], strlen(CurrentExhaustFanOption[0]), HAL_MAX_DELAY);
		if(POWER_ON_SELF_TEST == 1)
			MenuDisplay();
		StartExhaustFanOTF = 1;
		StopExhaustFanOTF = 0;
//	}

}
void StopExhaustFan()
{
//	if(StopExhaustFanOTF == 0)
//	{
		ExhaustFanStartMode = 0;
		ExhaustFanLoop = 0;
		HAL_UART_Transmit(&huart2, (uint8_t*)CurrentExhaustFanOption[1], strlen(CurrentExhaustFanOption[1]), HAL_MAX_DELAY);
//		HAL_TIM_PWM_Stop_IT(&htim2, TIM_CHANNEL_2);
		HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_2);
		if(POWER_ON_SELF_TEST == 1)
			MenuDisplay();
		StopExhaustFanOTF = 1;
		StartExhaustFanOTF = 0;
//	}
}

void ExhaustGateOpen()
{
	  htim3.Instance->CCR1 = 315;  // duty cycle is 1.5 milliseconds 90 degree
}
void ExhaustGateClose()
{
	  htim3.Instance->CCR1 = 125;  // duty cycle is .5 milliseconds 0 degree
}
void ExhaustOn()
{
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_SET);
}
void ExhaustOff()
{
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_RESET);
}

