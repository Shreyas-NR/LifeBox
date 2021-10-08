/*
 * Blower.c
 *
 *  Created on: Aug 6, 2020
 *      Author: shreyasnr
 */

#include "Blower/Blower.h"

uint8_t StartBlowerOTF = 0,StopBlowerOTF = 0;
uint8_t BlowerPWMValue = 0;
uint8_t IncBlowerSpeed = 0;
uint8_t DecBlowerSpeed = 0;
void StartBlower()
{
	if(StartBlowerOTF == 0)
	{
		BlowerStartMode = 1;
		MainMenuMode = 0;
		ExhaustFanStartMode = 0;
		BlowerSettingsDisplay();
		switch(RxBlowerOption[0]){
		case '1':
			BlowerPWMValue = 0;
			break;
		case '2':
			BlowerPWMValue = 30;
			break;
		case '3':
			BlowerPWMValue = 60;
			break;
		case '4':
			BlowerPWMValue = 80;
			break;
		case '5':
			BlowerPWMValue = 100;
//			BlowerLoop = 1;
			break;
		default:
			BlowerPWMValue = 10;
			break;
		}
		TIM2_Init(BlowerPWMValue,BLOWER);
//		HAL_TIM_PWM_Start_IT(&htim2, TIM_CHANNEL_1);
		HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_RESET);
		HAL_UART_Transmit(&huart2, (uint8_t*)CurrentBlowerOption[0], strlen(CurrentBlowerOption[0]), HAL_MAX_DELAY);
		//if(!BlowerLoop){
		if(POWER_ON_SELF_TEST == 1)
			MenuDisplay();
		//}
		StartBlowerOTF = 1;
		StopBlowerOTF = 0;
	}
}

void StopBlower()
{
	if(StopBlowerOTF == 0)
	{
		BlowerStartMode = 0;
		BlowerLoop = 0;
		HAL_UART_Transmit(&huart2, (uint8_t*)CurrentBlowerOption[1], strlen(CurrentBlowerOption[1]), HAL_MAX_DELAY);
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_SET);
//		HAL_TIM_PWM_Stop_IT(&htim2, TIM_CHANNEL_1);
		HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);
		if(POWER_ON_SELF_TEST == 1)
			MenuDisplay();
		StopBlowerOTF = 1;
		StartBlowerOTF = 0;
	}
}
void StopBlowerPID()
{
	BlowerOff();//Switch OFF the Blower Power Supply
//	HAL_TIM_PWM_Stop_IT(&htim2, TIM_CHANNEL_1);
	HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);
}
void BlowerVaryingSpeed()
{
	if((PulseCounter % 1000) == 0){

		if(BlowerPWMValue == 1)
		{
			DecBlowerSpeed = 0;
			IncBlowerSpeed = 1;
		}
		if(BlowerPWMValue == 100)
		{
			DecBlowerSpeed = 1;
			IncBlowerSpeed = 0;
		}
		if((IncBlowerSpeed == 1)&&(DecBlowerSpeed == 0))
		{
			BlowerPWMValue++;
		}
		if((IncBlowerSpeed == 0)&&(DecBlowerSpeed == 1))
		{
			BlowerPWMValue--;
		}
//		HAL_TIM_PWM_Stop_IT(&htim2, TIM_CHANNEL_1);
		HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);
		TIM2_Init(BlowerPWMValue,BLOWER);
//		HAL_TIM_PWM_Start_IT(&htim2, TIM_CHANNEL_1);
		HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
	}
}

void BlowerVaryingPIDSpeed()
{
//	HAL_TIM_PWM_Stop_IT(&htim2, TIM_CHANNEL_1);
	HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);
	TIM2_Init(PWM,BLOWER);
//	HAL_TIM_PWM_Start_IT(&htim2, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
}

void BlowerOn()
{
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_SET);
}
void BlowerOff()
{
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_RESET);
}
