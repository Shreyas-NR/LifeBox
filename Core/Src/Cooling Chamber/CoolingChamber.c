/*
 * CoolingChamber.c
 *
 *  Created on: Oct 10, 2020
 *      Author: shreyasnr
 */

#include "Cooling Chamber/CoolingChamber.h"

void StartCoolingChamber()
{
/*	HAL_TIM_Base_Start_IT(&htim5);//Start the timer for the PID execution (10ms)
//	HAL_TIM_PWM_Start_IT(&htim2, TIM_CHANNEL_1);//Start the timer for the Blower(25KHz)
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);//Start the timer for the Blower(25KHz)
	BlowerOn();//Switch ON the Blower Power Supply
	BlowerPIDLoop = 1;*/
	PIDTest();
	HAL_TIM_Base_Start_IT(&htim5);//Start the timer for the PID execution (1s)
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);//Start the timer for the Blower(25KHz)
	BlowerOn();//Switch ON the Blower Power Supply
	BlowerPIDLoop = 1;
}

void StopCoolingChamber()
{
/*//	HAL_TIM_PWM_Stop_IT(&htim2, TIM_CHANNEL_1);//Stop the timer of the Blower(25KHz)
	HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);//Stop the timer of the Blower(25KHz)
	BlowerOff();//Switch OFF the Blower Power Supply
	HAL_TIM_Base_Stop_IT(&htim5);//Stop the timer of the PID execution (10ms)
	BlowerPIDLoop = 0;*/

	HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);//Stop the timer of the Blower(25KHz)
	BlowerOff();//Switch OFF the Blower Power Supply
	HAL_TIM_Base_Stop_IT(&htim5);//Stop the timer of the PID execution (10ms)
	BlowerPIDLoop = 0;
}
