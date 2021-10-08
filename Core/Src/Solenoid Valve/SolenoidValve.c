/*
 * SolenoidValve.c
 *
 *  Created on: 07-Aug-2020
 *      Author: shreyasnr
 */


#include "Solenoid Valve/SolenoidValve.h"
char usr_msg[50];

void OpenSolenoidValve(){
	sprintf(usr_msg, "\n\tWORK IN PROGRESS\n\r");
	if(HAL_UART_Transmit(&huart2, (uint8_t*)usr_msg, sizeof(usr_msg), HAL_MAX_DELAY)!= HAL_OK)
	{
		Error_Handler();
	}
	if(POWER_ON_SELF_TEST == 1)
		MenuDisplay();
}
void CloseSolenoidValve(){
	sprintf(usr_msg, "\n\tWORK IN PROGRESS\n\r");
	if(HAL_UART_Transmit(&huart2, (uint8_t*)usr_msg, sizeof(usr_msg), HAL_MAX_DELAY)!= HAL_OK)
	{
		Error_Handler();
	}
	if(POWER_ON_SELF_TEST == 1)
		MenuDisplay();
}
