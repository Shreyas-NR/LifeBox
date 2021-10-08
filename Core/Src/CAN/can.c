/*
 * can.c
 *
 *  Created on: 13-Aug-2020
 *      Author: shreyasnr
 */

#include "CAN/can.h"
char usr_msg[50];

void TestCommunication()
{
	sprintf(usr_msg, "\n\tWORK IN PROGRESS\n\r");
	if(HAL_UART_Transmit(&huart2, (uint8_t*)usr_msg, sizeof(usr_msg), HAL_MAX_DELAY)!= HAL_OK)
	{
		Error_Handler();
	}
	if(POWER_ON_SELF_TEST == 1)
		MenuDisplay();
}
