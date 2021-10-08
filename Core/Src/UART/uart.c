/*
 * uart.c
 *
 *  Created on: Aug 24, 2020
 *      Author: shreyasnr
 */

#include "UART/uart.h"
#include "Serial/Serial.h"

char msg[100];
uint8_t BufCount = 0;
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	if(huart->Instance == USART2)//PC for unit test
	{
		if (RxData == '\r') {
			Complete = 1;
			//RxOptionData[Cnt++] = '\r';
			//HAL_UART_Transmit(&huart2, RxOptionData, Cnt, HAL_MAX_DELAY);
			Cnt = 0;
		} else {
			if (MainMenuMode == 1) {
				RxOptionData[Cnt++] = RxData;
			} else if (BlowerStartMode == 1) {
				RxBlowerOption[Cnt++] = RxData;
			} else if (ExhaustFanStartMode == 1) {
				RxExhaustFanOption[Cnt++] = RxData;
			}
		}
	}
	if(huart->Instance == USART3)//raspberry Pi temperature
	{
		if (RcvData == '\r')
		{
			Temperature = 0;
			Temperature = (RcvDataBuffer[0]-48) * 100;
			Temperature += (RcvDataBuffer[1]-48) * 10;
			Temperature += (RcvDataBuffer[3]-48);
			Temperature /= 10.0;
			Count = 0;
			IterationNumber++;
			for(uint8_t i =0;i<7;i++)
			{
				RcvDataBuffer[i] = 0;
			}
//			sprintf(msg,"\r\nCurrent Temp = %f\tUART Number = %d", Temperature,IterationNumber);
//			HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);

			if(TemperatureAvailable == 0)
			{
				TemperatureAvailable = 1;
				HAL_TIM_Base_Start_IT(&htim5);
//				HAL_TIM_PWM_Start_IT(&htim2, TIM_CHANNEL_1);
				HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_SET);
				BlowerPIDLoop = 1;
			}
		}
		else
		{
			RcvDataBuffer[Count++] = RcvData;
		}
	}
	if(huart->Instance == UART4)//touch screen GUI
	{
			if(ReqPerfusionData == 1)
			{
				PerfusionDataBuf[BufCount++] = RcvData;
				if(BufCount == 2)
				{
					ReqPerfusionTime = 1;
				}
				if(BufCount == 3)
				{
					BufCount = 0;
					ReqPerfusionData = 0;
					ReqPerfusionTime = 0;
					PerfusionSettingUpdated = 1;
				}
			}
			else
			{
				RcvDataBuf[Count++] = RcvData;
				if(Count == 1)
				{
					Count = 0;
					RcvData = 0;
				}
			}

	}
	if(huart->Instance == UART5)//RoboClaw
	{
		SERIAL_HandleTypeDef* serial_handler = get_serial_handler(huart);

		ring_buffer_queue_arr(serial_handler->buffer_Rx, serial_handler->hal_reg_Rx, PRIMARY_REG_SIZE);
		HAL_UART_Receive_DMA(huart, serial_handler->hal_reg_Rx, PRIMARY_REG_SIZE);
	}
}
void MenuDisplay() {
	MainMenuMode = 1;
	Complete = 0;
	Cnt = 0;
	RxOptionData[Cnt] = 0;
	HAL_UART_Transmit(&huart2, (uint8_t*) menu_data, strlen(menu_data), HAL_MAX_DELAY);
	HAL_UART_Receive_IT(&huart2, &RxData, 1);
}
void BlowerSettingsDisplay() {
	Complete = 0;
	Cnt = 0;
	HAL_UART_Transmit(&huart2, (uint8_t*) BlowerSetting_data,
			strlen(BlowerSetting_data), HAL_MAX_DELAY);
	while (Complete != 1) {
		HAL_UART_Receive_IT(&huart2, &RxData, 1);
	}
}
void ExhaustFanSettingsDisplay() {
	Complete = 0;
	Cnt = 0;
	HAL_UART_Transmit(&huart2, (uint8_t*) ExhaustFanSetting_data,
			strlen(ExhaustFanSetting_data), HAL_MAX_DELAY);
	while (Complete != 1) {
		HAL_UART_Receive_IT(&huart2, &RxData, 1);
	}
}
void InvalidOption() {

	HAL_UART_Transmit(&huart2, (uint8_t*) CurrentBlowerOption[2],
			strlen(CurrentBlowerOption[2]), HAL_MAX_DELAY);
	if(POWER_ON_SELF_TEST == 1)
		MenuDisplay();

}
void ExitMenu() {

	HAL_UART_Transmit(&huart2, (uint8_t*) exit_data, strlen(exit_data),
			HAL_MAX_DELAY);

//	StopBlower();
//	StopExhaustFan();
}
void PumpOneSettingsDisplay() {
	Complete = 0;
	Cnt = 0;
	HAL_UART_Transmit(&huart2, (uint8_t*) PumpOneSetting_data, strlen(PumpOneSetting_data), HAL_MAX_DELAY);
	while (Complete != 1) {
		HAL_UART_Receive_IT(&huart2, &RxData, 1);
	}
}
