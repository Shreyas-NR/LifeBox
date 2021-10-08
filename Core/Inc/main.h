/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32l4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "string.h"
#include "Blower/Blower.h"
#include "Exhaust Fan/ExhaustFan.h"
#include "LoadCell/LoadCell.h"
#include "dwt_stm32_delay.h"
#include "Solenoid Valve/SolenoidValve.h"
#include "Temperature Sensors/TemperatureSensor.h"
#include "Timer/Timer.h"
#include "HX711/hx711.h"
#include "Peristaltic Pump/PeristalticPump.h"
#include <ADS1115/ADS1115.h>
#include "CAN/can.h"
#include "PID/pid.h"
#include "POST/post.h"
#include "UART/uart.h"
#include "Cooling Chamber/CoolingChamber.h"
#include "Perfusion/Perfusion.h"
#include "AD7193/AD7193.h"
#include "Pressure Sensor/PressureSensor.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
extern UART_HandleTypeDef huart2;
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern I2C_HandleTypeDef hi2c1;
extern I2C_HandleTypeDef hi2c2;
extern I2C_HandleTypeDef hi2c3;
extern TIM_HandleTypeDef htim5;
extern TIM_HandleTypeDef htim6;
extern UART_HandleTypeDef huart4;
extern SPI_HandleTypeDef hspi2;
//extern SMBUS_HandleTypeDef hsmbus2;

extern char *user_data;
extern char *exit_data;
extern char *menu_data;
extern char *CurrentBlowerOption[3];
extern char *BlowerSetting_data;
extern char *PumpOneSetting_data;
extern char *CurrentExhaustFanOption[3];
extern char *ExhaustFanSetting_data;
extern uint8_t RxData;
extern uint8_t RxDataBuffer[100];
extern uint8_t RxOptionData[10],RxBlowerOption[10],RxExhaustFanOption[10];
extern uint8_t Cnt,Complete;
extern uint8_t MainMenuMode, BlowerStartMode, ExhaustFanStartMode;
extern uint8_t BlowerLoop;
extern uint8_t ExhaustFanLoop;
extern uint32_t PulseCounter;
extern uint8_t BlowerPWMValue;
extern uint8_t ExhaustFanPWMValue;
extern uint8_t TemperatureSensorOne;
extern uint8_t TemperatureSensorTwo;
extern uint8_t AmbTempOneRegValue[2];
extern uint8_t AmbTempTwoRegValue[2];
extern uint8_t ObjTempRegValue[2];
extern uint16_t TempOneRegValue;
extern uint16_t TempTwoRegValue;
extern uint16_t HeartTempRegValue;
extern float TemperatureOne;
extern float TemperatureTwo;
extern float HeartTemperature;
extern float TemperatureDifference;
extern uint32_t iTemperatureDifference;
extern uint32_t aTemperatureDifference[200];
extern uint32_t TemperauteDifferenceAvg;


extern uint8_t RcvDataBuffer[7],RcvDataBuf[20],PerfusionDataBuf[5];
extern uint8_t Count;
extern uint8_t RcvData;
extern uint8_t TemperatureAvailable;
extern float Temperature;
extern uint32_t IterationNumber;
extern float Temp_IN_PID_Output[5000][3];
extern uint8_t PWM;
extern uint8_t LastPWM;
extern uint8_t BlowerPIDLoop;
extern uint8_t PwmZero;
extern uint8_t ON,OFF;
extern float PidOutput;
extern int UnitWeight;
extern uint16_t NumOfSamples;
extern char usr_msg1[100];
extern int DryIceLoad, SalineOneLoad, SalineTwoLoad;
extern uint8_t BlowerFunctionalTime;
extern uint8_t BlowerOnOff;
extern uint8_t ExhaustOnOff;
extern float pHValue;
extern float PressureValue;
extern uint8_t ReqPerfusionData;
extern uint8_t PerfusionSettingUpdated, ReqPerfusionTime;
extern uint16_t PerfusionFlowRate;
extern uint8_t PerfusionTimeInterval;
extern uint8_t PerfusionTime;
extern uint32_t SecondCount, GsecCounter;
extern uint32_t MinuteCount, GminCounter;
extern uint8_t MainPumpSpeedValue;
extern uint8_t StopPumpOneMotor;
extern uint8_t StartPumpOneMotor;
extern uint8_t StopPumpTwoMotor;
extern uint8_t StartPumpTwoMotor;
extern uint8_t NumOfPerfusionCycles;

extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;
/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
void MenuDisplay();
void InvalidOption();
void ExitMenu();
void InitializeLoadCell();
void InitializeTemperatureSensor();
void InitializeRoboClaw();
void InitializeServo();
void InitializePressureSensor();
void LoadDefaultSettings();
void SendToGUI(uint8_t *pData);
void SendToUSB(uint8_t *pData);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define B1_Pin GPIO_PIN_13
#define B1_GPIO_Port GPIOC
#define USART_TX_Pin GPIO_PIN_2
#define USART_TX_GPIO_Port GPIOA
#define USART_RX_Pin GPIO_PIN_3
#define USART_RX_GPIO_Port GPIOA
#define AD7193_CS_PIN_Pin GPIO_PIN_12
#define AD7193_CS_PIN_GPIO_Port GPIOB
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define SWO_Pin GPIO_PIN_3
#define SWO_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */
#define POWER_ON_SELF_TEST 		0
#define F_I(f) ( f>0?(uint32_t)(f + 0.5):(uint32_t)(f - 0.5))
#define IN_LINE_PRESSURE_SENSOR 0
#define FLOW_SENSOR 			0
#define DRY_ICE_LOAD			0
#define PERFUSION_FLUID_LOAD	1
#define WASTE_FLUID_LOAD		2
#define DEBUG_PERFUSION			0
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
