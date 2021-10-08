/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "Serial/Serial.h"
#include "roboClaw/roboClaw.h"
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
//#define POWER_ON_SELF_TEST	0
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define F_I(f) ( f>0?(uint32_t)(f + 0.5):(uint32_t)(f - 0.5))
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;
I2C_HandleTypeDef hi2c3;

SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim5;
TIM_HandleTypeDef htim6;

UART_HandleTypeDef huart4;
UART_HandleTypeDef huart5;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_uart5_rx;
DMA_HandleTypeDef hdma_uart5_tx;

/* USER CODE BEGIN PV */
SERIAL_HandleTypeDef *hserial_uart5;
RoboClaw_HandleTypeDef hroboclaw_mc1;

Pressure_t PressureLimits;
uint8_t debug_buff[50];
uint32_t m1_enc_cnt;
uint32_t m1_speed;
uint8_t status;
bool valid;
char *user_data = "\tHello User! Welcome to LifeBox...\r\n";
char *exit_data = "\n\tThankyou for using LifeBox...\r\n";
char *menu_data = "\r\n"
		"**************************************************************************************\r\n"
		"*****************************************MAIN MENU************************************\r\n"
		"**************************************************************************************\r\n"
		"1. START BLOWER\r\n"
		"2. STOP BLOWER\r\n"
		"3. START EXHAUST FAN\r\n"
		"4. STOP EXHAUST FAN\r\n"
		"5. OPEN SOLENOID VALVE\r\n"
		"6. CLOSE SOLENOID VALVE\r\n"
		"7. READ TEMPERATURE SENSOR 1 MCP9808\r\n"
		"8. READ TEMPERATURE SENSOR 2 MCP9808\r\n"
		"9. READ HEART TEMPERATURE SENSOR MLX90614\r\n"
		"a. READ HUMIDITY + TEMPERATURE SENSOR BME280\r\n"
		"b. READ LOAD SENSOR 1\r\n"
		"c. START PUMP 1\r\n"
		"d. STOP PUMP 1\r\n"
		"e. READ ENCODER\r\n"
		"f. START PUMP 2\r\n"
		"g. STOP PUMP 2\r\n"
		"h. READ pH SENSOR\r\n"
		"i. READ PRESSURE SENSOR\r\n"
		"j. READ FLOW SENSOR\r\n"
		"k. TEST COMMNUNICATION WITH RASPBERRY-PI\r\n"
		"p. START COOLING CHAMBER\r\n"
		"s. STOP COOLING CHAMBER\r\n"
		"x. EXIT APPLICATION\r\n"
		"Enter your Option Number\r\n";
char *CurrentBlowerOption[3] = {"\n\t--------BLOWER STARTED SUCCESSFULLY--------\r\n",
		"\n\t--------BLOWER STOPPED SUCCESSFULLY--------\r\n",
		"\n\t--------INVALID OPTION SELECTED--------\r\n"};
char *BlowerSetting_data = "\n"
		"*************************************************************************************\r\n"
		"********************************BLOWER SETTINGS TABLE********************************\r\n"
		"*************************************************************************************\r\n"
		"1. 0% DUTY CYCLE\r\n"
		"2. 30% DUTY CYCLE\r\n"
		"3. 60% DUTY CYCLE\r\n"
		"4. 80% DUTY CYCLE\r\n"
		"5. 100% DUTY CYCLE\r\n"
		"Enter your Option Number and press the ENTER key\r\n";
char *PumpOneSetting_data = "\n"
		"*************************************************************************************\r\n"
		"********************************BLOWER SETTINGS TABLE********************************\r\n"
		"*************************************************************************************\r\n"
		"1. 10% DUTY CYCLE\r\n"
		"2. 100% DUTY CYCLE\r\n"
		"Enter your Option Number and press the ENTER key\r\n";
char *CurrentExhaustFanOption[3] = {"\n\t--------EXHAUST FAN STARTED SUCCESSFULLY--------\r\n",
		"\n\t--------EXHAUST FAN STOPPED SUCCESSFULLY--------\r\n",
		"\n\t--------INVALID OPTION SELECTED--------\r\n"};
char *ExhaustFanSetting_data = "\n"
		"**********************************************************************************\r\n"
		"*****************************EXHAUST FAN SETTINGS TABLE***************************\r\n"
		"**********************************************************************************\r\n"
		"1. 25% DUTY CYCLE\r\n"
		"2. 50% DUTY CYCLE\r\n"
		"3. 75% DUTY CYCLE\r\n"
		"4. 100% DUTY CYCLE\r\n"
		"5. DUTY CYCLE in LOOP 1% to 100% and vice-versa\r\n"
		"Enter your Option Number and press the ENTER key\r\n";

uint8_t RxData = 0;
uint8_t RxDataBuffer[100];
uint8_t RxOptionData[10],RxBlowerOption[10],RxExhaustFanOption[10];
uint8_t Cnt = 0,Complete = 0;
uint8_t MainMenuMode = 0,BlowerStartMode = 0,ExhaustFanStartMode = 0, BlowerLoop = 0, ExhaustFanLoop = 0;
uint32_t PulseCounter = 0;
uint8_t RcvDataBuffer[7],RcvDataBuf[20],PerfusionDataBuf[5];
uint8_t Count = 0;
uint8_t RcvData = 0;
uint8_t TemperatureAvailable = 0;
float Temperature = 0;
uint32_t IterationNumber = 0;
float Temp_IN_PID_Output[5000][3];
uint8_t PWM = 0;
uint8_t LastPWM = 0;
uint8_t BlowerPIDLoop = 0;
uint8_t PwmZero = 0;
uint16_t adcValue = 0;
int UnitWeight = 0;
float CalibrationValue = 0;
uint16_t NumOfSamples = 4;
char usr_msg1[100];
uint8_t ON = 0,OFF = 0;
int DryIceLoad = 0;
int SalineOneLoad = 0;
int SalineTwoLoad = 0;
uint8_t BlowerFunctionalTime = 0;
uint8_t BlowerOnOff = 0;
uint8_t ExhaustOnOff = 0;
float pHValue = 0;
float PressureValue = 0;
uint8_t PerfusionSettingUpdated = 0,ReqPerfusionTime = 0;
uint8_t ReqPerfusionData = 0;
uint16_t PerfusionFlowRate = 0;
uint8_t PerfusionTimeInterval = 0;
uint8_t PerfusionTime = 0;
uint32_t SecondCount = 0, GsecCounter = 0;
uint32_t MinuteCount = 0, GminCounter = 0;
uint8_t MainPumpSpeedValue = 0;
uint8_t StopPumpOneMotor = 0;
uint8_t StartPumpOneMotor = 0;
uint8_t StopPumpTwoMotor = 0;
uint8_t StartPumpTwoMotor = 0;
uint8_t NumOfPerfusionCycles = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C2_Init(void);
static void MX_I2C3_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM5_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_UART4_Init(void);
static void MX_UART5_Init(void);
static void MX_SPI2_Init(void);
static void MX_TIM6_Init(void);
/* USER CODE BEGIN PFP */
void MenuDisplay();
void ResetVariables();
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */
  

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM2_Init();
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_I2C3_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_TIM5_Init();
  MX_USART3_UART_Init();
  MX_UART4_Init();
  MX_UART5_Init();
  MX_SPI2_Init();
  MX_TIM6_Init();
  /* USER CODE BEGIN 2 */
  InitializeRoboClaw();
  LoadDefaultSettings();
  InitializePressureSensor();
  InitializeTemperatureSensor();
  InitializeLoadCell();
  //  InitializeServo();
  /*to run unit level test un-comment below two lines*/
  if(POWER_ON_SELF_TEST == 1)
  {
	  HAL_UART_Transmit(&huart2, (uint8_t*)user_data, strlen(user_data), HAL_MAX_DELAY);
	  MenuDisplay();
  }
  else
  {
	  ReadTemperatureSensorOne();
	  ReadTemperatureSensorTwo();
	  ReadHeartTemperatureSensor();
  }


  /* USER CODE END 2 */
 
 

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
/*	  BackupCodes
//	  to run PID through RaspberryPi Temperature reception un-comment below line
//	  HAL_UART_Receive_IT(&huart3, &RcvData, 1);


//	  to run Servo Motor un-comment the below Lines of code
//	  ExhaustGateOpen();
//	  HAL_Delay(2000);
//	  ExhaustGateClose();
//	  HAL_Delay(2000);

//	  to run External ADC un-comment the below Lines of code
//	  adcValue = readADC_SingleEnded(2);
//	  HAL_Delay(100);

//	  to run Exhaust FAN with servo motor un-comment the below Lines of code
//	  ExhaustGateOpen();
//	  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_SET);
//	  HAL_Delay(3000);
//	  ExhaustGateClose();
//	  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_RESET);
//	  HAL_Delay(3000);

//To run Cooling Chamber PID uncomment the following lines of code
 * read temperature sensor values and Load cell value here
 * calculate the weighted average of the two temperature sensors
 * 70% for the bottom Temperature sensor 1
 * 30% for the top Temperature sensor 2
 * monitor for the emergency button

//	  if(BlowerPIDLoop == 1)
//	  {
//		  ReadTemperatureSensorOne();
//		  ReadTemperatureSensorTwo();
//		  Temperature = (((TemperatureOne * 0.7) + (TemperatureTwo * 0.3)) );
//		  sprintf(usr_msg1, "\n\tTemperature Sensor Value =  %f *C\n\r", (Temperature));
//		  if(HAL_UART_Transmit(&huart2, (uint8_t*)usr_msg1, sizeof(usr_msg1), HAL_MAX_DELAY)!= HAL_OK)
//		  {
//			  Error_Handler();
//		  }
//	  }
//	  to run unit level test un-comment below two lines*/
	  if(POWER_ON_SELF_TEST == 1)
		  PowerOnSelfTest();
	  else
	  {
		  ReadLoadcell();
		  ReadTemperatureSensorOne();
		  ReadTemperatureSensorTwo();
		  CalculateTempAverage();
		  ReadHeartTemperatureSensor();
		  pHValue = ReadPHSensor();
		  PressureValue = ReadPressureSensor();
		  HAL_UART_Receive_IT(&huart4, &RcvData, 1);

		  if(RcvDataBuf[0] == '1')
		  {
			  ResetVariables();
			  memset(usr_msg1, 0, sizeof(usr_msg1)); // clear the array
			  sprintf(usr_msg1, "A\n\r");
			  SendToGUI((uint8_t*)usr_msg1);
			  RcvDataBuf[Count] = 0;
			  ON = 1;
			  OFF = 0;
			  HAL_Delay(100);
		  }
		  else if(RcvDataBuf[0] == '2')
		  {
			  memset(usr_msg1, 0, sizeof(usr_msg1)); // clear the array
			  sprintf(usr_msg1, "B\n\r");
			  SendToGUI((uint8_t*)usr_msg1);
			  RcvDataBuf[Count] = 0;
			  OFF = 1;
			  ON = 0;
			  HAL_Delay(100);
		  }
		  else if(RcvDataBuf[0] == 'C')
		  {
			  StartCoolingChamber();
			  RcvDataBuf[Count] = 0;
		  }
		  else if(RcvDataBuf[0] == 'c')
		  {
			  StopCoolingChamber();
			  ExhaustGateClose();
			  ExhaustOff();
			  RcvDataBuf[Count] = 0;
		  }
		  else if(RcvDataBuf[0] == 'X')
		  {
			  ExhaustGateOpen();
			  ExhaustOn();
			  RcvDataBuf[Count] = 0;
		  }
		  else if(RcvDataBuf[0] == 'x')
		  {
			  ExhaustGateClose();
			  ExhaustOff();
			  RcvDataBuf[Count] = 0;
		  }
		  else if(RcvDataBuf[0] == 'T')
		  {
			  HX711_Tare(NumOfSamples,DRY_ICE_LOAD);
			  RcvDataBuf[Count] = 0;
		  }
		  else if(RcvDataBuf[0] == 'L')
		  {
			  /*Tare the main reservoir load cell*/
			  HX711_Tare(NumOfSamples,PERFUSION_FLUID_LOAD);
			  RcvDataBuf[Count] = 0;
		  }
		  else if(RcvDataBuf[0] == 'l')
		  {
			  /*Tare the waste reservoir load cell*/
			  HX711_Tare(NumOfSamples,WASTE_FLUID_LOAD);
			  RcvDataBuf[Count] = 0;
		  }
		  else if(RcvDataBuf[0] == 'U')
		  {
			  /*Update the perfusion settings*/
			  ReqPerfusionData = 1;
			  RcvDataBuf[Count] = 0;
		  }
		  else if(PerfusionSettingUpdated == 1)
		  {
			  GetPerfusionData();
			  PerfusionSettingUpdated = 0;
		  }
		  else if(RcvDataBuf[0] == 'P')
		  {
			  /*Start the perfusion at the given rate
			   * start the interval timer here*/
			  StartPerfusion();
			  RcvDataBuf[Count] = 0;
		  }
		  else if(RcvDataBuf[0] == 'p')
		  {
			  /*Stop the perfusion*/
			  StopPerfusion();
			  RcvDataBuf[Count] = 0;
		  }
		  else if(RcvDataBuf[0] == 'M')
		  {
			  /*Start the Motor 1 in forward direction*/
			  ForwardM1(&hroboclaw_mc1,64);
			  RcvDataBuf[Count] = 0;
		  }
		  else if(RcvDataBuf[0] == 'm')
		  {
			  /*Stop the Motor 1*/
			  ForwardM1(&hroboclaw_mc1,0);
			  RcvDataBuf[Count] = 0;
		  }
		  else if(RcvDataBuf[0] == 'S')
		  {
			  /*Start the Motor 2 in forward direction*/
			  ForwardM2(&hroboclaw_mc1,127);
			  RcvDataBuf[Count] = 0;
		  }
		  else if(RcvDataBuf[0] == 's')
		  {
			  /*Stop the Motor 2*/
			  ForwardM2(&hroboclaw_mc1,0);
			  RcvDataBuf[Count] = 0;
		  }
		  else if(RcvDataBuf[0] == 'F')
		  {
			  /*Start the Motor 1 in backward direction*/
			  BackwardM1(&hroboclaw_mc1,64);
			  RcvDataBuf[Count] = 0;
		  }
		  else if(RcvDataBuf[0] == 'f')
		  {
			  /*Stop the Motor 1*/
			  BackwardM1(&hroboclaw_mc1,0);
			  RcvDataBuf[Count] = 0;
		  }
		  else if(RcvDataBuf[0] == 'R')
		  {
			  /*Start the Motor 2 in backward direction*/
			  BackwardM2(&hroboclaw_mc1,127);
			  RcvDataBuf[Count] = 0;
		  }
		  else if(RcvDataBuf[0] == 'r')
		  {
			  /*Stop the Motor 2*/
			  BackwardM2(&hroboclaw_mc1,0);
			  RcvDataBuf[Count] = 0;
		  }
		  else if(RcvDataBuf[0] == 'I')
		  {
			  /*Reset the perfusion data*/
			  ResetPerfusionData();
			  RcvDataBuf[Count] = 0;
		  }
		  else if(RcvDataBuf[0] == 'Z')
		  {
			  RcvDataBuf[Count] = 0;
		  }
		  if(ON == 1)
		  {
			  memset(usr_msg1, 0, sizeof(usr_msg1)); // clear the array
			  sprintf(usr_msg1, "***\n\r");
			  SendToGUI((uint8_t*)usr_msg1);

			  memset(usr_msg1, 0, sizeof(usr_msg1)); // clear the array
			  sprintf(usr_msg1, "%f\n\r", (TemperatureOne));
			  SendToGUI((uint8_t*)usr_msg1);
//			  SendToUSB((uint8_t*)usr_msg1);

			  memset(usr_msg1, 0, sizeof(usr_msg1)); // clear the array
			  sprintf(usr_msg1, "%f\n\r", (TemperatureTwo));
			  SendToGUI((uint8_t*)usr_msg1);
//			  SendToUSB((uint8_t*)usr_msg1);

			  memset(usr_msg1, 0, sizeof(usr_msg1)); // clear the array
			  sprintf(usr_msg1, "%f\n\r", (Temperature));
			  SendToGUI((uint8_t*)usr_msg1);
//			  SendToUSB((uint8_t*)usr_msg1);

			  memset(usr_msg1, 0, sizeof(usr_msg1)); // clear the array
			  sprintf(usr_msg1, "%f\n\r", (HeartTemperature));
			  SendToGUI((uint8_t*)usr_msg1);
//			  SendToUSB((uint8_t*)usr_msg1);

			  memset(usr_msg1, 0, sizeof(usr_msg1)); // clear the array
			  sprintf(usr_msg1, "%d\n\r", (DryIceLoad));
			  SendToGUI((uint8_t*)usr_msg1);
//			  SendToUSB((uint8_t*)usr_msg1);

			  memset(usr_msg1, 0, sizeof(usr_msg1)); // clear the array
			  sprintf(usr_msg1, "%d\n\r", (SalineOneLoad));
			  SendToGUI((uint8_t*)usr_msg1);
//			  SendToUSB((uint8_t*)usr_msg1);

			  memset(usr_msg1, 0, sizeof(usr_msg1)); // clear the array
			  sprintf(usr_msg1, "%d\n\r", (SalineTwoLoad));
			  SendToGUI((uint8_t*)usr_msg1);
//			  SendToUSB((uint8_t*)usr_msg1);

			  memset(usr_msg1, 0, sizeof(usr_msg1)); // clear the array
			  sprintf(usr_msg1, "%f\n\r", (PidOutput));
			  SendToGUI((uint8_t*)usr_msg1);
//			  SendToUSB((uint8_t*)usr_msg1);

			  memset(usr_msg1, 0, sizeof(usr_msg1)); // clear the array
//			  sprintf(usr_msg1, "%d\n\r", (PWM));
			  sprintf(usr_msg1, "%d\n\r", (BlowerFunctionalTime));
			  SendToGUI((uint8_t*)usr_msg1);
//			  SendToUSB((uint8_t*)usr_msg1);

			  memset(usr_msg1, 0, sizeof(usr_msg1)); // clear the array
			  sprintf(usr_msg1, "%d\n\r", (BlowerOnOff));
			  SendToGUI((uint8_t*)usr_msg1);
//			  SendToUSB((uint8_t*)usr_msg1);

			  memset(usr_msg1, 0, sizeof(usr_msg1)); // clear the array
			  sprintf(usr_msg1, "%d\n\r", (ExhaustOnOff));
			  SendToGUI((uint8_t*)usr_msg1);
//			  SendToUSB((uint8_t*)usr_msg1);

			  memset(usr_msg1, 0, sizeof(usr_msg1)); // clear the array
			  sprintf(usr_msg1, "%d\n\r", (ReqPerfusionData));
			  SendToGUI((uint8_t*)usr_msg1);
//			  SendToUSB((uint8_t*)usr_msg1);

			  memset(usr_msg1, 0, sizeof(usr_msg1)); // clear the array
			  sprintf(usr_msg1, "%d\n\r", (ReqPerfusionTime));
			  SendToGUI((uint8_t*)usr_msg1);
//			  SendToUSB((uint8_t*)usr_msg1);

			  memset(usr_msg1, 0, sizeof(usr_msg1)); // clear the array
			  sprintf(usr_msg1, "%ld\n\r", (GsecCounter));
			  SendToGUI((uint8_t*)usr_msg1);
//			  SendToUSB((uint8_t*)usr_msg1);

			  memset(usr_msg1, 0, sizeof(usr_msg1)); // clear the array
			  sprintf(usr_msg1, "%ld\n\r", (SecondCount));
			  SendToGUI((uint8_t*)usr_msg1);
//			  SendToUSB((uint8_t*)usr_msg1);

			  memset(usr_msg1, 0, sizeof(usr_msg1)); // clear the array
			  sprintf(usr_msg1, "%d\n\r", (StartPumpOneMotor));
			  SendToGUI((uint8_t*)usr_msg1);
//			  SendToUSB((uint8_t*)usr_msg1);

			  memset(usr_msg1, 0, sizeof(usr_msg1)); // clear the array
			  sprintf(usr_msg1, "%d\n\r", (StartPumpTwoMotor));
			  SendToGUI((uint8_t*)usr_msg1);
//			  SendToUSB((uint8_t*)usr_msg1);

			  memset(usr_msg1, 0, sizeof(usr_msg1)); // clear the array
			  sprintf(usr_msg1, "%f\n\r", (PressureValue));
			  SendToGUI((uint8_t*)usr_msg1);
//			  SendToUSB((uint8_t*)usr_msg1);

			  memset(usr_msg1, 0, sizeof(usr_msg1)); // clear the array
			  sprintf(usr_msg1, "%f\n\r", (pHValue));
			  SendToGUI((uint8_t*)usr_msg1);
//			  SendToUSB((uint8_t*)usr_msg1);

			  memset(usr_msg1, 0, sizeof(usr_msg1)); // clear the array
			  sprintf(usr_msg1, "%d\n\r", (NumOfPerfusionCycles));
			  SendToGUI((uint8_t*)usr_msg1);
//			  SendToUSB((uint8_t*)usr_msg1);
		  }
		  else
		  {
			  /*Stop all timers and motor here*/
			  HAL_TIM_Base_Stop_IT(&htim6);
			  ForwardM1(&hroboclaw_mc1,0);
			  ForwardM2(&hroboclaw_mc1,0);
			  StopPumpOneMotor = 0;
			  StartPumpOneMotor = 0;
			  StopPumpTwoMotor = 0;
			  StartPumpTwoMotor = 0;
		  }
		  if(StopPumpOneMotor == 1)
		  {
			  ForwardM1(&hroboclaw_mc1,0);
		  }
		  if(StartPumpOneMotor == 1)
		  {
			  ForwardM1(&hroboclaw_mc1,MainPumpSpeedValue);
		  }
		  if(StopPumpTwoMotor == 1)
		  {
			  ForwardM2(&hroboclaw_mc1,0);
		  }
		  if(StartPumpTwoMotor == 1)
		  {
			  ForwardM2(&hroboclaw_mc1,127);//max speed
		  }
	  }


    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2|RCC_PERIPHCLK_USART3
                              |RCC_PERIPHCLK_UART4|RCC_PERIPHCLK_UART5
                              |RCC_PERIPHCLK_I2C1|RCC_PERIPHCLK_I2C2
                              |RCC_PERIPHCLK_I2C3;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.Usart3ClockSelection = RCC_USART3CLKSOURCE_PCLK1;
  PeriphClkInit.Uart4ClockSelection = RCC_UART4CLKSOURCE_PCLK1;
  PeriphClkInit.Uart5ClockSelection = RCC_UART5CLKSOURCE_PCLK1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
  PeriphClkInit.I2c2ClockSelection = RCC_I2C2CLKSOURCE_PCLK1;
  PeriphClkInit.I2c3ClockSelection = RCC_I2C3CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure the main internal regulator output voltage 
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x10909CEC;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Analogue filter 
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Digital filter 
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.Timing = 0x10909CEC;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Analogue filter 
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Digital filter 
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief I2C3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C3_Init(void)
{

  /* USER CODE BEGIN I2C3_Init 0 */

  /* USER CODE END I2C3_Init 0 */

  /* USER CODE BEGIN I2C3_Init 1 */

  /* USER CODE END I2C3_Init 1 */
  hi2c3.Instance = I2C3;
  hi2c3.Init.Timing = 0x10909CEC;
  hi2c3.Init.OwnAddress1 = 0;
  hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c3.Init.OwnAddress2 = 0;
  hi2c3.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c3) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Analogue filter 
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c3, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Digital filter 
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c3, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C3_Init 2 */

  /* USER CODE END I2C3_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi2.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 7;
  hspi2.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi2.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 40000;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 127;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 62500-1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 31250;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.Pulse = 0;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 63;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 25000-1;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 255;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 312500-1;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 2047;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 39062;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief UART4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART4_Init(void)
{

  /* USER CODE BEGIN UART4_Init 0 */

  /* USER CODE END UART4_Init 0 */

  /* USER CODE BEGIN UART4_Init 1 */

  /* USER CODE END UART4_Init 1 */
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 115200;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  huart4.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart4.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */

  /* USER CODE END UART4_Init 2 */

}

/**
  * @brief UART5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART5_Init(void)
{

  /* USER CODE BEGIN UART5_Init 0 */

  /* USER CODE END UART5_Init 0 */

  /* USER CODE BEGIN UART5_Init 1 */

  /* USER CODE END UART5_Init 1 */
  huart5.Instance = UART5;
  huart5.Init.BaudRate = 38400;
  huart5.Init.WordLength = UART_WORDLENGTH_8B;
  huart5.Init.StopBits = UART_STOPBITS_1;
  huart5.Init.Parity = UART_PARITY_NONE;
  huart5.Init.Mode = UART_MODE_TX_RX;
  huart5.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart5.Init.OverSampling = UART_OVERSAMPLING_16;
  huart5.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart5.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart5) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART5_Init 2 */

  /* USER CODE END UART5_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Channel1_IRQn);
  /* DMA2_Channel2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Channel2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Channel2_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(AD7193_CS_PIN_GPIO_Port, AD7193_CS_PIN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8|GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : AD7193_CS_PIN_Pin */
  GPIO_InitStruct.Pin = AD7193_CS_PIN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(AD7193_CS_PIN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PC8 PC9 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void ResetVariables()
{
	  ON = 0;
	  OFF = 0;
	  PWM = 0;
	  BlowerFunctionalTime = 0;
	  LastPWM = 0;
}
void InitializeLoadCell()
{
	if(DWT_Delay_Init()==1)
	  Error_Handler();
	HX711_init();
	CalibrationValue = 100;
	HX711_set_scale(CalibrationValue,DRY_ICE_LOAD);
	HX711_set_scale(CalibrationValue,PERFUSION_FLUID_LOAD);
	HX711_set_scale(CalibrationValue,WASTE_FLUID_LOAD);
	DryIceLoad = HX711_valueUnits(NumOfSamples,DRY_ICE_LOAD);
	SalineOneLoad = HX711_valueUnits(NumOfSamples,PERFUSION_FLUID_LOAD);
	SalineTwoLoad = HX711_valueUnits(NumOfSamples,WASTE_FLUID_LOAD);
	for(uint8_t i = 0; i<3;i++){
	  for(uint8_t j = 0;j<5; j++){
		  HX711_Tare(NumOfSamples,i);
		  HAL_Delay(50);
		}
	}
	/*for(uint8_t j = 0;j<5; j++){
	  HX711_Tare(NumOfSamples,DRY_ICE_LOAD);
	  HAL_Delay(50);
	}
	for(uint8_t j = 0;j<5; j++){
	  HX711_Tare(NumOfSamples,PERFUSION_FLUID_LOAD);
	  HAL_Delay(50);
	}
	for(uint8_t j = 0;j<5; j++){
	  HX711_Tare(NumOfSamples,WASTE_FLUID_LOAD);
	  HAL_Delay(50);
	}*/
	DryIceLoad = HX711_valueUnits(NumOfSamples,DRY_ICE_LOAD);
	SalineOneLoad = HX711_valueUnits(NumOfSamples,PERFUSION_FLUID_LOAD);
	SalineTwoLoad = HX711_valueUnits(NumOfSamples,WASTE_FLUID_LOAD);
}

void InitializeTemperatureSensor()
{
	  SetTempSensorResolution(COOLING_TEMPERATURE_SENSOR_ONE,FIFTEEN_SAMPLES_PER_SEC);
	  SetTempSensorResolution(COOLING_TEMPERATURE_SENSOR_TWO,FIFTEEN_SAMPLES_PER_SEC);
}
void InitializeRoboClaw()
{
	  /* initialize serial communication for roboClaw controller*/
	  hserial_uart5 = serial_init(&huart5);
	  /* Initialize roboClaw */
	  hroboclaw_mc1.hserial = hserial_uart5;
	  hroboclaw_mc1.packetserial_address = 0x80;
	  if (roboClaw_init(&hroboclaw_mc1) != ROBOCLAW_OK)
	  {
		  Error_Handler();
	  }
}
void InitializeServo()
{
	  /*to run Servo Motor un-comment the below Lines of code*/
	  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
}
void InitializePressureSensor()
{
	   // Device setup///////////////////////////////////
	   //This will append status bits onto end of data - is required for library to work properly
	   AD7193AppendStatusValuetoData();
	   // Set the gain of the PGA
	   AD7193SetPGAGain(128);
	   // Set the Averaging
	   AD7193SetAveraging(100);
	   /////////////////////////////////////
	   // Calibrate with given PGA settings - need to recalibrate if PGA setting is changed
	   AD7193ReadRegisterMap();
	   AD7193Calibrate();
	   // Debug - Check register map values
	   AD7193ReadRegisterMap();
	   PressureLimits.InputMax = 26300;//16300;
	   PressureLimits.InputMin = 19000;//9650;
	   PressureLimits.OutputMax = -600;
	   PressureLimits.OutputMin = 0;
	   //////////////////////////////////////
}
void LoadDefaultSettings()
{
//	  ExhaustGateClose();
	  ForwardM1(&hroboclaw_mc1,0);
	  ForwardM2(&hroboclaw_mc1,0);
}
void SendToGUI(uint8_t *pData)
{
	  if(HAL_UART_Transmit(&huart4, (uint8_t*)usr_msg1, strlen(usr_msg1), 100)!= HAL_OK)
	  {
		Error_Handler();
	  }
}
void SendToUSB(uint8_t *pData)
{
	if(HAL_UART_Transmit(&huart2, (uint8_t*)usr_msg1, sizeof(usr_msg1), 100)!= HAL_OK)
	{
		Error_Handler();
	}
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(char *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
