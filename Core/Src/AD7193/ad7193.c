/*
 * ad7193.c
 *
 *  Created on: Feb 9, 2021
 *      Author: shreyasnr
 */


#include "AD7193/AD7193.h"


// default register settings
unsigned long registerMap[4] = {
  0x00,
  0x080060,
  0x000117,
  0x000000
};

int registerSize[8] = {
  1,
  3,
  3,
  3,
  1,
  1,
  3,
  3
};
void AD7193Reset(void)
{
#if DEBUG_PERFUSION
	memset(usr_msg1, 0, sizeof(usr_msg1));
	sprintf(usr_msg1, "\nReset AD7193...");
	SendToUSB((uint8_t*)usr_msg1);
#endif

	HAL_GPIO_WritePin(AD7193_CS_PIN_GPIO_Port, AD7193_CS_PIN_Pin, GPIO_PIN_RESET);
	HAL_Delay(100);
	char i = 0;
	uint8_t ResetData= 0xFF;
	for(i = 0; i < 6; i++)
	{
//		SPI.transfer(0xFF);
		if(HAL_SPI_TransmitReceive(&hspi2, &ResetData, &ResetData, sizeof(ResetData), HAL_MAX_DELAY) != HAL_OK)
		{
			Error_Handler();
		}
	}
	HAL_GPIO_WritePin(AD7193_CS_PIN_GPIO_Port, AD7193_CS_PIN_Pin, GPIO_PIN_SET);
	HAL_Delay(100);
}

void AD7193SetPGAGain(int gain)
{
#if DEBUG_PERFUSION
	memset(usr_msg1, 0, sizeof(usr_msg1));
	sprintf(usr_msg1, "\nSetting PGA Gain to %d",gain);
	SendToUSB((uint8_t*)usr_msg1);
#endif

  unsigned long gainSetting;

  if(gain == 1)         {gainSetting = 0x0;}
  else if (gain == 8)   {gainSetting = 0x3;}
  else if (gain == 16)  {gainSetting = 0x4;}
  else if (gain == 32)  {gainSetting = 0x5;}
  else if (gain == 64)  {gainSetting = 0x6;}
  else if (gain == 128) {gainSetting = 0x7;}
  else
  {
	#if DEBUG_PERFUSION
		memset(usr_msg1, 0, sizeof(usr_msg1));
		sprintf(usr_msg1, "\n\tERROR - Invalid Gain Setting - no changes made.  Valid Gain settings are 1, 8, 16, 32, 64, 128");
		SendToUSB((uint8_t*)usr_msg1);
	#endif
	  return;
  }

  int regAddress = AD7193_REG_CONF;

  registerMap[regAddress] &= 0xFFFFF8; //keep all bit values except gain bits
  registerMap[regAddress] |= gainSetting;

  AD7193SetRegisterValue(regAddress, registerMap[regAddress], registerSize[regAddress], 1);
}
void AD7193SetAveraging(int filterRate)  {

#if DEBUG_PERFUSION
	memset(usr_msg1, 0, sizeof(usr_msg1));
	sprintf(usr_msg1, "\nSetting Filter Rate Select Bits to %d",filterRate);
	SendToUSB((uint8_t*)usr_msg1);
#endif

  if(filterRate > 0x3ff)
  {
	#if DEBUG_PERFUSION
	  memset(usr_msg1, 0, sizeof(usr_msg1));
	  sprintf(usr_msg1, "\n\tERROR - Invalid Filter Rate Setting - no changes made.  Filter Rate is a 10-bit value");
		SendToUSB((uint8_t*)usr_msg1);
	#endif

    return;
  }

  registerMap[1] &= 0xFFFC00; //keep all bit values except filter setting bits
  registerMap[1] |= filterRate;

  AD7193SetRegisterValue(1, registerMap[1], registerSize[1], 1);

}

void AD7193ReadRegisterMap(void)
{
	#if DEBUG_PERFUSION
		memset(usr_msg1, 0, sizeof(usr_msg1));
		sprintf(usr_msg1, "\nRead All Register Values (helpful for troubleshooting)");
		SendToUSB((uint8_t*)usr_msg1);
	#endif
  AD7193GetRegisterValue(0, 1, 1);
  AD7193GetRegisterValue(1, 3, 1);
  AD7193GetRegisterValue(2, 3, 1);
  AD7193GetRegisterValue(3, registerSize[3], 1);
  AD7193GetRegisterValue(4, 1, 1);
  AD7193GetRegisterValue(6, 3, 1);
  AD7193GetRegisterValue(7, 3, 1);
  HAL_Delay(100);
}
void AD7193SetPsuedoDifferentialInputs(void)  {
#if DEBUG_PERFUSION
	memset(usr_msg1, 0, sizeof(usr_msg1));
	sprintf(usr_msg1, "\nSwitching from differential input to pseudo differential inputs...");
	SendToUSB((uint8_t*)usr_msg1);
#endif

//  unsigned long psuedoBit = 0x040000;
  registerMap[2] &= 0xFBFFFF;
  registerMap[2] |= 0x040000;

  AD7193SetRegisterValue(2, registerMap[2], registerSize[2], 1);

  //Serial.print(" - on next register refresh, new Config Reg value will be: ");
  //Serial.println(registerMap[2], HEX);
}
void AD7193AppendStatusValuetoData(void) {
#if DEBUG_PERFUSION
	memset(usr_msg1, 0, sizeof(usr_msg1));
	sprintf(usr_msg1, "\nEnabling DAT_STA Bit (appends status register to data register when reading)");
	SendToUSB((uint8_t*)usr_msg1);
#endif


  registerMap[1] &= 0xEFFFFF; //keep all bit values except DAT_STA bit
  registerMap[1] |= 0x100000;  // set DAT_STA to 1

  AD7193SetRegisterValue(1, registerMap[1], registerSize[1], 1);

  //Serial.print(" - New Mode Reg Value: ");
  //Serial.println(registerMap[1], HEX);

  registerSize[3] = 4; // change register size to 4, b/c status register is now appended
}
void AD7193Calibrate(void) {
#if DEBUG_PERFUSION
	memset(usr_msg1, 0, sizeof(usr_msg1));
	sprintf(usr_msg1, "\nInitiate Internal Calibration, starting with Zero-scale calibration...");
	SendToUSB((uint8_t*)usr_msg1);
#endif

  // Begin Communication cycle, bring CS low manually
  HAL_GPIO_WritePin(AD7193_CS_PIN_GPIO_Port, AD7193_CS_PIN_Pin, GPIO_PIN_RESET);
  HAL_Delay(100);

  registerMap[1] &= 0x1FFFFF; //keep all bit values except Channel bits
  registerMap[1] |= 0x800000; // internal zero scale calibration

  AD7193SetRegisterValue(1, registerMap[1], 3, 0);  // overwriting previous MODE reg setting

  AD7193WaitForADC();
  //HAL_Delay(100);

#if DEBUG_PERFUSION
  memset(usr_msg1, 0, sizeof(usr_msg1));
    sprintf(usr_msg1, "\n\nNow full-scale calibration...");
	SendToUSB((uint8_t*)usr_msg1);
#endif

  registerMap[1] &= 0x1FFFFF; //keep all bit values except Channel bits
  registerMap[1] |= 0xA00000; // internal full scale calibration

  AD7193SetRegisterValue(1, registerMap[1], 3, 0);  // overwriting previous MODE reg setting

  AD7193WaitForADC();
  //delay(100);

  HAL_GPIO_WritePin(AD7193_CS_PIN_GPIO_Port, AD7193_CS_PIN_Pin, GPIO_PIN_SET);
  HAL_Delay(100);
}
void AD7193WaitForADC(void)  {
    int breakTime = 0;

	#if DEBUG_PERFUSION
		memset(usr_msg1, 0, sizeof(usr_msg1));
		sprintf(usr_msg1, "\nWaiting for Conversion");
		SendToUSB((uint8_t*)usr_msg1);
	#endif
    while(1){
//      if (digitalRead(AD7193_RDY_STATE) == 0){      // Break if ready goes low
    	if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_14) == 0){      // Break if ready goes low
        break;
      }

      if (breakTime > 5000)
      {                       // Break after five seconds - avoids program hanging up
		#if DEBUG_PERFUSION
				  memset(usr_msg1, 0, sizeof(usr_msg1));
				  sprintf(usr_msg1, "Data Ready never went low!");
					SendToUSB((uint8_t*)usr_msg1);
		#endif
        break;
      }

//      if (digitalRead(AD7193_RDY_STATE)) {Serial.print(".");}
      if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_14))
      {
		#if DEBUG_PERFUSION
    	  memset(usr_msg1, 0, sizeof(usr_msg1));
		  sprintf(usr_msg1, ".");
			SendToUSB((uint8_t*)usr_msg1);
		#endif
      }
      HAL_Delay(1);
      breakTime = breakTime + 1;
    }
}
void AD7193IntitiateSingleConversion(void) {
  //Serial.print("    Initiate Single Conversion... (Device will go into low pwer mode when conversion complete)");

  // Begin Communication cycle, bring CS low manually
  HAL_GPIO_WritePin(AD7193_CS_PIN_GPIO_Port, AD7193_CS_PIN_Pin, GPIO_PIN_RESET);
  HAL_Delay(100);

  registerMap[1] &= 0x1FFFFF; //keep all bit values except Channel bits
  registerMap[1] |= 0x200000; // single conversion mode bits

  AD7193SetRegisterValue(1, registerMap[1], 3, 0);  // overwriting previous MODE reg setting
}

unsigned long AD7193ReadADCData(void)  {

    unsigned char byteIndex = 0;
    unsigned long buffer = 0;
    unsigned char receiveBuffer = 0;
    unsigned char dataLength = registerSize[3];  // data length depends on if Status register is appended to Data read - see AppendStatusValuetoData()
    uint8_t StartReadData = 0x58;
    uint8_t ZeroData = 0;

//    SPI.transfer(0x58);  // command to start read data
	if(HAL_SPI_TransmitReceive(&hspi2, &StartReadData, &StartReadData, sizeof(StartReadData), HAL_MAX_DELAY) != HAL_OK)
	{
		Error_Handler();
	}
    while(byteIndex < dataLength)
    {
//      receiveBuffer = SPI.transfer(0);
//      receiveBuffer = HAL_SPI_Transmit(&hspi2, &(ZeroData), sizeof(uint8_t), HAL_MAX_DELAY);
      HAL_SPI_TransmitReceive(&hspi2, &(ZeroData), &(receiveBuffer), sizeof(uint8_t), HAL_MAX_DELAY);
      buffer = (buffer << 8) + receiveBuffer;
      byteIndex++;
    }

    return(buffer);
}


void AD7193SetChannel(int channel) {

    // generate Channel settings bits for Configuration write
    unsigned long shiftvalue = 0x00000100;
    unsigned long channelBits = shiftvalue << channel;

    // Write Channel bits to Config register, keeping other bits as is
    registerMap[2] &= 0xFC00FF; //keep all bit values except Channel bits
    registerMap[2] |= channelBits;

    // write channel selected to Configuration register
    AD7193SetRegisterValue(2, registerMap[2], registerSize[2], 1);
    HAL_Delay(10);
}

unsigned long AD7193ReadADCChannel(int channel)  {

	AD7193SetChannel(channel);

    // write command to initial conversion
    AD7193IntitiateSingleConversion();
    //delay(100); // hardcoded wait time for data to be ready
    // should scale the wait time by averaging

    AD7193WaitForADC();

    unsigned long ADCdata = AD7193ReadADCData();
    HAL_Delay(10);

    // end communication cycle, bringing CS pin High manually
    HAL_GPIO_WritePin(AD7193_CS_PIN_GPIO_Port, AD7193_CS_PIN_Pin, GPIO_PIN_SET);
    HAL_Delay(10);

    return(ADCdata);
}



float AD7193DataToVoltage(long rawData)  {
  float voltage = 0;
//  char mGain = 0;
  float mVref = 2.5;
  char mPolarity = 0;

  int PGASetting = registerMap[2] & 0x000007;  // keep only the PGA setting bits
  int PGAGain;

  if (PGASetting == 0) {
    PGAGain = 1;
  } else if (PGASetting == 3) {
    PGAGain = 8;
  } else if (PGASetting == 4) {
    PGAGain = 16;
  } else if (PGASetting == 5) {
    PGAGain = 32;
  } else if (PGASetting == 6) {
    PGAGain = 64;
  } else if (PGASetting == 7) {
    PGAGain = 128;
  } else {
    PGAGain = 1;
  }


  //Serial.print("PGA Gain = ");
  //Serial.println(PGAGain);


  if(mPolarity == 1)
  {
    voltage = ((double)rawData / 16777216 / (1 << PGAGain)) * mVref;
  }
  if(mPolarity == 0)
  {
    voltage = (((float)rawData / (float)8388608) - (float)1) * (mVref / (float)PGAGain);
//    voltage = (((float)rawData / (1ul << 23)) - 1) * (mVref / PGAGain);
  }


  return(voltage);
}

// See "Temperature Sensor" section of AD7193 Datasheet - page 39
float AD7193TempSensorDataToDegC(unsigned long rawData)  {
        float degC = ((rawData - 0x800000) / 2815.0) - 273;
//        float degF = (degC * 9 / 5) + 32;
        /*Serial.print(degC);
        Serial.print(" degC, ");
        Serial.print(degF);
        Serial.print(" degF\t");*/
        return(degC);
}

/*! Writes data into a register. */
void AD7193SetRegisterValue(unsigned char registerAddress,  unsigned long registerValue,  unsigned char bytesNumber,  unsigned char modifyCS)  {//setregistervalue
    unsigned char commandByte = 0;
    unsigned char txBuffer[4] = {0, 0, 0 ,0};

    commandByte = AD7193_COMM_WRITE | AD7193_COMM_ADDR(registerAddress);

    txBuffer[0] = (registerValue >> 0)  & 0x000000FF;
    txBuffer[1] = (registerValue >> 8)  & 0x000000FF;
    txBuffer[2] = (registerValue >> 16) & 0x000000FF;
    txBuffer[3] = (registerValue >> 24) & 0x000000FF;
    if(modifyCS == 1)
    {
      HAL_GPIO_WritePin(AD7193_CS_PIN_GPIO_Port, AD7193_CS_PIN_Pin, GPIO_PIN_RESET);
    }

//    SPI.transfer(commandByte);
	if(HAL_SPI_TransmitReceive(&hspi2, &commandByte, &commandByte, sizeof(commandByte), HAL_MAX_DELAY) != HAL_OK)
	{
		Error_Handler();
	}
    while(bytesNumber > 0)
    {
//        SPI.transfer(txBuffer[bytesNumber - 1]);
    	if(HAL_SPI_TransmitReceive(&hspi2, &(txBuffer[bytesNumber - 1]), &(txBuffer[bytesNumber - 1]), sizeof(txBuffer[bytesNumber - 1]), HAL_MAX_DELAY) != HAL_OK)
    	{
    		Error_Handler();
    	}
    	bytesNumber--;
    }
    if(modifyCS == 1)
    {
      HAL_GPIO_WritePin(AD7193_CS_PIN_GPIO_Port, AD7193_CS_PIN_Pin, GPIO_PIN_SET);
    }

    /*Serial.print("    Write Register Address: ");
    Serial.print(registerAddress, HEX);
    Serial.print(", command: ");
    Serial.print(commandByte, HEX);
    Serial.print(",     sent: ");
    Serial.println(registerValue, HEX);*/
}
/*! Reads the value of a register. */
unsigned long AD7193GetRegisterValue(unsigned char registerAddress, unsigned char bytesNumber, unsigned char modifyCS)  {//getregistervalue

    unsigned char receiveBuffer = 0;
    unsigned char writeByte = 0;
    unsigned char byteIndex = 0;
    unsigned long buffer = 0;
    uint8_t ZeroData = 0;
    unsigned char rxdByte = 0;

    writeByte = AD7193_COMM_READ | AD7193_COMM_ADDR(registerAddress);
    if(modifyCS == 1)
    {
      HAL_GPIO_WritePin(AD7193_CS_PIN_GPIO_Port, AD7193_CS_PIN_Pin, GPIO_PIN_RESET);
    }

//    SPI.transfer(writeByte);
	if(HAL_SPI_TransmitReceive(&hspi2, &(writeByte), &(rxdByte), sizeof(writeByte), HAL_MAX_DELAY) != HAL_OK)
	{
		Error_Handler();
	}
    while(byteIndex < bytesNumber)
    {
//        receiveBuffer = SPI.transfer(0);
        HAL_SPI_TransmitReceive(&hspi2, &(ZeroData), &(receiveBuffer), sizeof(uint8_t), HAL_MAX_DELAY);
        buffer = (buffer << 8) + receiveBuffer;
        byteIndex++;
    }

    if(modifyCS == 1)
    {
      HAL_GPIO_WritePin(AD7193_CS_PIN_GPIO_Port, AD7193_CS_PIN_Pin, GPIO_PIN_SET);
    }

//    char str[32];
//    sprintf(str, "%06x", buffer);

	#if DEBUG_PERFUSION
		memset(usr_msg1, 0, sizeof(usr_msg1));
		sprintf(usr_msg1, "\nRead Register Address: %X", registerAddress);
		SendToUSB((uint8_t*)usr_msg1);
	#endif
	#if DEBUG_PERFUSION
		memset(usr_msg1, 0, sizeof(usr_msg1));
		sprintf(usr_msg1, ", command: %X", writeByte);
		SendToUSB((uint8_t*)usr_msg1);
	#endif
	#if DEBUG_PERFUSION
		memset(usr_msg1, 0, sizeof(usr_msg1));
		sprintf(usr_msg1, ", recieved: %lX", buffer);
		SendToUSB((uint8_t*)usr_msg1);
	#endif
    //Serial.print(" - ");
    //Serial.println(str);
    return(buffer);
}
float CalculatePressure(Pressure_t* PressureVal, float PressureSensorInput)
{
	int16_t InMax = PressureVal->InputMax;
	int16_t InMin = PressureVal->InputMin;
	int16_t OutMax = PressureVal->OutputMax;
	int16_t OutMin = PressureVal->OutputMin;
	float PressureOut = 0;
	PressureOut = (((PressureSensorInput - InMin)*(OutMax - OutMin)/(InMax - InMin)+OutMin));
	return PressureOut;
}
