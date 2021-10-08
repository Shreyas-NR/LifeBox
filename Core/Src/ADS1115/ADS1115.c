/*
 * ADS1115.c
 *
 *  Created on: 13-Aug-2020
 *      Author: shreyasnr
 */


#include <ADS1115/ADS1115.h>
char usr_msg[50];
uint8_t ConfigRegValue[3] = {0,0,0};
uint8_t ConvRegValue[2] = {0,0};
uint16_t ConversionValue = 0;
const float VoltageConv = 6.144 / 32768.0;
//const float VoltageConv = 5 / 16384.0;
float Volt = 0;
uint16_t DigitalReading = 0;
float AnalogValue = 0;
int Slope_ph = -6;
float yIntercept_ph = 16.36;

float ReadPHSensor()
{
	float temp = 0;
	DigitalReading = readADC_SingleEnded(1);
	AnalogValue = DigitalReading * VoltageConv;
	temp = Slope_ph * AnalogValue + yIntercept_ph;
//	return AnalogValue;
	return (temp);
}
void ReadFlowSensor()
{
	sprintf(usr_msg, "\n\tWORK IN PROGRESS\n\r");
	if(HAL_UART_Transmit(&huart2, (uint8_t*)usr_msg, sizeof(usr_msg), HAL_MAX_DELAY)!= HAL_OK)
	{
		Error_Handler();
	}
	if(POWER_ON_SELF_TEST == 1)
		MenuDisplay();
}
uint16_t readADC_SingleEnded(uint8_t channel)
{
	if (channel > 3) {
	    return 0;
	  }

	  // Start with default values
	  uint16_t config =
	      ADS1115_REG_CONFIG_CQUE_NONE |    // Disable the comparator (default val)
	      ADS1115_REG_CONFIG_CLAT_NONLAT |  // Non-latching (default val)
	      ADS1115_REG_CONFIG_CPOL_ACTVLOW | // Alert/Rdy active low   (default val)
	      ADS1115_REG_CONFIG_CMODE_TRAD |   // Traditional comparator (default val)
	      ADS1115_REG_CONFIG_DR_1600SPS |   // 1600 samples per second (default)
		  ADS1115_REG_CONFIG_MODE_SINGLE;   // Single-shot mode (default)

	  // Set PGA/voltage range
	  config |= GAIN_TWOTHIRDS;

	  // Set single-ended input channel
	  switch (channel) {
	  case (0):
	    config |= ADS1115_REG_CONFIG_MUX_SINGLE_0;
	    break;
	  case (1):
	    config |= ADS1115_REG_CONFIG_MUX_SINGLE_1;
	    break;
	  case (2):
	    config |= ADS1115_REG_CONFIG_MUX_SINGLE_2;
	    break;
	  case (3):
	    config |= ADS1115_REG_CONFIG_MUX_SINGLE_3;
	    break;
	  }

	  // Set 'start single-conversion' bit
	  config |= ADS1115_REG_CONFIG_OS_SINGLE;

	  ConfigRegValue[0] = ADS1115_REG_POINTER_CONFIG;
	  ConfigRegValue[1] = ((config & 0xFF00)>>8);
	  ConfigRegValue[2] = config & 0x00FF;

	  // Write config register to the ADC
	  HAL_I2C_Master_Transmit(&hi2c3, ADS1115_ADDRESS,ConfigRegValue,3, 100);
	  // Write Address Pointer register to change the Pointer register to Conversion register of ADC
	  HAL_I2C_Master_Transmit(&hi2c3, ADS1115_ADDRESS,ADS1115_REG_POINTER_CONVERT,1, 100);
	  // Wait for the conversion to complete
	  HAL_Delay(ADS1115_CONVERSIONDELAY);
	  // Read the conversion results
	  HAL_I2C_Master_Receive (&hi2c3, ADS1115_ADDRESS,ConvRegValue,2, 100);
	  ConversionValue = (ConvRegValue[0] << 8 | ConvRegValue[1]);
	  Volt = ConversionValue * VoltageConv;
	  return ConversionValue;
}
