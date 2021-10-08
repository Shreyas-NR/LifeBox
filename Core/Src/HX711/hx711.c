/*
 * hx711.c
 *
 *  Created on: Aug 12, 2020
 *      Author: shreyasnr
 */

#include "HX711/hx711.h"


#define HX711_delay(x)    HAL_Delay(x)
int OFFSET[3] = {0,0,0},SCALE[3] = {0,0,0};
//float DryIceSlope = 0.47671778;
//float DryIceIntercept = 0.388094;
float DryIceSlope = 0.469314079;//0.476009139;//0.5636;
float DryIceIntercept = 112.0469314;//-80.33891851;//-14.909;
float SalineOneSlope = 0.503174603;
float SalineOneIntercept = -6.519047619;
float SalineTwoSlope = 0.470326409;
float SalineTwoIntercept = -3.5;

__STATIC_INLINE void HX711_delay_us(uint32_t microseconds)
{
  uint32_t clk_cycle_start = DWT->CYCCNT;
  microseconds *= (HAL_RCC_GetHCLKFreq() / 1000000);
  while ((DWT->CYCCNT - clk_cycle_start) < microseconds);
}

void  HX711_init(void)
{
//  GPIO_InitTypeDef  gpio;
  GPIO_InitTypeDef gpio = {0};
  gpio.Mode = GPIO_MODE_OUTPUT_PP;
  gpio.Pull = GPIO_PULLUP;
  gpio.Speed = GPIO_SPEED_FREQ_LOW;
  gpio.Pin = _HX711_ONE_SCK_PIN;
  HAL_GPIO_Init(_HX711_ONE_SCK_GPIO, &gpio);
  gpio.Mode = GPIO_MODE_INPUT;
  gpio.Pull = GPIO_PULLUP;
  gpio.Speed = GPIO_SPEED_FREQ_LOW;
  gpio.Pin = _HX711_ONE_DI_PIN;
  HAL_GPIO_Init(_HX711_ONE_DI_GPIO, &gpio);
  HAL_GPIO_WritePin(_HX711_ONE_SCK_GPIO, _HX711_ONE_SCK_PIN, GPIO_PIN_SET);
  HX711_delay(10);
  HAL_GPIO_WritePin(_HX711_ONE_SCK_GPIO, _HX711_ONE_SCK_PIN, GPIO_PIN_RESET);
  HX711_delay(10);

  gpio.Mode = GPIO_MODE_OUTPUT_PP;
  gpio.Pull = GPIO_PULLUP;
  gpio.Speed = GPIO_SPEED_FREQ_LOW;
  gpio.Pin = _HX711_TWO_SCK_PIN;
  HAL_GPIO_Init(_HX711_TWO_SCK_GPIO, &gpio);
  gpio.Mode = GPIO_MODE_INPUT;
  gpio.Pull = GPIO_PULLUP;
  gpio.Speed = GPIO_SPEED_FREQ_LOW;
  gpio.Pin = _HX711_TWO_DI_PIN;
  HAL_GPIO_Init(_HX711_TWO_DI_GPIO, &gpio);
  HAL_GPIO_WritePin(_HX711_TWO_SCK_GPIO, _HX711_TWO_SCK_PIN, GPIO_PIN_SET);
  HX711_delay(10);
  HAL_GPIO_WritePin(_HX711_TWO_SCK_GPIO, _HX711_TWO_SCK_PIN, GPIO_PIN_RESET);
  HX711_delay(10);

  gpio.Mode = GPIO_MODE_OUTPUT_PP;
  gpio.Pull = GPIO_PULLUP;
  gpio.Speed = GPIO_SPEED_FREQ_LOW;
  gpio.Pin = _HX711_THREE_SCK_PIN;
  HAL_GPIO_Init(_HX711_THREE_SCK_GPIO, &gpio);
  gpio.Mode = GPIO_MODE_INPUT;
  gpio.Pull = GPIO_PULLUP;
  gpio.Speed = GPIO_SPEED_FREQ_LOW;
  gpio.Pin = _HX711_THREE_DI_PIN;
  HAL_GPIO_Init(_HX711_THREE_DI_GPIO, &gpio);
  HAL_GPIO_WritePin(_HX711_THREE_SCK_GPIO, _HX711_THREE_SCK_PIN, GPIO_PIN_SET);
  HX711_delay(10);
  HAL_GPIO_WritePin(_HX711_THREE_SCK_GPIO, _HX711_THREE_SCK_PIN, GPIO_PIN_RESET);
  HX711_delay(10);
}

/*
int HX711_value(void)
{
  uint32_t  data = 0;
//  uint8_t isNegative = 0;
  uint32_t  startTime = HAL_GetTick();
//  HAL_GPIO_WritePin(_HX711_SCK_GPIO, _HX711_SCK_PIN, GPIO_PIN_RESET);

  while(HAL_GPIO_ReadPin(_HX711_ONE_DI_GPIO, _HX711_ONE_DI_PIN) == GPIO_PIN_SET)
  {
    if(HAL_GetTick() - startTime > 300)
      return 0;
  }
  for(uint8_t i = 0; i < 24 ; i++)
  {
    HAL_GPIO_WritePin(_HX711_ONE_SCK_GPIO, _HX711_ONE_SCK_PIN, GPIO_PIN_SET);
//    HX711_delay_us(1);
    data = data << 1;
    if(HAL_GPIO_ReadPin(_HX711_ONE_DI_GPIO, _HX711_ONE_DI_PIN) == GPIO_PIN_SET)
    {
//    	if(i == 0)
//    	{
//    		isNegative = 1;
//    	}
    	data++;
    }
    HAL_GPIO_WritePin(_HX711_ONE_SCK_GPIO, _HX711_ONE_SCK_PIN, GPIO_PIN_RESET);

//    HX711_delay_us(1);

  }
//  if(isNegative == 1)
//  {
//	  data = data ^ 0xFF000000;
//  }
//  HAL_GPIO_WritePin(_HX711_SCK_GPIO, _HX711_SCK_PIN, GPIO_PIN_SET);
//  HX711_delay_us(1);
//  data = data ^ 0x800000;
//  HAL_GPIO_WritePin(_HX711_SCK_GPIO, _HX711_SCK_PIN, GPIO_PIN_RESET);
//  HX711_delay_us(1);
//	for (int i = 0; i < 128; i++) {
		HAL_GPIO_WritePin(_HX711_ONE_SCK_GPIO, _HX711_ONE_SCK_PIN, GPIO_PIN_SET);
		HAL_GPIO_WritePin(_HX711_ONE_SCK_GPIO, _HX711_ONE_SCK_PIN, GPIO_PIN_RESET);
//	}
  return data;
}
*/

int HX711_value(uint8_t LoadCellNum)
{
	uint32_t  data = 0;
//	uint32_t  startTime = HAL_GetTick();

	switch(LoadCellNum)
	{
		case 0:
			__disable_irq();
			while(HAL_GPIO_ReadPin(_HX711_ONE_DI_GPIO, _HX711_ONE_DI_PIN) == GPIO_PIN_SET);
//			{
//				if(HAL_GetTick() - startTime > 300)
//				return 0;
//			}
			for(uint8_t i = 0; i < 24 ; i++)
			{
				HAL_GPIO_WritePin(_HX711_ONE_SCK_GPIO, _HX711_ONE_SCK_PIN, GPIO_PIN_SET);
				data = data << 1;
				if(HAL_GPIO_ReadPin(_HX711_ONE_DI_GPIO, _HX711_ONE_DI_PIN) == GPIO_PIN_SET)
				{
					data++;
				}
				HAL_GPIO_WritePin(_HX711_ONE_SCK_GPIO, _HX711_ONE_SCK_PIN, GPIO_PIN_RESET);
			}
			HAL_GPIO_WritePin(_HX711_ONE_SCK_GPIO, _HX711_ONE_SCK_PIN, GPIO_PIN_SET);
			HAL_GPIO_WritePin(_HX711_ONE_SCK_GPIO, _HX711_ONE_SCK_PIN, GPIO_PIN_RESET);
			data = data ^ 0x80000000;
			__enable_irq();
			return data;
//			break;
		case 1:
			__disable_irq();
			while(HAL_GPIO_ReadPin(_HX711_TWO_DI_GPIO, _HX711_TWO_DI_PIN) == GPIO_PIN_SET);
//			{
//				if(HAL_GetTick() - startTime > 300)
//				return 0;
//			}
			for(uint8_t i = 0; i < 24 ; i++)
			{
				HAL_GPIO_WritePin(_HX711_TWO_SCK_GPIO, _HX711_TWO_SCK_PIN, GPIO_PIN_SET);
				data = data << 1;
				if(HAL_GPIO_ReadPin(_HX711_TWO_DI_GPIO, _HX711_TWO_DI_PIN) == GPIO_PIN_SET)
				{
					data++;
				}
				HAL_GPIO_WritePin(_HX711_TWO_SCK_GPIO, _HX711_TWO_SCK_PIN, GPIO_PIN_RESET);
			}
			HAL_GPIO_WritePin(_HX711_TWO_SCK_GPIO, _HX711_TWO_SCK_PIN, GPIO_PIN_SET);
			HAL_GPIO_WritePin(_HX711_TWO_SCK_GPIO, _HX711_TWO_SCK_PIN, GPIO_PIN_RESET);
			data = data ^ 0x80000000;
			__enable_irq();
			return data;
//			break;
		case 2:
			__disable_irq();
			while(HAL_GPIO_ReadPin(_HX711_THREE_DI_GPIO, _HX711_THREE_DI_PIN) == GPIO_PIN_SET);
//			{
//				if(HAL_GetTick() - startTime > 300)
//				return 0;
//			}
			for(uint8_t i = 0; i < 24 ; i++)
			{
				HAL_GPIO_WritePin(_HX711_THREE_SCK_GPIO, _HX711_THREE_SCK_PIN, GPIO_PIN_SET);
				data = data << 1;
				if(HAL_GPIO_ReadPin(_HX711_THREE_DI_GPIO, _HX711_THREE_DI_PIN) == GPIO_PIN_SET)
				{
					data++;
				}
				HAL_GPIO_WritePin(_HX711_THREE_SCK_GPIO, _HX711_THREE_SCK_PIN, GPIO_PIN_RESET);
			}
			HAL_GPIO_WritePin(_HX711_THREE_SCK_GPIO, _HX711_THREE_SCK_PIN, GPIO_PIN_SET);
			HAL_GPIO_WritePin(_HX711_THREE_SCK_GPIO, _HX711_THREE_SCK_PIN, GPIO_PIN_RESET);
			data = data ^ 0x80000000;
			__enable_irq();
			return data;
//			break;
		default:
			return 0;
//			break;
	}
}

int HX711_valueAve(uint16_t sample, uint8_t LoadCellNum)
{
  int64_t  ave = 0;
  for(uint16_t i=0 ; i<sample ; i++)
    ave += HX711_value(LoadCellNum);
  return (int32_t)(ave / sample);
}

int HX711_OffsettedValue(uint16_t sample, uint8_t LoadCellNum)
{
	int temp = HX711_valueAve(sample, LoadCellNum);
	temp = temp - OFFSET[LoadCellNum];
	return (temp);
}

int HX711_valueUnits (uint16_t sample, uint8_t LoadCellNum)
{
	int temp = HX711_OffsettedValue(sample, LoadCellNum);
	temp = temp / SCALE[LoadCellNum];
	UnitWeight = temp;
	switch(LoadCellNum)
	{
		case 0:
			UnitWeight = (UnitWeight * DryIceSlope) + DryIceIntercept;
			return (UnitWeight);
		case 1:
			UnitWeight = (UnitWeight * SalineOneSlope) + SalineOneIntercept;
			return (UnitWeight);
		case 2:
			UnitWeight = (UnitWeight * SalineTwoSlope) + SalineTwoIntercept;
			return (UnitWeight);
		default:
			return (UnitWeight);
	}
	return (UnitWeight);
}

void HX711_Tare(uint16_t sample, uint8_t LoadCellNum)
{
	int32_t sum = HX711_valueAve(sample,LoadCellNum);
	HX711_set_offset(sum,LoadCellNum);
}

void HX711_set_scale(float scale, uint8_t LoadCellNum)
{
	switch(LoadCellNum)
	{
		case 0:
			SCALE[0] = scale;
			break;
		case 1:
			SCALE[1] = scale;
			break;
		case 2:
			SCALE[2] = scale;
			break;
		default:
			break;
	}
}

void HX711_set_offset(int32_t offset, uint8_t LoadCellNum)
{
	switch(LoadCellNum)
	{
		case 0:
			OFFSET[0] = offset;
			break;
		case 1:
			OFFSET[1] = offset;
			break;
		case 2:
			OFFSET[2] = offset;
			break;
		default:
			break;
	}
}
