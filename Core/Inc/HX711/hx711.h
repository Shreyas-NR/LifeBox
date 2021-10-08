/*
 * hx711.h
 *
 *  Created on: Aug 12, 2020
 *      Author: shreyasnr
 */

#ifndef INC_HX711_H_
#define INC_HX711_H_

#include "main.h"

/*Load Cell 1 - DryIce
 * LoadCell 2 - Perfusion Fluid
 * LoadCell 3 - Waste Fluid*/



#define		_HX711_ONE_SCK_GPIO			GPIOC
#define		_HX711_ONE_SCK_PIN			GPIO_PIN_2
#define		_HX711_ONE_DI_GPIO			GPIOC
#define		_HX711_ONE_DI_PIN			GPIO_PIN_3

#define		_HX711_TWO_SCK_GPIO			GPIOC
#define		_HX711_TWO_SCK_PIN			GPIO_PIN_7
#define		_HX711_TWO_DI_GPIO			GPIOC
#define		_HX711_TWO_DI_PIN			GPIO_PIN_4

#define		_HX711_THREE_SCK_GPIO		GPIOC
#define		_HX711_THREE_SCK_PIN		GPIO_PIN_6
#define		_HX711_THREE_DI_GPIO		GPIOC
#define		_HX711_THREE_DI_PIN			GPIO_PIN_5

float DryIceSlope;
float DryIceIntercept;
float SalineOneSlope;
float SalineOneIntercept;
float SalineTwoSlope;
float SalineTwoIntercept;


void HX711_init(void);
//int HX711_value(void);
int HX711_value(uint8_t LoadCellNum);
//int32_t     HX711_value(void);
int HX711_valueAve(uint16_t sample, uint8_t LoadCellNum);
int HX711_OffsettedValue(uint16_t sample, uint8_t LoadCellNum);
int HX711_valueUnits (uint16_t sample, uint8_t LoadCellNum);
void HX711_Tare(uint16_t sample, uint8_t LoadCellNum);
void HX711_set_scale(float scale, uint8_t LoadCellNum);
void HX711_set_offset(int32_t offset, uint8_t LoadCellNum);

#endif /* INC_HX711_H_ */
