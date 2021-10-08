/*
 * Timer.h
 *
 *  Created on: Aug 6, 2020
 *      Author: shreyasnr
 */

#ifndef INC_TIMER_TIMER_H_
#define INC_TIMER_TIMER_H_

#include "main.h"

#define BLOWER	0x00000000U
#define EXHAUST	0x00000004U
void TIM2_Init(uint8_t PulseWidth,uint8_t ChannelNum);
#endif /* INC_TIMER_TIMER_H_ */
