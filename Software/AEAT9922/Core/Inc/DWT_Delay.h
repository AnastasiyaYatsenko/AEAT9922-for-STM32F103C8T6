/*
 * DWT_Delay.h
 *
 *  Created on: Jun 22, 2020
 *  Author: Khaled Magdy
 */

#ifndef DWT_DELAY_H_
#define DWT_DELAY_H_

#include "stm32f1xx_hal.h"


uint32_t DWT_Delay_Init();
void DWT_Delay_us(volatile uint32_t au32_microseconds);
void DWT_Delay_ms(volatile uint32_t au32_milliseconds);


#endif /* DWT_DELAY_H_ */
