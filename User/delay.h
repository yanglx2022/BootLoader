/**********************
*系统tick延时
**********************/

#ifndef __DELAY_H
#define __DELAY_H

#include "stm32f10x.h"

void Delay_Init(void);
void Delay_10us(uint32_t dlyTicks);
void Delay_Ms(uint32_t dlyTicks);

#endif  /* __DELAY_H */
