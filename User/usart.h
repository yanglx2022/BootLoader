/**********************
* 串口调试信息
**********************/

#ifndef __USART_H
#define __USART_H

#include "stm32f10x.h"
#include <stdio.h>

//#define DEBUG		// 启用debug时需要在设置中勾选Use MicroLIB

#ifdef DEBUG

int fputc(int ch, FILE* p);
void Debug_Init(void);

#endif

#endif  /* __USART_H */
