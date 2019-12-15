/**********************
* 串口输出调试信息
* TX PA9
**********************/

#include "usart.h"

#ifdef DEBUG

int fputc(int ch, FILE* p)
{
	USART_SendData(USART1, (uint8_t)ch);
	while(USART_GetFlagStatus(USART1, USART_FLAG_TC) != SET);
	return ch;
}

void Debug_Init(void)
{
	// 开启外设时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
	
	// 设置IO PA9 PA10
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_9;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IN_FLOATING;
	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_10;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	// 设置USART1
	USART_DeInit(USART1);
	USART_InitTypeDef USART_InitStructure;
	USART_InitStructure.USART_BaudRate			= 9600;
	USART_InitStructure.USART_HardwareFlowControl	= USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode					= USART_Mode_Tx/*|USART_Mode_Rx*/;
	USART_InitStructure.USART_Parity				= USART_Parity_No;
	USART_InitStructure.USART_StopBits			= USART_StopBits_1;
	USART_InitStructure.USART_WordLength		= USART_WordLength_8b;
	USART_Init(USART1, &USART_InitStructure);

	// 串口使能
	USART_Cmd(USART1, ENABLE);
	USART_ClearFlag(USART1, USART_FLAG_TC);
}

#endif

