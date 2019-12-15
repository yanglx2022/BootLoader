/**********************
*系统tick延时
**********************/

#include "delay.h"

volatile uint32_t Ticks;   // counts 10us timeTicks

/*----------------------------------------------------------------------------
 * SysTick_Handler:
 *----------------------------------------------------------------------------*/
void SysTick_Handler(void) {
  Ticks++;
}

/*----------------------------------------------------------------------------
 * Delay: Init
 *----------------------------------------------------------------------------*/
void Delay_Init()
{
	SysTick_Config(SystemCoreClock / 100000); // 10us
}
/*----------------------------------------------------------------------------
 * Delay: delays a number of Systicks (10us)
 *----------------------------------------------------------------------------*/
void Delay_10us (uint32_t dlyTicks) 
{
  uint32_t curTicks;

  curTicks = Ticks;
  while ((Ticks - curTicks) < dlyTicks) { __NOP(); }
}

void Delay_Ms(uint32_t dlyTicks)
{
	Delay_10us(100 * dlyTicks);
}






