/*
 * systick.c
 *
 *  Created on: Nov 19, 2025
 *      Author: Harisama1
 */


#include "systick.h"

#define ReLoadVal 16000
#define CountFlag (1U << 16)


void SystickDelayMS(uint16_t delay)
{
	//Step 1: Program reload value.
	/*
	 * Calculate the load value
	 * Using system clock 16Mhz
	 * so 1 tick = 1/(16 * 10^6)s
	 * ==> 1 mili second = 16000 ticks
	 * ==> Reload Value = 16000 - 1
	 */
	SYSTICK_SET_RELOAD(ReLoadVal - 1);

	//Step 2: Clear current value.
	SYSTICK_CLEAR();

	//Step 3: Program Control and Status register.
	SYSTICK_SET_CLK_CPU();
	SYSTICK_ENABLE();
	while(delay > 0)
	{
		while((SysTick->CTRL & CountFlag) == 0);
		delay--;
	}
	SYSTICK_DISABLE();
}
