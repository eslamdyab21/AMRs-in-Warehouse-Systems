/*
 * main.c
 *
 *  Created on: Mar 10, 2023
 *      Author: HP
 */

#include"STD_TYPES.h"
#include"BIT_MATH.h"

#include"RCC_interface.h"
#include"DIO_interface.h"
#include"TIM2_interface.h"

void main (void)
{
	/*initialize clocks*/
	RCC_voidInitSysClock();
	/*Enable GPIOA clock */
	RCC_voidEnableClock(RCC_APB2,2);
	RCC_voidEnableClock(RCC_APB2,3);
	/*Enable AFIO clock */
	RCC_voidEnableClock(RCC_APB2,0);
	/*Enable Timer2 clock*/
	RCC_voidEnableClock(RCC_APB1,0);

	/*mode must be alternative function push pull*/
	MGPIO_VoidSetPinDirection(GPIOA,PIN3,OUTPUT_2MHZ_AFPP);

	MTIM2_voidInit();
	MTIM2_voidOutputPWM(100);

	while(1)
	{

	}
}

