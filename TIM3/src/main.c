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
#include"TIM3_interface.h"

void main (void)
{
	/*initialize clocks*/
	RCC_voidInitSysClock();
	/*Enable GPIOA clock */
	RCC_voidEnableClock(RCC_APB2,2);
	RCC_voidEnableClock(RCC_APB2,3);
	/*Enable AFIO clock */
	RCC_voidEnableClock(RCC_APB2,0);
	/*Enable Timer3 clock*/
	RCC_voidEnableClock(RCC_APB1,1);

	/*mode must be alternative function push pull*/
	MGPIO_VoidSetPinDirection(GPIOA,PIN6,OUTPUT_2MHZ_AFPP);

	MTIM3_voidInit();
	MTIM3_voidOutputPWM(500);

	while(1)
	{

	}
}

