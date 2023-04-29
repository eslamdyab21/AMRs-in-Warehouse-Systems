/*
 * main.c
 *
 *  Created on: Mar 10, 2023
 *      Author: Gehad
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

	/*mode must be alternative function push pull (not considered as GPIO)*/
	MGPIO_VoidSetPinDirection(GPIOA,PIN6,OUTPUT_2MHZ_AFPP);
	MGPIO_VoidSetPinDirection(GPIOA,PIN5,OUTPUT_2MHZ_PP);

	MTIM3_voidInit();


	while(1)
	{
		MTIM3_voidOutputPWM(0);
		MGPIO_VoidSetPinValue(GPIOA,PIN5,HIGH);
	}
}

