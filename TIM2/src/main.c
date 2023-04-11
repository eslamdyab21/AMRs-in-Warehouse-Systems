//*
 * main.c
 *
 *  Created on: Mar 10, 2023
 *      Author: Gehad
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
	MGPIO_VoidSetPinDirection(GPIOA,PIN2,OUTPUT_2MHZ_PP);
	//MGPIO_VoidSetPinDirection(GPIOA,PIN0,OUTPUT_2MHZ_PP);

	MTIM2_voidInit();

	while(1)
	{

		MGPIO_VoidSetPinValue(GPIOA , PIN2 ,HIGH);
		MTIM2_voidOutputPWM(0);

	}
}

