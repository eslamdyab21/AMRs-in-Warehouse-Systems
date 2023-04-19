/*
 * main.c
 *
 *  Created on: Feb 2, 2023
 *      Author: Gehad Elkoumy
 */

#include"STD_TYPES.h"
#include"BIT_MATH.h"

#include"RCC_interface.h"
#include"DIO_interface.h"
#include"NVIC_interface.h"
#include"AFIO_interface.h"
#include"EXTI_interface.h"
#include"STK_interface.h"

void EXTI0_ISR0(void)
{
	MGPIO_VoidSetPinValue(GPIOA,PIN2,HIGH);
	MSTK_voidSetBusyWait(1000000);  /*delay 1sec*/
	MGPIO_VoidSetPinValue(GPIOA,PIN2,LOW);
	MSTK_voidSetBusyWait(1000000);
}


void main (void)
{
	/*initialize clocks*/
	RCC_voidInitSysClock();
	/*Enable GPIOA clock */
	RCC_voidEnableClock(RCC_APB2,2);
	/*Enable AFIOA clock */
	RCC_voidEnableClock(RCC_APB2,0);

	/*initialize timer*/
	MSTK_voidInit();
	/*AFIO*/
	MAFIO_voidSetEXTIConfiguration(LINE0 , AFIOA);

	EXTI0_voidSetCallBack(EXTI0_ISR0);

	/*pin modes*/
	MGPIO_VoidSetPinDirection(GPIOA,PIN0,INPUT_PULLUP_PULLDOWN);
	MGPIO_VoidSetPinDirection(GPIOA,PIN2,OUTPUT_2MHZ_PP);

	MGPIO_VoidSetPinValue(GPIOA,PIN0,HIGH);

	/*initialize EXTI*/
	MEXTI_voidInit();
	MEXTI_voidSetSignalLatch(LINE0 , FALLING);


	/*Enable EXTI0 interrupt from NVIC*/
	MNVIC_voidEnableInterrupt(6);
	/*set priority for EXT0*/
	MNVIC_voidSetPriority(6,1,0);
/**/


	while(1)
	{

	}
}



