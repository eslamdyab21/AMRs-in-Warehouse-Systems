/*
 * main.c
 *
 *  Created on: Feb 18, 2023
 *      Author: HP
 */

#include"STD_TYPES.h"
#include"BIT_MATH.h"

#include"RCC_interface.h"
#include"DIO_interface.h"
#include"NVIC_interface.h"
#include"AFIO_interface.h"
#include"EXTI_interface.h"

void TakeAction8(void)
{
	MGPIO_VoidSetPinValue(GPIOA,PIN0,HIGH);
}

void TakeAction9(void)
{
	MGPIO_VoidSetPinValue(GPIOA,PIN1,HIGH);
}


void main (void)
{
	/*initialize clocks*/
	RCC_voidInitSysClock();
	/*Enable GPIOA clock */
	RCC_voidEnableClock(RCC_APB2,2);
	//RCC_voidEnableClock(RCC_APB2,3);
	/*Enable AFIO clock */
	RCC_voidEnableClock(RCC_APB2,0);

	/*AFIO*//**/
	MAFIO_voidSetEXTIConfiguration(LINE8 , AFIOA);
	MAFIO_voidSetEXTIConfiguration(LINE9 , AFIOA);


	EXTI_voidSetCallBack(TakeAction9 , LINE9);
	EXTI_voidSetCallBack(TakeAction8 , LINE8);

	/*pin modes*/
	MGPIO_VoidSetPinDirection(GPIOA,PIN8,INPUT_PULLUP_PULLDOWN);
	MGPIO_VoidSetPinDirection(GPIOA,PIN9,INPUT_PULLUP_PULLDOWN);

	MGPIO_VoidSetPinDirection(GPIOA,PIN0,OUTPUT_2MHZ_PP);
	MGPIO_VoidSetPinDirection(GPIOA,PIN1,OUTPUT_2MHZ_PP);

	MGPIO_VoidSetPinValue(GPIOA,PIN8,HIGH);
	MGPIO_VoidSetPinValue(GPIOA,PIN9,HIGH);

	/*initialize EXTI*/
	MEXTI_voidInit();
	MEXTI_voidSetSignalLatch(LINE8 , FALLING);
	MEXTI_voidSetSignalLatch(LINE9 , FALLING);


	/*Enable EXTI0 interrupt from NVIC*/
	MNVIC_voidEnableInterrupt(23);
	/*set priority for EXT0*/
	//MNVIC_voidSetPriority(23,1,0);



	while(1)
	{

	}
}

