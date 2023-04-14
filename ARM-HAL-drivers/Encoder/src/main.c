/*
 * main.c
 *
 *  Created on: Feb 11, 2023
 *      Author: Eslam
 */


#include"STD_TYPES.h"
#include"BIT_MATH.h"

#include"RCC_interface.h"
#include"DIO_interface.h"
#include"NVIC_interface.h"
#include"AFIO_interface.h"
#include"EXTI_interface.h"
#include"STK_interface.h"
//#include"TIM2_interface.h"
#include"ENCODER_interface.h"

s32 count = 0, RPM = 0;
u8 direction;
/*count 1sec : RCC = HSE crystal(8MHz) + systick clock = AHB/8 = (1tick -- 1usec)*/

/*ISR of Systick*/
void EncoderCounts(void)
{
	RPM = (count/230)*60;
	if (count < 0)
	{
		direction = CounterClockwiseDirection;
	}
	else
	{
		direction = ClockwiseDirection;
	}
	count = 0;

}

/*ISR of EXTI8*/
void GetReading (void)
{
	u8 current_state = 0 ;
	current_state = MGPIO_u8GetPinValue(GPIOA, PIN8);
	if(MGPIO_u8GetPinValue(GPIOA, PIN9) != current_state)
	{
		count++;
	}
	else
	{
		count--;
	}

}


void main(void)
{
	/*initialize RCC*/
	RCC_voidInitSysClock();
	/*Enable GPIOA ,B ,C clock*/
	RCC_voidEnableClock(RCC_APB2,2);
	RCC_voidEnableClock(RCC_APB2,3);
	RCC_voidEnableClock(RCC_APB2,4);
	/*Enable AFIOA clock */
	RCC_voidEnableClock(RCC_APB2,0);

	/*AFIO*/
	MAFIO_voidSetEXTIConfiguration(LINE8 , AFIOA);
	//MAFIO_voidSetEXTIConfiguration(LINE9 , AFIOA);

	/*pin directions*/
	/*input floating --- 0 or 1*/
	MGPIO_VoidSetPinDirection(GPIOA,PIN8,INPUT_FLOATING);
	MGPIO_VoidSetPinDirection(GPIOA,PIN9,INPUT_FLOATING);
	/*motor*/
	MGPIO_VoidSetPinDirection(GPIOA,PIN2,OUTPUT_2MHZ_PP);
	MGPIO_VoidSetPinDirection(GPIOA,PIN3,OUTPUT_2MHZ_PP);
	/*output pins for test*/
	MGPIO_VoidSetPinDirection(GPIOC,PIN13,OUTPUT_2MHZ_PP);

	/*EXTI initialize*/
	/*before initializing EXTI , initiate call back*/
	EXTI_voidSetCallBack(GetReading,LINE8);
	MEXTI_voidInit();
	MEXTI_voidSetSignalLatch(LINE8,RISING);
	//MEXTI_voidSetSignalLatch(LINE9,RISING);

	/*Enalbe EXTI(5:9) interrupt from NVIC*/
	MNVIC_voidEnableInterrupt(23);

	/*systick initialize*/
	MSTK_voidInit();
	/*start timer 1sec*/
	MSTK_voidSetIntervalPeriodic(1000000,EncoderCounts);

	while(1)
	{

		MGPIO_VoidSetPinValue(GPIOA,PIN2,HIGH);
		MGPIO_VoidSetPinValue(GPIOA,PIN3,HIGH);
	}
}
