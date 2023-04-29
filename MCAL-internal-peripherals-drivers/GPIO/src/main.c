/*
 * main.c
 *
 *  Created on: Dec 8, 2022
 *      Author: Gehad Elkoumy
 */

#include"STD_TYPES.h"
#include"BIT_MATH.h"

#include"RCC_interface.h"
#include"DIO_interface.h"


void main(void)
{
	/*initialize clock system*/
	RCC_voidInitSysClock();

	/*enable peripheral clk (GPIOA&B)*/
	RCC_voidEnableClock(RCC_APB2 , 2);
	RCC_voidEnableClock(RCC_APB2 , 3);


    /*mode for pin*/
	MGPIO_VoidSetPinDirection(GPIOA,PIN0,OUTPUT_2MHZ_PP);
	MGPIO_VoidSetPinDirection(GPIOA,PIN1,OUTPUT_2MHZ_PP);
	MGPIO_VoidSetPinDirection(GPIOA,PIN2,OUTPUT_2MHZ_PP);
	MGPIO_VoidSetPinDirection(GPIOA,PIN3,OUTPUT_2MHZ_PP);
	MGPIO_VoidSetPinDirection(GPIOA,PIN4,OUTPUT_2MHZ_PP);
	MGPIO_VoidSetPinDirection(GPIOA,PIN5,OUTPUT_2MHZ_PP);
	MGPIO_VoidSetPinDirection(GPIOA,PIN6,OUTPUT_2MHZ_PP);
	MGPIO_VoidSetPinDirection(GPIOA,PIN7,OUTPUT_2MHZ_PP);
	MGPIO_VoidSetPinDirection(GPIOA,PIN8,OUTPUT_2MHZ_PP);
	MGPIO_VoidSetPinDirection(GPIOA,PIN9,OUTPUT_2MHZ_PP);
	MGPIO_VoidSetPinDirection(GPIOA,PIN10,OUTPUT_2MHZ_PP);
	MGPIO_VoidSetPinDirection(GPIOA,PIN11,OUTPUT_2MHZ_PP);
	MGPIO_VoidSetPinDirection(GPIOA,PIN12,OUTPUT_2MHZ_PP);
	MGPIO_VoidSetPinDirection(GPIOA,PIN13,OUTPUT_2MHZ_PP);
	MGPIO_VoidSetPinDirection(GPIOA,PIN14,OUTPUT_2MHZ_PP);
	MGPIO_VoidSetPinDirection(GPIOA,PIN15,OUTPUT_2MHZ_PP);
	//u16 w = 500;


	while(1)
	{

		MGPIO_VoidSetPinValue(GPIOA,PIN0,HIGH);
		MGPIO_VoidSetPinValue(GPIOA,PIN1,HIGH);
		MGPIO_VoidSetPinValue(GPIOA,PIN2,HIGH);
		MGPIO_VoidSetPinValue(GPIOA,PIN3,HIGH);
		MGPIO_VoidSetPinValue(GPIOA,PIN4,HIGH);
		MGPIO_VoidSetPinValue(GPIOA,PIN5,HIGH);
		MGPIO_VoidSetPinValue(GPIOA,PIN6,HIGH);
		MGPIO_VoidSetPinValue(GPIOA,PIN7,HIGH);
		MGPIO_VoidSetPinValue(GPIOA,PIN8,HIGH);
		MGPIO_VoidSetPinValue(GPIOA,PIN9,HIGH);
		MGPIO_VoidSetPinValue(GPIOA,PIN10,HIGH);
		MGPIO_VoidSetPinValue(GPIOA,PIN11,HIGH);
		MGPIO_VoidSetPinValue(GPIOA,PIN12,HIGH);
		MGPIO_VoidSetPinValue(GPIOA,PIN13,HIGH);
		MGPIO_VoidSetPinValue(GPIOA,PIN14,HIGH);
		MGPIO_VoidSetPinValue(GPIOA,PIN15,HIGH);

		/*delay with assembly (loop 500)*/
		/*for(u16 i=0;i<w;i++)
		{
			for(u16 j=0;j<w;j++)
			{
				asm("NOP");
			}

		}

		MGPIO_VoidSetPinValue(GPIOA,PIN0,HIGH);*/
		/*delay with assembly (loop 500)*/
		/*for(u16 i=0;i<w;i++)
		{
			for(u16 j=0;j<w;j++)
			{
				asm("NOP");
			}

		}*/

	}


}



