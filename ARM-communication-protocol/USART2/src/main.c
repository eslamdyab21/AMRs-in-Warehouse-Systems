/*
 * main.c
 *
 *  Created on: Jun 11, 2023
 *      Author: Gehad Elkoumy
 */

#include"STD_TYPES.h"
#include"BIT_MATH.h"

#include"RCC_interface.h"
#include"DIO_interface.h"
#include"USART_interface.h"

u8 x = 22;
void main (void)
{
	/*initialize clocks*/
	RCC_voidInitSysClock();

	MUSART2_voidInit();
	//MUSART2_voidSendString("hello world");
	//s32 x=255;

	while(1)
		{
			//MUSART2_voidSendString((u8*)"hello");
			//MUSART2_voidSendString((u8*)"\r\n");
			MUSART2_voidSendNumbers(x);
			MUSART2_voidSendString((u8*)"\r\n");
			//x++;

		}



}

