/**********************************************************/
/* Author    : Gehad Elkoumy                              */
/* Date      : 30 NOV 2022                                */
/* Version   : V01                                        */
/**********************************************************/
#include "STD_TYPES.h"
#include "BIT_MATH.h"

#include "AFIO_interface.h"
#include "AFIO_config.h"
#include "AFIO_private.h"


void	MAFIO_voidSetEXTIConfiguration(u8 Copy_u8EXTILine ,u8 Copy_u8PortMap)
{
	u8 Local_u8RegIndex = 0 ;		
	/* Assign to EXTICRX register									*/
	if(Copy_u8EXTILine <= 3 )
	{
			Local_u8RegIndex = 0;
	}
	else if(Copy_u8EXTILine <= 7)
	{	
		Local_u8RegIndex = 1;
		
		/*bit0 to bit3 not 4 to 7*/
		Copy_u8EXTILine -= 4;
	}
	else if(Copy_u8EXTILine <= 11)
	{
		Local_u8RegIndex = 2;
		/*4bits from 0 to 3*/
		Copy_u8EXTILine -= 8;
	
	}
	else if(Copy_u8EXTILine <= 15)
	{
		Local_u8RegIndex = 3;
		/*4bits from 0 to 3*/
		Copy_u8EXTILine -= 12;
	}

	/*reset first --> (avoiding overwrite)*/
	AFIO->EXTICR[Local_u8RegIndex] &= ~((0b1111) << (Copy_u8EXTILine * 4));
	
	/*set*/
	AFIO->EXTICR[Local_u8RegIndex]  |= ((Copy_u8PortMap) << (Copy_u8EXTILine * 4));	
}
