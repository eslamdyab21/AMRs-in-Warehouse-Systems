/**********************************************************/
/* Author    : Gehad Ekoumy                               */
/* Date      : 5 DEC 2022                                 */
/* Version   : V01                                        */
/**********************************************************/
#include "STD_TYPES.h"
#include "BIT_MATH.h"

#include "NVIC_interface.h"
#include "NVIC_config.h"
#include "NVIC_private.h"


void MNVIC_voidEnableInterrupt (u8 Copy_u8IntNumber)
{
	if ( Copy_u8IntNumber <= 31)
	{
		NVIC_ISER0 = (1 << Copy_u8IntNumber);
	}
	
	else if (  Copy_u8IntNumber <= 59)
	{
		Copy_u8IntNumber -= 32;
		NVIC_ISER1 = (1 << Copy_u8IntNumber);
	}
	
}


void MNVIC_voidDisableInterrupt (u8 Copy_u8IntNumber)
{
	if ( Copy_u8IntNumber <= 31)
	{
		NVIC_ICER0 = (1 << Copy_u8IntNumber);
	}
	
	else if (  Copy_u8IntNumber <= 59)
	{
		Copy_u8IntNumber -= 32;
		NVIC_ICER1 = (1 << Copy_u8IntNumber);
	}
	
}


		/*for testing & debugging*/
void MNVIC_voidSetPendingFlag(u8 Copy_u8IntNumber)
{
	if ( Copy_u8IntNumber <= 31)
	{
		NVIC_ISPR0 = (1 << Copy_u8IntNumber);
	}
	
	else if (  Copy_u8IntNumber <= 59)
	{
		Copy_u8IntNumber -= 32;
		NVIC_ISPR1 = (1 << Copy_u8IntNumber);
	}
	
}


void MNVIC_voidClearPendingFlag(u8 Copy_u8IntNumber)
{
	if ( Copy_u8IntNumber <= 31)
	{
		NVIC_ICPR0 = (1 << Copy_u8IntNumber);
	}
	
	else if (  Copy_u8IntNumber <= 59)
	{
		Copy_u8IntNumber -= 32;
		NVIC_ICPR1 = (1 << Copy_u8IntNumber);
	}
	
}


u8 MNVIC_u8GetActiveFlag(u8 Copy_u8IntNumber)
{
	u8 Local_u8Result = 0;

	if ( Copy_u8IntNumber <= 31)
	{
		Local_u8Result = GET_BIT(NVIC_IABR0,Copy_u8IntNumber);
	}
	
	else if (  Copy_u8IntNumber <= 59)
	{
		Copy_u8IntNumber -= 32;
		Local_u8Result = GET_BIT(NVIC_IABR1,Copy_u8IntNumber);
	}
	
	return Local_u8Result;      /*1 : interrupt is executing*/
}


/*function takes : peripheral + group proirity + sub proirity*/
void MNVIC_voidSetPriority(u8 Copy_u8IntNumber , u8 Copy_u8GroupPriority ,u8 Copy_u8SubPriority )
{				
				  /* group priority value is shifted by number of sub bits */			
				  /* priority = sub priority value | group priority value */
/* ex : 0x05FA0400 3 Group & 1 sub priority (group priority is shifted by 1)*/
	u8 Local_u8Priority = Copy_u8SubPriority | (Copy_u8GroupPriority << ((NO_OF_GROUPS_SUB - 0x05FA0300)/256));
	
	/* external peripheral */ /*EXTI0 = 6*/
	if(Copy_u8IntNumber <= 59)
	{
		/*each byte --> first 4 bits are reserved*/
		NVIC_IPR[Copy_u8IntNumber] = Local_u8Priority << 4 ;
	}
	
	SCB_AIRCR = NO_OF_GROUPS_SUB ;
}
