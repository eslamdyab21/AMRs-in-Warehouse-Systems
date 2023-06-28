/**********************************************************/
/* Author    : Gehad Elkoumy                              */
/* Date      : 6 DEC 2022                                 */
/* Version   : V01                                        */
/**********************************************************/

#include "STD_TYPES/STD_TYPES.h"
#include "BIT_MATH/BIT_MATH.h"

#include "EXTI/EXTI_interface.h"
#include "EXTI/EXTI_config.h"
#include "EXTI/EXTI_private.h"



void MEXTI_voidInit()
{
	#if 	EXTI_MODE	== 	RISING
		SET_BIT(EXTI -> RTSR , EXTI_LINE);
	#elif 	EXTI_MODE	== 	FALLING
		SET_BIT(EXTI -> FTSR , EXTI_LINE);
	#elif	EXTI_MODE	==	ON_CHANGE
		SET_BIT(EXTI -> RTSR , EXTI_LINE);
		SET_BIT(EXTI -> FTSR , EXTI_LINE);
	#else 	
		#error "Wrong Mode"
	#endif
	
	/*Disable interrupt*/
	CLR_BIT(EXTI -> IMR , EXTI_LINE);
}


void MEXTI_voidEnableEXTI(u8 Copy_u8EXTILine)
{
	SET_BIT(EXTI -> IMR , Copy_u8EXTILine);
}


void MEXTI_voidDisableEXTI(u8 Copy_u8EXTILine)
{
	CLR_BIT(EXTI -> IMR , Copy_u8EXTILine);
}


void MEXTI_voidSoftwareTrigger(u8 Copy_u8EXTILine)
{
	SET_BIT(EXTI -> IMR , Copy_u8EXTILine);
	CLR_BIT(EXTI -> PR , Copy_u8EXTILine);
	SET_BIT(EXTI -> SWIER , Copy_u8EXTILine);
}

 
             /*changing mode & line in run time*/
void MEXTI_voidSetSignalLatch(u8 Copy_u8EXTILine , u8 Copy_u8EXTIMode)
{
	switch(Copy_u8EXTIMode)
	{
		case RISING :
			SET_BIT(EXTI -> RTSR , Copy_u8EXTILine);
			break;
		
		case FALLING :
			SET_BIT(EXTI -> FTSR , Copy_u8EXTILine); 	
			break;
		
		case ON_CHANGE :
			SET_BIT(EXTI -> RTSR , Copy_u8EXTILine);
			SET_BIT(EXTI -> FTSR , Copy_u8EXTILine); 	
			break;
	}
	SET_BIT(EXTI -> IMR , Copy_u8EXTILine);
}


void EXTI_voidSetCallBack(void (*ptr) (void) , u8 Copy_u8EXTILine)
{
	EXTI_GlobalPtr[Copy_u8EXTILine] = ptr;
}


void EXTI0_IRQHandler(void)
{
	EXTI_GlobalPtr[0]();
	/*clear pending bit*/
	SET_BIT(EXTI -> PR , 0);
}

void EXTI1_IRQHandler(void)
{
	EXTI_GlobalPtr[1]();
	/*clear pending bit*/
	SET_BIT(EXTI -> PR , 1);
}

void EXTI2_IRQHandler(void)
{
	EXTI_GlobalPtr[2]();
	/*clear pending bit*/
	SET_BIT(EXTI -> PR , 2);
}

void EXTI3_IRQHandler(void)
{
	EXTI_GlobalPtr[3]();
	/*clear pending bit*/
	SET_BIT(EXTI -> PR , 3);
}

void EXTI4_IRQHandler(void)
{
	EXTI_GlobalPtr[4]();
	/*clear pending bit*/
	SET_BIT(EXTI -> PR , 4);
}

void EXTI9_5_IRQHandler(void)
{
	u8 PinValue_5 , PinValue_6 , PinValue_7 , PinValue_8 , PinValue_9;

	PinValue_5 = GET_BIT(EXTI->PR,5);
	PinValue_6 = GET_BIT(EXTI->PR,6);
	PinValue_7 = GET_BIT(EXTI->PR,7);
	PinValue_8 = GET_BIT(EXTI->PR,8);
	PinValue_9 = GET_BIT(EXTI->PR,9);

	if (PinValue_5 == 1)
	{
		EXTI_GlobalPtr[5]();
		SET_BIT(EXTI -> PR , 5);
	}

	if (PinValue_6 == 1)
	{
		EXTI_GlobalPtr[6]();
		SET_BIT(EXTI -> PR , 6);
	}

	if (PinValue_7 == 1)
	{
		EXTI_GlobalPtr[7]();
		SET_BIT(EXTI -> PR , 7);
	}

	if (PinValue_8 == 1)
	{
		EXTI_GlobalPtr[8]();
		SET_BIT(EXTI -> PR , 8);
	}

	if (PinValue_9 == 1)
	{
		EXTI_GlobalPtr[9]();
		SET_BIT(EXTI -> PR , 9);
	}

}

void EXTI15_10_IRQHandler(void)
{
	u8 PinValue_10 , PinValue_11 , PinValue_12 , PinValue_13 , PinValue_14 , PinValue_15;

	PinValue_10 = GET_BIT(EXTI->PR,10);
	PinValue_11 = GET_BIT(EXTI->PR,11);
	PinValue_12 = GET_BIT(EXTI->PR,12);
	PinValue_13 = GET_BIT(EXTI->PR,13);
	PinValue_14 = GET_BIT(EXTI->PR,14);
	PinValue_15 = GET_BIT(EXTI->PR,14);

	if (PinValue_10 == 1)
	{
		EXTI_GlobalPtr[10]();
		SET_BIT(EXTI -> PR , 10);
	}

	if (PinValue_11 == 1)
	{
		EXTI_GlobalPtr[11]();
		SET_BIT(EXTI -> PR , 11);
	}

	if (PinValue_12 == 1)
	{
		EXTI_GlobalPtr[12]();
		SET_BIT(EXTI -> PR , 12);
	}

	if (PinValue_13 == 1)
	{
		EXTI_GlobalPtr[13]();
		SET_BIT(EXTI -> PR , 13);
	}

	if (PinValue_14 == 1)
	{
		EXTI_GlobalPtr[14]();
		SET_BIT(EXTI -> PR , 14);
	}

	if (PinValue_15 == 1)
	{
		EXTI_GlobalPtr[15]();
		SET_BIT(EXTI -> PR , 15);
	}

}
