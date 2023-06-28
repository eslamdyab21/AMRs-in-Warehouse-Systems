/*********************************************************************************/
/* Author    : Gehad Elkoumy                                                     */
/* Version   : V01                                                               */
/* Data      : 5 NOV                                                             */
/*********************************************************************************/
#include "STD_TYPES/STD_TYPES.h"
#include "BIT_MATH/BIT_MATH.h"

#include "RCC/RCC_interface.h"
#include "RCC/RCC_private.h"
#include "RCC/RCC_config.h"

void RCC_voidInitSysClock(void)
{
	#if     RCC_CLOCK_TYPE == RCC_HSE_CRYSTAL
		RCC_CR   = 0x00010000; /* Enable HSE with no bypass */
		RCC_CFGR = 0x00000001;
		
	#elif   RCC_CLOCK_TYPE == RCC_HSE_RC
		RCC_CR   = 0x00050000; /* Enable HSE with bypass */
		RCC_CFGR = 0x00000001;
		
	#elif   RCC_CLOCK_TYPE == RCC_HSI
		RCC_CR   = 0x00000081; /* Enable HSI + Trimming(default) = 0 */
		RCC_CFGR = 0x00000000;
	
	#elif   RCC_CLOCK_TYPE == RCC_PLL
		#if   RCC_PLL_MUL_VALUE  ==  2
			RCC_CFGR = 0x00000000;
		#elif RCC_PLL_MUL_VALUE  ==  3
			RCC_CFGR = 0x00040000;
		#elif RCC_PLL_MUL_VALUE  ==  4
			RCC_CFGR = 0x00080000;
		#elif RCC_PLL_MUL_VALUE  ==  5
			RCC_CFGR = 0x000C0000;
		#elif RCC_PLL_MUL_VALUE  ==  6
			RCC_CFGR = 0x00100000;
		#elif RCC_PLL_MUL_VALUE  ==  7
			RCC_CFGR = 0x00140000;
		#elif RCC_PLL_MUL_VALUE  ==  8
			RCC_CFGR = 0x00180000;
		#elif RCC_PLL_MUL_VALUE  ==  9
			RCC_CFGR = 0x001C0000;
		#elif RCC_PLL_MUL_VALUE  ==  10
			RCC_CFGR = 0x00200000;
		#elif RCC_PLL_MUL_VALUE  ==  11
			RCC_CFGR = 0x00240000;
		#elif RCC_PLL_MUL_VALUE  ==  12
			RCC_CFGR = 0x00280000;
		#elif RCC_PLL_MUL_VALUE  ==  13
			RCC_CFGR = 0x002C0000;
		#elif RCC_PLL_MUL_VALUE  ==  14
			RCC_CFGR = 0x00300000;
		#elif RCC_PLL_MUL_VALUE  ==  15
			RCC_CFGR = 0x00340000;
		#elif RCC_PLL_MUL_VALUE  ==  16
			RCC_CFGR = 0x00380000;
		#endif
		
		#if   RCC_PLL_INPUT == RCC_PLL_IN_HSI_DIV_2
			RCC_CR   = 0x00000001;  /*HSI*/
			/*PLL as system clock + HSI/2*/
			CLR_BIT(RCC_CFGR,SW0);
			SET_BIT(RCC_CFGR,SW1);
			CLR_BIT(RCC_CFGR,PLLSRC);
			/*Enable PLL*/
			SET_BIT(RCC_CR , PLL_ON);
		
		#elif RCC_PLL_INPUT == RCC_PLL_IN_HSE_DIV_2
			RCC_CR   = 0x00010000;  /*HSE*/
			/*PLL as system clock + HSE/2*/
			CLR_BIT(RCC_CFGR,SW0);
			SET_BIT(RCC_CFGR,SW1);
			SET_BIT(RCC_CFGR,PLLSRC);
			SET_BIT(RCC_CFGR,PLLXTPRE);
			/*Enable PLL*/
			SET_BIT(RCC_CR , PLL_ON);
	
		#elif RCC_PLL_INPUT == RCC_PLL_IN_HSE
			RCC_CR   = 0x00010000;  /*HSE*/
			/*PLL as system clock + HSE*/
			CLR_BIT(RCC_CFGR,SW0);
			SET_BIT(RCC_CFGR,SW1);
			SET_BIT(RCC_CFGR,PLLSRC);
			CLR_BIT(RCC_CFGR,PLLXTPRE);
			/*Enable PLL*/
			SET_BIT(RCC_CR , PLL_ON);
		
		#endif
	
	#else
		#error("You chosed Wrong Clock type")
	#endif
}


void RCC_voidEnableClock(u8 Copy_u8BusId, u8 Copy_u8PeripheralId)
{
	/*check input validation (32 bits)*/
	if (Copy_u8PeripheralId <= 31)     
	{
		switch (Copy_u8BusId)
		{
			case RCC_AHB  : SET_BIT(RCC_AHBENR  ,Copy_u8PeripheralId);   
			break;
			
			case RCC_APB1 : SET_BIT(RCC_APB1ENR ,Copy_u8PeripheralId);   
			break;
			
			case RCC_APB2 : SET_BIT(RCC_APB2ENR ,Copy_u8PeripheralId);   
			break;
		}
	}
	
	/*else
	{
		// Return Error 
	}*/

}


void RCC_voidDisableClock(u8 Copy_u8BusId, u8 Copy_u8PeripheralId)
{
	if (Copy_u8PeripheralId <= 31)
	{
		switch (Copy_u8BusId)
		{
			case RCC_AHB  : CLR_BIT(RCC_AHBENR  ,Copy_u8PeripheralId);   
			break;
			
			case RCC_APB1 : CLR_BIT(RCC_APB1ENR ,Copy_u8PeripheralId);   
			break;
			
			case RCC_APB2 : CLR_BIT(RCC_APB2ENR ,Copy_u8PeripheralId);   
			break;
		}
	}
	
	/*else
	{
		// Return Error 
	}*/

}

