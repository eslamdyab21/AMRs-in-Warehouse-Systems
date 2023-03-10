/*****************************************/
/* Author  :  Gehad Elkoumy              */
/* Version :  V01                        */
/* Date    :  10 MAR 2023                */
/*****************************************/

#include "STD_TYPES.h"
#include "BIT_MATH.h"

#include "TIM2_interface.h"
#include "TIM2_private.h"
#include "TIM2_config.h"

void MTIM2_voidInit (void)
{
	
	/*direction of counter when it is edge aligned mode , no need for this bit if centered aligned*/
	CLR_BIT(TIM2 -> CR1 , 4);
	TIM2->CR1 |= (CR1_DIR << 4);
	
	
	
	/*prescaler value*/
	TIM2->PSC = TIM_PRESCALER;
	
	/*if there is interrupt then DIER_UIE must be enabled*/
	
}

void MTIM2_voidOutputPWM (u16 Copy_u16CompareValue)
{
	/*enable channel*/
	SET_BIT(TIM2->CCER , 12);
	
	/*enable pwm mode 1*/
	CLR_BIT(TIM2->CCMR2 , 12);
	SET_BIT(TIM2->CCMR2 , 13);
	SET_BIT(TIM2->CCMR2 , 14);
	
	/*enable output compare -- update value after overflow or immediately*/
	CLR_BIT(TIM2->CCMR2 , 11);   //immediately
	
	
	/*enable auto reload preload for PWM*/
	SET_BIT(TIM2 -> CR1 , 7);
	
	
	
	/*desired value*/
	TIM2->ARR  = 1000;
	CCR4 = (TIM2->ARR) - (Copy_u16CompareValue);    //channel4
	
	/*enable counter*/
	SET_BIT(TIM2 -> CR1 , 0);
	/*enable update generation*/
	SET_BIT(TIM2 -> EGR , 0);
	
}
