/*****************************************/
/* Author  :  Gehad Elkoumy              */
/* Version :  V02                        */
/* Date    :  10 MAR 2023                */
/*****************************************/

#include "STD_TYPES/STD_TYPES.h"
#include "BIT_MATH/BIT_MATH.h"

#include "TIM2/TIM2_interface.h"
#include "TIM2/TIM2_private.h"
#include "TIM2/TIM2_config.h"

void MTIM2_voidInitC2 (void)
{
	
	/*direction of counter when it is edge aligned mode , no need for this bit if centered aligned*/
	CLR_BIT(TIM2 -> CR1 , 4);
	TIM2->CR1 |= (CR1_DIR << 4);
	
	/*prescaler value*/
	TIM2->PSC = TIM_PRESCALER;
	
	/*if there is interrupt then DIER_UIE must be enabled*/
	SET_BIT(TIM2 -> DIER , 0);
	
	/*enable channel2*/
	SET_BIT(TIM2->CCER , 4);

	/*enable pwm mode 1 - channel 2*/
	CLR_BIT(TIM2->CCMR1 , 12);
	SET_BIT(TIM2->CCMR1 , 13);
	SET_BIT(TIM2->CCMR1 , 14);

	/*enable output compare -- update value after overflow or immediately*/
	CLR_BIT(TIM2->CCMR1 , 11);   //immediately

	/*enable auto reload preload for PWM*/
	SET_BIT(TIM2 -> CR1 , 7);

	/*load desired value of ARR*/
	TIM2->ARR  = 100;

	/*enable counter*/
	SET_BIT(TIM2 -> CR1 , 0);
	/*enable update generation*/
	SET_BIT(TIM2 -> EGR , 0);

}

void MTIM2_voidInitC4 (void)
{

	/*direction of counter when it is edge aligned mode , no need for this bit if centered aligned*/
	CLR_BIT(TIM2 -> CR1 , 4);
	TIM2->CR1 |= (CR1_DIR << 4);

	/*prescaler value*/
	TIM2->PSC = TIM_PRESCALER;

	/*if there is interrupt then DIER_UIE must be enabled*/
	SET_BIT(TIM2 -> DIER , 0);

	/*enable channel4*/
	SET_BIT(TIM2->CCER , 12);

	/*enable pwm mode 1 - channel 4*/
	CLR_BIT(TIM2->CCMR2 , 12);
	SET_BIT(TIM2->CCMR2 , 13);
	SET_BIT(TIM2->CCMR2 , 14);

	/*enable output compare -- update value after overflow or immediately*/
	CLR_BIT(TIM2->CCMR2 , 11);   //immediately

	/*enable auto reload preload for PWM*/
	SET_BIT(TIM2 -> CR1 , 7);

	/*load desired value of ARR*/
	TIM2->ARR  = 100;

	/*enable counter*/
	SET_BIT(TIM2 -> CR1 , 0);
	/*enable update generation*/
	SET_BIT(TIM2 -> EGR , 0);

}


void MTIM2_voidSetBusyWait(u32 Copy_u16Ticks)
{
	/* Load ticks to ARR register */
	TIM2 -> ARR = Copy_u16Ticks;
	/*start counter*/
	SET_BIT(TIM2 -> CR1 , 0);
	/* Wait till flag is raised */
	while( (GET_BIT(TIM2 -> SR , 0)) == 0);
	/*Stop counter*/
	CLR_BIT(TIM2 -> CR1 , 0);
	/*Clear flag*/
	CLR_BIT(TIM2 -> SR , 0);
	
}

void MTIM2_voidOutputPWM_C2 (u16 Copy_16CompareValue)
{
	/*load the desired value -- channel2*/
	CCR2 = Copy_16CompareValue;
}

void MTIM2_voidOutputPWM_C4 (u16 Copy_16CompareValue)
{
	/*load the desired value -- channel4*/
	CCR4 = Copy_16CompareValue;
}
