/*****************************************/
/* Author  :  Gehad Elkoumy              */
/* Version :  V01                        */
/* Date    :  10 MAR 2023                */
/*****************************************/

#ifndef TIM2_PRIVATE_H
#define TIM2_PRIVATE_H

#define 	TIM2_Base_Address		0x40000000

typedef struct
{
	volatile u32 CR1   ;
	volatile u32 CR2   ;
	volatile u32 SMCR  ;
	volatile u32 DIER  ;
	volatile u32 SR    ;
	volatile u32 EGR   ;
	volatile u32 CCMR1 ;
	volatile u32 CCMR2 ;
	volatile u32 CCER  ;
	volatile u32 CNT   ;
	volatile u32 PSC   ; 
	volatile u32 ARR   ;
	
}TIMER_t;

#define     TIM2    ((TIMER_t*) TIM2_Base_Address)


#define 	CCR1	*((volatile u32*)(TIM2_Base_Address + 0x34))
#define 	CCR2	*((volatile u32*)(TIM2_Base_Address + 0x38)) 
#define 	CCR3	*((volatile u32*)(TIM2_Base_Address + 0x3C)) 
#define 	CCR4	*((volatile u32*)(TIM2_Base_Address + 0x40))
#define 	DCR		*((volatile u32*)(TIM2_Base_Address + 0x48)) 
#define 	DMAR	*((volatile u32*)(TIM2_Base_Address + 0x4C)) 



#endif
