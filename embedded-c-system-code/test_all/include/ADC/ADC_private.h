/*****************************************/
/* Author  :  Gehad Elkoumy              */
/* Version :  V01                        */
/* Date    :  23 April 2023              */
/*****************************************/

#ifndef ADC_PRIVATE_H
#define ADC_PRIVATE_H


typedef	struct
{
	volatile    u32		SR;
	volatile	u32		CR1;
	volatile	u32		CR2;
	volatile	u32		SMPR1;
	volatile	u32		SMPR2;
	volatile	u32		JOFR1;
	volatile	u32		JOFR2;
	volatile	u32		JOFR3;
	volatile	u32		JOFR4;
	volatile	u32		HTR;
	volatile	u32		LTR;
	volatile	u32		SQR1;
	volatile	u32		SQR2;
	volatile	u32		SQR3;
	volatile	u32		JSQR;
	volatile	u32		JDR1;
	volatile	u32		JDR2;
	volatile	u32		JDR3;
	volatile	u32		JDR4;
	volatile	u32		DR;
	
}ADC1_Type;   

#define		ADC1		(( ADC1_Type * )0X40012400)

//static void (*ADC1_GlobalPtr) (void) = '\0';

#endif
