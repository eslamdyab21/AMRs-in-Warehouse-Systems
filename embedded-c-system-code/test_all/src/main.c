/*
 * main.c
 *
 *  Created on: Jun 24, 2023
 *      Author: Gehad Elkoumy
 */


#include"STD_TYPES/STD_TYPES.h"
#include"BIT_MATH/BIT_MATH.h"

#include"RCC/RCC_interface.h"
#include"DIO/DIO_interface.h"
#include"AFIO/AFIO_interface.h"
#include"STK/STK_interface.h"
#include"NVIC/NVIC_interface.h"
#include"EXTI/EXTI_interface.h"
#include"TIM2/TIM2_interface.h"
#include"TIM3/TIM3_interface.h"
#include"ADC/ADC_interface.h"
#include"UART2/USART_interface.h"
#include"Encoder/ENCODER_interface.h"

#include <math.h>

/* Notes : stepper motor , while + communication */


/* Left Encoder */
u8 left_MotorDirection;
s32 left_counts = 0, left_countsPerSec = 0;
f32 left_RevPerMin = 0, left_distance = 0;


/* Right Encoder */
 u8 right_MotorDirection;
s32 right_counts = 0, right_countsPerSec = 0;
f32 right_RevPerMin = 0, right_distance = 0;


/* ADC variables */
u16 adc_value = 0;
f32 reading = 0 ,adc_volt = 0;
u16 R1 = 30000, R2 = 7500;

/*ISR of Systick*/
void EncoderCounts(void)
{
	static s32 left_LastCounts = 0;
	static s32 right_LastCounts = 0;

	/* left motor (encoder readings) */
	left_MotorDirection = HENCODER_u8GetMotorDirection(GPIOA,PIN8);

	//left_counts = HENCODER_s32GetEncoderCounts();
	left_countsPerSec = left_counts - left_LastCounts;
	left_LastCounts = left_counts;

	left_RevPerMin = HENCODER_s32GetRevPerMin(left_countsPerSec);
	left_distance = HENCODER_f32GetDistance(left_counts);

	/* right motor (encoder readings) */
	right_MotorDirection = HENCODER_u8GetMotorDirection(GPIOB,PIN10);

	//right_counts = HENCODER_s32GetEncoderCounts();
	right_countsPerSec = right_counts - right_LastCounts;
	right_LastCounts = right_counts;

	right_RevPerMin = HENCODER_s32GetRevPerMin(right_countsPerSec);
	right_distance = HENCODER_f32GetDistance(right_counts);
}

/*ISR of EXTI8 (left encoder)*/
void LeftEncoderGetReading (void)
{
	left_counts = HENCODER_voidEncoderCounts(GPIOA,PIN8);
}

/*ISR of EXTI10 (right encoder)*/
void RightEncoderGetReading (void)
{
	right_counts = HENCODER_voidEncoderCounts(GPIOB,PIN10);
}

void RotateLeft()
{
	s16 left_angle = 0;
	MTIM2_voidOutputPWM_C2(30);
	MTIM3_voidOutputPWM(0);

	/* motor direction */
	MGPIO_VoidSetPinValue(GPIOA,PIN5,HIGH);
	MGPIO_VoidSetPinValue(GPIOA,PIN0,HIGH);

	while(1)
	{
		left_angle = (left_counts/230)*(7/31)*360;
		if(left_angle >= 90)
		{
			MTIM2_voidOutputPWM_C2(0);
			break;
		}
	}
}

void RotateRight()
{
	s16 right_angle = 0;
	MTIM2_voidOutputPWM_C2(0);
	MTIM3_voidOutputPWM(30);

	/* motor direction */
	MGPIO_VoidSetPinValue(GPIOA,PIN5,HIGH);
	MGPIO_VoidSetPinValue(GPIOA,PIN0,HIGH);

	while(1)
	{
		right_angle = (right_counts/230)*(7/31)*360;
		if(right_angle >= 90)
		{
			MTIM3_voidOutputPWM(0);
			break;
		}
	}
}

void VoltageReading()
{
	adc_value = MADC1_u16ReadValue();
	adc_volt = (adc_value*2.82)/4096;    //3.3
	reading = (adc_volt*(R1+R2))/R2;

	reading = floor(reading * 100) / 100;    // %.2f

	MUSART2_voidSendNumbers(reading);
}

void main (void)
{
	/*initialize RCC*/
	RCC_voidInitSysClock();

	/*initialize peripherals clock - GPIO, AFIO, TIM2, TIM3, ADC1*/
	RCC_voidEnableClock(RCC_APB2 , 2);
	RCC_voidEnableClock(RCC_APB2 , 3);
	RCC_voidEnableClock(RCC_APB2 , 0);
	RCC_voidEnableClock(RCC_APB1 , 0);
	RCC_voidEnableClock(RCC_APB1 , 1);
	RCC_voidEnableClock(RCC_APB2 , 9);

	/* cytron 1 */
	//MGPIO_VoidSetPinDirection(GPIOA,PIN3,OUTPUT_2MHZ_AFPP);
	//MGPIO_VoidSetPinDirection(GPIOA,PIN2,OUTPUT_2MHZ_PP);
	MGPIO_VoidSetPinDirection(GPIOA,PIN1,OUTPUT_2MHZ_AFPP);		//pwm
	MGPIO_VoidSetPinDirection(GPIOA,PIN0,OUTPUT_2MHZ_PP);       //direction

	/* cytron 2 */
	MGPIO_VoidSetPinDirection(GPIOA,PIN6,OUTPUT_2MHZ_AFPP);		//pwm
	MGPIO_VoidSetPinDirection(GPIOA,PIN5,OUTPUT_2MHZ_PP);		//direction

	/* Right encoder */
	MGPIO_VoidSetPinDirection(GPIOA,PIN8,INPUT_FLOATING);
	MGPIO_VoidSetPinDirection(GPIOA,PIN9,INPUT_FLOATING);

	/* Left encoder */
	MGPIO_VoidSetPinDirection(GPIOA,PIN10,INPUT_FLOATING);
	MGPIO_VoidSetPinDirection(GPIOA,PIN11,INPUT_FLOATING);

	/* voltage sensor */
	MGPIO_VoidSetPinDirection(GPIOB, PIN0, INPUT_ANALOG);

	/* AFIO for EXTI (Encoders) */
	MAFIO_voidSetEXTIConfiguration(LINE8 , AFIOA);
	MAFIO_voidSetEXTIConfiguration(LINE10 , AFIOB);

	/* call back for EXTI8,10*/
	EXTI_voidSetCallBack(LeftEncoderGetReading,LINE8);
	EXTI_voidSetCallBack(RightEncoderGetReading,LINE10);

	/* EXTI initialization */
	MEXTI_voidInit();
	MEXTI_voidSetSignalLatch(LINE8,RISING);
	MEXTI_voidSetSignalLatch(LINE10,RISING);

	/* ADC initialization & conversion */
	MADC1_voidInit();
	MADC1_voidStartConversion(8,1);

	/*Enable EXTI from NVIC*/
	MNVIC_voidEnableInterrupt(23);
	MNVIC_voidEnableInterrupt(40);

	/* Initialization */
	MSTK_voidInit();
	MTIM2_voidInitC2();
	MTIM3_voidInit();
	MUSART2_voidInit();


	/*start timer 1sec*/
	MSTK_voidSetIntervalPeriodic(1000000,EncoderCounts);

	MSTK_voidSetIntervalPeriodic(1000000,VoltageReading);

	while(1)
	{
//		MGPIO_VoidSetPinValue(GPIOA,PIN5,HIGH);
//		MGPIO_VoidSetPinValue(GPIOA,PIN0,HIGH);
	}



}
