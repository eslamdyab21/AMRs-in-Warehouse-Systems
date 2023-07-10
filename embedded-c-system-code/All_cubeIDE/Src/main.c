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

#include <stdio.h>
#include <stdlib.h>
#include <math.h>

/**/
/* Notes : stepper motor */

#define step_distance 	50

/* Encoders */
s32 right_counts = 0 , left_counts = 0;

/* ADC variables */
u16 adc_value = 0;
f32 reading = 0 ,adc_volt = 0;
u16 R1 = 30000, R2 = 7500;

/* Communication */
u8 Rx_arrlength = 0, data_arr[20] = {0} ;
u16 Rx_pwm = 0 ;
s16 Rx_mpu = 0 ;
s16 Local_Reading = 0;
s16 PID_Error = 0 ;
s16 Final_Value = 0 ;
s16 Reading = 0 ;

/*ISR of EXTI8 (left encoder)*/
void RightEncoderGetReading (void)
{
	right_counts = HENCODER_voidEncoderCounts(GPIOA,PIN8);
}

/*ISR of EXTI10 (right encoder)*/
void LeftEncoderGetReading (void)
{
	left_counts = HENCODER_voidEncoderCounts(GPIOB,PIN10);
}


s16 Get_Reading(void)
{
	Rx_arrlength = MUSART2_u8ReceiveDataBlock(data_arr);
	Rx_mpu = 0;

	if(data_arr[0] == 'a')
	{
		if (data_arr[1] == '-')
		{
			for(u8 i = 2 ; i<Rx_arrlength ; i++)
			{
				Rx_mpu = Rx_mpu + (data_arr[i]-48)*pow(10,Rx_arrlength-1-i);
			}
			Rx_mpu = Rx_mpu*-1;

		}
		else
		{
			for(u8 i = 1 ; i<Rx_arrlength ; i++)
			{
				Rx_mpu = Rx_mpu + (data_arr[i]-48)*pow(10,Rx_arrlength-1-i);
			}
		}
	}
		return Rx_mpu ;
}


void RotateRight()
{
	s16 error = 0;

	Reading = Get_Reading() ;
	Final_Value = Reading + 90 ;


	if (Final_Value > 180 )
	{
		Final_Value = Final_Value - 360 ;
	}

	error = Final_Value - Reading ;

	while (1)
	{
		MGPIO_VoidSetPinValue(GPIOA, 0, HIGH);
		MGPIO_VoidSetPinValue(GPIOA, 5, LOW);

		MTIM2_voidOutputPWM_C2(15);
		MTIM3_voidOutputPWM(15);

	  if (error < -180 )
	  {
		  error += 360 ;

	  }

	if ( error <= 0 )
	{

		MTIM3_voidOutputPWM(0);
		MTIM2_voidOutputPWM_C2(0);

		HENCODER_s32GetZeroCounts(PIN8);
		HENCODER_s32GetZeroCounts(PIN10);

		MUSART2_voidSendString((u8*)"s2");
		MUSART2_voidSendString((u8*)"\r\n");

		MUSART2_voidSendString((u8*)"p");
		MUSART2_voidSendNumbers(Final_Value);
		MUSART2_voidSendString((u8*)"\r\n");

		MUSART2_voidSendString((u8*)"v");
		MUSART2_voidSendNumbers(reading);
		MUSART2_voidSendString((u8*)"\r\n");
		break ;
	}
	 Reading = Get_Reading() ;
	error = Final_Value - Reading ;

    }

}


void RotateLeft()
{
	s16 error = 0;

	Reading = Get_Reading() ;
	Final_Value = Reading - 90 ;


	if (Final_Value < -180 )
	{
		Final_Value = Final_Value + 360 ;
	}

	error = Final_Value - Reading ;

	while (1)
	{
		MGPIO_VoidSetPinValue(GPIOA, 0, LOW);
		MGPIO_VoidSetPinValue(GPIOA, 5, HIGH);

		MTIM2_voidOutputPWM_C2(15);
		MTIM3_voidOutputPWM(15);

	  if (error  > 180 )
	  {
		  error -= 360 ;

	  }

	if ( error >= 0 )
	{

		MTIM3_voidOutputPWM(0);
		MTIM2_voidOutputPWM_C2(0);

		HENCODER_s32GetZeroCounts(PIN8);
		HENCODER_s32GetZeroCounts(PIN10);

		MUSART2_voidSendString((u8*)"s2");
		MUSART2_voidSendString((u8*)"\r\n");

		MUSART2_voidSendString((u8*)"p");
		MUSART2_voidSendNumbers(Final_Value);
		MUSART2_voidSendString((u8*)"\r\n");

		MUSART2_voidSendString((u8*)"v");
		MUSART2_voidSendNumbers(reading);
		MUSART2_voidSendString((u8*)"\r\n");

		break ;
	}
	Reading = Get_Reading() ;
	error = Final_Value - Reading ;

    }

}


void TargetDistance()
{
	u32 target_count = 0;
	s16 error = 0 ;


	HENCODER_s32GetZeroCounts(PIN8);
	HENCODER_s32GetZeroCounts(PIN10);
	HENCODER_f32GetZeroDistance();

	target_count = step_distance*10.23;

	while(1)
	{
		Reading = Get_Reading();
		PID_Error = Final_Value - Reading ;
		 error = 4*PID_Error ;
		/*left then right*/
		MTIM2_voidOutputPWM_C2((Rx_pwm + error) );
		MTIM3_voidOutputPWM((Rx_pwm - error));

		MUSART2_voidSendString((u8*)"t");
		MUSART2_voidSendNumbers(error);
		MUSART2_voidSendString((u8*)"\r\n");

		if(abs(right_counts) >= target_count)
		{
			HENCODER_s32GetZeroCounts(PIN8);
			HENCODER_s32GetZeroCounts(PIN10);

			Rx_pwm = 0;
			MTIM3_voidOutputPWM(0);
			MTIM2_voidOutputPWM_C2(0);

			MUSART2_voidSendString((u8*) "s1");
			MUSART2_voidSendString((u8*)"\r\n");

			MUSART2_voidSendString((u8*)"v");
			MUSART2_voidSendNumbers(reading);
			MUSART2_voidSendString((u8*)"\r\n");

			break;
		}
	}
}


void VoltageReading()
{
	adc_value = MADC1_u16ReadValue();
	adc_volt = (adc_value*3.3)/4096;
	reading = (adc_volt*(R1+R2))/R2;

}


int main (void)
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
	MGPIO_VoidSetPinDirection(GPIOB,PIN10,INPUT_FLOATING);
	MGPIO_VoidSetPinDirection(GPIOB,PIN11,INPUT_FLOATING);

	/* voltage sensor */
	MGPIO_VoidSetPinDirection(GPIOB, PIN0, INPUT_ANALOG);

	/* AFIO for EXTI (Encoders) */
	MAFIO_voidSetEXTIConfiguration(LINE8 , AFIOA);
	MAFIO_voidSetEXTIConfiguration(LINE10 , AFIOB);

	/* call back for EXTI8,10*/
	EXTI_voidSetCallBack(LeftEncoderGetReading,LINE10);
	EXTI_voidSetCallBack(RightEncoderGetReading,LINE8);

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
	MSTK_voidSetIntervalPeriodic(1000000, VoltageReading);

	MTIM2_voidOutputPWM_C2(0);
	MTIM3_voidOutputPWM(0);

	while(1)
	{
		Rx_arrlength = MUSART2_u8ReceiveDataBlock(data_arr);

		if (data_arr[0] == 'y')
		{
			MTIM2_voidOutputPWM_C2(0);
			MTIM3_voidOutputPWM(0);

			if (data_arr[1] == '-')
			{
				MGPIO_VoidSetPinValue(GPIOA, 0, LOW);
				MGPIO_VoidSetPinValue(GPIOA, 5, LOW);

				for(u8 i = 2 ; i<Rx_arrlength ; i++)
				{
					Rx_pwm = Rx_pwm + (data_arr[i]-48)*pow(10,Rx_arrlength-1-i);
				}

				TargetDistance();

			}
			else
			{
				MGPIO_VoidSetPinValue(GPIOA, 0, HIGH);
				MGPIO_VoidSetPinValue(GPIOA, 5, HIGH);

				for(u8 i = 1 ; i<Rx_arrlength ; i++)
				{
					Rx_pwm = Rx_pwm + (data_arr[i]-48)*pow(10,Rx_arrlength-1-i);
				}

				TargetDistance(Rx_pwm);

			}

		}

		else if(data_arr[0] == 'm')
		{
			MTIM2_voidOutputPWM_C2(0);
			MTIM3_voidOutputPWM(0);

			if(data_arr[1] == '-')
			{
				/* rotate right */
				RotateRight();
			}
			else
			{
				/* rotate left */
				RotateLeft();
			}
		}

		else
		{
			MTIM3_voidOutputPWM(0);
			MTIM2_voidOutputPWM_C2(0);
		}

	}
}
