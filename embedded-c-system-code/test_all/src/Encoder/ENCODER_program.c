/*****************************************/
/* Author  :  Gehad Elkoumy              */
/* Version :  V01                        */
/* Date    : 9 FEB 2023                  */
/*****************************************/

#include"STD_TYPES/STD_TYPES.h"
#include"BIT_MATH/BIT_MATH.h"

#include "DIO/DIO_interface.h"

#include "ENCODER/ENCODER_interface.h"
#include "ENCODER/ENCODER_private.h"
#include "ENCODER/ENCODER_config.h"

u8 Global_u8MotorRightDirection = 0;
u8 Global_u8MotorLeftDirection = 0;
static s32 Global_s32NumOfRightCounts = 0 ;
static s32 Global_s32NumOfLeftCounts = 0 ;
static f32 Global_f32Distance = 0;

u8 HENCODER_u8GetMotorDirection(u8 Copy_u8Channel2Pin)
{
	if(Copy_u8Channel2Pin == 8)
	{
		return Global_u8MotorLeftDirection;
	}
	else //if (Copy_u8Channel2Pin == 10)
	{
		return Global_u8MotorRightDirection;
	}
}

s32 HENCODER_voidEncoderCounts(u8 Copy_u8Channel2Port ,u8 Copy_u8Channel2Pin)
{
	//u8 MotorDirection ;
	u8 Local_u8ChannelState ;

	Local_u8ChannelState = MGPIO_u8GetPinValue(Copy_u8Channel2Port ,Copy_u8Channel2Pin+1);
	
	if (Copy_u8Channel2Pin == 8)
	{
		if(Local_u8ChannelState == HIGH)
		{
			Global_u8MotorLeftDirection = ClockwiseDirection;
			Global_s32NumOfLeftCounts--;
		}
		else if (Local_u8ChannelState == LOW)
		{
			Global_u8MotorLeftDirection = CounterClockwiseDirection;
			Global_s32NumOfLeftCounts++;
		}

	return Global_s32NumOfLeftCounts;
	}

	else //if (Copy_u8Channel2Pin == 10)
	{
		if(Local_u8ChannelState == HIGH)
		{
			Global_u8MotorRightDirection = ClockwiseDirection;
			Global_s32NumOfRightCounts--;
		}
		else if (Local_u8ChannelState == LOW)
		{
			Global_u8MotorRightDirection = CounterClockwiseDirection;
			Global_s32NumOfRightCounts++;
		}

	return Global_s32NumOfRightCounts;
	}
}


void HENCODER_s32GetZeroCounts(u8 Copy_u8Channel2Pin)
{
	if(Copy_u8Channel2Pin == 8)
	{
		Global_s32NumOfLeftCounts = 0;
	}
	else if (Copy_u8Channel2Pin == 10)
	{
		Global_s32NumOfRightCounts = 0;
	}
	//return Global_s32NumOfCounts;
}


s32 HENCODER_s32GetRevPerMin(s32 Copy_s32EncoderCounts, u32 Copy_u32Time)
{
	s32 Local_s32RPM = 0;
	Local_s32RPM = (Copy_s32EncoderCounts*60)/(EncoderResolution*Copy_u32Time/1000000); // need to handle if not 1sec
	return Local_s32RPM;
}


f32 HENCODER_f32GetDistance( s32 Copy_s32EncoderCounts )
{
	Global_f32Distance = Global_f32Distance + (Copy_s32EncoderCounts*2*Pi*WheelRadius)/EncoderResolution;
	return Global_f32Distance;
}

void HENCODER_f32GetZeroDistance(void)
{
	Global_f32Distance = 0;
}

