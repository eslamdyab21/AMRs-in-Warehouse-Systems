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

static s32 Global_s32NumOfCounts = 0 ;

/*input parameter : encoder channels */
u8 HENCODER_u8GetMotorDirection(u8 Copy_u8Channel2Port , u8 Copy_u8Channel2Pin)
{

	u8 Local_u8MotorDirection = 0;
	u8 Local_u8ChannelState;
	
	Local_u8ChannelState = MGPIO_u8GetPinValue(Copy_u8Channel2Port ,Copy_u8Channel2Pin+1);
	if(Local_u8ChannelState == HIGH)
	{
		Local_u8MotorDirection = ClockwiseDirection;
	}
	else if (Local_u8ChannelState == LOW)
	{
		Local_u8MotorDirection = CounterClockwiseDirection;
	}
	
	return Local_u8MotorDirection;
}


s32 HENCODER_voidEncoderCounts(u8 Copy_u8Channel2Port ,u8 Copy_u8Channel2Pin)
{
	u8 MotorDirection ;

	MotorDirection = HENCODER_u8GetMotorDirection(Copy_u8Channel2Port , Copy_u8Channel2Pin);
	
	if(MotorDirection == ClockwiseDirection)
	{
		Global_s32NumOfCounts++;
	}
	else
	{
		Global_s32NumOfCounts--;
	}

	return Global_s32NumOfCounts;
}


//s32 HENCODER_s32GetEncoderCounts(void)
//{
//	return Global_s32NumOfCounts;
//}


s32 HENCODER_s32GetRevPerMin(s32 Copy_s32EncoderCounts)
{
	s32 Local_s32RPM = 0;
	Local_s32RPM = (Copy_s32EncoderCounts*60)/EncoderResolution; // need to handle if not 1sec
	return Local_s32RPM;
}


f32 HENCODER_f32GetDistance( s32 Copy_s32EncoderCounts )
{
	f32 Local_f32Distance = 0;
	Local_f32Distance = (Copy_s32EncoderCounts*2*Pi*WheelRadius)/EncoderResolution;
	return Local_f32Distance;
}

