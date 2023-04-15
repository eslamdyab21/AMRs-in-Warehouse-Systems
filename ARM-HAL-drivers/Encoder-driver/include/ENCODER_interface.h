/*****************************************/
/* Author  :  Gehad Elkoumy              */
/* Version :  V01                        */
/* Date    : 9 FEB 2023                  */
/*****************************************/

#ifndef ENCODER_INTERFACE_H
#define ENCODER_INTERFACE_H

u8 HENCODER_u8GetMotorDirection(u8 Copy_u8Channel2Port , u8 Copy_u8Channel2Pin);
void HENCODER_voidEncoderCounts(u8 Copy_u8Channel2Port ,u8 Copy_u8Channel2Pin);
s32 HENCODER_s32GetEncoderCounts(void);
s32 HENCODER_s32GetRevPerMin(s32 Copy_s32EncoderCounts);
f32 HENCODER_f32GetDistance( s32 Copy_s32EncoderCounts );

#define 	ClockwiseDirection     				1
#define 	CounterClockwiseDirection    		2

#endif
