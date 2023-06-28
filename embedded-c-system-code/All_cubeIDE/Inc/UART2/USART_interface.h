/*****************************************/
/* Author  :  Gehad Elkoumy              */
/* Version :  V01                        */
/* Date    :  11 Jun 2023                */
/*****************************************/

#ifndef USART_INTERFACE_H
#define USART_INTERFACE_H

void MUSART2_voidInit(void);
void MUSART2_voidSendData(u8 Copy_u16Data);
void MUSART2_voidSendString(u8 *Copy_u8String);
void MUSART2_voidSendNumbers(f32 Copy_s32Number);
u8 MUSART2_u8ReceiveData(void);

#endif






