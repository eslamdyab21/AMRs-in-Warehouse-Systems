/**********************************************************/
/* Author    : Gehad Ekoumy                               */
/* Date      : 10 DEC 2022                                */
/* Version   : V01                                        */
/**********************************************************/
#ifndef NVIC_INTERFACE_H
#define NVIC_INTERFACE_H

void MNVIC_voidEnableInterrupt (u8 Copy_u8IntNumber);
void MNVIC_voidDisableInterrupt (u8 Copy_u8IntNumber);
void MNVIC_voidSetPendingFlag   (u8 Copy_u8IntNumber);
void MNVIC_voidClearPendingFlag (u8 Copy_u8IntNumber);
u8 MNVIC_u8GetActiveFlag        (u8 Copy_u8IntNumber);
void MNVIC_voidSetPriority(u8 Copy_u8IntNumber , u8 Copy_u8GroupPriority ,u8 Copy_u8SubPriority );


#endif
