/*********************************************************************************/
/* Author    : Gehad Elkoumy                                                     */
/* Version   : V01                                                               */
/* Data      : 5 NOV                                                             */
/*********************************************************************************/
#ifndef RCC_INTERFACE_H
#define RCC_INTERFACE_H


/*Bus Id*/
#define RCC_AHB       0 
#define RCC_APB1      1
#define RCC_APB2      2


void RCC_voidInitSysClock(void);
void RCC_voidEnableClock(u8 Copy_u8BusId, u8 Copy_u8PeripheralId);
void RCC_voidDisableClock(u8 Copy_u8BusId, u8 Copy_u8PeripheralId);


#endif