/*****************************************/
/* Author  :  Gehad Elkoumy              */
/* Version :  V02                        */
/* Date    :  10 MAR 2023                */
/*****************************************/

#ifndef TIM2_INTERFACE_H
#define TIM2_INTERFACE_H

void MTIM2_voidInit (void);
void MTIM2_voidSetBusyWait(u32 Copy_u16Ticks);
void MTIM2_voidOutputPWM (u16 Copy_16CompareValue);

#endif




/*

PWM freq = Fclk/(PSC*ARR)
PWM duty cycle = CCR / ARR

ex: if we want freq of 1KHz then Fclk = 8MHZ , ARR = 1000 , PSC = 8
 pwm freq = 8M/(8*1000) = 1KHZ

*/
