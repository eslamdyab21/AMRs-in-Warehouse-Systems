/*****************************************/
/* Author  :  Gehad Elkoumy              */
/* Version :  V01                        */
/* Date    :  23 April 2023              */
/*****************************************/

#ifndef ADC_INTERFACE_H
#define ADC_INTERFACE_H

void MADC1_voidInit(void);
void MADC1_voidStartConversion(u8 Copy_u8ChannelID , u8 Copy_u8SeqID);
//void MADC1_voidSetCallBack(void (*ptr) (void));


#endif

