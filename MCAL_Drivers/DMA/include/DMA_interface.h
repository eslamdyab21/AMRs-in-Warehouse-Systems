/*****************************************/
/* Author  :  Gehad Elkoumy              */
/* Version :  V01                        */
/* Date    :  23 April 2023              */
/*****************************************/

#ifndef DMA_INTERFACE_H_
#define DMA_INTERFACE_H_

void MDMA_voidChannel1Init(void);
void MDMA_voidChannel1Start(u32 *Copy_Pointeru32SourceAddress, u32 *Copy_Pointer32DestinationAddress, u16 Copy_u16BlockLength);

#endif