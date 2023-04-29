/*****************************************/
/* Author  :  Gehad Elkoumy              */
/* Version :  V01                        */
/* Date    :  23 April 2023              */
/*****************************************/

#include "STD_TYPES.h"
#include "BIT_MATH.h"

#include "DMA_interface.h"
#include "DMA_private.h"
#include "DMA_config.h"

void MDMA_voidChannel1Init(void)
{
	/* channel priority level is low */
	/* Memory size of 16-bits */
	SET_BIT(DMA->Channel[0].CCR , 10);
	CLR_BIT(DMA->Channel[0].CCR , 11);
	
	/* Peripheral size of 16-bits */
	SET_BIT(DMA->Channel[0].CCR , 8);
	CLR_BIT(DMA->Channel[0].CCR , 9);
	
	/* Enable memory increment mode */
	SET_BIT(DMA->Channel[0].CCR , 7);
	
	/* Enable circular mode */
	SET_BIT(DMA->Channel[0].CCR , 5);
	
}


void MDMA_voidChannel1Start(u32 *Copy_Pointer32SourceAddress, u32 *Copy_Pointeru32DestinationAddress, u16 Copy_u16BlockLength)
{
	/* channel must be disabled first */
	CLR_BIT(DMA->Channel[0].CCR , 0);
	
	/*Load the source & destination addresses*/
	DMA->Channel[0].CPAR = Copy_Pointer32SourceAddress;
	DMA->Channel[0].CMAR = Copy_Pointeru32DestinationAddress;
	
	/*Load the block length*/
	DMA->Channel[0].CNDTR = Copy_u16BlockLength;
	
	/*Enable channel*/
	SET_BIT(DMA->Channel[0].CCR , 0);
}
