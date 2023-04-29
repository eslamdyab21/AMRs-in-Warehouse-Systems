/*****************************************/
/* Author  :  Gehad Elkoumy              */
/* Version :  V01                        */
/* Date    :  23 April 2023              */
/*****************************************/

#ifndef DMA_PRIVATE_H_
#define DMA_PRIVATE_H_

/*Array for 7 channels*/
typedef struct
{
	volatile u32 CCR;
	volatile u32 CNDTR;
	volatile u32 CPAR;
	volatile u32 CMAR;
	volatile u32 Reserved;
}DMA_Channel;

typedef struct
{
	/*flags for the 7 channels*/
	volatile u32 ISR;
	volatile u32 IFCR;
	
	/*each register is an array of 7 elements(each channel has a register)*/
	DMA_Channel Channel[7];
}DMA_t;

#define 	DMA		((volatile DMA_t*)0x40020000)

#endif