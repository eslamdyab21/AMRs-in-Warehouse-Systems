/**********************************************************/
/* Author    : Gehad Elkoumy                              */
/* Date      : 6 DEC 2022                                 */
/* Version   : V01                                        */
/**********************************************************/
#ifndef EXTI_PRIVATE_H
#define EXTI_PRIVATE_H

/*must be ordered*/
typedef struct{
	volatile u32 IMR;
	volatile u32 EMR;
	volatile u32 RTSR;
	volatile u32 FTSR;
	volatile u32 SWIER;
	volatile u32 PR;
	
}EXTI_t;

/*without dereference as it act as pointer*/
#define		EXTI  	 	((volatile EXTI_t *) 0x40010400 )

static void (*EXTI_GlobalPtr[16]) (void) ;

#endif
