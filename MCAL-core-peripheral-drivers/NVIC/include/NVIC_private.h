/**********************************************************/
/* Author    : Gehad Ekoumy                               */
/* Date      : 10 DEC 2022                                */
/* Version   : V01                                        */
/**********************************************************/
#ifndef NVIC_PRIVATE_H
#define NVIC_PRIVATE_H

/*59 peripherals , use 0 & 1 registers only*/
        /*Base address + offset*/
#define NVIC_ISER0       *((volatile u32*)0xE000E100)  /* Enable External interrupts from 0  to 31 */    
#define NVIC_ISER1       *((volatile u32*)0xE000E104)  /* Enable External interrupts from 32 to 63 */

#define NVIC_ICER0       *((volatile u32*)0xE000E180)  /* Disable External interrupts from 0  to 31 */
#define NVIC_ICER1       *((volatile u32*)0xE000E184)  /* Disable External interrupts from 32 to 63 */

#define NVIC_ISPR0       *((volatile u32*)0xE000E200)  /* Set Pending Flag from 0 to 31 */
#define NVIC_ISPR1       *((volatile u32*)0xE000E204)  /* Set Pending Flag from 32 to 63 */

#define NVIC_ICPR0       *((volatile u32*)0xE000E280)  /* Clear Pending Flag from 0 to 31 */
#define NVIC_ICPR1       *((volatile u32*)0xE000E284)  /* Clear Pending Flag from 32 to 63 */

	/*Read only register , volatile must be existed*/
#define NVIC_IABR0       *((volatile u32*)0xE000E300)  /* Read Active Flag from 0 to 31 */
#define NVIC_IABR1       *((volatile u32*)0xE000E304)  /* Read Active Flag from 32 to 63 */

/*without Dereference, accessed as an array(byte accessible)*/
#define NVIC_IPR		  ((volatile u8 *)(0xE000E100 + 0x300))

//_____________________________________________________________________________________________________
/* For dividing group & sub priority bits , SCB core peripheral is needed*/
		   /*for priority group controlling IPR*/
#define SCB_AIRCR		 *((volatile u32*)0xE000ED0C)
//_____________________________________________________________________________________________________


/* 4 bits in (IPR) to determine group number */
/* in (SCB_AIRCR) 4 bits for Group and 0 bit for sub */
#define		MNVIC_GROUP_4_SUB_0		0x05FA0300 
/* in (SCB_AIRCR) 3 bits for Group and 1 bit for sub */
#define		MNVIC_GROUP_3_SUB_1		0x05FA0400 
/* in (SCB_AIRCR) 2 bits for Group and 2 bit for sub */
#define		MNVIC_GROUP_2_SUB_2		0x05FA0500 
/* in (SCB_AIRCR) 1 bits for Group and 3 bit for sub */
#define		MNVIC_GROUP_1_SUB_3		0x05FA0600 
/* in (SCB_AIRCR) 0 bits for Group and 4 bits for sub */
#define		MNVIC_GROUP_0_SUB_4		0x05FA0700 

#endif
