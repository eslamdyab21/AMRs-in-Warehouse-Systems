/*********************************************************************************/
/* Author    : Gehad Elkoumy                                                     */
/* Version   : V01                                                               */
/* Date      : 29 NOV 2022                                                       */
/*********************************************************************************/
#ifndef DIO_PRIVATE_H
#define DIO_PRIVATE_H


#define GPIOA_Base_Address        0x40010800            // Base address of GPIO port A
#define GPIOB_Base_Address        0x40010C00            // Base address of GPIO port B
#define GPIOC_Base_Address        0x40011000            // Base address of GPIO port C

/*  Register Definitions for Port A (Base address + offset)  */
		
#define GPIOA_CRL         *((volatile u32*)(GPIOA_Base_Address + 0x00))
#define GPIOA_CRH         *((volatile u32*)(GPIOA_Base_Address + 0x04))
#define GPIOA_IDR         *((volatile u32*)(GPIOA_Base_Address + 0x08))
#define GPIOA_ODR         *((volatile u32*)(GPIOA_Base_Address + 0x0c))
#define GPIOA_BSRR        *((volatile u32*)(GPIOA_Base_Address + 0x10))
#define GPIOA_BRR         *((volatile u32*)(GPIOA_Base_Address + 0x14))
#define GPIOA_LCKR        *((volatile u32*)(GPIOA_Base_Address + 0x18))


/*  Register Definitions for Port B (Base address + offset)  */
		
#define GPIOB_CRL         *((volatile u32*)(GPIOB_Base_Address + 0x00))
#define GPIOB_CRH         *((volatile u32*)(GPIOB_Base_Address + 0x04))
#define GPIOB_IDR         *((volatile u32*)(GPIOB_Base_Address + 0x08))
#define GPIOB_ODR         *((volatile u32*)(GPIOB_Base_Address + 0x0c))
#define GPIOB_BSRR        *((volatile u32*)(GPIOB_Base_Address + 0x10))
#define GPIOB_BRR         *((volatile u32*)(GPIOB_Base_Address + 0x14))
#define GPIOB_LCKR        *((volatile u32*)(GPIOB_Base_Address + 0x18))


/*  Register Definitions for Port C (Base address + offset)  */
		
#define GPIOC_CRL         *((volatile u32*)(GPIOC_Base_Address + 0x00))
#define GPIOC_CRH         *((volatile u32*)(GPIOC_Base_Address + 0x04))
#define GPIOC_IDR         *((volatile u32*)(GPIOC_Base_Address + 0x08))
#define GPIOC_ODR         *((volatile u32*)(GPIOC_Base_Address + 0x0c))
#define GPIOC_BSRR        *((volatile u32*)(GPIOC_Base_Address + 0x10))
#define GPIOC_BRR         *((volatile u32*)(GPIOC_Base_Address + 0x14))
#define GPIOC_LCKR        *((volatile u32*)(GPIOC_Base_Address + 0x18))

/*lock key bit*/
#define LCKK  16

#endif