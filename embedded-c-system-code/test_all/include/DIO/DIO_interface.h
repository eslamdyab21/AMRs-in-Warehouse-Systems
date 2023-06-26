/*********************************************************************************/
/* Author    : Gehad Elkoumy                                                     */
/* Version   : V01                                                               */
/* Date      : 29 NOV 2022                                                       */
/*********************************************************************************/
#ifndef DIO_INTERFACE_H
#define DIO_INTERFACE_H

#define HIGH    1
#define LOW     0

/* Port Id */
#define GPIOA   0
#define GPIOB   1
#define GPIOC   2

/* Pin Id*/
#define PIN0    0
#define PIN1    1
#define PIN2    2
#define PIN3    3
#define PIN4    4
#define PIN5    5
#define PIN6    6
#define PIN7    7
#define PIN8    8
#define PIN9    9
#define PIN10    10
#define PIN11    11
#define PIN12    12
#define PIN13    13
#define PIN14    14
#define PIN15    15

/*Input mode with configuration*/
#define INPUT_ANALOG              0b0000
#define INPUT_FLOATING            0b0100
#define INPUT_PULLUP_PULLDOWN     0b1000

/*Output mode 10MHZ with configuration*/
#define OUTPUT_10MHZ_PP           0b0001     //push pull
#define OUTPUT_10MHZ_OD           0b0101     //open drain
#define OUTPUT_10MHZ_AFPP         0b1001     //Alternative function push pull
#define OUTPUT_10MHZ_AFOD         0b1101     //Alernative function open drain 

/*Output mode 2MHZ with configuration*/
#define OUTPUT_2MHZ_PP            0b0010     //push pull
#define OUTPUT_2MHZ_OD            0b0110     //open drain
#define OUTPUT_2MHZ_AFPP          0b1010     //Alternative function push pull
#define OUTPUT_2MHZ_AFOD          0b1110     //Alernative function open drain

/*Output mode 50MHZ with configuration*/
#define OUTPUT_50MHZ_PP           0b0011     //push pull
#define OUTPUT_50MHZ_OD           0b0111     //open drain
#define OUTPUT_50MHZ_AFPP         0b1011     //Alternative function push pull
#define OUTPUT_50MHZ_AFOD         0b1111     //Alernative function open drain


void MGPIO_VoidSetPinDirection(u8 Copy_u8Port , u8 Copy_u8Pin , u8 Copy_u8Mode);
void MGPIO_VoidSetPinValue(u8 Copy_u8Port , u8 Copy_u8Pin , u8 Copy_u8Value);
u8 MGPIO_u8GetPinValue(u8 Copy_u8Port , u8 Copy_u8Pin);
void MGPIO_VoidLockPin(u8 Copy_u8Port , u8 Copy_u8Pin);


#endif
