/**********************************************************/
/* Author    : Gehad Elkoumy                              */
/* Date      : 30 NOV 2022                                */
/* Version   : V01                                        */
/**********************************************************/

#ifndef AFIO_INTERFACE_H
#define AFIO_INTERFACE_H

void	MAFIO_voidSetEXTIConfiguration(u8 Copy_u8Line ,u8 Copy_u8PortMap);

/*portmap used as an interrupt*/
#define AFIOA   0b0000
#define AFIOB   0b0001
#define AFIOC   0b0010

/*lines are in EXTI driver*/

#endif