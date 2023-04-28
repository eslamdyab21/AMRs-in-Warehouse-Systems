//*****************************************/
/* Author  :  Gehad Elkoumy              */
/* Version :  V01                        */
/* Date    : 30 JAN 2022                 */
/*****************************************/

#ifndef STK_PRIVATE_H
#define STK_PRIVATE_H

typedef struct
{
	volatile u32 CTRL  ;
	volatile u32 LOAD  ;
	volatile u32 VAL   ;
	
}MSTK_t;

#define     MSTK    ((MSTK_t*) 0xE000E010)


#define     MSTK_SRC_AHB           0
#define     MSTK_SRC_AHB_8         1

#define     MSTK_SINGLE_INTERVAL    0
#define     MSTK_PERIOD_INTERVAL    1

/* Define Callback Global Variable */
static void(*MSTK_GlobalPtr)(void) = '\0';

#endif
