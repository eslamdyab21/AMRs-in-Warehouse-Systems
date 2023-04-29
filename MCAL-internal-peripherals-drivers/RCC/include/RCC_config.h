/*********************************************************************************/
/* Author    : Gehad Elkoumy                                                     */
/* Version   : V01                                                               */
/* Data      : 5 NOV                                                             */
/*********************************************************************************/
#ifndef _RCC_CONFIG_H
#define _RCC_CONFIG_H
/*__________________________________________________________________________*/
/* Options:    RCC_HSE_CRYSTAL
               RCC_HSE_RC
               RCC_HSI
               RCC_PLL			   */
#define RCC_CLOCK_TYPE    RCC_HSE_CRYSTAL

/*__________________________________________________________________________*/
/* Note: Select value only if you have PLL as input clock source */
#if RCC_CLOCK_TYPE == RCC_PLL

/* Options:     RCC_PLL_IN_HSI_DIV_2
                RCC_PLL_IN_HSE_DIV_2
				RCC_PLL_IN_HSE      */
#define RCC_PLL_INPUT     RCC_PLL_IN_HSE_DIV_2

/* Options: 2 to 16 */
#define RCC_PLL_MUL_VALUE     4
#endif 

#endif