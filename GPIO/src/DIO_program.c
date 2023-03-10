/*********************************************************************************/
/* Author    : Gehad Elkoumy                                                     */
/* Version   : V01                                                               */
/* Date      : 29 NOV 2022                                                       */
/*********************************************************************************/

#include "STD_TYPES.h"
#include "BIT_MATH.h"

#include "DIO_interface.h"
#include "DIO_private.h"
#include "DIO_config.h"

void MGPIO_VoidSetPinDirection(u8 Copy_u8Port , u8 Copy_u8Pin , u8 Copy_u8Mode)
{
	switch(Copy_u8Port)
	{
	case GPIOA:
		/*CRL controls form pin0 to pin7 (each pin has 4 bits)*/
		if(Copy_u8Pin <= 7 )
		{
			GPIOA_CRL &= ~ ( ( 0b1111 )  << ( Copy_u8Pin * 4 ));  /*Reset or clear pin (4bits)*/
			GPIOA_CRL |= ( Copy_u8Mode ) << ( Copy_u8Pin * 4 );   /*apply mode on pin (interface file)*/
		}
		
		/*CRH controls form pin8 to pin15 (each pin has 4 bits)*/
		else if(Copy_u8Pin <= 15 )
		{
			Copy_u8Pin = Copy_u8Pin - 8;
			GPIOA_CRH &= ~ ( ( 0b1111 )  << ( Copy_u8Pin * 4 ) );
			GPIOA_CRH |= ( Copy_u8Mode ) << ( Copy_u8Pin * 4 );
		}

		break;

	case GPIOB:
		/*CRL controls form pin0 to pin7 (each pin has 4 bits)*/
		if(Copy_u8Pin <= 7 )
		{
			/*Reset pin then overwrite*/
			GPIOB_CRL &= ~ ( ( 0b1111 )  << ( Copy_u8Pin * 4 ) );    /*Reset or clear pin*/
			GPIOB_CRL |= ( Copy_u8Mode ) << ( Copy_u8Pin * 4 )  ;    /*apply mode on pin*/
		}
		
		/*CRH controls form pin8 to pin15 (each pin has 4 bits)*/
		else if(Copy_u8Pin <= 15 )
		{
			Copy_u8Pin = Copy_u8Pin - 8;
			GPIOB_CRH &= ~ ( ( 0b1111 )  << ( Copy_u8Pin * 4 ) );
			GPIOB_CRH |= ( Copy_u8Mode ) << ( Copy_u8Pin * 4 )  ;
		}
		
		break;
		
	case GPIOC:
		/*CRL controls form pin0 to pin7 (each pin has 4 bits)*/
		if(Copy_u8Pin <= 7 )
		{
			GPIOC_CRL &= ~ ( ( 0b1111 )  << ( Copy_u8Pin * 4 ) );// R M W
			GPIOC_CRL |= ( Copy_u8Mode ) << ( Copy_u8Pin * 4 )  ;
		}
		
		/*CRH controls form pin8 to pin15 (each pin has 4 bits)*/
		else if(Copy_u8Pin <= 15 )
		{
			Copy_u8Pin = Copy_u8Pin - 8;
			GPIOC_CRH &= ~ ( ( 0b1111 )  << ( Copy_u8Pin * 4 ) );
			GPIOC_CRH |= ( Copy_u8Mode ) << ( Copy_u8Pin * 4 )  ;
		}
		
		break;

	}
}


void MGPIO_VoidSetPinValue(u8 Copy_u8Port , u8 Copy_u8Pin , u8 Copy_u8Value)
{
	switch(Copy_u8Port)
	{
	/*output data high or low for each pin*/
	case GPIOA:
		if( Copy_u8Value == HIGH )
		{
			//SET_BIT( GPIOA_ODR , Copy_u8Pin );
			GPIOA_BSRR = (1 << Copy_u8Pin);     /*speed up -- 0 has no effect*/
		}
		else if( Copy_u8Value == LOW )
		{
			//CLR_BIT( GPIOA_ODR , Copy_u8Pin );
			GPIOA_BRR = (1 << Copy_u8Pin);    /*speed up*/
			//GPIOA_BSRR = (1 << (Copy_u8Pin + 16));
		}
		break;
		
	case GPIOB:
		if( Copy_u8Value == HIGH )
		{
			//SET_BIT( GPIOB_ODR  , Copy_u8Pin );
			GPIOB_BSRR = (1 << Copy_u8Pin);
		}
		else if( Copy_u8Value == LOW )
		{
			//CLR_BIT( GPIOB_ODR  , Copy_u8Pin );
			GPIOB_BRR = (1 << Copy_u8Pin);
		}
		break;
		
	case GPIOC:
		if( Copy_u8Value == HIGH )
		{
			//SET_BIT( GPIOC_ODR , Copy_u8Pin );
			GPIOC_BSRR = (1 << Copy_u8Pin);
		}
		else if( Copy_u8Value == LOW )
		{
			//CLR_BIT( GPIOC_ODR , Copy_u8Pin );
			GPIOC_BRR = (1 << Copy_u8Pin);
		}
		break;

	}
}


u8 MGPIO_u8GetPinValue(u8 Copy_u8Port , u8 Copy_u8Pin)
{
	u8 LOC_u8Result = 0 ;   //return value

	switch(Copy_u8Port)
	{
	/*get input data*/
	case GPIOA:
		LOC_u8Result = GET_BIT( GPIOA_IDR , Copy_u8Pin );
		break;
		
	case GPIOB:
		LOC_u8Result = GET_BIT( GPIOB_IDR , Copy_u8Pin );
		break;
		
	case GPIOC:
		LOC_u8Result = GET_BIT( GPIOC_IDR , Copy_u8Pin );
		break;
	}
	
	return LOC_u8Result;
}


                             /*lock pin mode*/
/*Each lock bit freezes the corresponding 4 bits of the control register(CRL,CRH)*/
void MGPIO_VoidLockPin(u8 Copy_u8Port , u8 Copy_u8Pin)
{
	switch(Copy_u8Port)
	{
	case GPIOA:
	
		/*locked pin*/
		CLR_BIT(GPIOA_LCKR , LCKK);
		GPIOA_LCKR = (1 << Copy_u8Pin);
		
		/*activate the lock key*/
		SET_BIT(GPIOA_LCKR , LCKK);
		
		/*writing sequence*/
		SET_BIT(GPIOA_LCKR , LCKK);
		CLR_BIT(GPIOA_LCKR , LCKK);
		SET_BIT(GPIOA_LCKR , LCKK);
		CLR_BIT(GPIOA_LCKR , LCKK);
		
		break;
		
		
	case GPIOB:
		/*locked pin*/
		CLR_BIT(GPIOB_LCKR , LCKK);
		GPIOB_LCKR = (1 << Copy_u8Pin);
		
		/*activate the lock key*/
		SET_BIT(GPIOB_LCKR , LCKK);
		
		/*writing sequence*/
		SET_BIT(GPIOB_LCKR , LCKK);
		CLR_BIT(GPIOB_LCKR , LCKK);
		SET_BIT(GPIOB_LCKR , LCKK);
		CLR_BIT(GPIOB_LCKR , LCKK);
		
		break;
		
		
	case GPIOC:
		/*locked pin*/
		CLR_BIT(GPIOC_LCKR , LCKK);
		GPIOC_LCKR = (1 << Copy_u8Pin);
		
		/*activate the lock key*/
		SET_BIT(GPIOC_LCKR , LCKK);
		
		/*writing sequence*/
		SET_BIT(GPIOC_LCKR , LCKK);
		CLR_BIT(GPIOC_LCKR , LCKK);
		SET_BIT(GPIOC_LCKR , LCKK);
		CLR_BIT(GPIOC_LCKR , LCKK);
		
		break;
	}
	
}
