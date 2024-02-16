#include <stdint.h>
#include "stm32F446xx.h"
#include "ErrType.h"
#include "NVIC_Interface.h"

Error_State_t NVIC_EnableIRQ( IRQNum_t IRQ_Number )
{
	Error_State_t Local_u8ErrorStatus = OK ;

	/* Variable to Holds Reg Number For the Specified IRQn */
	uint8_t REG_Number = ( IRQ_Number/32 ) ;

	/* Variable to Hold Bits Start For the Specified IRQn */
	uint8_t BITS_Start = ( IRQ_Number%32 ) ;

	if( IRQ_Number >= WWDG_IRQ && IRQ_Number < NUMBER_OF_IRQs )
	{
		NVIC->ISER[ REG_Number ] = ( 1 << BITS_Start ) ;
	}
	else
	{
		Local_u8ErrorStatus = NOK ;
	}
	return Local_u8ErrorStatus ;
}



Error_State_t NVIC_DisableIRQ( IRQNum_t IRQ_Number )
{
	Error_State_t Local_u8ErrorStatus = OK ;

	/* Variable to Holds Reg Number For the Specified IRQn */
	uint8_t REG_Number = ( IRQ_Number/32 ) ;

	/* Variable to Hold Bits Start For the Specified IRQn */
	uint8_t BITS_Start = ( IRQ_Number%32 ) ;

	if( IRQ_Number >= WWDG_IRQ && IRQ_Number < NUMBER_OF_IRQs )
	{
		NVIC->ICER[ REG_Number ] = ( 1 << BITS_Start ) ;
	}
	else
	{
		Local_u8ErrorStatus = NOK ;
	}
	return Local_u8ErrorStatus ;
}


Error_State_t NVIC_SetPendingIRQ( IRQNum_t IRQ_Number )
{
	Error_State_t Local_u8ErrorStatus = OK ;

	/* Variable to Holds Reg Number For the Specified IRQn */
	uint8_t REG_Number = ( IRQ_Number / 32 ) ;

	/* Variable to Hold Bits Start For the Specified IRQn */
	uint8_t BITS_Start = ( IRQ_Number % 32 ) ;

	if( IRQ_Number >= WWDG_IRQ && IRQ_Number < NUMBER_OF_IRQs )
	{
		NVIC->ISPR[ REG_Number ] = ( 1 << BITS_Start ) ;
	}
	else
	{
		Local_u8ErrorStatus = NOK ;
	}
	return Local_u8ErrorStatus ;
}



Error_State_t NVIC_ClearPendingIRQ( IRQNum_t IRQ_Number )
{
	Error_State_t Local_u8ErrorStatus = OK ;

	/* Variable to Holds Reg Number For the Specified IRQn */
	uint8_t REG_Number = ( IRQ_Number/32 ) ;

	/* Variable to Hold Bits Start For the Specified IRQn */
	uint8_t BITS_Start = ( IRQ_Number%32 ) ;

	if( (IRQ_Number >= WWDG_IRQ) && (IRQ_Number < NUMBER_OF_IRQs) )
	{
		NVIC->ICPR[ REG_Number ] = ( 1 << BITS_Start ) ;
	}
	else
	{
		Local_u8ErrorStatus = NOK ;
	}
	return Local_u8ErrorStatus ;
}


Error_State_t NVIC_GetPendingIRQ( IRQNum_t IRQ_Number , PENDING_t * Pending_State  )
{
	Error_State_t Local_u8ErrorStatus = OK ;

	/* Variable to Holds Reg Number For the Specified IRQn */
	uint8_t REG_Number = ( IRQ_Number/32 ) ;

	/* Variable to Hold Bits Start For the Specified IRQn */
	uint8_t BITS_Start = ( IRQ_Number%32 ) ;

	if(  NULL != Pending_State )
	{
		if( (IRQ_Number >= WWDG_IRQ) && (IRQ_Number < NUMBER_OF_IRQs)  )
		{
			if( ( ( NVIC->ISPR[ REG_Number ] >> ( BITS_Start ) ) & 0x01 ) )
			{
				*Pending_State = PENDING ;
			}
			else
			{
				*Pending_State = NOT_PENDING ;
			}
		}
		else
		{
			Local_u8ErrorStatus = NOK ;
		}
	}
	else
	{
		Local_u8ErrorStatus = NULL_POINTER ;
	}
	return Local_u8ErrorStatus ;
}



Error_State_t NVIC_GetActiveIRQ( IRQNum_t IRQ_Number , ACTIVE_t * Active_State )
{
	Error_State_t Local_u8ErrorStatus = OK ;

	/* Variable to Holds Reg Number For the Specified IRQn */
	uint8_t REG_Number = ( IRQ_Number/32 ) ;

	/* Variable to Hold Bits Start For the Specified IRQn */
	uint8_t BITS_Start = ( IRQ_Number%32 ) ;

	if( NULL !=  Active_State )
	{
		if( (IRQ_Number >= WWDG_IRQ) && (IRQ_Number < NUMBER_OF_IRQs)  )
		{
			if( ( ( NVIC->IABR[ REG_Number ] >> BITS_Start ) & 0x01 ) )
			{
				*Active_State = ACTIVE ;
			}
			else
			{
				*Active_State = NOT_ACTIVE ;
			}
		}
		else
		{
			Local_u8ErrorStatus = NOK ;
		}
	}
	else
	{
		Local_u8ErrorStatus = NULL_POINTER ;
	}
	return Local_u8ErrorStatus ;
}


Error_State_t NVIC_SetPriority( IRQNum_t IRQ_Number , uint8_t  Copy_u8Priority )
{
	Error_State_t Local_u8ErrorStatus = OK ;

	if( (IRQ_Number >= WWDG_IRQ) && (IRQ_Number  < NUMBER_OF_IRQs) )
	{
		/* BIT MASKING */
		NVIC->IPR[IRQ_Number] &= ( ~ ( PRIORITY_REG_MASK ) ) ;
		NVIC->IPR[IRQ_Number] = ( Copy_u8Priority << 4 ) ;
	}
	else
	{
		Local_u8ErrorStatus = NOK ;
	}
	return Local_u8ErrorStatus ;
}