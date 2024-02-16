#include "stdint.h"
#include "ErrType.h"
#include "stm32F446xx.h"
#include "SYSTICK_Interface.h"

extern SYSTICK_CONFIG_t SYSTICK_TIMER_CONFIG ;
   
ERRORS_t SYSTICK_Delayms(uint32_t Copy_u32TimeInMillis)
{
	ERRORS_t Local_u8ErrorStatus = OK ;

	/* Variable To Carry RELOAD Value */
	uint32_t RELOAD_Value = 0 ;

	/* Check on TIMER_CLK  */
	if( SYSTICK_TIMER_CONFIG.CLK == SYSTICK_AHB )
	{
		RELOAD_Value = ( Copy_u32TimeInMillis * 1000UL ) / AHB_TICK_TIME ;
	}
	else if( SYSTICK_TIMER_CONFIG.CLK == SYSTICK_AHB_BY8 )
	{
		RELOAD_Value = ( Copy_u32TimeInMillis * 1000UL ) / AHB_BY8_TICK_TIME ;
	}
	else
	{
		Local_u8ErrorStatus = NOK ;
	}

	/* Setting Reload Value --- register which the value put */
	( SYSTICK->SYST_RVR ) = RELOAD_Value ;

	/* Clear Current --- register which count in it */
	( SYSTICK->SYST_CVR ) = 0 ;

	/* Set Exception */
	( SYSTICK->SYST_CSR ) &= ~(1<<CSR_TICKINT) ;
	( SYSTICK->SYST_CSR ) |= ( ( SYSTICK_TIMER_CONFIG.Exception ) << CSR_TICKINT ) ;

	/* Set Clock Source */
	( SYSTICK->SYST_CSR ) &= ~(1<<CSR_CLKSOURCE) ;
	( SYSTICK->SYST_CSR ) |= ( ( SYSTICK_TIMER_CONFIG.CLK ) << CSR_CLKSOURCE ) ;

	/* Enable Timer */
	( SYSTICK->SYST_CSR ) |= ( 1 << CSR_ENABLE ) ;

	/* Check on Flag */
	while( !( ( (SYSTICK->SYST_CSR)>>CSR_COUNTFLAG )&0x01) ) ;

	/* Disable Timer */
	( SYSTICK->SYST_CSR ) &= ~( 1 << CSR_ENABLE ) ;

	return Local_u8ErrorStatus ;
}



ERRORS_t SYSTICK_Delayus(uint32_t Copy_u32TimeInMicroSeconds)
{
	ERRORS_t Local_u8ErrorStatus = OK ;

	/* Variable To Carry RELOAD Value */
	uint32_t RELOAD_Value = 0 ;

	/* Check on TIMER_CLK  */
	if( SYSTICK_TIMER_CONFIG.CLK == SYSTICK_AHB )
	{
		RELOAD_Value = ( Copy_u32TimeInMicroSeconds  / AHB_TICK_TIME ) ;
	}
	else if( SYSTICK_TIMER_CONFIG.CLK == SYSTICK_AHB_BY8 )
	{
		RELOAD_Value = ( Copy_u32TimeInMicroSeconds  / AHB_BY8_TICK_TIME ) ;
	}

	/* Setting Reload Value */
	( SYSTICK->SYST_RVR ) = RELOAD_Value ;

	/* Clear Current */
	( SYSTICK->SYST_CVR ) = 0 ;

	/* Set Exception */
	( SYSTICK->SYST_CSR ) &= ~(1<<CSR_TICKINT) ;
	( SYSTICK->SYST_CSR ) |= ( ( SYSTICK_TIMER_CONFIG.Exception ) << CSR_TICKINT ) ;

	/* Set Clock Source */
	( SYSTICK->SYST_CSR ) &= ~(1<<CSR_CLKSOURCE) ;
	( SYSTICK->SYST_CSR ) |= ( ( SYSTICK_TIMER_CONFIG.CLK ) << CSR_CLKSOURCE ) ;

	/* Enable Timer */
	( SYSTICK->SYST_CSR ) |= ( 1 << CSR_ENABLE ) ;

	/* Check on Flag */
	while( !( ( (SYSTICK->SYST_CSR)>>CSR_COUNTFLAG )&0x01) ) ;

	/* Disable Timer */
	( SYSTICK->SYST_CSR ) &= ~( 1 << CSR_ENABLE ) ;

	return Local_u8ErrorStatus ;
	
	
	
	
}