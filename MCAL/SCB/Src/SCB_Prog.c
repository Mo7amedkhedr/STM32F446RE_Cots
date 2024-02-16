#include <stdint.h>
#include "stm32F446xx.h"
#include "ErrType.h"
#include "SCB_Interface.h"

static void (*SCB_PTR_TO_FUNCTION[8])(void) = {NULL};

void SCB_VoidSetPriorityGroup(PRIGROUP_t Local_u8PriorityGroup)
{
	
	SCB->AIRCR = Local_u8PriorityGroup;
	
}

void SCB_VoidEnableFaultException(Fault_t Local_Fault)
{
	
	SCB->SHCSR |= (1 << Local_Fault);
	
}

void SCB_VoidDisableFaultException(Fault_t Local_Fault)
{
	
	SCB->SHCSR &= ~(1 << Local_Fault);
	
}

uint8_t SCB_u8SetHandler(HANDLER_t Local_Handler, void (*Local_ptrToFunction)(void))
{
	uint8_t Local_u8ErrorState = OK;
	if (Local_Handler < PENDSV_HANDLER && Local_ptrToFunction != NULL)
	{
		SCB_PTR_TO_FUNCTION[Local_Handler] = Local_ptrToFunction;
	}
	else
	{
		Local_u8ErrorState = NOK;
	}
	return Local_u8ErrorState;
}

void SCB_VoidSetCorePriority (SystemFault_t SystemFault , uint8_t  Copy_u8Priority)
{
	if( (SystemFault == MemoryManagment_FAULT) || (SystemFault ==BusFault_FAULT) || (SystemFault ==Usage_FAULT) )
	{
		SCB->SHPR[0] &= ~(15 << SystemFault);
		SCB->SHPR[0] |= (Copy_u8Priority << SystemFault);

	}
	else if (SystemFault == SVCCALL_FAULT)
	{
		SCB->SHPR[1] &= ~(15 << SystemFault);
		SCB->SHPR[1] |= (Copy_u8Priority << SystemFault);
	}
	else if( (SystemFault == SYSTICK_FAULT) || (SystemFault == PENDSV_FAULT) )
	{
		SCB->SHPR[2] &= ~(15 << SystemFault);
		SCB->SHPR[2] |= (Copy_u8Priority << SystemFault);
	}
	else
	{
		/* Error */
	}

}


/*==============================================================================================================================================
 * HANDLERS SECTION
 *==============================================================================================================================================*/
 void NMI_Handler(void)
{
	if (SCB_PTR_TO_FUNCTION[NMI_HANDLER] != NULL)
	{
		SCB_PTR_TO_FUNCTION[NMI_HANDLER]();
	}
}

void HardFault_Handler(void)
{
	if (SCB_PTR_TO_FUNCTION[HARDFAULT_HANDLER] != NULL)
	{
		SCB_PTR_TO_FUNCTION[HARDFAULT_HANDLER]();
	}
}

void MemManage_Handler(void)
{
	if (SCB_PTR_TO_FUNCTION[MEMMANAGE_HANDLER] != NULL)
	{
		SCB_PTR_TO_FUNCTION[MEMMANAGE_HANDLER]();
	}
}

void BusFault_Handler(void)
{
	if (SCB_PTR_TO_FUNCTION[BUSFAULT_HANDLER] != NULL)
	{
		SCB_PTR_TO_FUNCTION[BUSFAULT_HANDLER]();
	}
}

void UsageFault_Handler(void)
{
	if (SCB_PTR_TO_FUNCTION[USAGEFAULT_HANDLER] != NULL)
	{
		SCB_PTR_TO_FUNCTION[USAGEFAULT_HANDLER]();
	}
}

void SVC_Handler(void)
{
	if (SCB_PTR_TO_FUNCTION[SVCALL_HANDLER] != NULL)
	{
		SCB_PTR_TO_FUNCTION[SVCALL_HANDLER]();
	}
}

void DebugMon_Handler(void)
{
	if (SCB_PTR_TO_FUNCTION[DEBUGMONITOR_HANDLER] != NULL)
	{
		SCB_PTR_TO_FUNCTION[DEBUGMONITOR_HANDLER]();
	}
}

void PendSV_Handler(void)
{
	if (SCB_PTR_TO_FUNCTION[PENDSV_HANDLER] != NULL)
	{
		SCB_PTR_TO_FUNCTION[PENDSV_HANDLER]();
	}
}
}
}
}
}
}
}
}
}
}
}
}