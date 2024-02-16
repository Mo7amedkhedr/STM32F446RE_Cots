#ifndef SCB_INTERFACE_H
#define SCB_INTERFACE_H

typedef enum
{
    GP_16_SP_0 = 0x05FA0000UL, // 16 Group of Priority & 0 Sub Group of Priority
    GP_8_SP_2 = 0x05FA0400UL,  // 8 Group of Priority & 2 Sub Group of Priority
    GP_4_SP_4 = 0x05FA0500UL,  // 4 Group of Priority & 4 Sub Group of Priority
    GP_2_SP_8 = 0x05FA0600UL,  // 2 Group of Priority & 8 Sub Group of Priority
    GP_0_SP_16 = 0x05FA0700UL, // 0 Group of Priority & 16 Sub Group of Priority
	
} PRIGROUP_t;



typedef enum
{
    MEMFAULT = 16,
    BUSFAULT,
    USAGEFAULT
	
} Fault_t;

typedef enum
{
    NMI_HANDLER = 0,
    HARDFAULT_HANDLER,
    MEMMANAGE_HANDLER,
    BUSFAULT_HANDLER,
    USAGEFAULT_HANDLER,
    SVCALL_HANDLER,
    DEBUGMONITOR_HANDLER,
    PENDSV_HANDLER
	
} HANDLER_t;



typedef enum
{
MemoryManagment_FAULT =4   ,
BusFault_FAULT=12,
Usage_FAULT=20,
SVCCALL_FAULT=28,
PENDSV_FAULT=20  ,
SYSTICK_FAULT=28,
}SystemFault_t;



void SCB_VoidSetPriorityGroup(PRIGROUP_t Local_u8PriorityGroup);

void SCB_VoidEnableFaultException(Fault_t Local_Fault);

void SCB_VoidDisableFaultException(Fault_t Local_Fault);

uint8_t SCB_u8SetHandler(HANDLER_t Local_Handler, void (*Local_ptrToFunction)(void))

void SCB_VoidSetCorePriority (SystemFault_t SystemFault , uint8_t  Copy_u8Priority);
















#endif