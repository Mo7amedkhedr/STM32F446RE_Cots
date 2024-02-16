#include <stdint.h>
#include "stm32F446xx.h"
#include "ErrType.h"
#include "SysCfg_interface.h"


void SYSFG_voidSetEXTIPort(EXTI_t Local_u8Line, GPIO_PORT_t Local_u8Port)
{
    /*Variable To Hold Register Index */
    uint8_t Local_u8RegIndex = Local_u8Line / 4;
    /*Variable To Hold Bit Index */
    uint8_t Local_u8BitIndex = (Local_u8Line % 4) * 4;
    /*Clearing The 4 bits Required */
    SYSCFG->EXTICR[Local_u8RegIndex] &= ~(0xF << Local_u8BitIndex);
    /*Setting The 4 bits Required */
    SYSCFG->EXTICR[Local_u8RegIndex] |= (Local_u8Port << Local_u8BitIndex);
}