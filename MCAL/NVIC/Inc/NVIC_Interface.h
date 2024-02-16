#ifndef NVIC_INTERFACE_H_
#define NVIC_INTERFACE_H_

#define PRIORITY_REG_MASK 0xF0


typedef enum
{
	WWDG_IRQ = 0 , PVD_IRQ , TAMP_STAMP_IRQ ,RTC_WKUP_IRQ , FLASH_IRQ , RCC_IRQ , EXTI0_IRQ , EXTI1_IRQ , EXTI2_IRQ , EXTI3_IRQ , EXTI4_IRQ ,
	DMA1_Stream0_IRQ , DMA1_Stream1_IRQ , DMA1_Stream2_IRQ , DMA1_Stream3_IRQ , DMA1_Stream4_IRQ , DMA1_Stream5_IRQ , DMA1_Stream6_IRQ , ADC_IRQ ,
	CAN1_TX_IRQ , CAN1_RX0_IRQ , CAN1_RX1_IRQ , CAN1_SCE_IRQ , EXTI9_5_IRQ , TIM1_BRK_TIM9_IRQ , TIM1_UP_TIM10_IRQ ,
	TIM1_TRG_COM_TIM11_IRQ , TIM1_CC_IRQ , TIM2_IRQ , TIM3_IRQ , TIM4_IRQ , I2C1_EV_IRQ , I2C1_ER_IRQ , I2C2_EV_IRQ ,
	I2C2_ER_IRQ , SPI1_IRQ , SPI2_IRQ , USART1_IRQ , USART2_IRQ , USART3_IRQ ,EXTI15_10_IRQ , RTC_Alarm_IRQ ,OTG_FS_WKUP_IRQ , TIM8_BRK_TIM12_IRQ , TIM8_UP_TIM13_IRQ ,	TIM8_TRG_COM_TIM14_IRQ ,TIM8_CC_IRQ ,DMA1_Stream7_IRQ , FMC_IRQ , SDIO_IRQ , TIM5_IRQ ,SPI3_IRQ , UART4_IRQ ,
	UART5_IRQ ,	TIM6_DAC_IRQ , TIM7_IRQ ,DMA2_Stream0_IRQ , DMA2_Stream1_IRQ , DMA2_Stream2_IRQ , DMA2_Stream3_IRQ , DMA2_Stream4_IRQ ,
    CAN2_TX_IRQ = 63 , CAN2_RX0_IRQ ,CAN2_RX1_IRQ , CAN2_SCE_IRQ , OTG_FS_IRQ ,DMA2_Stream5_IRQ , DMA2_Stream6_IRQ , DMA2_Stream7_IRQ , USART6_IRQ ,
	I2C3_EV_IRQ , I2C3_ER_IRQ ,OTG_HS_EP1_OUT_IRQ , OTG_HS_EP1_IN_IRQ ,	OTG_HS_WKUP_IRQ ,OTG_HS_IRQ , DCMI_IRQ , FPU_IRQ = 81 ,
	SPI4_IRQ = 84 ,SAI1_IRQ = 87 , SAI2_IRQ = 91 , QuadSPI_IRQ , HDMI_CEC_IRQ , SPDIF_Rx_IRQ , FMPI2C1_IRQ ,FMPI2C1_ERROR_IRQ , NUMBER_OF_IRQs

}IRQNum_t;



typedef enum
{
	NOT_PENDING = 0 ,	PENDING

}PENDING_t;



typedef enum
{
  NOT_ACTIVE = 0 , ACTIVE

}ACTIVE_t;



Error_State_t NVIC_EnableIRQ( IRQNum_t IRQ_Number ) ;

Error_State_t NVIC_DisableIRQ( IRQNum_t IRQ_Number ) ;

Error_State_t NVIC_SetPendingIRQ( IRQNum_t IRQ_Number ) ;

Error_State_t NVIC_ClearPendingIRQ( IRQNum_t IRQ_Number ) ;


Error_State_t NVIC_GetPendingIRQ( IRQNum_t IRQ_Number , PENDING_t * Pending_State  );

Error_State_t NVIC_GetActiveIRQ( IRQNum_t IRQ_Number , ACTIVE_t * Active_State ) ;

Error_State_t NVIC_SetPriority( IRQNum_t IRQ_Number , uint8_t  Copy_u8Priority ) ;

#endif 