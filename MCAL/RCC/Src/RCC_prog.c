#include "stdint.h"
#include "stm32F446xx.h"
#include "bit_math.h"
#include "ErrType.h"
#include "RCC_interface.h"


uint8_t RCC_SetClkStatus(RCC_CLK_TYPE clkType, RCC_CLK_STATUS status)
{
	uint32_t counter=0;
	
	switch(clkType)
	{
		
		case HSI :
		    if(status == CLK_ON)
			{
				
				RCC->CR |= (1<<CR_HSION);
				while ((counter <= TIME_OUT) && (!(GET_BIT(RCC->CR,CR_HSIRDY)))
				{
					counter++;
				}
				if (counter == TIME_OUT)
				{
					counter = 0;
				}
			}
			if(status == CLK_OFF)
			{
				RCC->CR &= ~(1<<CR_HSION);
				
			}
			break;
  		case HSE:
            if(status == CLK_ON)
			{
				RCC->CR |= (1<<CR_HSEON);
				while ((counter <= TIME_OUT) && (!(GET_BIT(RCC->CR,CR_HSERDY)))
				{
					counter++;
				}
				if (counter == TIME_OUT)
				{
					counter = 0;
				}
			}
			if(status == CLK_OFF)
			{
				RCC->CR &= ~(1<<CR_HSEON);
				
			}
			break;				
		case PLL_MAIN:
		    if (status == CLK_ON)
			{
				RCC->CR |= (1<<CR_PLLON);
				while ((counter <= TIME_OUT) && (!(GET_BIT(RCC->CR,CR_PLLRDY)))
				{
					counter++;
				}
				if (counter == TIME_OUT)
				{
					counter = 0;
				}
			}
			if(status == CLK_OFF)
			{
				RCC->CR &= ~(1<<CR_PLLON);
				
			}
			break;
		
		
		
	}
	
	
	
}



void RCC_SetSysClk(SYSCLK_SRC sysClkType)
{
	
	 RCC->CFGR &= ~(TWO_BIT_MASKING << CFGR_SW);
     RCC->CFGR |= (sysClkType << CFGR_SW);
	
	
}

void RCC_HSEConfig(RCC_HSEConfig HseConfig)
{
	
	    RCC->CR &= ~(ONE_BIT_MASKING << CR_HSEBYP);
        RCC->CR |= (HseConfig << CR_HSEBYP);
		
}

void RCC_PLLConfig(RCC_PLL_CONFIG_t *pllConfig)
{
		RCC->PLLCFGR &=~(0x7F437FFF);
RCC ->PLLCFGR |= (pllConfig->PLLM_FACTOR << PLLCFGR_PLLM);
RCC ->PLLCFGR |= (pllConfig->PLLN_FACTOR << PLLCFGR_PLLN);
RCC ->PLLCFGR |= (pllConfig->PLLP_FACTOR << PLLCFGR_PLLP);
RCC ->PLLCFGR |= (pllConfig->PLLQ_FACTOR << PLLCFGR_PLLQ);
RCC ->PLLCFGR |= (pllConfig->PLLR_FACTOR << PLLCFGR_PLLR);
RCC ->PLLCFGR |= (pllConfig->PLL_SRC << PLLCFGR_PLLSRC);
	
	
}

void RCC_AHB1EnableCLK(RCC_AHB1ENReg_BITS_t Prephiral)
{
	RCC ->AHB1ENR |=(1<< Prephiral);
	
}

void RCC_AHB2EnableCLK(RCC_AHB2ENReg_BITS_t Prephiral)
{
	
	RCC ->AHB2ENR |=(1<< Prephiral);
}

void RCC_AHB3EnableCLK(RCC_AHB3ENReg_BITS_t Prephiral)
{
	RCC ->AHB3ENR |=( 1<< Prephiral);
}

void RCC_APB1EnableCLK(RCC_APB1ENReg_BITS_t Prephiral)
{
	RCC ->APB1ENR |=( 1<< Prephiral );
}
void RCC_APB2EnableCLK(RCC_APB2ENReg_BITS_t Prephiral)
{
	RCC ->APB2ENR |=(1<< Prephiral);
}